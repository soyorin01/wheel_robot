#include <libobsensor/ObSensor.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace wheel_robot
{

class AstraCameraNode : public rclcpp::Node
{
public:
  AstraCameraNode()
  : Node("astra_camera"), running_(false)
  {
    declare_parameters();
    read_parameters();
    validate_parameters();

    // 实时相机画面只保留最新一帧。处理端偶尔变慢时允许丢帧，避免旧帧
    // 在队列中累积后造成肉眼可见的显示延迟。
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (enable_color_) {
      try {
        open_color_camera();
        color_camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
          this, color_camera_name_, color_camera_info_url_);
        color_image_pub_ = create_publisher<sensor_msgs::msg::Image>("color/image_raw", qos);
        color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("color/camera_info", qos);

        if (color_camera_info_manager_->isCalibrated()) {
          RCLCPP_INFO(
            get_logger(), "Loaded color calibration for camera '%s' from %s",
            color_camera_name_.c_str(), color_camera_info_url_.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Color camera is uncalibrated; /camera/color/camera_info will have K[0] == 0");
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(),
          "Color camera unavailable (%s); continuing with depth only for distance test",
          error.what());
        color_capture_.release();
        enable_color_ = false;
      }
    }

    try {
      if (enable_depth_) {
        depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", qos);
        depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", qos);
        open_depth_camera();
      }
    } catch (...) {
      color_capture_.release();
      close_depth_camera();
      throw;
    }

    running_.store(true);
    if (enable_color_) {
      color_thread_ = std::thread(&AstraCameraNode::color_loop, this);
    }
    if (enable_depth_) {
      depth_thread_ = std::thread(&AstraCameraNode::depth_loop, this);
    }

    RCLCPP_INFO(
      get_logger(),
      "Astra Pro camera started (color=%s, depth=%s). No control topics are published.",
      enable_color_ ? "on" : "off", enable_depth_ ? "on" : "off");
  }

  ~AstraCameraNode() override
  {
    running_.store(false);
    stop_depth_pipeline();
    color_capture_.release();

    if (color_thread_.joinable()) {
      color_thread_.join();
    }
    if (depth_thread_.joinable()) {
      depth_thread_.join();
    }
    close_depth_camera();
  }

private:
  void declare_parameters()
  {
    declare_parameter("enable_color", true);
    declare_parameter("enable_depth", true);
    declare_parameter(
      "color_device",
      "/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0");
    declare_parameter("color_width", 640);
    declare_parameter("color_height", 480);
    declare_parameter("color_fps", 30.0);
    declare_parameter("color_pixel_format", "AUTO");
    declare_parameter("color_camera_name", "astra_pro_color");
    declare_parameter("color_camera_info_url", "");
    declare_parameter("depth_width", 640);
    declare_parameter("depth_height", 480);
    declare_parameter("depth_fps", 30);
    declare_parameter("depth_unit_mm", 10.0);
    declare_parameter("color_frame_id", "camera_color_optical_frame");
    declare_parameter("depth_frame_id", "camera_depth_optical_frame");
  }

  void read_parameters()
  {
    enable_color_ = get_parameter("enable_color").as_bool();
    enable_depth_ = get_parameter("enable_depth").as_bool();
    color_device_ = get_parameter("color_device").as_string();
    color_width_ = static_cast<int>(get_parameter("color_width").as_int());
    color_height_ = static_cast<int>(get_parameter("color_height").as_int());
    color_fps_ = get_parameter("color_fps").as_double();
    color_pixel_format_ = get_parameter("color_pixel_format").as_string();
    color_camera_name_ = get_parameter("color_camera_name").as_string();
    color_camera_info_url_ = get_parameter("color_camera_info_url").as_string();
    depth_width_ = static_cast<int>(get_parameter("depth_width").as_int());
    depth_height_ = static_cast<int>(get_parameter("depth_height").as_int());
    depth_fps_ = static_cast<int>(get_parameter("depth_fps").as_int());
    depth_unit_mm_ = get_parameter("depth_unit_mm").as_double();
    color_frame_id_ = get_parameter("color_frame_id").as_string();
    depth_frame_id_ = get_parameter("depth_frame_id").as_string();
  }

  void validate_parameters() const
  {
    if (!enable_color_ && !enable_depth_) {
      throw std::invalid_argument("enable_color and enable_depth cannot both be false");
    }
    if (enable_color_ && color_device_.empty()) {
      throw std::invalid_argument("color_device cannot be empty when color is enabled");
    }
    if (color_width_ <= 0 || color_height_ <= 0 || color_fps_ <= 0.0) {
      throw std::invalid_argument("color dimensions and frame rate must be positive");
    }
    if (depth_width_ <= 0 || depth_height_ <= 0 || depth_fps_ <= 0) {
      throw std::invalid_argument("depth dimensions and frame rate must be positive");
    }
    if (depth_unit_mm_ <= 0.0) {
      throw std::invalid_argument("depth_unit_mm must be positive");
    }
    if (color_pixel_format_ != "AUTO" && color_pixel_format_.size() != 4U) {
      throw std::invalid_argument("color_pixel_format must be AUTO or a four-character code");
    }
    if (enable_color_ && color_camera_name_.empty()) {
      throw std::invalid_argument("color_camera_name cannot be empty when color is enabled");
    }
  }

  void open_color_camera()
  {
    // icn.py 用 CAP_V4L2。CAP_ANY 在这块板上会走 GStreamer，by-id 路径经常打不开。
    color_capture_.open(color_device_, cv::CAP_V4L2);
    if (!color_capture_.isOpened()) {
      RCLCPP_WARN(
        get_logger(), "V4L2 open failed for %s, retrying CAP_ANY", color_device_.c_str());
      color_capture_.open(color_device_, cv::CAP_ANY);
    }
    if (!color_capture_.isOpened()) {
      throw std::runtime_error("failed to open color camera: " + color_device_);
    }

    color_capture_.set(cv::CAP_PROP_FRAME_WIDTH, color_width_);
    color_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, color_height_);
    if (color_pixel_format_ != "AUTO") {
      const int fourcc = cv::VideoWriter::fourcc(
        color_pixel_format_[0], color_pixel_format_[1],
        color_pixel_format_[2], color_pixel_format_[3]);
      color_capture_.set(cv::CAP_PROP_FOURCC, fourcc);
    }
    color_capture_.set(cv::CAP_PROP_FPS, color_fps_);

    RCLCPP_INFO(
      get_logger(), "Color camera: %s, %.0fx%.0f @ %.1f fps, backend=%s",
      color_device_.c_str(), color_capture_.get(cv::CAP_PROP_FRAME_WIDTH),
      color_capture_.get(cv::CAP_PROP_FRAME_HEIGHT),
      color_capture_.get(cv::CAP_PROP_FPS), color_capture_.getBackendName().c_str());
  }

  void open_depth_camera()
  {
    try {
      ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_ERROR);
      depth_pipeline_ = std::make_unique<ob::Pipeline>();
      const auto profiles = depth_pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
      const auto profile = profiles->getVideoStreamProfile(
        depth_width_, depth_height_, OB_FORMAT_Y11, depth_fps_);

      depth_config_ = std::make_shared<ob::Config>();
      depth_config_->setDepthScaleRequire(true);
      depth_config_->enableStream(profile);
      depth_pipeline_->start(depth_config_);
      depth_started_ = true;

      depth_camera_info_.width = profile->width();
      depth_camera_info_.height = profile->height();
      depth_camera_info_.distortion_model = "plumb_bob";
      try {
        const auto intrinsic = profile->getIntrinsic();
        const auto distortion = profile->getDistortion();
        depth_camera_info_.d = {
          distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3};
        depth_camera_info_.k = {
          intrinsic.fx, 0.0, intrinsic.cx,
          0.0, intrinsic.fy, intrinsic.cy,
          0.0, 0.0, 1.0};
        depth_camera_info_.r = {
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0};
        depth_camera_info_.p = {
          intrinsic.fx, 0.0, intrinsic.cx, 0.0,
          0.0, intrinsic.fy, intrinsic.cy, 0.0,
          0.0, 0.0, 1.0, 0.0};
      } catch (const ob::Error & error) {
        RCLCPP_WARN(
          get_logger(),
          "Depth factory calibration is unavailable (%s); publishing uncalibrated CameraInfo",
          error.getMessage());
      }

      const auto device_info = depth_pipeline_->getDevice()->getDeviceInfo();
      RCLCPP_INFO(
        get_logger(),
        "Depth camera: %s, PID=0x%04x, %ux%u @ %u fps, Y11 unit=%.3f mm",
        device_info->name(), device_info->pid(), profile->width(), profile->height(),
        profile->fps(), depth_unit_mm_);
    } catch (const ob::Error & error) {
      throw std::runtime_error(
              std::string("failed to start Astra depth stream: ") + error.getMessage());
    }
  }

  void close_depth_camera() noexcept
  {
    stop_depth_pipeline();
    depth_config_.reset();
    depth_pipeline_.reset();
  }

  void stop_depth_pipeline() noexcept
  {
    if (depth_pipeline_ && depth_started_) {
      try {
        depth_pipeline_->stop();
      } catch (...) {
        // Destructors and shutdown paths must not throw.
      }
      depth_started_ = false;
    }
  }

  sensor_msgs::msg::CameraInfo make_camera_info(
    const rclcpp::Time & stamp, const std::string & frame_id, uint32_t width,
    uint32_t height) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    // K[0] == 0 follows the CameraInfo convention for an uncalibrated camera.
    return info;
  }

  void color_loop()
  {
    cv::Mat frame;
    while (running_.load() && rclcpp::ok()) {
      if (!color_capture_.read(frame) || frame.empty()) {
        if (!running_.load()) {
          break;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to read an Astra color frame");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      if (frame.type() != CV_8UC3) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Unexpected color frame type: %d", frame.type());
        continue;
      }
      if (!frame.isContinuous()) {
        frame = frame.clone();
      }

      const auto stamp = now();
      auto image = std::make_unique<sensor_msgs::msg::Image>();
      image->header.stamp = stamp;
      image->header.frame_id = color_frame_id_;
      image->height = static_cast<uint32_t>(frame.rows);
      image->width = static_cast<uint32_t>(frame.cols);
      image->encoding = sensor_msgs::image_encodings::BGR8;
      image->is_bigendian = false;
      image->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
      image->data.resize(static_cast<size_t>(image->step) * image->height);
      std::memcpy(image->data.data(), frame.data, image->data.size());

      auto color_info = color_camera_info_manager_->isCalibrated() ?
        color_camera_info_manager_->getCameraInfo() :
        make_camera_info(stamp, color_frame_id_, image->width, image->height);
      color_info.header.stamp = stamp;
      color_info.header.frame_id = color_frame_id_;
      color_info_pub_->publish(color_info);
      color_image_pub_->publish(std::move(image));
    }
  }

  void depth_loop()
  {
    while (running_.load() && rclcpp::ok()) {
      std::shared_ptr<ob::DepthFrame> frame;
      try {
        const auto frames = depth_pipeline_->waitForFrames(500);
        if (frames) {
          frame = frames->depthFrame();
        }
      } catch (const ob::Error & error) {
        if (!running_.load()) {
          break;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to read Astra depth frame: %s",
          error.getMessage());
        continue;
      }
      if (!frame) {
        continue;
      }

      const auto stamp = now();
      auto image = std::make_unique<sensor_msgs::msg::Image>();
      image->header.stamp = stamp;
      image->header.frame_id = depth_frame_id_;
      image->height = frame->height();
      image->width = frame->width();
      image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      image->is_bigendian = false;
      image->step = image->width * sizeof(uint16_t);
      image->data.resize(static_cast<size_t>(image->step) * image->height);
      const size_t source_size = frame->dataSize();
      if (source_size < image->data.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Short depth frame: expected %zu bytes, received %zu", image->data.size(), source_size);
        continue;
      }

      const float scale_mm = static_cast<float>(depth_unit_mm_);
      if (std::abs(scale_mm - 1.0F) < 1.0e-6F) {
        std::memcpy(image->data.data(), frame->data(), image->data.size());
      } else {
        const auto * source = static_cast<const uint16_t *>(frame->data());
        auto * target = reinterpret_cast<uint16_t *>(image->data.data());
        const size_t pixel_count = static_cast<size_t>(image->width) * image->height;
        for (size_t index = 0; index < pixel_count; ++index) {
          const auto value_mm = std::lround(source[index] * scale_mm);
          target[index] = static_cast<uint16_t>(
            std::clamp<long>(value_mm, 0L, std::numeric_limits<uint16_t>::max()));
        }
      }

      depth_camera_info_.header.stamp = stamp;
      depth_camera_info_.header.frame_id = depth_frame_id_;
      depth_info_pub_->publish(depth_camera_info_);
      depth_image_pub_->publish(std::move(image));
    }
  }

  bool enable_color_{true};
  bool enable_depth_{true};
  std::string color_device_;
  int color_width_{640};
  int color_height_{480};
  double color_fps_{30.0};
  std::string color_pixel_format_;
  std::string color_camera_name_;
  std::string color_camera_info_url_;
  int depth_width_{640};
  int depth_height_{480};
  int depth_fps_{30};
  double depth_unit_mm_{10.0};
  std::string color_frame_id_;
  std::string depth_frame_id_;

  cv::VideoCapture color_capture_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> color_camera_info_manager_;
  bool depth_started_{false};
  std::unique_ptr<ob::Pipeline> depth_pipeline_;
  std::shared_ptr<ob::Config> depth_config_;
  sensor_msgs::msg::CameraInfo depth_camera_info_;
  std::atomic_bool running_;
  std::thread color_thread_;
  std::thread depth_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
};

}  // namespace wheel_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_robot::AstraCameraNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_camera"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
