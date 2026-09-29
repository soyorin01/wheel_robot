#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/header.hpp>
#include <wheel_robot/msg/obstacle_status.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace wheel_robot
{

struct RegionMeasurement
{
  float raw_distance_m{std::numeric_limits<float>::quiet_NaN()};
  float filtered_distance_m{std::numeric_limits<float>::quiet_NaN()};
  size_t valid_pixels{0U};
  int total_pixels{0};
  cv::Rect rectangle;
  uint8_t status{msg::ObstacleStatus::UNKNOWN};
};

class DepthDistanceNode : public rclcpp::Node
{
public:
  DepthDistanceNode()
  : Node("depth_distance")
  {
    roi_width_ = declare_parameter("roi_width", 180);
    roi_height_ = declare_parameter("roi_height", 150);
    roi_center_y_ratio_ = declare_parameter("roi_center_y_ratio", 0.42);
    distance_percentile_ = declare_parameter("distance_percentile", 0.10);
    min_valid_pixels_ = declare_parameter("min_valid_pixels", 50);
    temporal_window_ = declare_parameter("temporal_window", 3);
    min_distance_m_ = declare_parameter("min_distance_m", 0.15);
    max_distance_m_ = declare_parameter("max_distance_m", 8.0);
    field_of_view_rad_ = declare_parameter("field_of_view_rad", 0.27);
    stop_distance_m_ = declare_parameter("stop_distance_m", 0.60);
    caution_distance_m_ = declare_parameter("caution_distance_m", 1.00);
    visualization_max_distance_m_ =
      declare_parameter("visualization_max_distance_m", 1.20);
    visualization_auto_range_ = declare_parameter("visualization_auto_range", true);
    visualization_median_size_ = declare_parameter("visualization_median_size", 5);

    validate_parameters();

    // 障碍检测优先使用最新数据；队列深度为 1 可防止可视化计算较慢时
    // 继续处理已经过时的深度帧。
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    left_range_pub_ =
      create_publisher<sensor_msgs::msg::Range>("depth/left_distance", sensor_qos);
    center_range_pub_ =
      create_publisher<sensor_msgs::msg::Range>("depth/center_distance", sensor_qos);
    right_range_pub_ =
      create_publisher<sensor_msgs::msg::Range>("depth/right_distance", sensor_qos);
    visualization_pub_ =
      create_publisher<sensor_msgs::msg::Image>("depth/obstacle_view", sensor_qos);
    status_pub_ = create_publisher<msg::ObstacleStatus>("depth/obstacle_status", 10);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "depth/image_raw", sensor_qos,
      std::bind(&DepthDistanceNode::on_depth_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Obstacle detection started: L/C/R regions=%dx%d at y=%.2f, percentile=%.0f%%, "
      "CAUTION<=%.2fm, STOP<=%.2fm",
      roi_width_, roi_height_, roi_center_y_ratio_,
      distance_percentile_ * 100.0,
      caution_distance_m_, stop_distance_m_);
  }

private:
  void validate_parameters() const
  {
    if (roi_width_ <= 0 || roi_height_ <= 0) {
      throw std::invalid_argument("roi_width and roi_height must be positive");
    }
    if (roi_center_y_ratio_ < 0.10 || roi_center_y_ratio_ > 0.90) {
      throw std::invalid_argument("roi_center_y_ratio must be between 0.10 and 0.90");
    }
    if (distance_percentile_ < 0.0 || distance_percentile_ > 1.0) {
      throw std::invalid_argument("distance_percentile must be between 0 and 1");
    }
    if (min_valid_pixels_ <= 0 || temporal_window_ <= 0) {
      throw std::invalid_argument("min_valid_pixels and temporal_window must be positive");
    }
    if (min_distance_m_ < 0.0 || max_distance_m_ <= min_distance_m_) {
      throw std::invalid_argument("distance limits are invalid");
    }
    if (stop_distance_m_ <= min_distance_m_ ||
      caution_distance_m_ <= stop_distance_m_ ||
      caution_distance_m_ >= max_distance_m_)
    {
      throw std::invalid_argument("obstacle status thresholds are invalid");
    }
    if (visualization_max_distance_m_ <= min_distance_m_) {
      throw std::invalid_argument("visualization_max_distance_m is invalid");
    }
    if (visualization_median_size_ < 1 || visualization_median_size_ % 2 == 0) {
      throw std::invalid_argument("visualization_median_size must be an odd integer >= 1");
    }
  }

  static uint16_t read_depth_mm(
    const sensor_msgs::msg::Image & image, int x, int y)
  {
    const size_t offset = static_cast<size_t>(y) * image.step +
      static_cast<size_t>(x) * sizeof(uint16_t);
    uint16_t depth_mm = 0;
    std::memcpy(&depth_mm, image.data.data() + offset, sizeof(depth_mm));
    if (image.is_bigendian) {
      depth_mm = static_cast<uint16_t>((depth_mm >> 8U) | (depth_mm << 8U));
    }
    return depth_mm;
  }

  RegionMeasurement measure_region(
    const sensor_msgs::msg::Image & image, int center_x, int center_y)
  {
    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    const int sample_width = std::min(roi_width_, width);
    const int sample_height = std::min(roi_height_, height);
    const int start_x = std::clamp(center_x - sample_width / 2, 0, width - sample_width);
    const int start_y = std::clamp(center_y - sample_height / 2, 0, height - sample_height);
    const uint16_t min_mm = static_cast<uint16_t>(min_distance_m_ * 1000.0);
    const uint16_t max_mm = static_cast<uint16_t>(max_distance_m_ * 1000.0);

    valid_depths_.clear();
    valid_depths_.reserve(static_cast<size_t>(sample_width) * sample_height);
    for (int y = start_y; y < start_y + sample_height; ++y) {
      for (int x = start_x; x < start_x + sample_width; ++x) {
        const uint16_t depth_mm = read_depth_mm(image, x, y);
        if (depth_mm >= min_mm && depth_mm <= max_mm) {
          valid_depths_.push_back(depth_mm);
        }
      }
    }

    RegionMeasurement measurement;
    measurement.valid_pixels = valid_depths_.size();
    measurement.total_pixels = sample_width * sample_height;
    measurement.rectangle = cv::Rect(start_x, start_y, sample_width, sample_height);
    if (valid_depths_.size() >= static_cast<size_t>(min_valid_pixels_)) {
      const size_t index = static_cast<size_t>(
        distance_percentile_ * static_cast<double>(valid_depths_.size() - 1U));
      const auto selected = valid_depths_.begin() + static_cast<std::ptrdiff_t>(index);
      std::nth_element(valid_depths_.begin(), selected, valid_depths_.end());
      measurement.raw_distance_m = static_cast<float>(*selected) / 1000.0F;
    }
    return measurement;
  }

  float update_temporal_filter(size_t zone, float value)
  {
    auto & history = distance_history_[zone];
    history.push_back(value);
    while (history.size() > static_cast<size_t>(temporal_window_)) {
      history.pop_front();
    }

    temporal_values_.clear();
    for (const float sample : history) {
      if (std::isfinite(sample)) {
        temporal_values_.push_back(sample);
      }
    }
    const size_t required_samples = (history.size() + 1U) / 2U;
    if (temporal_values_.size() < required_samples) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    const auto middle = temporal_values_.begin() + temporal_values_.size() / 2U;
    std::nth_element(temporal_values_.begin(), middle, temporal_values_.end());
    return *middle;
  }

  uint8_t classify_distance(float distance_m) const
  {
    if (!std::isfinite(distance_m)) {
      return msg::ObstacleStatus::UNKNOWN;
    }
    if (distance_m <= stop_distance_m_) {
      return msg::ObstacleStatus::STOP;
    }
    if (distance_m <= caution_distance_m_) {
      return msg::ObstacleStatus::CAUTION;
    }
    return msg::ObstacleStatus::CLEAR;
  }

  static uint8_t overall_status(const std::array<RegionMeasurement, 3> & measurements)
  {
    for (const auto & measurement : measurements) {
      if (measurement.status == msg::ObstacleStatus::STOP) {
        return msg::ObstacleStatus::STOP;
      }
    }

    // The forward zone is mandatory.  A single blind side is degraded to
    // CAUTION so the patrol controller can move slowly and steer away from it;
    // losing both side zones still means that the wide vehicle cannot judge
    // its available corridor safely.
    const bool left_unknown =
      measurements[0].status == msg::ObstacleStatus::UNKNOWN;
    const bool center_unknown =
      measurements[1].status == msg::ObstacleStatus::UNKNOWN;
    const bool right_unknown =
      measurements[2].status == msg::ObstacleStatus::UNKNOWN;
    if (center_unknown || (left_unknown && right_unknown)) {
      return msg::ObstacleStatus::UNKNOWN;
    }
    if (left_unknown || right_unknown) {
      return msg::ObstacleStatus::CAUTION;
    }
    for (const auto & measurement : measurements) {
      if (measurement.status == msg::ObstacleStatus::CAUTION) {
        return msg::ObstacleStatus::CAUTION;
      }
    }
    return msg::ObstacleStatus::CLEAR;
  }

  static const char * status_name(uint8_t status)
  {
    switch (status) {
      case msg::ObstacleStatus::CLEAR:
        return "CLEAR";
      case msg::ObstacleStatus::CAUTION:
        return "CAUTION";
      case msg::ObstacleStatus::STOP:
        return "STOP";
      default:
        return "UNKNOWN";
    }
  }

  static cv::Scalar status_color(uint8_t status)
  {
    switch (status) {
      case msg::ObstacleStatus::CLEAR:
        return cv::Scalar(0, 210, 0);
      case msg::ObstacleStatus::CAUTION:
        return cv::Scalar(0, 215, 255);
      case msg::ObstacleStatus::STOP:
        return cv::Scalar(0, 0, 255);
      default:
        return cv::Scalar(170, 170, 170);
    }
  }

  sensor_msgs::msg::Range make_range(
    const std_msgs::msg::Header & header, float distance_m) const
  {
    sensor_msgs::msg::Range range;
    range.header = header;
    range.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range.field_of_view = static_cast<float>(field_of_view_rad_);
    range.min_range = static_cast<float>(min_distance_m_);
    range.max_range = static_cast<float>(max_distance_m_);
    range.range = distance_m;
    return range;
  }

  msg::ObstacleStatus make_status_message(
    const std_msgs::msg::Header & header,
    const std::array<RegionMeasurement, 3> & measurements,
    uint8_t status) const
  {
    msg::ObstacleStatus output;
    output.header = header;
    output.status = status;
    output.left_status = measurements[0].status;
    output.center_status = measurements[1].status;
    output.right_status = measurements[2].status;
    output.left_distance = measurements[0].filtered_distance_m;
    output.center_distance = measurements[1].filtered_distance_m;
    output.right_distance = measurements[2].filtered_distance_m;
    return output;
  }

  sensor_msgs::msg::Image make_visualization(
    const sensor_msgs::msg::Image & image,
    const std::array<RegionMeasurement, 3> & measurements,
    uint8_t status) const
  {
    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    const uint16_t min_mm = static_cast<uint16_t>(min_distance_m_ * 1000.0);
    const uint16_t hard_max_mm = static_cast<uint16_t>(max_distance_m_ * 1000.0);

    cv::Mat depth_mm(height, width, CV_16UC1, cv::Scalar(0));
    for (int y = 0; y < height; ++y) {
      auto * row = depth_mm.ptr<uint16_t>(y);
      for (int x = 0; x < width; ++x) {
        const uint16_t value = read_depth_mm(image, x, y);
        if (value >= min_mm && value <= hard_max_mm) {
          row[x] = value;
        }
      }
    }
    if (visualization_median_size_ >= 3) {
      cv::medianBlur(depth_mm, depth_mm, visualization_median_size_);
    }

    std::vector<uint16_t> samples;
    samples.reserve(static_cast<size_t>(width * height / 4));
    for (int y = 0; y < height; ++y) {
      const auto * row = depth_mm.ptr<uint16_t>(y);
      for (int x = 0; x < width; ++x) {
        if (row[x] >= min_mm) {
          samples.push_back(row[x]);
        }
      }
    }

    float vis_min_mm = static_cast<float>(min_mm);
    float vis_max_mm = static_cast<float>(
      std::min(visualization_max_distance_m_, max_distance_m_) * 1000.0);
    if (visualization_auto_range_ && samples.size() >= 64U) {
      auto percentile = [&samples](double fraction) -> uint16_t {
        const size_t index = static_cast<size_t>(
          fraction * static_cast<double>(samples.size() - 1U));
        auto selected = samples.begin() + static_cast<std::ptrdiff_t>(index);
        std::nth_element(samples.begin(), selected, samples.end());
        return *selected;
      };
      vis_min_mm = static_cast<float>(percentile(0.05));
      vis_max_mm = static_cast<float>(percentile(0.95));
      if (vis_max_mm - vis_min_mm < 250.0F) {
        const float mid = 0.5F * (vis_min_mm + vis_max_mm);
        vis_min_mm = mid - 125.0F;
        vis_max_mm = mid + 125.0F;
      }
    }
    vis_min_mm = std::max(vis_min_mm, static_cast<float>(min_mm));
    vis_max_mm = std::max(vis_max_mm, vis_min_mm + 80.0F);

    cv::Mat intensity(height, width, CV_8UC1, cv::Scalar(0));
    cv::Mat valid_mask(height, width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < height; ++y) {
      const auto * depth_row = depth_mm.ptr<uint16_t>(y);
      auto * intensity_row = intensity.ptr<uint8_t>(y);
      auto * mask_row = valid_mask.ptr<uint8_t>(y);
      for (int x = 0; x < width; ++x) {
        const uint16_t value = depth_row[x];
        if (value < min_mm) {
          continue;
        }
        const float clipped = std::clamp(static_cast<float>(value), vis_min_mm, vis_max_mm);
        const float normalized = (clipped - vis_min_mm) / (vis_max_mm - vis_min_mm);
        intensity_row[x] = static_cast<uint8_t>(255.0F * (1.0F - normalized));
        mask_row[x] = 255U;
      }
    }

    cv::Mat visualization;
    cv::applyColorMap(intensity, visualization, cv::COLORMAP_TURBO);
    visualization.setTo(cv::Scalar(0, 0, 0), valid_mask == 0);

    cv::Mat depth32;
    depth_mm.convertTo(depth32, CV_32F);
    cv::Mat grad_x;
    cv::Mat grad_y;
    cv::Sobel(depth32, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(depth32, grad_y, CV_32F, 0, 1, 3);
    cv::Mat edge_mag;
    cv::magnitude(grad_x, grad_y, edge_mag);
    const double edge_threshold = std::max(18.0, 0.06 * (vis_max_mm - vis_min_mm));
    cv::Mat edge_mask;
    cv::threshold(edge_mag, edge_mask, edge_threshold, 255.0, cv::THRESH_BINARY);
    edge_mask.convertTo(edge_mask, CV_8U);
    cv::bitwise_and(edge_mask, valid_mask, edge_mask);
    visualization.setTo(cv::Scalar(255, 255, 255), edge_mask);

    constexpr std::array<const char *, 3> labels{"L", "C", "R"};
    for (size_t index = 0; index < measurements.size(); ++index) {
      const auto & measurement = measurements[index];
      const cv::Scalar color = status_color(measurement.status);
      cv::rectangle(visualization, measurement.rectangle, color, 3, cv::LINE_AA);

      std::string label = labels[index] + std::string(" ");
      if (std::isfinite(measurement.filtered_distance_m)) {
        label += cv::format("%.2fm", measurement.filtered_distance_m);
      } else {
        label += "N/A";
      }
      const cv::Point text_position(
        measurement.rectangle.x + 7, measurement.rectangle.y + 27);
      cv::putText(
        visualization, label, text_position, cv::FONT_HERSHEY_SIMPLEX,
        0.65, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
      cv::putText(
        visualization, label, text_position, cv::FONT_HERSHEY_SIMPLEX,
        0.65, color, 2, cv::LINE_AA);
    }

    cv::rectangle(visualization, cv::Rect(0, 0, width, 48), cv::Scalar(0, 0, 0), cv::FILLED);
    const std::string overall_label = std::string("OBSTACLE: ") + status_name(status) +
      cv::format(
        "  viz %.2f-%.2fm  STOP<=%.2fm",
        vis_min_mm / 1000.0F, vis_max_mm / 1000.0F, stop_distance_m_);
    cv::putText(
      visualization, overall_label, cv::Point(12, 32), cv::FONT_HERSHEY_SIMPLEX,
      0.58, status_color(status), 2, cv::LINE_AA);

    // The local display is 800x480 while the Astra image is 640x480.  Use the
    // otherwise empty 160-pixel strip as a readable status panel instead of
    // stretching the depth image and distorting its geometry.
    const int status_panel_width = std::max(160, width / 4);
    cv::Mat canvas(
      height, width + status_panel_width, CV_8UC3, cv::Scalar(22, 22, 22));
    visualization.copyTo(canvas(cv::Rect(0, 0, width, height)));
    cv::line(
      canvas, cv::Point(width, 0), cv::Point(width, height),
      status_color(status), 4, cv::LINE_AA);

    const int panel_x = width + 14;
    cv::putText(
      canvas, "STATUS", cv::Point(panel_x, 36), cv::FONT_HERSHEY_SIMPLEX,
      0.55, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    cv::putText(
      canvas, status_name(status), cv::Point(panel_x, 68), cv::FONT_HERSHEY_SIMPLEX,
      0.72, status_color(status), 2, cv::LINE_AA);

    constexpr std::array<const char *, 3> panel_labels{"LEFT", "CENTER", "RIGHT"};
    for (size_t index = 0; index < measurements.size(); ++index) {
      const int panel_y = 135 + static_cast<int>(index) * 108;
      const auto & measurement = measurements[index];
      const cv::Scalar color = status_color(measurement.status);
      cv::putText(
        canvas, panel_labels[index], cv::Point(panel_x, panel_y),
        cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
      const std::string distance_label = std::isfinite(measurement.filtered_distance_m) ?
        cv::format("%.2f m", measurement.filtered_distance_m) : "N/A";
      cv::putText(
        canvas, distance_label, cv::Point(panel_x, panel_y + 32),
        cv::FONT_HERSHEY_SIMPLEX, 0.68, color, 2, cv::LINE_AA);
      cv::putText(
        canvas, status_name(measurement.status), cv::Point(panel_x, panel_y + 57),
        cv::FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv::LINE_AA);
    }

    sensor_msgs::msg::Image output;
    output.header = image.header;
    output.height = static_cast<uint32_t>(canvas.rows);
    output.width = static_cast<uint32_t>(canvas.cols);
    output.encoding = sensor_msgs::image_encodings::BGR8;
    output.is_bigendian = false;
    output.step = output.width * 3U;
    output.data.assign(canvas.datastart, canvas.dataend);
    return output;
  }

  void on_depth_image(const sensor_msgs::msg::Image::ConstSharedPtr image)
  {
    if (image->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Expected 16UC1 depth image, received %s",
        image->encoding.c_str());
      return;
    }
    if (image->width == 0U || image->height == 0U || image->step < image->width * 2U) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid depth image layout");
      return;
    }
    if (image->data.size() < static_cast<size_t>(image->step) * image->height) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Truncated depth image data");
      return;
    }

    const int width = static_cast<int>(image->width);
    const int center_y = static_cast<int>(std::lround(
      static_cast<double>(image->height) * roi_center_y_ratio_));
    std::array<RegionMeasurement, 3> measurements{
      measure_region(*image, width / 6, center_y),
      measure_region(*image, width / 2, center_y),
      measure_region(*image, width * 5 / 6, center_y),
    };

    for (size_t index = 0; index < measurements.size(); ++index) {
      measurements[index].filtered_distance_m =
        update_temporal_filter(index, measurements[index].raw_distance_m);
      measurements[index].status =
        classify_distance(measurements[index].filtered_distance_m);
    }
    const uint8_t status = overall_status(measurements);

    left_range_pub_->publish(make_range(image->header, measurements[0].filtered_distance_m));
    center_range_pub_->publish(make_range(image->header, measurements[1].filtered_distance_m));
    right_range_pub_->publish(make_range(image->header, measurements[2].filtered_distance_m));
    status_pub_->publish(make_status_message(image->header, measurements, status));
    visualization_pub_->publish(make_visualization(*image, measurements, status));

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Obstacle: %s | L=%.2fm(%s) C=%.2fm(%s) R=%.2fm(%s)",
      status_name(status),
      measurements[0].filtered_distance_m, status_name(measurements[0].status),
      measurements[1].filtered_distance_m, status_name(measurements[1].status),
      measurements[2].filtered_distance_m, status_name(measurements[2].status));
  }

  int roi_width_{180};
  int roi_height_{150};
  double roi_center_y_ratio_{0.42};
  double distance_percentile_{0.10};
  int min_valid_pixels_{50};
  int temporal_window_{3};
  double min_distance_m_{0.15};
  double max_distance_m_{8.0};
  double field_of_view_rad_{0.27};
  double stop_distance_m_{0.60};
  double caution_distance_m_{1.00};
  double visualization_max_distance_m_{1.20};
  bool visualization_auto_range_{true};
  int visualization_median_size_{5};

  std::vector<uint16_t> valid_depths_;
  std::vector<float> temporal_values_;
  std::array<std::deque<float>, 3> distance_history_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr left_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr center_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr right_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
  rclcpp::Publisher<msg::ObstacleStatus>::SharedPtr status_pub_;
};

}  // namespace wheel_robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_robot::DepthDistanceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_distance"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
