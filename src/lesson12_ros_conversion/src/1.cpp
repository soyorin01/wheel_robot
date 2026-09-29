#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int calculate(int value)
{
  return value * 2;
}

class CppDoubler : public rclcpp::Node
{
public:
  CppDoubler() : Node("cpp_doubler")
  {
    result_publisher_ = create_publisher<std_msgs::msg::Int32>(
      "/lesson12/doubled_value", 10);

    input_subscription_ = create_subscription<std_msgs::msg::Int32>(
      "/lesson12/input_value", 10,
      std::bind(&CppDoubler::on_input, this, std::placeholders::_1));
  }

private:
  void on_input(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std_msgs::msg::Int32 result;
    result.data = calculate(msg->data);
    result_publisher_->publish(result);
    RCLCPP_INFO(
      get_logger(), "receive=%d, publish=%d", msg->data, result.data);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr result_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr input_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppDoubler>());
  rclcpp::shutdown();
  return 0;
}