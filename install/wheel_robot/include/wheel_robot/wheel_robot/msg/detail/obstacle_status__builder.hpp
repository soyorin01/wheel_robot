// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheel_robot:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__BUILDER_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheel_robot/msg/detail/obstacle_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheel_robot
{

namespace msg
{

namespace builder
{

class Init_ObstacleStatus_right_distance
{
public:
  explicit Init_ObstacleStatus_right_distance(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  ::wheel_robot::msg::ObstacleStatus right_distance(::wheel_robot::msg::ObstacleStatus::_right_distance_type arg)
  {
    msg_.right_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_center_distance
{
public:
  explicit Init_ObstacleStatus_center_distance(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  Init_ObstacleStatus_right_distance center_distance(::wheel_robot::msg::ObstacleStatus::_center_distance_type arg)
  {
    msg_.center_distance = std::move(arg);
    return Init_ObstacleStatus_right_distance(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_left_distance
{
public:
  explicit Init_ObstacleStatus_left_distance(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  Init_ObstacleStatus_center_distance left_distance(::wheel_robot::msg::ObstacleStatus::_left_distance_type arg)
  {
    msg_.left_distance = std::move(arg);
    return Init_ObstacleStatus_center_distance(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_right_status
{
public:
  explicit Init_ObstacleStatus_right_status(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  Init_ObstacleStatus_left_distance right_status(::wheel_robot::msg::ObstacleStatus::_right_status_type arg)
  {
    msg_.right_status = std::move(arg);
    return Init_ObstacleStatus_left_distance(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_center_status
{
public:
  explicit Init_ObstacleStatus_center_status(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  Init_ObstacleStatus_right_status center_status(::wheel_robot::msg::ObstacleStatus::_center_status_type arg)
  {
    msg_.center_status = std::move(arg);
    return Init_ObstacleStatus_right_status(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_left_status
{
public:
  explicit Init_ObstacleStatus_left_status(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  Init_ObstacleStatus_center_status left_status(::wheel_robot::msg::ObstacleStatus::_left_status_type arg)
  {
    msg_.left_status = std::move(arg);
    return Init_ObstacleStatus_center_status(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_status
{
public:
  explicit Init_ObstacleStatus_status(::wheel_robot::msg::ObstacleStatus & msg)
  : msg_(msg)
  {}
  Init_ObstacleStatus_left_status status(::wheel_robot::msg::ObstacleStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ObstacleStatus_left_status(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

class Init_ObstacleStatus_header
{
public:
  Init_ObstacleStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleStatus_status header(::wheel_robot::msg::ObstacleStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleStatus_status(msg_);
  }

private:
  ::wheel_robot::msg::ObstacleStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheel_robot::msg::ObstacleStatus>()
{
  return wheel_robot::msg::builder::Init_ObstacleStatus_header();
}

}  // namespace wheel_robot

#endif  // WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__BUILDER_HPP_
