// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheel_robot:msg/StepDetection.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__BUILDER_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheel_robot/msg/detail/step_detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheel_robot
{

namespace msg
{

namespace builder
{

class Init_StepDetection_required_confirm_frames
{
public:
  explicit Init_StepDetection_required_confirm_frames(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  ::wheel_robot::msg::StepDetection required_confirm_frames(::wheel_robot::msg::StepDetection::_required_confirm_frames_type arg)
  {
    msg_.required_confirm_frames = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_confirm_frames
{
public:
  explicit Init_StepDetection_confirm_frames(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_required_confirm_frames confirm_frames(::wheel_robot::msg::StepDetection::_confirm_frames_type arg)
  {
    msg_.confirm_frames = std::move(arg);
    return Init_StepDetection_required_confirm_frames(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_lateral_offset_m
{
public:
  explicit Init_StepDetection_lateral_offset_m(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_confirm_frames lateral_offset_m(::wheel_robot::msg::StepDetection::_lateral_offset_m_type arg)
  {
    msg_.lateral_offset_m = std::move(arg);
    return Init_StepDetection_confirm_frames(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_edge_angle_rad
{
public:
  explicit Init_StepDetection_edge_angle_rad(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_lateral_offset_m edge_angle_rad(::wheel_robot::msg::StepDetection::_edge_angle_rad_type arg)
  {
    msg_.edge_angle_rad = std::move(arg);
    return Init_StepDetection_lateral_offset_m(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_pose_valid
{
public:
  explicit Init_StepDetection_pose_valid(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_edge_angle_rad pose_valid(::wheel_robot::msg::StepDetection::_pose_valid_type arg)
  {
    msg_.pose_valid = std::move(arg);
    return Init_StepDetection_edge_angle_rad(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_confidence
{
public:
  explicit Init_StepDetection_confidence(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_pose_valid confidence(::wheel_robot::msg::StepDetection::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_StepDetection_pose_valid(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_height
{
public:
  explicit Init_StepDetection_height(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_confidence height(::wheel_robot::msg::StepDetection::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_StepDetection_confidence(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_distance
{
public:
  explicit Init_StepDetection_distance(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_height distance(::wheel_robot::msg::StepDetection::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_StepDetection_height(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_detected
{
public:
  explicit Init_StepDetection_detected(::wheel_robot::msg::StepDetection & msg)
  : msg_(msg)
  {}
  Init_StepDetection_distance detected(::wheel_robot::msg::StepDetection::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_StepDetection_distance(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

class Init_StepDetection_header
{
public:
  Init_StepDetection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StepDetection_detected header(::wheel_robot::msg::StepDetection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StepDetection_detected(msg_);
  }

private:
  ::wheel_robot::msg::StepDetection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheel_robot::msg::StepDetection>()
{
  return wheel_robot::msg::builder::Init_StepDetection_header();
}

}  // namespace wheel_robot

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__BUILDER_HPP_
