// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wheel_robot:msg/StepApproachStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__BUILDER_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wheel_robot/msg/detail/step_approach_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wheel_robot
{

namespace msg
{

namespace builder
{

class Init_StepApproachStatus_remaining
{
public:
  explicit Init_StepApproachStatus_remaining(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  ::wheel_robot::msg::StepApproachStatus remaining(::wheel_robot::msg::StepApproachStatus::_remaining_type arg)
  {
    msg_.remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_traveled
{
public:
  explicit Init_StepApproachStatus_traveled(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_remaining traveled(::wheel_robot::msg::StepApproachStatus::_traveled_type arg)
  {
    msg_.traveled = std::move(arg);
    return Init_StepApproachStatus_remaining(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_target_travel
{
public:
  explicit Init_StepApproachStatus_target_travel(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_traveled target_travel(::wheel_robot::msg::StepApproachStatus::_target_travel_type arg)
  {
    msg_.target_travel = std::move(arg);
    return Init_StepApproachStatus_traveled(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_stop_distance
{
public:
  explicit Init_StepApproachStatus_stop_distance(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_target_travel stop_distance(::wheel_robot::msg::StepApproachStatus::_stop_distance_type arg)
  {
    msg_.stop_distance = std::move(arg);
    return Init_StepApproachStatus_target_travel(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_confidence
{
public:
  explicit Init_StepApproachStatus_confidence(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_stop_distance confidence(::wheel_robot::msg::StepApproachStatus::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_StepApproachStatus_stop_distance(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_height
{
public:
  explicit Init_StepApproachStatus_height(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_confidence height(::wheel_robot::msg::StepApproachStatus::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_StepApproachStatus_confidence(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_detected_distance
{
public:
  explicit Init_StepApproachStatus_detected_distance(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_height detected_distance(::wheel_robot::msg::StepApproachStatus::_detected_distance_type arg)
  {
    msg_.detected_distance = std::move(arg);
    return Init_StepApproachStatus_height(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_step_detected
{
public:
  explicit Init_StepApproachStatus_step_detected(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_detected_distance step_detected(::wheel_robot::msg::StepApproachStatus::_step_detected_type arg)
  {
    msg_.step_detected = std::move(arg);
    return Init_StepApproachStatus_detected_distance(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_locked
{
public:
  explicit Init_StepApproachStatus_locked(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_step_detected locked(::wheel_robot::msg::StepApproachStatus::_locked_type arg)
  {
    msg_.locked = std::move(arg);
    return Init_StepApproachStatus_step_detected(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_state
{
public:
  explicit Init_StepApproachStatus_state(::wheel_robot::msg::StepApproachStatus & msg)
  : msg_(msg)
  {}
  Init_StepApproachStatus_locked state(::wheel_robot::msg::StepApproachStatus::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_StepApproachStatus_locked(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

class Init_StepApproachStatus_header
{
public:
  Init_StepApproachStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StepApproachStatus_state header(::wheel_robot::msg::StepApproachStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StepApproachStatus_state(msg_);
  }

private:
  ::wheel_robot::msg::StepApproachStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wheel_robot::msg::StepApproachStatus>()
{
  return wheel_robot::msg::builder::Init_StepApproachStatus_header();
}

}  // namespace wheel_robot

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__BUILDER_HPP_
