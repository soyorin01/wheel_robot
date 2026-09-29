// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheel_robot:msg/StepApproachStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__TRAITS_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wheel_robot/msg/detail/step_approach_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace wheel_robot
{

namespace msg
{

inline void to_flow_style_yaml(
  const StepApproachStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: locked
  {
    out << "locked: ";
    rosidl_generator_traits::value_to_yaml(msg.locked, out);
    out << ", ";
  }

  // member: step_detected
  {
    out << "step_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.step_detected, out);
    out << ", ";
  }

  // member: detected_distance
  {
    out << "detected_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.detected_distance, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: stop_distance
  {
    out << "stop_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_distance, out);
    out << ", ";
  }

  // member: target_travel
  {
    out << "target_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.target_travel, out);
    out << ", ";
  }

  // member: traveled
  {
    out << "traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.traveled, out);
    out << ", ";
  }

  // member: remaining
  {
    out << "remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StepApproachStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: locked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "locked: ";
    rosidl_generator_traits::value_to_yaml(msg.locked, out);
    out << "\n";
  }

  // member: step_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "step_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.step_detected, out);
    out << "\n";
  }

  // member: detected_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detected_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.detected_distance, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: stop_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_distance, out);
    out << "\n";
  }

  // member: target_travel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.target_travel, out);
    out << "\n";
  }

  // member: traveled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traveled: ";
    rosidl_generator_traits::value_to_yaml(msg.traveled, out);
    out << "\n";
  }

  // member: remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StepApproachStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace wheel_robot

namespace rosidl_generator_traits
{

[[deprecated("use wheel_robot::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wheel_robot::msg::StepApproachStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheel_robot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheel_robot::msg::to_yaml() instead")]]
inline std::string to_yaml(const wheel_robot::msg::StepApproachStatus & msg)
{
  return wheel_robot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wheel_robot::msg::StepApproachStatus>()
{
  return "wheel_robot::msg::StepApproachStatus";
}

template<>
inline const char * name<wheel_robot::msg::StepApproachStatus>()
{
  return "wheel_robot/msg/StepApproachStatus";
}

template<>
struct has_fixed_size<wheel_robot::msg::StepApproachStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wheel_robot::msg::StepApproachStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wheel_robot::msg::StepApproachStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__TRAITS_HPP_
