// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheel_robot:msg/StepDetection.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__TRAITS_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wheel_robot/msg/detail/step_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace wheel_robot
{

namespace msg
{

inline void to_flow_style_yaml(
  const StepDetection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: detected
  {
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
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

  // member: pose_valid
  {
    out << "pose_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_valid, out);
    out << ", ";
  }

  // member: edge_angle_rad
  {
    out << "edge_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.edge_angle_rad, out);
    out << ", ";
  }

  // member: lateral_offset_m
  {
    out << "lateral_offset_m: ";
    rosidl_generator_traits::value_to_yaml(msg.lateral_offset_m, out);
    out << ", ";
  }

  // member: confirm_frames
  {
    out << "confirm_frames: ";
    rosidl_generator_traits::value_to_yaml(msg.confirm_frames, out);
    out << ", ";
  }

  // member: required_confirm_frames
  {
    out << "required_confirm_frames: ";
    rosidl_generator_traits::value_to_yaml(msg.required_confirm_frames, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StepDetection & msg,
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

  // member: detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
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

  // member: pose_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_valid, out);
    out << "\n";
  }

  // member: edge_angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "edge_angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.edge_angle_rad, out);
    out << "\n";
  }

  // member: lateral_offset_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lateral_offset_m: ";
    rosidl_generator_traits::value_to_yaml(msg.lateral_offset_m, out);
    out << "\n";
  }

  // member: confirm_frames
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confirm_frames: ";
    rosidl_generator_traits::value_to_yaml(msg.confirm_frames, out);
    out << "\n";
  }

  // member: required_confirm_frames
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "required_confirm_frames: ";
    rosidl_generator_traits::value_to_yaml(msg.required_confirm_frames, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StepDetection & msg, bool use_flow_style = false)
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
  const wheel_robot::msg::StepDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheel_robot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheel_robot::msg::to_yaml() instead")]]
inline std::string to_yaml(const wheel_robot::msg::StepDetection & msg)
{
  return wheel_robot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wheel_robot::msg::StepDetection>()
{
  return "wheel_robot::msg::StepDetection";
}

template<>
inline const char * name<wheel_robot::msg::StepDetection>()
{
  return "wheel_robot/msg/StepDetection";
}

template<>
struct has_fixed_size<wheel_robot::msg::StepDetection>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<wheel_robot::msg::StepDetection>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<wheel_robot::msg::StepDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__TRAITS_HPP_
