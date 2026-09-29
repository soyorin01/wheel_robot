// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wheel_robot:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__TRAITS_HPP_
#define WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wheel_robot/msg/detail/obstacle_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace wheel_robot
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObstacleStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: left_status
  {
    out << "left_status: ";
    rosidl_generator_traits::value_to_yaml(msg.left_status, out);
    out << ", ";
  }

  // member: center_status
  {
    out << "center_status: ";
    rosidl_generator_traits::value_to_yaml(msg.center_status, out);
    out << ", ";
  }

  // member: right_status
  {
    out << "right_status: ";
    rosidl_generator_traits::value_to_yaml(msg.right_status, out);
    out << ", ";
  }

  // member: left_distance
  {
    out << "left_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.left_distance, out);
    out << ", ";
  }

  // member: center_distance
  {
    out << "center_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.center_distance, out);
    out << ", ";
  }

  // member: right_distance
  {
    out << "right_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.right_distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObstacleStatus & msg,
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

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: left_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_status: ";
    rosidl_generator_traits::value_to_yaml(msg.left_status, out);
    out << "\n";
  }

  // member: center_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_status: ";
    rosidl_generator_traits::value_to_yaml(msg.center_status, out);
    out << "\n";
  }

  // member: right_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_status: ";
    rosidl_generator_traits::value_to_yaml(msg.right_status, out);
    out << "\n";
  }

  // member: left_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.left_distance, out);
    out << "\n";
  }

  // member: center_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.center_distance, out);
    out << "\n";
  }

  // member: right_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_distance: ";
    rosidl_generator_traits::value_to_yaml(msg.right_distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObstacleStatus & msg, bool use_flow_style = false)
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
  const wheel_robot::msg::ObstacleStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  wheel_robot::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wheel_robot::msg::to_yaml() instead")]]
inline std::string to_yaml(const wheel_robot::msg::ObstacleStatus & msg)
{
  return wheel_robot::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wheel_robot::msg::ObstacleStatus>()
{
  return "wheel_robot::msg::ObstacleStatus";
}

template<>
inline const char * name<wheel_robot::msg::ObstacleStatus>()
{
  return "wheel_robot/msg/ObstacleStatus";
}

template<>
struct has_fixed_size<wheel_robot::msg::ObstacleStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<wheel_robot::msg::ObstacleStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<wheel_robot::msg::ObstacleStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__TRAITS_HPP_
