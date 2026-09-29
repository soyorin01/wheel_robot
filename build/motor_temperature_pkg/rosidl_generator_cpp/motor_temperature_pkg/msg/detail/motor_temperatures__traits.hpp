// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__TRAITS_HPP_
#define MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_temperature_pkg/msg/detail/motor_temperatures__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace motor_temperature_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorTemperatures & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: name
  {
    if (msg.name.size() == 0) {
      out << "name: []";
    } else {
      out << "name: [";
      size_t pending_items = msg.name.size();
      for (auto item : msg.name) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: temperature_c
  {
    if (msg.temperature_c.size() == 0) {
      out << "temperature_c: []";
    } else {
      out << "temperature_c: [";
      size_t pending_items = msg.temperature_c.size();
      for (auto item : msg.temperature_c) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: warning_threshold_c
  {
    out << "warning_threshold_c: ";
    rosidl_generator_traits::value_to_yaml(msg.warning_threshold_c, out);
    out << ", ";
  }

  // member: overheated_index
  {
    if (msg.overheated_index.size() == 0) {
      out << "overheated_index: []";
    } else {
      out << "overheated_index: [";
      size_t pending_items = msg.overheated_index.size();
      for (auto item : msg.overheated_index) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: overheated
  {
    out << "overheated: ";
    rosidl_generator_traits::value_to_yaml(msg.overheated, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorTemperatures & msg,
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

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.name.size() == 0) {
      out << "name: []\n";
    } else {
      out << "name:\n";
      for (auto item : msg.name) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: temperature_c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.temperature_c.size() == 0) {
      out << "temperature_c: []\n";
    } else {
      out << "temperature_c:\n";
      for (auto item : msg.temperature_c) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: warning_threshold_c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "warning_threshold_c: ";
    rosidl_generator_traits::value_to_yaml(msg.warning_threshold_c, out);
    out << "\n";
  }

  // member: overheated_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.overheated_index.size() == 0) {
      out << "overheated_index: []\n";
    } else {
      out << "overheated_index:\n";
      for (auto item : msg.overheated_index) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: overheated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overheated: ";
    rosidl_generator_traits::value_to_yaml(msg.overheated, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorTemperatures & msg, bool use_flow_style = false)
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

}  // namespace motor_temperature_pkg

namespace rosidl_generator_traits
{

[[deprecated("use motor_temperature_pkg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_temperature_pkg::msg::MotorTemperatures & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_temperature_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_temperature_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_temperature_pkg::msg::MotorTemperatures & msg)
{
  return motor_temperature_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_temperature_pkg::msg::MotorTemperatures>()
{
  return "motor_temperature_pkg::msg::MotorTemperatures";
}

template<>
inline const char * name<motor_temperature_pkg::msg::MotorTemperatures>()
{
  return "motor_temperature_pkg/msg/MotorTemperatures";
}

template<>
struct has_fixed_size<motor_temperature_pkg::msg::MotorTemperatures>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<motor_temperature_pkg::msg::MotorTemperatures>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<motor_temperature_pkg::msg::MotorTemperatures>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__TRAITS_HPP_
