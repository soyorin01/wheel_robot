// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__BUILDER_HPP_
#define MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_temperature_pkg/msg/detail/motor_temperatures__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_temperature_pkg
{

namespace msg
{

namespace builder
{

class Init_MotorTemperatures_overheated
{
public:
  explicit Init_MotorTemperatures_overheated(::motor_temperature_pkg::msg::MotorTemperatures & msg)
  : msg_(msg)
  {}
  ::motor_temperature_pkg::msg::MotorTemperatures overheated(::motor_temperature_pkg::msg::MotorTemperatures::_overheated_type arg)
  {
    msg_.overheated = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_temperature_pkg::msg::MotorTemperatures msg_;
};

class Init_MotorTemperatures_overheated_index
{
public:
  explicit Init_MotorTemperatures_overheated_index(::motor_temperature_pkg::msg::MotorTemperatures & msg)
  : msg_(msg)
  {}
  Init_MotorTemperatures_overheated overheated_index(::motor_temperature_pkg::msg::MotorTemperatures::_overheated_index_type arg)
  {
    msg_.overheated_index = std::move(arg);
    return Init_MotorTemperatures_overheated(msg_);
  }

private:
  ::motor_temperature_pkg::msg::MotorTemperatures msg_;
};

class Init_MotorTemperatures_warning_threshold_c
{
public:
  explicit Init_MotorTemperatures_warning_threshold_c(::motor_temperature_pkg::msg::MotorTemperatures & msg)
  : msg_(msg)
  {}
  Init_MotorTemperatures_overheated_index warning_threshold_c(::motor_temperature_pkg::msg::MotorTemperatures::_warning_threshold_c_type arg)
  {
    msg_.warning_threshold_c = std::move(arg);
    return Init_MotorTemperatures_overheated_index(msg_);
  }

private:
  ::motor_temperature_pkg::msg::MotorTemperatures msg_;
};

class Init_MotorTemperatures_temperature_c
{
public:
  explicit Init_MotorTemperatures_temperature_c(::motor_temperature_pkg::msg::MotorTemperatures & msg)
  : msg_(msg)
  {}
  Init_MotorTemperatures_warning_threshold_c temperature_c(::motor_temperature_pkg::msg::MotorTemperatures::_temperature_c_type arg)
  {
    msg_.temperature_c = std::move(arg);
    return Init_MotorTemperatures_warning_threshold_c(msg_);
  }

private:
  ::motor_temperature_pkg::msg::MotorTemperatures msg_;
};

class Init_MotorTemperatures_name
{
public:
  explicit Init_MotorTemperatures_name(::motor_temperature_pkg::msg::MotorTemperatures & msg)
  : msg_(msg)
  {}
  Init_MotorTemperatures_temperature_c name(::motor_temperature_pkg::msg::MotorTemperatures::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MotorTemperatures_temperature_c(msg_);
  }

private:
  ::motor_temperature_pkg::msg::MotorTemperatures msg_;
};

class Init_MotorTemperatures_header
{
public:
  Init_MotorTemperatures_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorTemperatures_name header(::motor_temperature_pkg::msg::MotorTemperatures::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorTemperatures_name(msg_);
  }

private:
  ::motor_temperature_pkg::msg::MotorTemperatures msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_temperature_pkg::msg::MotorTemperatures>()
{
  return motor_temperature_pkg::msg::builder::Init_MotorTemperatures_header();
}

}  // namespace motor_temperature_pkg

#endif  // MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__BUILDER_HPP_
