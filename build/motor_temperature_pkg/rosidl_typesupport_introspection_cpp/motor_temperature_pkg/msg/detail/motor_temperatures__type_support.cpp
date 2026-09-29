// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_temperature_pkg/msg/detail/motor_temperatures__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace motor_temperature_pkg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorTemperatures_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) motor_temperature_pkg::msg::MotorTemperatures(_init);
}

void MotorTemperatures_fini_function(void * message_memory)
{
  auto typed_message = static_cast<motor_temperature_pkg::msg::MotorTemperatures *>(message_memory);
  typed_message->~MotorTemperatures();
}

size_t size_function__MotorTemperatures__name(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorTemperatures__name(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorTemperatures__name(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorTemperatures__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__MotorTemperatures__name(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__MotorTemperatures__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__MotorTemperatures__name(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__MotorTemperatures__name(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorTemperatures__temperature_c(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorTemperatures__temperature_c(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorTemperatures__temperature_c(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorTemperatures__temperature_c(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__MotorTemperatures__temperature_c(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__MotorTemperatures__temperature_c(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__MotorTemperatures__temperature_c(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__MotorTemperatures__temperature_c(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorTemperatures__overheated_index(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorTemperatures__overheated_index(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorTemperatures__overheated_index(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorTemperatures__overheated_index(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__MotorTemperatures__overheated_index(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__MotorTemperatures__overheated_index(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__MotorTemperatures__overheated_index(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__MotorTemperatures__overheated_index(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorTemperatures_message_member_array[6] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg::msg::MotorTemperatures, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg::msg::MotorTemperatures, name),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorTemperatures__name,  // size() function pointer
    get_const_function__MotorTemperatures__name,  // get_const(index) function pointer
    get_function__MotorTemperatures__name,  // get(index) function pointer
    fetch_function__MotorTemperatures__name,  // fetch(index, &value) function pointer
    assign_function__MotorTemperatures__name,  // assign(index, value) function pointer
    resize_function__MotorTemperatures__name  // resize(index) function pointer
  },
  {
    "temperature_c",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg::msg::MotorTemperatures, temperature_c),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorTemperatures__temperature_c,  // size() function pointer
    get_const_function__MotorTemperatures__temperature_c,  // get_const(index) function pointer
    get_function__MotorTemperatures__temperature_c,  // get(index) function pointer
    fetch_function__MotorTemperatures__temperature_c,  // fetch(index, &value) function pointer
    assign_function__MotorTemperatures__temperature_c,  // assign(index, value) function pointer
    resize_function__MotorTemperatures__temperature_c  // resize(index) function pointer
  },
  {
    "warning_threshold_c",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg::msg::MotorTemperatures, warning_threshold_c),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "overheated_index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg::msg::MotorTemperatures, overheated_index),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorTemperatures__overheated_index,  // size() function pointer
    get_const_function__MotorTemperatures__overheated_index,  // get_const(index) function pointer
    get_function__MotorTemperatures__overheated_index,  // get(index) function pointer
    fetch_function__MotorTemperatures__overheated_index,  // fetch(index, &value) function pointer
    assign_function__MotorTemperatures__overheated_index,  // assign(index, value) function pointer
    resize_function__MotorTemperatures__overheated_index  // resize(index) function pointer
  },
  {
    "overheated",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg::msg::MotorTemperatures, overheated),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorTemperatures_message_members = {
  "motor_temperature_pkg::msg",  // message namespace
  "MotorTemperatures",  // message name
  6,  // number of fields
  sizeof(motor_temperature_pkg::msg::MotorTemperatures),
  MotorTemperatures_message_member_array,  // message members
  MotorTemperatures_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorTemperatures_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorTemperatures_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorTemperatures_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace motor_temperature_pkg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<motor_temperature_pkg::msg::MotorTemperatures>()
{
  return &::motor_temperature_pkg::msg::rosidl_typesupport_introspection_cpp::MotorTemperatures_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, motor_temperature_pkg, msg, MotorTemperatures)() {
  return &::motor_temperature_pkg::msg::rosidl_typesupport_introspection_cpp::MotorTemperatures_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
