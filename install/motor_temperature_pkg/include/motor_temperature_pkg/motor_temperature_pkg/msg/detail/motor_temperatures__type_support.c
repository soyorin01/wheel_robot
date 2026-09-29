// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_temperature_pkg/msg/detail/motor_temperatures__rosidl_typesupport_introspection_c.h"
#include "motor_temperature_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_temperature_pkg/msg/detail/motor_temperatures__functions.h"
#include "motor_temperature_pkg/msg/detail/motor_temperatures__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `temperature_c`
// Member `overheated_index`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_temperature_pkg__msg__MotorTemperatures__init(message_memory);
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_fini_function(void * message_memory)
{
  motor_temperature_pkg__msg__MotorTemperatures__fini(message_memory);
}

size_t motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__size_function__MotorTemperatures__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__fetch_function__MotorTemperatures__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__assign_function__MotorTemperatures__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__resize_function__MotorTemperatures__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__size_function__MotorTemperatures__temperature_c(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__temperature_c(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__temperature_c(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__fetch_function__MotorTemperatures__temperature_c(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__temperature_c(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__assign_function__MotorTemperatures__temperature_c(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__temperature_c(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__resize_function__MotorTemperatures__temperature_c(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__size_function__MotorTemperatures__overheated_index(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__overheated_index(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__overheated_index(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__fetch_function__MotorTemperatures__overheated_index(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__overheated_index(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__assign_function__MotorTemperatures__overheated_index(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__overheated_index(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__resize_function__MotorTemperatures__overheated_index(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg__msg__MotorTemperatures, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg__msg__MotorTemperatures, name),  // bytes offset in struct
    NULL,  // default value
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__size_function__MotorTemperatures__name,  // size() function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__name,  // get_const(index) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__name,  // get(index) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__fetch_function__MotorTemperatures__name,  // fetch(index, &value) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__assign_function__MotorTemperatures__name,  // assign(index, value) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__resize_function__MotorTemperatures__name  // resize(index) function pointer
  },
  {
    "temperature_c",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg__msg__MotorTemperatures, temperature_c),  // bytes offset in struct
    NULL,  // default value
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__size_function__MotorTemperatures__temperature_c,  // size() function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__temperature_c,  // get_const(index) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__temperature_c,  // get(index) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__fetch_function__MotorTemperatures__temperature_c,  // fetch(index, &value) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__assign_function__MotorTemperatures__temperature_c,  // assign(index, value) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__resize_function__MotorTemperatures__temperature_c  // resize(index) function pointer
  },
  {
    "warning_threshold_c",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg__msg__MotorTemperatures, warning_threshold_c),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "overheated_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg__msg__MotorTemperatures, overheated_index),  // bytes offset in struct
    NULL,  // default value
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__size_function__MotorTemperatures__overheated_index,  // size() function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_const_function__MotorTemperatures__overheated_index,  // get_const(index) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__get_function__MotorTemperatures__overheated_index,  // get(index) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__fetch_function__MotorTemperatures__overheated_index,  // fetch(index, &value) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__assign_function__MotorTemperatures__overheated_index,  // assign(index, value) function pointer
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__resize_function__MotorTemperatures__overheated_index  // resize(index) function pointer
  },
  {
    "overheated",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_temperature_pkg__msg__MotorTemperatures, overheated),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_members = {
  "motor_temperature_pkg__msg",  // message namespace
  "MotorTemperatures",  // message name
  6,  // number of fields
  sizeof(motor_temperature_pkg__msg__MotorTemperatures),
  motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_member_array,  // message members
  motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_type_support_handle = {
  0,
  &motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_temperature_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_temperature_pkg, msg, MotorTemperatures)() {
  motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_type_support_handle.typesupport_identifier) {
    motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_temperature_pkg__msg__MotorTemperatures__rosidl_typesupport_introspection_c__MotorTemperatures_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
