// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__STRUCT_H_
#define MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'temperature_c'
// Member 'overheated_index'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MotorTemperatures in the package motor_temperature_pkg.
typedef struct motor_temperature_pkg__msg__MotorTemperatures
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__float__Sequence temperature_c;
  float warning_threshold_c;
  rosidl_runtime_c__uint8__Sequence overheated_index;
  bool overheated;
} motor_temperature_pkg__msg__MotorTemperatures;

// Struct for a sequence of motor_temperature_pkg__msg__MotorTemperatures.
typedef struct motor_temperature_pkg__msg__MotorTemperatures__Sequence
{
  motor_temperature_pkg__msg__MotorTemperatures * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_temperature_pkg__msg__MotorTemperatures__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_TEMPERATURE_PKG__MSG__DETAIL__MOTOR_TEMPERATURES__STRUCT_H_
