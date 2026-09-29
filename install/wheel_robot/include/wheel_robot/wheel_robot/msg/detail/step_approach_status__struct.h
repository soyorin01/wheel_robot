// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wheel_robot:msg/StepApproachStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__STRUCT_H_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__STRUCT_H_

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
// Member 'state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StepApproachStatus in the package wheel_robot.
typedef struct wheel_robot__msg__StepApproachStatus
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String state;
  bool locked;
  bool step_detected;
  float detected_distance;
  float height;
  float confidence;
  float stop_distance;
  float target_travel;
  float traveled;
  float remaining;
} wheel_robot__msg__StepApproachStatus;

// Struct for a sequence of wheel_robot__msg__StepApproachStatus.
typedef struct wheel_robot__msg__StepApproachStatus__Sequence
{
  wheel_robot__msg__StepApproachStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheel_robot__msg__StepApproachStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__STRUCT_H_
