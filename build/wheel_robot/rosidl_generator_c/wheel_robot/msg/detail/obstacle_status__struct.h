// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wheel_robot:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_H_
#define WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNKNOWN'.
enum
{
  wheel_robot__msg__ObstacleStatus__UNKNOWN = 0
};

/// Constant 'CLEAR'.
enum
{
  wheel_robot__msg__ObstacleStatus__CLEAR = 1
};

/// Constant 'CAUTION'.
enum
{
  wheel_robot__msg__ObstacleStatus__CAUTION = 2
};

/// Constant 'STOP'.
enum
{
  wheel_robot__msg__ObstacleStatus__STOP = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ObstacleStatus in the package wheel_robot.
typedef struct wheel_robot__msg__ObstacleStatus
{
  std_msgs__msg__Header header;
  uint8_t status;
  uint8_t left_status;
  uint8_t center_status;
  uint8_t right_status;
  float left_distance;
  float center_distance;
  float right_distance;
} wheel_robot__msg__ObstacleStatus;

// Struct for a sequence of wheel_robot__msg__ObstacleStatus.
typedef struct wheel_robot__msg__ObstacleStatus__Sequence
{
  wheel_robot__msg__ObstacleStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheel_robot__msg__ObstacleStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHEEL_ROBOT__MSG__DETAIL__OBSTACLE_STATUS__STRUCT_H_
