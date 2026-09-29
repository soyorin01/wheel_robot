// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wheel_robot:msg/StepDetection.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__STRUCT_H_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__STRUCT_H_

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

/// Struct defined in msg/StepDetection in the package wheel_robot.
typedef struct wheel_robot__msg__StepDetection
{
  std_msgs__msg__Header header;
  bool detected;
  float distance;
  float height;
  float confidence;
  /// 台阶前沿在相机地面坐标系中的姿态。
  /// edge_angle_rad=0 表示台阶横边与相机 x 轴平行，即车头正对台阶。
  /// 正值表示拟合直线 z=a*x+b 的 a>0；lateral_offset_m 正值表示台阶中心在车体右侧。
  bool pose_valid;
  float edge_angle_rad;
  float lateral_offset_m;
  uint32_t confirm_frames;
  uint32_t required_confirm_frames;
} wheel_robot__msg__StepDetection;

// Struct for a sequence of wheel_robot__msg__StepDetection.
typedef struct wheel_robot__msg__StepDetection__Sequence
{
  wheel_robot__msg__StepDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wheel_robot__msg__StepDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__STRUCT_H_
