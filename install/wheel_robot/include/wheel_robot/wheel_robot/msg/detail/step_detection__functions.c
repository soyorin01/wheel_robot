// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wheel_robot:msg/StepDetection.idl
// generated code does not contain a copyright notice
#include "wheel_robot/msg/detail/step_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
wheel_robot__msg__StepDetection__init(wheel_robot__msg__StepDetection * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wheel_robot__msg__StepDetection__fini(msg);
    return false;
  }
  // detected
  // distance
  // height
  // confidence
  // pose_valid
  // edge_angle_rad
  // lateral_offset_m
  // confirm_frames
  // required_confirm_frames
  return true;
}

void
wheel_robot__msg__StepDetection__fini(wheel_robot__msg__StepDetection * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detected
  // distance
  // height
  // confidence
  // pose_valid
  // edge_angle_rad
  // lateral_offset_m
  // confirm_frames
  // required_confirm_frames
}

bool
wheel_robot__msg__StepDetection__are_equal(const wheel_robot__msg__StepDetection * lhs, const wheel_robot__msg__StepDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detected
  if (lhs->detected != rhs->detected) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // pose_valid
  if (lhs->pose_valid != rhs->pose_valid) {
    return false;
  }
  // edge_angle_rad
  if (lhs->edge_angle_rad != rhs->edge_angle_rad) {
    return false;
  }
  // lateral_offset_m
  if (lhs->lateral_offset_m != rhs->lateral_offset_m) {
    return false;
  }
  // confirm_frames
  if (lhs->confirm_frames != rhs->confirm_frames) {
    return false;
  }
  // required_confirm_frames
  if (lhs->required_confirm_frames != rhs->required_confirm_frames) {
    return false;
  }
  return true;
}

bool
wheel_robot__msg__StepDetection__copy(
  const wheel_robot__msg__StepDetection * input,
  wheel_robot__msg__StepDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detected
  output->detected = input->detected;
  // distance
  output->distance = input->distance;
  // height
  output->height = input->height;
  // confidence
  output->confidence = input->confidence;
  // pose_valid
  output->pose_valid = input->pose_valid;
  // edge_angle_rad
  output->edge_angle_rad = input->edge_angle_rad;
  // lateral_offset_m
  output->lateral_offset_m = input->lateral_offset_m;
  // confirm_frames
  output->confirm_frames = input->confirm_frames;
  // required_confirm_frames
  output->required_confirm_frames = input->required_confirm_frames;
  return true;
}

wheel_robot__msg__StepDetection *
wheel_robot__msg__StepDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__StepDetection * msg = (wheel_robot__msg__StepDetection *)allocator.allocate(sizeof(wheel_robot__msg__StepDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheel_robot__msg__StepDetection));
  bool success = wheel_robot__msg__StepDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheel_robot__msg__StepDetection__destroy(wheel_robot__msg__StepDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheel_robot__msg__StepDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheel_robot__msg__StepDetection__Sequence__init(wheel_robot__msg__StepDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__StepDetection * data = NULL;

  if (size) {
    data = (wheel_robot__msg__StepDetection *)allocator.zero_allocate(size, sizeof(wheel_robot__msg__StepDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheel_robot__msg__StepDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheel_robot__msg__StepDetection__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
wheel_robot__msg__StepDetection__Sequence__fini(wheel_robot__msg__StepDetection__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      wheel_robot__msg__StepDetection__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

wheel_robot__msg__StepDetection__Sequence *
wheel_robot__msg__StepDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__StepDetection__Sequence * array = (wheel_robot__msg__StepDetection__Sequence *)allocator.allocate(sizeof(wheel_robot__msg__StepDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheel_robot__msg__StepDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheel_robot__msg__StepDetection__Sequence__destroy(wheel_robot__msg__StepDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheel_robot__msg__StepDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheel_robot__msg__StepDetection__Sequence__are_equal(const wheel_robot__msg__StepDetection__Sequence * lhs, const wheel_robot__msg__StepDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheel_robot__msg__StepDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheel_robot__msg__StepDetection__Sequence__copy(
  const wheel_robot__msg__StepDetection__Sequence * input,
  wheel_robot__msg__StepDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheel_robot__msg__StepDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheel_robot__msg__StepDetection * data =
      (wheel_robot__msg__StepDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheel_robot__msg__StepDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheel_robot__msg__StepDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheel_robot__msg__StepDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
