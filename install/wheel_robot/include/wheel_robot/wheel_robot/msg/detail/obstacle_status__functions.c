// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wheel_robot:msg/ObstacleStatus.idl
// generated code does not contain a copyright notice
#include "wheel_robot/msg/detail/obstacle_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
wheel_robot__msg__ObstacleStatus__init(wheel_robot__msg__ObstacleStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wheel_robot__msg__ObstacleStatus__fini(msg);
    return false;
  }
  // status
  // left_status
  // center_status
  // right_status
  // left_distance
  // center_distance
  // right_distance
  return true;
}

void
wheel_robot__msg__ObstacleStatus__fini(wheel_robot__msg__ObstacleStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // status
  // left_status
  // center_status
  // right_status
  // left_distance
  // center_distance
  // right_distance
}

bool
wheel_robot__msg__ObstacleStatus__are_equal(const wheel_robot__msg__ObstacleStatus * lhs, const wheel_robot__msg__ObstacleStatus * rhs)
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
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // left_status
  if (lhs->left_status != rhs->left_status) {
    return false;
  }
  // center_status
  if (lhs->center_status != rhs->center_status) {
    return false;
  }
  // right_status
  if (lhs->right_status != rhs->right_status) {
    return false;
  }
  // left_distance
  if (lhs->left_distance != rhs->left_distance) {
    return false;
  }
  // center_distance
  if (lhs->center_distance != rhs->center_distance) {
    return false;
  }
  // right_distance
  if (lhs->right_distance != rhs->right_distance) {
    return false;
  }
  return true;
}

bool
wheel_robot__msg__ObstacleStatus__copy(
  const wheel_robot__msg__ObstacleStatus * input,
  wheel_robot__msg__ObstacleStatus * output)
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
  // status
  output->status = input->status;
  // left_status
  output->left_status = input->left_status;
  // center_status
  output->center_status = input->center_status;
  // right_status
  output->right_status = input->right_status;
  // left_distance
  output->left_distance = input->left_distance;
  // center_distance
  output->center_distance = input->center_distance;
  // right_distance
  output->right_distance = input->right_distance;
  return true;
}

wheel_robot__msg__ObstacleStatus *
wheel_robot__msg__ObstacleStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__ObstacleStatus * msg = (wheel_robot__msg__ObstacleStatus *)allocator.allocate(sizeof(wheel_robot__msg__ObstacleStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheel_robot__msg__ObstacleStatus));
  bool success = wheel_robot__msg__ObstacleStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheel_robot__msg__ObstacleStatus__destroy(wheel_robot__msg__ObstacleStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheel_robot__msg__ObstacleStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheel_robot__msg__ObstacleStatus__Sequence__init(wheel_robot__msg__ObstacleStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__ObstacleStatus * data = NULL;

  if (size) {
    data = (wheel_robot__msg__ObstacleStatus *)allocator.zero_allocate(size, sizeof(wheel_robot__msg__ObstacleStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheel_robot__msg__ObstacleStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheel_robot__msg__ObstacleStatus__fini(&data[i - 1]);
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
wheel_robot__msg__ObstacleStatus__Sequence__fini(wheel_robot__msg__ObstacleStatus__Sequence * array)
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
      wheel_robot__msg__ObstacleStatus__fini(&array->data[i]);
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

wheel_robot__msg__ObstacleStatus__Sequence *
wheel_robot__msg__ObstacleStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__ObstacleStatus__Sequence * array = (wheel_robot__msg__ObstacleStatus__Sequence *)allocator.allocate(sizeof(wheel_robot__msg__ObstacleStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheel_robot__msg__ObstacleStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheel_robot__msg__ObstacleStatus__Sequence__destroy(wheel_robot__msg__ObstacleStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheel_robot__msg__ObstacleStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheel_robot__msg__ObstacleStatus__Sequence__are_equal(const wheel_robot__msg__ObstacleStatus__Sequence * lhs, const wheel_robot__msg__ObstacleStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheel_robot__msg__ObstacleStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheel_robot__msg__ObstacleStatus__Sequence__copy(
  const wheel_robot__msg__ObstacleStatus__Sequence * input,
  wheel_robot__msg__ObstacleStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheel_robot__msg__ObstacleStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheel_robot__msg__ObstacleStatus * data =
      (wheel_robot__msg__ObstacleStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheel_robot__msg__ObstacleStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheel_robot__msg__ObstacleStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheel_robot__msg__ObstacleStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
