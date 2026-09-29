// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wheel_robot:msg/StepApproachStatus.idl
// generated code does not contain a copyright notice
#include "wheel_robot/msg/detail/step_approach_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "rosidl_runtime_c/string_functions.h"

bool
wheel_robot__msg__StepApproachStatus__init(wheel_robot__msg__StepApproachStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wheel_robot__msg__StepApproachStatus__fini(msg);
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    wheel_robot__msg__StepApproachStatus__fini(msg);
    return false;
  }
  // locked
  // step_detected
  // detected_distance
  // height
  // confidence
  // stop_distance
  // target_travel
  // traveled
  // remaining
  return true;
}

void
wheel_robot__msg__StepApproachStatus__fini(wheel_robot__msg__StepApproachStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // locked
  // step_detected
  // detected_distance
  // height
  // confidence
  // stop_distance
  // target_travel
  // traveled
  // remaining
}

bool
wheel_robot__msg__StepApproachStatus__are_equal(const wheel_robot__msg__StepApproachStatus * lhs, const wheel_robot__msg__StepApproachStatus * rhs)
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
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // locked
  if (lhs->locked != rhs->locked) {
    return false;
  }
  // step_detected
  if (lhs->step_detected != rhs->step_detected) {
    return false;
  }
  // detected_distance
  if (lhs->detected_distance != rhs->detected_distance) {
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
  // stop_distance
  if (lhs->stop_distance != rhs->stop_distance) {
    return false;
  }
  // target_travel
  if (lhs->target_travel != rhs->target_travel) {
    return false;
  }
  // traveled
  if (lhs->traveled != rhs->traveled) {
    return false;
  }
  // remaining
  if (lhs->remaining != rhs->remaining) {
    return false;
  }
  return true;
}

bool
wheel_robot__msg__StepApproachStatus__copy(
  const wheel_robot__msg__StepApproachStatus * input,
  wheel_robot__msg__StepApproachStatus * output)
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
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // locked
  output->locked = input->locked;
  // step_detected
  output->step_detected = input->step_detected;
  // detected_distance
  output->detected_distance = input->detected_distance;
  // height
  output->height = input->height;
  // confidence
  output->confidence = input->confidence;
  // stop_distance
  output->stop_distance = input->stop_distance;
  // target_travel
  output->target_travel = input->target_travel;
  // traveled
  output->traveled = input->traveled;
  // remaining
  output->remaining = input->remaining;
  return true;
}

wheel_robot__msg__StepApproachStatus *
wheel_robot__msg__StepApproachStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__StepApproachStatus * msg = (wheel_robot__msg__StepApproachStatus *)allocator.allocate(sizeof(wheel_robot__msg__StepApproachStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wheel_robot__msg__StepApproachStatus));
  bool success = wheel_robot__msg__StepApproachStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wheel_robot__msg__StepApproachStatus__destroy(wheel_robot__msg__StepApproachStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wheel_robot__msg__StepApproachStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wheel_robot__msg__StepApproachStatus__Sequence__init(wheel_robot__msg__StepApproachStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__StepApproachStatus * data = NULL;

  if (size) {
    data = (wheel_robot__msg__StepApproachStatus *)allocator.zero_allocate(size, sizeof(wheel_robot__msg__StepApproachStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wheel_robot__msg__StepApproachStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wheel_robot__msg__StepApproachStatus__fini(&data[i - 1]);
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
wheel_robot__msg__StepApproachStatus__Sequence__fini(wheel_robot__msg__StepApproachStatus__Sequence * array)
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
      wheel_robot__msg__StepApproachStatus__fini(&array->data[i]);
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

wheel_robot__msg__StepApproachStatus__Sequence *
wheel_robot__msg__StepApproachStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wheel_robot__msg__StepApproachStatus__Sequence * array = (wheel_robot__msg__StepApproachStatus__Sequence *)allocator.allocate(sizeof(wheel_robot__msg__StepApproachStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wheel_robot__msg__StepApproachStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wheel_robot__msg__StepApproachStatus__Sequence__destroy(wheel_robot__msg__StepApproachStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wheel_robot__msg__StepApproachStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wheel_robot__msg__StepApproachStatus__Sequence__are_equal(const wheel_robot__msg__StepApproachStatus__Sequence * lhs, const wheel_robot__msg__StepApproachStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wheel_robot__msg__StepApproachStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wheel_robot__msg__StepApproachStatus__Sequence__copy(
  const wheel_robot__msg__StepApproachStatus__Sequence * input,
  wheel_robot__msg__StepApproachStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(wheel_robot__msg__StepApproachStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wheel_robot__msg__StepApproachStatus * data =
      (wheel_robot__msg__StepApproachStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wheel_robot__msg__StepApproachStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wheel_robot__msg__StepApproachStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wheel_robot__msg__StepApproachStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
