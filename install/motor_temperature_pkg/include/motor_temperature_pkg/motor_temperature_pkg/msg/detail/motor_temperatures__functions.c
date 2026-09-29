// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_temperature_pkg:msg/MotorTemperatures.idl
// generated code does not contain a copyright notice
#include "motor_temperature_pkg/msg/detail/motor_temperatures__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `temperature_c`
// Member `overheated_index`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
motor_temperature_pkg__msg__MotorTemperatures__init(motor_temperature_pkg__msg__MotorTemperatures * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motor_temperature_pkg__msg__MotorTemperatures__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    motor_temperature_pkg__msg__MotorTemperatures__fini(msg);
    return false;
  }
  // temperature_c
  if (!rosidl_runtime_c__float__Sequence__init(&msg->temperature_c, 0)) {
    motor_temperature_pkg__msg__MotorTemperatures__fini(msg);
    return false;
  }
  // warning_threshold_c
  // overheated_index
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->overheated_index, 0)) {
    motor_temperature_pkg__msg__MotorTemperatures__fini(msg);
    return false;
  }
  // overheated
  return true;
}

void
motor_temperature_pkg__msg__MotorTemperatures__fini(motor_temperature_pkg__msg__MotorTemperatures * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__Sequence__fini(&msg->name);
  // temperature_c
  rosidl_runtime_c__float__Sequence__fini(&msg->temperature_c);
  // warning_threshold_c
  // overheated_index
  rosidl_runtime_c__uint8__Sequence__fini(&msg->overheated_index);
  // overheated
}

bool
motor_temperature_pkg__msg__MotorTemperatures__are_equal(const motor_temperature_pkg__msg__MotorTemperatures * lhs, const motor_temperature_pkg__msg__MotorTemperatures * rhs)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // temperature_c
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->temperature_c), &(rhs->temperature_c)))
  {
    return false;
  }
  // warning_threshold_c
  if (lhs->warning_threshold_c != rhs->warning_threshold_c) {
    return false;
  }
  // overheated_index
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->overheated_index), &(rhs->overheated_index)))
  {
    return false;
  }
  // overheated
  if (lhs->overheated != rhs->overheated) {
    return false;
  }
  return true;
}

bool
motor_temperature_pkg__msg__MotorTemperatures__copy(
  const motor_temperature_pkg__msg__MotorTemperatures * input,
  motor_temperature_pkg__msg__MotorTemperatures * output)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // temperature_c
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->temperature_c), &(output->temperature_c)))
  {
    return false;
  }
  // warning_threshold_c
  output->warning_threshold_c = input->warning_threshold_c;
  // overheated_index
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->overheated_index), &(output->overheated_index)))
  {
    return false;
  }
  // overheated
  output->overheated = input->overheated;
  return true;
}

motor_temperature_pkg__msg__MotorTemperatures *
motor_temperature_pkg__msg__MotorTemperatures__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_temperature_pkg__msg__MotorTemperatures * msg = (motor_temperature_pkg__msg__MotorTemperatures *)allocator.allocate(sizeof(motor_temperature_pkg__msg__MotorTemperatures), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_temperature_pkg__msg__MotorTemperatures));
  bool success = motor_temperature_pkg__msg__MotorTemperatures__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_temperature_pkg__msg__MotorTemperatures__destroy(motor_temperature_pkg__msg__MotorTemperatures * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_temperature_pkg__msg__MotorTemperatures__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_temperature_pkg__msg__MotorTemperatures__Sequence__init(motor_temperature_pkg__msg__MotorTemperatures__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_temperature_pkg__msg__MotorTemperatures * data = NULL;

  if (size) {
    data = (motor_temperature_pkg__msg__MotorTemperatures *)allocator.zero_allocate(size, sizeof(motor_temperature_pkg__msg__MotorTemperatures), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_temperature_pkg__msg__MotorTemperatures__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_temperature_pkg__msg__MotorTemperatures__fini(&data[i - 1]);
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
motor_temperature_pkg__msg__MotorTemperatures__Sequence__fini(motor_temperature_pkg__msg__MotorTemperatures__Sequence * array)
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
      motor_temperature_pkg__msg__MotorTemperatures__fini(&array->data[i]);
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

motor_temperature_pkg__msg__MotorTemperatures__Sequence *
motor_temperature_pkg__msg__MotorTemperatures__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_temperature_pkg__msg__MotorTemperatures__Sequence * array = (motor_temperature_pkg__msg__MotorTemperatures__Sequence *)allocator.allocate(sizeof(motor_temperature_pkg__msg__MotorTemperatures__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_temperature_pkg__msg__MotorTemperatures__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_temperature_pkg__msg__MotorTemperatures__Sequence__destroy(motor_temperature_pkg__msg__MotorTemperatures__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_temperature_pkg__msg__MotorTemperatures__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_temperature_pkg__msg__MotorTemperatures__Sequence__are_equal(const motor_temperature_pkg__msg__MotorTemperatures__Sequence * lhs, const motor_temperature_pkg__msg__MotorTemperatures__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_temperature_pkg__msg__MotorTemperatures__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_temperature_pkg__msg__MotorTemperatures__Sequence__copy(
  const motor_temperature_pkg__msg__MotorTemperatures__Sequence * input,
  motor_temperature_pkg__msg__MotorTemperatures__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_temperature_pkg__msg__MotorTemperatures);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_temperature_pkg__msg__MotorTemperatures * data =
      (motor_temperature_pkg__msg__MotorTemperatures *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_temperature_pkg__msg__MotorTemperatures__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_temperature_pkg__msg__MotorTemperatures__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_temperature_pkg__msg__MotorTemperatures__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
