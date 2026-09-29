// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wheel_robot:msg/StepApproachStatus.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__FUNCTIONS_H_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "wheel_robot/msg/rosidl_generator_c__visibility_control.h"

#include "wheel_robot/msg/detail/step_approach_status__struct.h"

/// Initialize msg/StepApproachStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wheel_robot__msg__StepApproachStatus
 * )) before or use
 * wheel_robot__msg__StepApproachStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepApproachStatus__init(wheel_robot__msg__StepApproachStatus * msg);

/// Finalize msg/StepApproachStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepApproachStatus__fini(wheel_robot__msg__StepApproachStatus * msg);

/// Create msg/StepApproachStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wheel_robot__msg__StepApproachStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
wheel_robot__msg__StepApproachStatus *
wheel_robot__msg__StepApproachStatus__create();

/// Destroy msg/StepApproachStatus message.
/**
 * It calls
 * wheel_robot__msg__StepApproachStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepApproachStatus__destroy(wheel_robot__msg__StepApproachStatus * msg);

/// Check for msg/StepApproachStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepApproachStatus__are_equal(const wheel_robot__msg__StepApproachStatus * lhs, const wheel_robot__msg__StepApproachStatus * rhs);

/// Copy a msg/StepApproachStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepApproachStatus__copy(
  const wheel_robot__msg__StepApproachStatus * input,
  wheel_robot__msg__StepApproachStatus * output);

/// Initialize array of msg/StepApproachStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * wheel_robot__msg__StepApproachStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepApproachStatus__Sequence__init(wheel_robot__msg__StepApproachStatus__Sequence * array, size_t size);

/// Finalize array of msg/StepApproachStatus messages.
/**
 * It calls
 * wheel_robot__msg__StepApproachStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepApproachStatus__Sequence__fini(wheel_robot__msg__StepApproachStatus__Sequence * array);

/// Create array of msg/StepApproachStatus messages.
/**
 * It allocates the memory for the array and calls
 * wheel_robot__msg__StepApproachStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
wheel_robot__msg__StepApproachStatus__Sequence *
wheel_robot__msg__StepApproachStatus__Sequence__create(size_t size);

/// Destroy array of msg/StepApproachStatus messages.
/**
 * It calls
 * wheel_robot__msg__StepApproachStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepApproachStatus__Sequence__destroy(wheel_robot__msg__StepApproachStatus__Sequence * array);

/// Check for msg/StepApproachStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepApproachStatus__Sequence__are_equal(const wheel_robot__msg__StepApproachStatus__Sequence * lhs, const wheel_robot__msg__StepApproachStatus__Sequence * rhs);

/// Copy an array of msg/StepApproachStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepApproachStatus__Sequence__copy(
  const wheel_robot__msg__StepApproachStatus__Sequence * input,
  wheel_robot__msg__StepApproachStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_APPROACH_STATUS__FUNCTIONS_H_
