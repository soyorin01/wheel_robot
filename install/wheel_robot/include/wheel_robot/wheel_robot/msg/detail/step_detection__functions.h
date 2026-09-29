// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wheel_robot:msg/StepDetection.idl
// generated code does not contain a copyright notice

#ifndef WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__FUNCTIONS_H_
#define WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "wheel_robot/msg/rosidl_generator_c__visibility_control.h"

#include "wheel_robot/msg/detail/step_detection__struct.h"

/// Initialize msg/StepDetection message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wheel_robot__msg__StepDetection
 * )) before or use
 * wheel_robot__msg__StepDetection__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepDetection__init(wheel_robot__msg__StepDetection * msg);

/// Finalize msg/StepDetection message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepDetection__fini(wheel_robot__msg__StepDetection * msg);

/// Create msg/StepDetection message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wheel_robot__msg__StepDetection__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
wheel_robot__msg__StepDetection *
wheel_robot__msg__StepDetection__create();

/// Destroy msg/StepDetection message.
/**
 * It calls
 * wheel_robot__msg__StepDetection__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepDetection__destroy(wheel_robot__msg__StepDetection * msg);

/// Check for msg/StepDetection message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepDetection__are_equal(const wheel_robot__msg__StepDetection * lhs, const wheel_robot__msg__StepDetection * rhs);

/// Copy a msg/StepDetection message.
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
wheel_robot__msg__StepDetection__copy(
  const wheel_robot__msg__StepDetection * input,
  wheel_robot__msg__StepDetection * output);

/// Initialize array of msg/StepDetection messages.
/**
 * It allocates the memory for the number of elements and calls
 * wheel_robot__msg__StepDetection__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepDetection__Sequence__init(wheel_robot__msg__StepDetection__Sequence * array, size_t size);

/// Finalize array of msg/StepDetection messages.
/**
 * It calls
 * wheel_robot__msg__StepDetection__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepDetection__Sequence__fini(wheel_robot__msg__StepDetection__Sequence * array);

/// Create array of msg/StepDetection messages.
/**
 * It allocates the memory for the array and calls
 * wheel_robot__msg__StepDetection__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
wheel_robot__msg__StepDetection__Sequence *
wheel_robot__msg__StepDetection__Sequence__create(size_t size);

/// Destroy array of msg/StepDetection messages.
/**
 * It calls
 * wheel_robot__msg__StepDetection__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
void
wheel_robot__msg__StepDetection__Sequence__destroy(wheel_robot__msg__StepDetection__Sequence * array);

/// Check for msg/StepDetection message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wheel_robot
bool
wheel_robot__msg__StepDetection__Sequence__are_equal(const wheel_robot__msg__StepDetection__Sequence * lhs, const wheel_robot__msg__StepDetection__Sequence * rhs);

/// Copy an array of msg/StepDetection messages.
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
wheel_robot__msg__StepDetection__Sequence__copy(
  const wheel_robot__msg__StepDetection__Sequence * input,
  wheel_robot__msg__StepDetection__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WHEEL_ROBOT__MSG__DETAIL__STEP_DETECTION__FUNCTIONS_H_
