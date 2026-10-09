// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_control:msg/VelocityData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/velocity_data.h"


#ifndef ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__FUNCTIONS_H_
#define ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_control/msg/rosidl_generator_c__visibility_control.h"

#include "robot_control/msg/detail/velocity_data__struct.h"

/// Initialize msg/VelocityData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_control__msg__VelocityData
 * )) before or use
 * robot_control__msg__VelocityData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__VelocityData__init(robot_control__msg__VelocityData * msg);

/// Finalize msg/VelocityData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__VelocityData__fini(robot_control__msg__VelocityData * msg);

/// Create msg/VelocityData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_control__msg__VelocityData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
robot_control__msg__VelocityData *
robot_control__msg__VelocityData__create(void);

/// Destroy msg/VelocityData message.
/**
 * It calls
 * robot_control__msg__VelocityData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__VelocityData__destroy(robot_control__msg__VelocityData * msg);

/// Check for msg/VelocityData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__VelocityData__are_equal(const robot_control__msg__VelocityData * lhs, const robot_control__msg__VelocityData * rhs);

/// Copy a msg/VelocityData message.
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
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__VelocityData__copy(
  const robot_control__msg__VelocityData * input,
  robot_control__msg__VelocityData * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_type_hash_t *
robot_control__msg__VelocityData__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_runtime_c__type_description__TypeDescription *
robot_control__msg__VelocityData__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_runtime_c__type_description__TypeSource *
robot_control__msg__VelocityData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_control__msg__VelocityData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/VelocityData messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_control__msg__VelocityData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__VelocityData__Sequence__init(robot_control__msg__VelocityData__Sequence * array, size_t size);

/// Finalize array of msg/VelocityData messages.
/**
 * It calls
 * robot_control__msg__VelocityData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__VelocityData__Sequence__fini(robot_control__msg__VelocityData__Sequence * array);

/// Create array of msg/VelocityData messages.
/**
 * It allocates the memory for the array and calls
 * robot_control__msg__VelocityData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
robot_control__msg__VelocityData__Sequence *
robot_control__msg__VelocityData__Sequence__create(size_t size);

/// Destroy array of msg/VelocityData messages.
/**
 * It calls
 * robot_control__msg__VelocityData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__VelocityData__Sequence__destroy(robot_control__msg__VelocityData__Sequence * array);

/// Check for msg/VelocityData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__VelocityData__Sequence__are_equal(const robot_control__msg__VelocityData__Sequence * lhs, const robot_control__msg__VelocityData__Sequence * rhs);

/// Copy an array of msg/VelocityData messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__VelocityData__Sequence__copy(
  const robot_control__msg__VelocityData__Sequence * input,
  robot_control__msg__VelocityData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__FUNCTIONS_H_
