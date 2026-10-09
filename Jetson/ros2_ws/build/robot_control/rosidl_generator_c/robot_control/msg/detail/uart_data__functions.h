// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_control:msg/UARTData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/uart_data.h"


#ifndef ROBOT_CONTROL__MSG__DETAIL__UART_DATA__FUNCTIONS_H_
#define ROBOT_CONTROL__MSG__DETAIL__UART_DATA__FUNCTIONS_H_

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

#include "robot_control/msg/detail/uart_data__struct.h"

/// Initialize msg/UARTData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_control__msg__UARTData
 * )) before or use
 * robot_control__msg__UARTData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__UARTData__init(robot_control__msg__UARTData * msg);

/// Finalize msg/UARTData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__UARTData__fini(robot_control__msg__UARTData * msg);

/// Create msg/UARTData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_control__msg__UARTData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
robot_control__msg__UARTData *
robot_control__msg__UARTData__create(void);

/// Destroy msg/UARTData message.
/**
 * It calls
 * robot_control__msg__UARTData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__UARTData__destroy(robot_control__msg__UARTData * msg);

/// Check for msg/UARTData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__UARTData__are_equal(const robot_control__msg__UARTData * lhs, const robot_control__msg__UARTData * rhs);

/// Copy a msg/UARTData message.
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
robot_control__msg__UARTData__copy(
  const robot_control__msg__UARTData * input,
  robot_control__msg__UARTData * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_type_hash_t *
robot_control__msg__UARTData__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_runtime_c__type_description__TypeDescription *
robot_control__msg__UARTData__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_runtime_c__type_description__TypeSource *
robot_control__msg__UARTData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_control__msg__UARTData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/UARTData messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_control__msg__UARTData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__UARTData__Sequence__init(robot_control__msg__UARTData__Sequence * array, size_t size);

/// Finalize array of msg/UARTData messages.
/**
 * It calls
 * robot_control__msg__UARTData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__UARTData__Sequence__fini(robot_control__msg__UARTData__Sequence * array);

/// Create array of msg/UARTData messages.
/**
 * It allocates the memory for the array and calls
 * robot_control__msg__UARTData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
robot_control__msg__UARTData__Sequence *
robot_control__msg__UARTData__Sequence__create(size_t size);

/// Destroy array of msg/UARTData messages.
/**
 * It calls
 * robot_control__msg__UARTData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
void
robot_control__msg__UARTData__Sequence__destroy(robot_control__msg__UARTData__Sequence * array);

/// Check for msg/UARTData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_control
bool
robot_control__msg__UARTData__Sequence__are_equal(const robot_control__msg__UARTData__Sequence * lhs, const robot_control__msg__UARTData__Sequence * rhs);

/// Copy an array of msg/UARTData messages.
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
robot_control__msg__UARTData__Sequence__copy(
  const robot_control__msg__UARTData__Sequence * input,
  robot_control__msg__UARTData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL__MSG__DETAIL__UART_DATA__FUNCTIONS_H_
