// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/UARTData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/uart_data.h"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__UART_DATA__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__UART_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/UARTData in the package robot_interfaces.
typedef struct robot_interfaces__msg__UARTData
{
  int32_t x;
  int32_t y;
  int32_t z;
  uint32_t timestamp;
} robot_interfaces__msg__UARTData;

// Struct for a sequence of robot_interfaces__msg__UARTData.
typedef struct robot_interfaces__msg__UARTData__Sequence
{
  robot_interfaces__msg__UARTData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__UARTData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__UART_DATA__STRUCT_H_
