// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_control:msg/I2cData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/i2c_data.h"


#ifndef ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__STRUCT_H_
#define ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/I2cData in the package robot_control.
/**
  * I2cData.msg
 */
typedef struct robot_control__msg__I2cData
{
  float x;
  float y;
  float z;
  uint32_t timestamp;
} robot_control__msg__I2cData;

// Struct for a sequence of robot_control__msg__I2cData.
typedef struct robot_control__msg__I2cData__Sequence
{
  robot_control__msg__I2cData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control__msg__I2cData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL__MSG__DETAIL__I2C_DATA__STRUCT_H_
