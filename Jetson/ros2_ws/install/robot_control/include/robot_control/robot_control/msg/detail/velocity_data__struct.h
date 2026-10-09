// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_control:msg/VelocityData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_control/msg/velocity_data.h"


#ifndef ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__STRUCT_H_
#define ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/VelocityData in the package robot_control.
/**
  * VelocityData.msg
 */
typedef struct robot_control__msg__VelocityData
{
  float angular_speed_left;
  float angular_speed_right;
  float servo_angle;
} robot_control__msg__VelocityData;

// Struct for a sequence of robot_control__msg__VelocityData.
typedef struct robot_control__msg__VelocityData__Sequence
{
  robot_control__msg__VelocityData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_control__msg__VelocityData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_CONTROL__MSG__DETAIL__VELOCITY_DATA__STRUCT_H_
