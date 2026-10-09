// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/VelocityData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/velocity_data.h"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_DATA__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/VelocityData in the package robot_interfaces.
typedef struct robot_interfaces__msg__VelocityData
{
  float angular_speed_left;
  float angular_speed_right;
  float servo_angle;
} robot_interfaces__msg__VelocityData;

// Struct for a sequence of robot_interfaces__msg__VelocityData.
typedef struct robot_interfaces__msg__VelocityData__Sequence
{
  robot_interfaces__msg__VelocityData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__VelocityData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_DATA__STRUCT_H_
