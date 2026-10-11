// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/VelocityCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/velocity_command.h"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_COMMAND__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/VelocityCommand in the package robot_interfaces.
typedef struct robot_interfaces__msg__VelocityCommand
{
  float angular_speed_left;
  float angular_speed_right;
  float servo_angle;
} robot_interfaces__msg__VelocityCommand;

// Struct for a sequence of robot_interfaces__msg__VelocityCommand.
typedef struct robot_interfaces__msg__VelocityCommand__Sequence
{
  robot_interfaces__msg__VelocityCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__VelocityCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VELOCITY_COMMAND__STRUCT_H_
