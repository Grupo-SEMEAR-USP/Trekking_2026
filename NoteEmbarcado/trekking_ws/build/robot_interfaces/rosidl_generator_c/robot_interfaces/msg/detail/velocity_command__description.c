// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_interfaces:msg/VelocityCommand.idl
// generated code does not contain a copyright notice

#include "robot_interfaces/msg/detail/velocity_command__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_interfaces
const rosidl_type_hash_t *
robot_interfaces__msg__VelocityCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x78, 0xad, 0xbc, 0xec, 0x59, 0x36, 0x23, 0x71,
      0xb7, 0x55, 0xbc, 0xc1, 0xfd, 0xd7, 0x7d, 0x17,
      0xab, 0xe2, 0xa2, 0x4d, 0x7b, 0xf7, 0x9d, 0xf6,
      0xe7, 0x22, 0x58, 0x6f, 0xc0, 0x3b, 0x34, 0x72,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_interfaces__msg__VelocityCommand__TYPE_NAME[] = "robot_interfaces/msg/VelocityCommand";

// Define type names, field names, and default values
static char robot_interfaces__msg__VelocityCommand__FIELD_NAME__angular_speed_left[] = "angular_speed_left";
static char robot_interfaces__msg__VelocityCommand__FIELD_NAME__angular_speed_right[] = "angular_speed_right";
static char robot_interfaces__msg__VelocityCommand__FIELD_NAME__servo_angle[] = "servo_angle";

static rosidl_runtime_c__type_description__Field robot_interfaces__msg__VelocityCommand__FIELDS[] = {
  {
    {robot_interfaces__msg__VelocityCommand__FIELD_NAME__angular_speed_left, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VelocityCommand__FIELD_NAME__angular_speed_right, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VelocityCommand__FIELD_NAME__servo_angle, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_interfaces__msg__VelocityCommand__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_interfaces__msg__VelocityCommand__TYPE_NAME, 36, 36},
      {robot_interfaces__msg__VelocityCommand__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 angular_speed_left\n"
  "float32 angular_speed_right\n"
  "float32 servo_angle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_interfaces__msg__VelocityCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_interfaces__msg__VelocityCommand__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 74, 74},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_interfaces__msg__VelocityCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_interfaces__msg__VelocityCommand__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
