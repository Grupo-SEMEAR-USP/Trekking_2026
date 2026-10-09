// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_control:msg/VelocityData.idl
// generated code does not contain a copyright notice

#include "robot_control/msg/detail/velocity_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_type_hash_t *
robot_control__msg__VelocityData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb1, 0x4f, 0x5d, 0x89, 0xfb, 0x65, 0x1f, 0x3f,
      0x1a, 0x38, 0x37, 0x80, 0xf4, 0xd3, 0x39, 0x16,
      0x5e, 0x0d, 0x49, 0xe8, 0xa9, 0x41, 0x5a, 0x48,
      0x40, 0x03, 0x77, 0x7e, 0x88, 0x78, 0xf4, 0xa9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_control__msg__VelocityData__TYPE_NAME[] = "robot_control/msg/VelocityData";

// Define type names, field names, and default values
static char robot_control__msg__VelocityData__FIELD_NAME__angular_speed_left[] = "angular_speed_left";
static char robot_control__msg__VelocityData__FIELD_NAME__angular_speed_right[] = "angular_speed_right";
static char robot_control__msg__VelocityData__FIELD_NAME__servo_angle[] = "servo_angle";

static rosidl_runtime_c__type_description__Field robot_control__msg__VelocityData__FIELDS[] = {
  {
    {robot_control__msg__VelocityData__FIELD_NAME__angular_speed_left, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__VelocityData__FIELD_NAME__angular_speed_right, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__VelocityData__FIELD_NAME__servo_angle, 11, 11},
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
robot_control__msg__VelocityData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_control__msg__VelocityData__TYPE_NAME, 30, 30},
      {robot_control__msg__VelocityData__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# VelocityData.msg\n"
  "float32 angular_speed_left\n"
  "float32 angular_speed_right\n"
  "float32 servo_angle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_control__msg__VelocityData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_control__msg__VelocityData__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 93, 93},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_control__msg__VelocityData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_control__msg__VelocityData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
