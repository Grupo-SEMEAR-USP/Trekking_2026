// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_interfaces:msg/VelocityData.idl
// generated code does not contain a copyright notice

#include "robot_interfaces/msg/detail/velocity_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_interfaces
const rosidl_type_hash_t *
robot_interfaces__msg__VelocityData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb6, 0xaf, 0x0e, 0x66, 0x7a, 0x5f, 0xe2, 0xe2,
      0x3f, 0x29, 0xbe, 0x53, 0xe5, 0x36, 0x17, 0xc5,
      0xe4, 0xdd, 0xb6, 0xfe, 0x56, 0x5e, 0xf6, 0x81,
      0x12, 0x3e, 0x20, 0xf1, 0xc8, 0xa9, 0xb3, 0xb7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_interfaces__msg__VelocityData__TYPE_NAME[] = "robot_interfaces/msg/VelocityData";

// Define type names, field names, and default values
static char robot_interfaces__msg__VelocityData__FIELD_NAME__angular_speed_left[] = "angular_speed_left";
static char robot_interfaces__msg__VelocityData__FIELD_NAME__angular_speed_right[] = "angular_speed_right";
static char robot_interfaces__msg__VelocityData__FIELD_NAME__servo_angle[] = "servo_angle";

static rosidl_runtime_c__type_description__Field robot_interfaces__msg__VelocityData__FIELDS[] = {
  {
    {robot_interfaces__msg__VelocityData__FIELD_NAME__angular_speed_left, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VelocityData__FIELD_NAME__angular_speed_right, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VelocityData__FIELD_NAME__servo_angle, 11, 11},
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
robot_interfaces__msg__VelocityData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_interfaces__msg__VelocityData__TYPE_NAME, 33, 33},
      {robot_interfaces__msg__VelocityData__FIELDS, 3, 3},
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
robot_interfaces__msg__VelocityData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_interfaces__msg__VelocityData__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 74, 74},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_interfaces__msg__VelocityData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_interfaces__msg__VelocityData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
