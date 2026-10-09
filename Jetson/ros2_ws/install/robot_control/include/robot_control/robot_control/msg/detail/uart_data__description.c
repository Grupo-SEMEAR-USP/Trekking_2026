// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_control:msg/UARTData.idl
// generated code does not contain a copyright notice

#include "robot_control/msg/detail/uart_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_type_hash_t *
robot_control__msg__UARTData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xae, 0x2e, 0x84, 0x90, 0xd2, 0xd6, 0xb3, 0x6b,
      0xeb, 0x5e, 0xca, 0xa1, 0xc1, 0x62, 0x34, 0x69,
      0x26, 0x0d, 0xd4, 0x8e, 0x6c, 0x38, 0x28, 0x4c,
      0x8c, 0x93, 0x30, 0x6e, 0xee, 0x0a, 0xb4, 0xa2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_control__msg__UARTData__TYPE_NAME[] = "robot_control/msg/UARTData";

// Define type names, field names, and default values
static char robot_control__msg__UARTData__FIELD_NAME__x[] = "x";
static char robot_control__msg__UARTData__FIELD_NAME__y[] = "y";
static char robot_control__msg__UARTData__FIELD_NAME__z[] = "z";
static char robot_control__msg__UARTData__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field robot_control__msg__UARTData__FIELDS[] = {
  {
    {robot_control__msg__UARTData__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__UARTData__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__UARTData__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__UARTData__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_control__msg__UARTData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_control__msg__UARTData__TYPE_NAME, 26, 26},
      {robot_control__msg__UARTData__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# UARTData.msg\n"
  "float32 x\n"
  "float32 y\n"
  "float32 z\n"
  "uint32 timestamp ";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_control__msg__UARTData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_control__msg__UARTData__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 62, 62},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_control__msg__UARTData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_control__msg__UARTData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
