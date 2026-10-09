// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_control:msg/I2cData.idl
// generated code does not contain a copyright notice

#include "robot_control/msg/detail/i2c_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_control
const rosidl_type_hash_t *
robot_control__msg__I2cData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0x98, 0x94, 0x19, 0xf5, 0xd9, 0x5a, 0xe1,
      0x12, 0x5a, 0x96, 0x19, 0x3c, 0x09, 0xd1, 0x68,
      0xa1, 0xe9, 0x24, 0x12, 0x4e, 0xa2, 0x68, 0xe4,
      0xa3, 0xe6, 0xdc, 0xdd, 0xec, 0xf9, 0xf4, 0xfe,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_control__msg__I2cData__TYPE_NAME[] = "robot_control/msg/I2cData";

// Define type names, field names, and default values
static char robot_control__msg__I2cData__FIELD_NAME__x[] = "x";
static char robot_control__msg__I2cData__FIELD_NAME__y[] = "y";
static char robot_control__msg__I2cData__FIELD_NAME__z[] = "z";
static char robot_control__msg__I2cData__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field robot_control__msg__I2cData__FIELDS[] = {
  {
    {robot_control__msg__I2cData__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__I2cData__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__I2cData__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_control__msg__I2cData__FIELD_NAME__timestamp, 9, 9},
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
robot_control__msg__I2cData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_control__msg__I2cData__TYPE_NAME, 25, 25},
      {robot_control__msg__I2cData__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# I2cData.msg\n"
  "float32 x\n"
  "float32 y\n"
  "float32 z \n"
  "uint32 timestamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_control__msg__I2cData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_control__msg__I2cData__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 61, 61},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_control__msg__I2cData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_control__msg__I2cData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
