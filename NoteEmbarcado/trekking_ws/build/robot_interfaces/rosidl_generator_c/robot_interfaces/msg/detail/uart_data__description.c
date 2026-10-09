// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_interfaces:msg/UARTData.idl
// generated code does not contain a copyright notice

#include "robot_interfaces/msg/detail/uart_data__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_interfaces
const rosidl_type_hash_t *
robot_interfaces__msg__UARTData__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xac, 0x94, 0xda, 0xa8, 0xa5, 0xbb, 0x6c, 0x9a,
      0x86, 0x79, 0x25, 0x4f, 0xdc, 0x3a, 0x95, 0x33,
      0xb1, 0x42, 0x17, 0x6a, 0x96, 0x53, 0x83, 0xdb,
      0x57, 0x7f, 0x2d, 0x2e, 0x8c, 0xeb, 0xd7, 0x5a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_interfaces__msg__UARTData__TYPE_NAME[] = "robot_interfaces/msg/UARTData";

// Define type names, field names, and default values
static char robot_interfaces__msg__UARTData__FIELD_NAME__x[] = "x";
static char robot_interfaces__msg__UARTData__FIELD_NAME__y[] = "y";
static char robot_interfaces__msg__UARTData__FIELD_NAME__z[] = "z";
static char robot_interfaces__msg__UARTData__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field robot_interfaces__msg__UARTData__FIELDS[] = {
  {
    {robot_interfaces__msg__UARTData__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__UARTData__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__UARTData__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__UARTData__FIELD_NAME__timestamp, 9, 9},
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
robot_interfaces__msg__UARTData__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_interfaces__msg__UARTData__TYPE_NAME, 29, 29},
      {robot_interfaces__msg__UARTData__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 x\n"
  "int32 y\n"
  "int32 z\n"
  "uint32 timestamp";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_interfaces__msg__UARTData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_interfaces__msg__UARTData__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 40, 40},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_interfaces__msg__UARTData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_interfaces__msg__UARTData__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
