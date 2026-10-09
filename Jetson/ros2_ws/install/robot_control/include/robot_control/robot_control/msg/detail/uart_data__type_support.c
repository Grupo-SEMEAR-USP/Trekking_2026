// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_control:msg/UARTData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_control/msg/detail/uart_data__rosidl_typesupport_introspection_c.h"
#include "robot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_control/msg/detail/uart_data__functions.h"
#include "robot_control/msg/detail/uart_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_control__msg__UARTData__init(message_memory);
}

void robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_fini_function(void * message_memory)
{
  robot_control__msg__UARTData__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_member_array[4] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control__msg__UARTData, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control__msg__UARTData, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control__msg__UARTData, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_control__msg__UARTData, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_members = {
  "robot_control__msg",  // message namespace
  "UARTData",  // message name
  4,  // number of fields
  sizeof(robot_control__msg__UARTData),
  false,  // has_any_key_member_
  robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_member_array,  // message members
  robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_type_support_handle = {
  0,
  &robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_members,
  get_message_typesupport_handle_function,
  &robot_control__msg__UARTData__get_type_hash,
  &robot_control__msg__UARTData__get_type_description,
  &robot_control__msg__UARTData__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_control, msg, UARTData)() {
  if (!robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_type_support_handle.typesupport_identifier) {
    robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_control__msg__UARTData__rosidl_typesupport_introspection_c__UARTData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
