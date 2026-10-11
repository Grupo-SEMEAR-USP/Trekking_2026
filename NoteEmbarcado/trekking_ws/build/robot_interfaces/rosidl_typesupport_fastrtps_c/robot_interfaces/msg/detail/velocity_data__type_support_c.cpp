// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_interfaces:msg/VelocityData.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/velocity_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_interfaces/msg/detail/velocity_data__struct.h"
#include "robot_interfaces/msg/detail/velocity_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _VelocityData__ros_msg_type = robot_interfaces__msg__VelocityData;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
bool cdr_serialize_robot_interfaces__msg__VelocityData(
  const robot_interfaces__msg__VelocityData * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: angular_speed_left
  {
    cdr << ros_message->angular_speed_left;
  }

  // Field name: angular_speed_right
  {
    cdr << ros_message->angular_speed_right;
  }

  // Field name: servo_angle
  {
    cdr << ros_message->servo_angle;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
bool cdr_deserialize_robot_interfaces__msg__VelocityData(
  eprosima::fastcdr::Cdr & cdr,
  robot_interfaces__msg__VelocityData * ros_message)
{
  // Field name: angular_speed_left
  {
    cdr >> ros_message->angular_speed_left;
  }

  // Field name: angular_speed_right
  {
    cdr >> ros_message->angular_speed_right;
  }

  // Field name: servo_angle
  {
    cdr >> ros_message->servo_angle;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t get_serialized_size_robot_interfaces__msg__VelocityData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VelocityData__ros_msg_type * ros_message = static_cast<const _VelocityData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: angular_speed_left
  {
    size_t item_size = sizeof(ros_message->angular_speed_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: angular_speed_right
  {
    size_t item_size = sizeof(ros_message->angular_speed_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: servo_angle
  {
    size_t item_size = sizeof(ros_message->servo_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t max_serialized_size_robot_interfaces__msg__VelocityData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: angular_speed_left
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: angular_speed_right
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: servo_angle
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_interfaces__msg__VelocityData;
    is_plain =
      (
      offsetof(DataType, servo_angle) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
bool cdr_serialize_key_robot_interfaces__msg__VelocityData(
  const robot_interfaces__msg__VelocityData * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: angular_speed_left
  {
    cdr << ros_message->angular_speed_left;
  }

  // Field name: angular_speed_right
  {
    cdr << ros_message->angular_speed_right;
  }

  // Field name: servo_angle
  {
    cdr << ros_message->servo_angle;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t get_serialized_size_key_robot_interfaces__msg__VelocityData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VelocityData__ros_msg_type * ros_message = static_cast<const _VelocityData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: angular_speed_left
  {
    size_t item_size = sizeof(ros_message->angular_speed_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: angular_speed_right
  {
    size_t item_size = sizeof(ros_message->angular_speed_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: servo_angle
  {
    size_t item_size = sizeof(ros_message->servo_angle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t max_serialized_size_key_robot_interfaces__msg__VelocityData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: angular_speed_left
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: angular_speed_right
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: servo_angle
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_interfaces__msg__VelocityData;
    is_plain =
      (
      offsetof(DataType, servo_angle) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _VelocityData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const robot_interfaces__msg__VelocityData * ros_message = static_cast<const robot_interfaces__msg__VelocityData *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_robot_interfaces__msg__VelocityData(ros_message, cdr);
}

static bool _VelocityData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  robot_interfaces__msg__VelocityData * ros_message = static_cast<robot_interfaces__msg__VelocityData *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_robot_interfaces__msg__VelocityData(cdr, ros_message);
}

static uint32_t _VelocityData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_interfaces__msg__VelocityData(
      untyped_ros_message, 0));
}

static size_t _VelocityData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robot_interfaces__msg__VelocityData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_VelocityData = {
  "robot_interfaces::msg",
  "VelocityData",
  _VelocityData__cdr_serialize,
  _VelocityData__cdr_deserialize,
  _VelocityData__get_serialized_size,
  _VelocityData__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _VelocityData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VelocityData,
  get_message_typesupport_handle_function,
  &robot_interfaces__msg__VelocityData__get_type_hash,
  &robot_interfaces__msg__VelocityData__get_type_description,
  &robot_interfaces__msg__VelocityData__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, msg, VelocityData)() {
  return &_VelocityData__type_support;
}

#if defined(__cplusplus)
}
#endif
