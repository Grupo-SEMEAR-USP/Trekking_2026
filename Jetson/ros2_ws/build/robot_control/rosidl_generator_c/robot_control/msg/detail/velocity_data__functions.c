// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_control:msg/VelocityData.idl
// generated code does not contain a copyright notice
#include "robot_control/msg/detail/velocity_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_control__msg__VelocityData__init(robot_control__msg__VelocityData * msg)
{
  if (!msg) {
    return false;
  }
  // angular_speed_left
  // angular_speed_right
  // servo_angle
  return true;
}

void
robot_control__msg__VelocityData__fini(robot_control__msg__VelocityData * msg)
{
  if (!msg) {
    return;
  }
  // angular_speed_left
  // angular_speed_right
  // servo_angle
}

bool
robot_control__msg__VelocityData__are_equal(const robot_control__msg__VelocityData * lhs, const robot_control__msg__VelocityData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // angular_speed_left
  if (lhs->angular_speed_left != rhs->angular_speed_left) {
    return false;
  }
  // angular_speed_right
  if (lhs->angular_speed_right != rhs->angular_speed_right) {
    return false;
  }
  // servo_angle
  if (lhs->servo_angle != rhs->servo_angle) {
    return false;
  }
  return true;
}

bool
robot_control__msg__VelocityData__copy(
  const robot_control__msg__VelocityData * input,
  robot_control__msg__VelocityData * output)
{
  if (!input || !output) {
    return false;
  }
  // angular_speed_left
  output->angular_speed_left = input->angular_speed_left;
  // angular_speed_right
  output->angular_speed_right = input->angular_speed_right;
  // servo_angle
  output->servo_angle = input->servo_angle;
  return true;
}

robot_control__msg__VelocityData *
robot_control__msg__VelocityData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control__msg__VelocityData * msg = (robot_control__msg__VelocityData *)allocator.allocate(sizeof(robot_control__msg__VelocityData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control__msg__VelocityData));
  bool success = robot_control__msg__VelocityData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control__msg__VelocityData__destroy(robot_control__msg__VelocityData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control__msg__VelocityData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control__msg__VelocityData__Sequence__init(robot_control__msg__VelocityData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control__msg__VelocityData * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(robot_control__msg__VelocityData)) {
      return false;
    }
    data = (robot_control__msg__VelocityData *)allocator.zero_allocate(size, sizeof(robot_control__msg__VelocityData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control__msg__VelocityData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control__msg__VelocityData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_control__msg__VelocityData__Sequence__fini(robot_control__msg__VelocityData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_control__msg__VelocityData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_control__msg__VelocityData__Sequence *
robot_control__msg__VelocityData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control__msg__VelocityData__Sequence * array = (robot_control__msg__VelocityData__Sequence *)allocator.allocate(sizeof(robot_control__msg__VelocityData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control__msg__VelocityData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control__msg__VelocityData__Sequence__destroy(robot_control__msg__VelocityData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control__msg__VelocityData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control__msg__VelocityData__Sequence__are_equal(const robot_control__msg__VelocityData__Sequence * lhs, const robot_control__msg__VelocityData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control__msg__VelocityData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control__msg__VelocityData__Sequence__copy(
  const robot_control__msg__VelocityData__Sequence * input,
  robot_control__msg__VelocityData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(robot_control__msg__VelocityData)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(robot_control__msg__VelocityData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control__msg__VelocityData * data =
      (robot_control__msg__VelocityData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control__msg__VelocityData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control__msg__VelocityData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control__msg__VelocityData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
