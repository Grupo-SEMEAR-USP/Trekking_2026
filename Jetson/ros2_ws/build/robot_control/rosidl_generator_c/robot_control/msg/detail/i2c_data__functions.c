// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_control:msg/I2cData.idl
// generated code does not contain a copyright notice
#include "robot_control/msg/detail/i2c_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_control__msg__I2cData__init(robot_control__msg__I2cData * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // timestamp
  return true;
}

void
robot_control__msg__I2cData__fini(robot_control__msg__I2cData * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // timestamp
}

bool
robot_control__msg__I2cData__are_equal(const robot_control__msg__I2cData * lhs, const robot_control__msg__I2cData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  return true;
}

bool
robot_control__msg__I2cData__copy(
  const robot_control__msg__I2cData * input,
  robot_control__msg__I2cData * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // timestamp
  output->timestamp = input->timestamp;
  return true;
}

robot_control__msg__I2cData *
robot_control__msg__I2cData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control__msg__I2cData * msg = (robot_control__msg__I2cData *)allocator.allocate(sizeof(robot_control__msg__I2cData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_control__msg__I2cData));
  bool success = robot_control__msg__I2cData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_control__msg__I2cData__destroy(robot_control__msg__I2cData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_control__msg__I2cData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_control__msg__I2cData__Sequence__init(robot_control__msg__I2cData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control__msg__I2cData * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(robot_control__msg__I2cData)) {
      return false;
    }
    data = (robot_control__msg__I2cData *)allocator.zero_allocate(size, sizeof(robot_control__msg__I2cData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_control__msg__I2cData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_control__msg__I2cData__fini(&data[i - 1]);
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
robot_control__msg__I2cData__Sequence__fini(robot_control__msg__I2cData__Sequence * array)
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
      robot_control__msg__I2cData__fini(&array->data[i]);
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

robot_control__msg__I2cData__Sequence *
robot_control__msg__I2cData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_control__msg__I2cData__Sequence * array = (robot_control__msg__I2cData__Sequence *)allocator.allocate(sizeof(robot_control__msg__I2cData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_control__msg__I2cData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_control__msg__I2cData__Sequence__destroy(robot_control__msg__I2cData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_control__msg__I2cData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_control__msg__I2cData__Sequence__are_equal(const robot_control__msg__I2cData__Sequence * lhs, const robot_control__msg__I2cData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_control__msg__I2cData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_control__msg__I2cData__Sequence__copy(
  const robot_control__msg__I2cData__Sequence * input,
  robot_control__msg__I2cData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(robot_control__msg__I2cData)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(robot_control__msg__I2cData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_control__msg__I2cData * data =
      (robot_control__msg__I2cData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_control__msg__I2cData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_control__msg__I2cData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_control__msg__I2cData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
