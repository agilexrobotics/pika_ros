// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from data_msgs:msg/Gripper.idl
// generated code does not contain a copyright notice
#include "data_msgs/msg/detail/gripper__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
data_msgs__msg__Gripper__init(data_msgs__msg__Gripper * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    data_msgs__msg__Gripper__fini(msg);
    return false;
  }
  // angle
  // distance
  // effort
  // velocity
  // enable
  // set_zero
  // error
  // voltage
  // driver_temp
  // motor_temp
  // bus_current
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    data_msgs__msg__Gripper__fini(msg);
    return false;
  }
  return true;
}

void
data_msgs__msg__Gripper__fini(data_msgs__msg__Gripper * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // angle
  // distance
  // effort
  // velocity
  // enable
  // set_zero
  // error
  // voltage
  // driver_temp
  // motor_temp
  // bus_current
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
data_msgs__msg__Gripper__are_equal(const data_msgs__msg__Gripper * lhs, const data_msgs__msg__Gripper * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // effort
  if (lhs->effort != rhs->effort) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // enable
  if (lhs->enable != rhs->enable) {
    return false;
  }
  // set_zero
  if (lhs->set_zero != rhs->set_zero) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // driver_temp
  if (lhs->driver_temp != rhs->driver_temp) {
    return false;
  }
  // motor_temp
  if (lhs->motor_temp != rhs->motor_temp) {
    return false;
  }
  // bus_current
  if (lhs->bus_current != rhs->bus_current) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
data_msgs__msg__Gripper__copy(
  const data_msgs__msg__Gripper * input,
  data_msgs__msg__Gripper * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // angle
  output->angle = input->angle;
  // distance
  output->distance = input->distance;
  // effort
  output->effort = input->effort;
  // velocity
  output->velocity = input->velocity;
  // enable
  output->enable = input->enable;
  // set_zero
  output->set_zero = input->set_zero;
  // error
  output->error = input->error;
  // voltage
  output->voltage = input->voltage;
  // driver_temp
  output->driver_temp = input->driver_temp;
  // motor_temp
  output->motor_temp = input->motor_temp;
  // bus_current
  output->bus_current = input->bus_current;
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

data_msgs__msg__Gripper *
data_msgs__msg__Gripper__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__Gripper * msg = (data_msgs__msg__Gripper *)allocator.allocate(sizeof(data_msgs__msg__Gripper), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(data_msgs__msg__Gripper));
  bool success = data_msgs__msg__Gripper__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
data_msgs__msg__Gripper__destroy(data_msgs__msg__Gripper * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    data_msgs__msg__Gripper__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
data_msgs__msg__Gripper__Sequence__init(data_msgs__msg__Gripper__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__Gripper * data = NULL;

  if (size) {
    data = (data_msgs__msg__Gripper *)allocator.zero_allocate(size, sizeof(data_msgs__msg__Gripper), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = data_msgs__msg__Gripper__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        data_msgs__msg__Gripper__fini(&data[i - 1]);
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
data_msgs__msg__Gripper__Sequence__fini(data_msgs__msg__Gripper__Sequence * array)
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
      data_msgs__msg__Gripper__fini(&array->data[i]);
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

data_msgs__msg__Gripper__Sequence *
data_msgs__msg__Gripper__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__Gripper__Sequence * array = (data_msgs__msg__Gripper__Sequence *)allocator.allocate(sizeof(data_msgs__msg__Gripper__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = data_msgs__msg__Gripper__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
data_msgs__msg__Gripper__Sequence__destroy(data_msgs__msg__Gripper__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    data_msgs__msg__Gripper__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
data_msgs__msg__Gripper__Sequence__are_equal(const data_msgs__msg__Gripper__Sequence * lhs, const data_msgs__msg__Gripper__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!data_msgs__msg__Gripper__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
data_msgs__msg__Gripper__Sequence__copy(
  const data_msgs__msg__Gripper__Sequence * input,
  data_msgs__msg__Gripper__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(data_msgs__msg__Gripper);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data_msgs__msg__Gripper * data =
      (data_msgs__msg__Gripper *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!data_msgs__msg__Gripper__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          data_msgs__msg__Gripper__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!data_msgs__msg__Gripper__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
