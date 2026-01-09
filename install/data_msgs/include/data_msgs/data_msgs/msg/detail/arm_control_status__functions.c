// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from data_msgs:msg/ArmControlStatus.idl
// generated code does not contain a copyright notice
#include "data_msgs/msg/detail/arm_control_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
data_msgs__msg__ArmControlStatus__init(data_msgs__msg__ArmControlStatus * msg)
{
  if (!msg) {
    return false;
  }
  // over_limit
  return true;
}

void
data_msgs__msg__ArmControlStatus__fini(data_msgs__msg__ArmControlStatus * msg)
{
  if (!msg) {
    return;
  }
  // over_limit
}

bool
data_msgs__msg__ArmControlStatus__are_equal(const data_msgs__msg__ArmControlStatus * lhs, const data_msgs__msg__ArmControlStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // over_limit
  if (lhs->over_limit != rhs->over_limit) {
    return false;
  }
  return true;
}

bool
data_msgs__msg__ArmControlStatus__copy(
  const data_msgs__msg__ArmControlStatus * input,
  data_msgs__msg__ArmControlStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // over_limit
  output->over_limit = input->over_limit;
  return true;
}

data_msgs__msg__ArmControlStatus *
data_msgs__msg__ArmControlStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__ArmControlStatus * msg = (data_msgs__msg__ArmControlStatus *)allocator.allocate(sizeof(data_msgs__msg__ArmControlStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(data_msgs__msg__ArmControlStatus));
  bool success = data_msgs__msg__ArmControlStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
data_msgs__msg__ArmControlStatus__destroy(data_msgs__msg__ArmControlStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    data_msgs__msg__ArmControlStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
data_msgs__msg__ArmControlStatus__Sequence__init(data_msgs__msg__ArmControlStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__ArmControlStatus * data = NULL;

  if (size) {
    data = (data_msgs__msg__ArmControlStatus *)allocator.zero_allocate(size, sizeof(data_msgs__msg__ArmControlStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = data_msgs__msg__ArmControlStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        data_msgs__msg__ArmControlStatus__fini(&data[i - 1]);
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
data_msgs__msg__ArmControlStatus__Sequence__fini(data_msgs__msg__ArmControlStatus__Sequence * array)
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
      data_msgs__msg__ArmControlStatus__fini(&array->data[i]);
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

data_msgs__msg__ArmControlStatus__Sequence *
data_msgs__msg__ArmControlStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__ArmControlStatus__Sequence * array = (data_msgs__msg__ArmControlStatus__Sequence *)allocator.allocate(sizeof(data_msgs__msg__ArmControlStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = data_msgs__msg__ArmControlStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
data_msgs__msg__ArmControlStatus__Sequence__destroy(data_msgs__msg__ArmControlStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    data_msgs__msg__ArmControlStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
data_msgs__msg__ArmControlStatus__Sequence__are_equal(const data_msgs__msg__ArmControlStatus__Sequence * lhs, const data_msgs__msg__ArmControlStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!data_msgs__msg__ArmControlStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
data_msgs__msg__ArmControlStatus__Sequence__copy(
  const data_msgs__msg__ArmControlStatus__Sequence * input,
  data_msgs__msg__ArmControlStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(data_msgs__msg__ArmControlStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data_msgs__msg__ArmControlStatus * data =
      (data_msgs__msg__ArmControlStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!data_msgs__msg__ArmControlStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          data_msgs__msg__ArmControlStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!data_msgs__msg__ArmControlStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
