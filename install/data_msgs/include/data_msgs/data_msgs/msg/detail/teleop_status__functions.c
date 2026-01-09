// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from data_msgs:msg/TeleopStatus.idl
// generated code does not contain a copyright notice
#include "data_msgs/msg/detail/teleop_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"

bool
data_msgs__msg__TeleopStatus__init(data_msgs__msg__TeleopStatus * msg)
{
  if (!msg) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__init(&msg->topics, 0)) {
    data_msgs__msg__TeleopStatus__fini(msg);
    return false;
  }
  // fail
  // quit
  return true;
}

void
data_msgs__msg__TeleopStatus__fini(data_msgs__msg__TeleopStatus * msg)
{
  if (!msg) {
    return;
  }
  // topics
  rosidl_runtime_c__String__Sequence__fini(&msg->topics);
  // fail
  // quit
}

bool
data_msgs__msg__TeleopStatus__are_equal(const data_msgs__msg__TeleopStatus * lhs, const data_msgs__msg__TeleopStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->topics), &(rhs->topics)))
  {
    return false;
  }
  // fail
  if (lhs->fail != rhs->fail) {
    return false;
  }
  // quit
  if (lhs->quit != rhs->quit) {
    return false;
  }
  return true;
}

bool
data_msgs__msg__TeleopStatus__copy(
  const data_msgs__msg__TeleopStatus * input,
  data_msgs__msg__TeleopStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->topics), &(output->topics)))
  {
    return false;
  }
  // fail
  output->fail = input->fail;
  // quit
  output->quit = input->quit;
  return true;
}

data_msgs__msg__TeleopStatus *
data_msgs__msg__TeleopStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__TeleopStatus * msg = (data_msgs__msg__TeleopStatus *)allocator.allocate(sizeof(data_msgs__msg__TeleopStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(data_msgs__msg__TeleopStatus));
  bool success = data_msgs__msg__TeleopStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
data_msgs__msg__TeleopStatus__destroy(data_msgs__msg__TeleopStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    data_msgs__msg__TeleopStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
data_msgs__msg__TeleopStatus__Sequence__init(data_msgs__msg__TeleopStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__TeleopStatus * data = NULL;

  if (size) {
    data = (data_msgs__msg__TeleopStatus *)allocator.zero_allocate(size, sizeof(data_msgs__msg__TeleopStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = data_msgs__msg__TeleopStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        data_msgs__msg__TeleopStatus__fini(&data[i - 1]);
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
data_msgs__msg__TeleopStatus__Sequence__fini(data_msgs__msg__TeleopStatus__Sequence * array)
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
      data_msgs__msg__TeleopStatus__fini(&array->data[i]);
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

data_msgs__msg__TeleopStatus__Sequence *
data_msgs__msg__TeleopStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__TeleopStatus__Sequence * array = (data_msgs__msg__TeleopStatus__Sequence *)allocator.allocate(sizeof(data_msgs__msg__TeleopStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = data_msgs__msg__TeleopStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
data_msgs__msg__TeleopStatus__Sequence__destroy(data_msgs__msg__TeleopStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    data_msgs__msg__TeleopStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
data_msgs__msg__TeleopStatus__Sequence__are_equal(const data_msgs__msg__TeleopStatus__Sequence * lhs, const data_msgs__msg__TeleopStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!data_msgs__msg__TeleopStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
data_msgs__msg__TeleopStatus__Sequence__copy(
  const data_msgs__msg__TeleopStatus__Sequence * input,
  data_msgs__msg__TeleopStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(data_msgs__msg__TeleopStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data_msgs__msg__TeleopStatus * data =
      (data_msgs__msg__TeleopStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!data_msgs__msg__TeleopStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          data_msgs__msg__TeleopStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!data_msgs__msg__TeleopStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
