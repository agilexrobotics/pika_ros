// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from data_msgs:msg/CaptureStatus.idl
// generated code does not contain a copyright notice
#include "data_msgs/msg/detail/capture_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"
// Member `count_in_seconds`
// Member `frequencies`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
data_msgs__msg__CaptureStatus__init(data_msgs__msg__CaptureStatus * msg)
{
  if (!msg) {
    return false;
  }
  // topics
  if (!rosidl_runtime_c__String__Sequence__init(&msg->topics, 0)) {
    data_msgs__msg__CaptureStatus__fini(msg);
    return false;
  }
  // count_in_seconds
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->count_in_seconds, 0)) {
    data_msgs__msg__CaptureStatus__fini(msg);
    return false;
  }
  // frequencies
  if (!rosidl_runtime_c__float__Sequence__init(&msg->frequencies, 0)) {
    data_msgs__msg__CaptureStatus__fini(msg);
    return false;
  }
  // fail
  // quit
  return true;
}

void
data_msgs__msg__CaptureStatus__fini(data_msgs__msg__CaptureStatus * msg)
{
  if (!msg) {
    return;
  }
  // topics
  rosidl_runtime_c__String__Sequence__fini(&msg->topics);
  // count_in_seconds
  rosidl_runtime_c__int32__Sequence__fini(&msg->count_in_seconds);
  // frequencies
  rosidl_runtime_c__float__Sequence__fini(&msg->frequencies);
  // fail
  // quit
}

bool
data_msgs__msg__CaptureStatus__are_equal(const data_msgs__msg__CaptureStatus * lhs, const data_msgs__msg__CaptureStatus * rhs)
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
  // count_in_seconds
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->count_in_seconds), &(rhs->count_in_seconds)))
  {
    return false;
  }
  // frequencies
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->frequencies), &(rhs->frequencies)))
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
data_msgs__msg__CaptureStatus__copy(
  const data_msgs__msg__CaptureStatus * input,
  data_msgs__msg__CaptureStatus * output)
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
  // count_in_seconds
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->count_in_seconds), &(output->count_in_seconds)))
  {
    return false;
  }
  // frequencies
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->frequencies), &(output->frequencies)))
  {
    return false;
  }
  // fail
  output->fail = input->fail;
  // quit
  output->quit = input->quit;
  return true;
}

data_msgs__msg__CaptureStatus *
data_msgs__msg__CaptureStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__CaptureStatus * msg = (data_msgs__msg__CaptureStatus *)allocator.allocate(sizeof(data_msgs__msg__CaptureStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(data_msgs__msg__CaptureStatus));
  bool success = data_msgs__msg__CaptureStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
data_msgs__msg__CaptureStatus__destroy(data_msgs__msg__CaptureStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    data_msgs__msg__CaptureStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
data_msgs__msg__CaptureStatus__Sequence__init(data_msgs__msg__CaptureStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__CaptureStatus * data = NULL;

  if (size) {
    data = (data_msgs__msg__CaptureStatus *)allocator.zero_allocate(size, sizeof(data_msgs__msg__CaptureStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = data_msgs__msg__CaptureStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        data_msgs__msg__CaptureStatus__fini(&data[i - 1]);
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
data_msgs__msg__CaptureStatus__Sequence__fini(data_msgs__msg__CaptureStatus__Sequence * array)
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
      data_msgs__msg__CaptureStatus__fini(&array->data[i]);
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

data_msgs__msg__CaptureStatus__Sequence *
data_msgs__msg__CaptureStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__msg__CaptureStatus__Sequence * array = (data_msgs__msg__CaptureStatus__Sequence *)allocator.allocate(sizeof(data_msgs__msg__CaptureStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = data_msgs__msg__CaptureStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
data_msgs__msg__CaptureStatus__Sequence__destroy(data_msgs__msg__CaptureStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    data_msgs__msg__CaptureStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
data_msgs__msg__CaptureStatus__Sequence__are_equal(const data_msgs__msg__CaptureStatus__Sequence * lhs, const data_msgs__msg__CaptureStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!data_msgs__msg__CaptureStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
data_msgs__msg__CaptureStatus__Sequence__copy(
  const data_msgs__msg__CaptureStatus__Sequence * input,
  data_msgs__msg__CaptureStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(data_msgs__msg__CaptureStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data_msgs__msg__CaptureStatus * data =
      (data_msgs__msg__CaptureStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!data_msgs__msg__CaptureStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          data_msgs__msg__CaptureStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!data_msgs__msg__CaptureStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
