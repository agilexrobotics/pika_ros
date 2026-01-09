// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from data_msgs:srv/CaptureService.idl
// generated code does not contain a copyright notice
#include "data_msgs/srv/detail/capture_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `dataset_dir`
// Member `instructions`
#include "rosidl_runtime_c/string_functions.h"

bool
data_msgs__srv__CaptureService_Request__init(data_msgs__srv__CaptureService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // start
  // end
  // episode_index
  // dataset_dir
  if (!rosidl_runtime_c__String__init(&msg->dataset_dir)) {
    data_msgs__srv__CaptureService_Request__fini(msg);
    return false;
  }
  // instructions
  if (!rosidl_runtime_c__String__init(&msg->instructions)) {
    data_msgs__srv__CaptureService_Request__fini(msg);
    return false;
  }
  return true;
}

void
data_msgs__srv__CaptureService_Request__fini(data_msgs__srv__CaptureService_Request * msg)
{
  if (!msg) {
    return;
  }
  // start
  // end
  // episode_index
  // dataset_dir
  rosidl_runtime_c__String__fini(&msg->dataset_dir);
  // instructions
  rosidl_runtime_c__String__fini(&msg->instructions);
}

bool
data_msgs__srv__CaptureService_Request__are_equal(const data_msgs__srv__CaptureService_Request * lhs, const data_msgs__srv__CaptureService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start
  if (lhs->start != rhs->start) {
    return false;
  }
  // end
  if (lhs->end != rhs->end) {
    return false;
  }
  // episode_index
  if (lhs->episode_index != rhs->episode_index) {
    return false;
  }
  // dataset_dir
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dataset_dir), &(rhs->dataset_dir)))
  {
    return false;
  }
  // instructions
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->instructions), &(rhs->instructions)))
  {
    return false;
  }
  return true;
}

bool
data_msgs__srv__CaptureService_Request__copy(
  const data_msgs__srv__CaptureService_Request * input,
  data_msgs__srv__CaptureService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // start
  output->start = input->start;
  // end
  output->end = input->end;
  // episode_index
  output->episode_index = input->episode_index;
  // dataset_dir
  if (!rosidl_runtime_c__String__copy(
      &(input->dataset_dir), &(output->dataset_dir)))
  {
    return false;
  }
  // instructions
  if (!rosidl_runtime_c__String__copy(
      &(input->instructions), &(output->instructions)))
  {
    return false;
  }
  return true;
}

data_msgs__srv__CaptureService_Request *
data_msgs__srv__CaptureService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__srv__CaptureService_Request * msg = (data_msgs__srv__CaptureService_Request *)allocator.allocate(sizeof(data_msgs__srv__CaptureService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(data_msgs__srv__CaptureService_Request));
  bool success = data_msgs__srv__CaptureService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
data_msgs__srv__CaptureService_Request__destroy(data_msgs__srv__CaptureService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    data_msgs__srv__CaptureService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
data_msgs__srv__CaptureService_Request__Sequence__init(data_msgs__srv__CaptureService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__srv__CaptureService_Request * data = NULL;

  if (size) {
    data = (data_msgs__srv__CaptureService_Request *)allocator.zero_allocate(size, sizeof(data_msgs__srv__CaptureService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = data_msgs__srv__CaptureService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        data_msgs__srv__CaptureService_Request__fini(&data[i - 1]);
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
data_msgs__srv__CaptureService_Request__Sequence__fini(data_msgs__srv__CaptureService_Request__Sequence * array)
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
      data_msgs__srv__CaptureService_Request__fini(&array->data[i]);
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

data_msgs__srv__CaptureService_Request__Sequence *
data_msgs__srv__CaptureService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__srv__CaptureService_Request__Sequence * array = (data_msgs__srv__CaptureService_Request__Sequence *)allocator.allocate(sizeof(data_msgs__srv__CaptureService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = data_msgs__srv__CaptureService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
data_msgs__srv__CaptureService_Request__Sequence__destroy(data_msgs__srv__CaptureService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    data_msgs__srv__CaptureService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
data_msgs__srv__CaptureService_Request__Sequence__are_equal(const data_msgs__srv__CaptureService_Request__Sequence * lhs, const data_msgs__srv__CaptureService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!data_msgs__srv__CaptureService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
data_msgs__srv__CaptureService_Request__Sequence__copy(
  const data_msgs__srv__CaptureService_Request__Sequence * input,
  data_msgs__srv__CaptureService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(data_msgs__srv__CaptureService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data_msgs__srv__CaptureService_Request * data =
      (data_msgs__srv__CaptureService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!data_msgs__srv__CaptureService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          data_msgs__srv__CaptureService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!data_msgs__srv__CaptureService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
data_msgs__srv__CaptureService_Response__init(data_msgs__srv__CaptureService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    data_msgs__srv__CaptureService_Response__fini(msg);
    return false;
  }
  return true;
}

void
data_msgs__srv__CaptureService_Response__fini(data_msgs__srv__CaptureService_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
data_msgs__srv__CaptureService_Response__are_equal(const data_msgs__srv__CaptureService_Response * lhs, const data_msgs__srv__CaptureService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
data_msgs__srv__CaptureService_Response__copy(
  const data_msgs__srv__CaptureService_Response * input,
  data_msgs__srv__CaptureService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

data_msgs__srv__CaptureService_Response *
data_msgs__srv__CaptureService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__srv__CaptureService_Response * msg = (data_msgs__srv__CaptureService_Response *)allocator.allocate(sizeof(data_msgs__srv__CaptureService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(data_msgs__srv__CaptureService_Response));
  bool success = data_msgs__srv__CaptureService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
data_msgs__srv__CaptureService_Response__destroy(data_msgs__srv__CaptureService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    data_msgs__srv__CaptureService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
data_msgs__srv__CaptureService_Response__Sequence__init(data_msgs__srv__CaptureService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__srv__CaptureService_Response * data = NULL;

  if (size) {
    data = (data_msgs__srv__CaptureService_Response *)allocator.zero_allocate(size, sizeof(data_msgs__srv__CaptureService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = data_msgs__srv__CaptureService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        data_msgs__srv__CaptureService_Response__fini(&data[i - 1]);
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
data_msgs__srv__CaptureService_Response__Sequence__fini(data_msgs__srv__CaptureService_Response__Sequence * array)
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
      data_msgs__srv__CaptureService_Response__fini(&array->data[i]);
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

data_msgs__srv__CaptureService_Response__Sequence *
data_msgs__srv__CaptureService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  data_msgs__srv__CaptureService_Response__Sequence * array = (data_msgs__srv__CaptureService_Response__Sequence *)allocator.allocate(sizeof(data_msgs__srv__CaptureService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = data_msgs__srv__CaptureService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
data_msgs__srv__CaptureService_Response__Sequence__destroy(data_msgs__srv__CaptureService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    data_msgs__srv__CaptureService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
data_msgs__srv__CaptureService_Response__Sequence__are_equal(const data_msgs__srv__CaptureService_Response__Sequence * lhs, const data_msgs__srv__CaptureService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!data_msgs__srv__CaptureService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
data_msgs__srv__CaptureService_Response__Sequence__copy(
  const data_msgs__srv__CaptureService_Response__Sequence * input,
  data_msgs__srv__CaptureService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(data_msgs__srv__CaptureService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    data_msgs__srv__CaptureService_Response * data =
      (data_msgs__srv__CaptureService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!data_msgs__srv__CaptureService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          data_msgs__srv__CaptureService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!data_msgs__srv__CaptureService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
