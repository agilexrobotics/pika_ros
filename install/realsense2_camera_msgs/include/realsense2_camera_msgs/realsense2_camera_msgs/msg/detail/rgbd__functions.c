// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from realsense2_camera_msgs:msg/RGBD.idl
// generated code does not contain a copyright notice
#include "realsense2_camera_msgs/msg/detail/rgbd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `rgb_camera_info`
// Member `depth_camera_info`
#include "sensor_msgs/msg/detail/camera_info__functions.h"
// Member `rgb`
// Member `depth`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
realsense2_camera_msgs__msg__RGBD__init(realsense2_camera_msgs__msg__RGBD * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    realsense2_camera_msgs__msg__RGBD__fini(msg);
    return false;
  }
  // rgb_camera_info
  if (!sensor_msgs__msg__CameraInfo__init(&msg->rgb_camera_info)) {
    realsense2_camera_msgs__msg__RGBD__fini(msg);
    return false;
  }
  // depth_camera_info
  if (!sensor_msgs__msg__CameraInfo__init(&msg->depth_camera_info)) {
    realsense2_camera_msgs__msg__RGBD__fini(msg);
    return false;
  }
  // rgb
  if (!sensor_msgs__msg__Image__init(&msg->rgb)) {
    realsense2_camera_msgs__msg__RGBD__fini(msg);
    return false;
  }
  // depth
  if (!sensor_msgs__msg__Image__init(&msg->depth)) {
    realsense2_camera_msgs__msg__RGBD__fini(msg);
    return false;
  }
  return true;
}

void
realsense2_camera_msgs__msg__RGBD__fini(realsense2_camera_msgs__msg__RGBD * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rgb_camera_info
  sensor_msgs__msg__CameraInfo__fini(&msg->rgb_camera_info);
  // depth_camera_info
  sensor_msgs__msg__CameraInfo__fini(&msg->depth_camera_info);
  // rgb
  sensor_msgs__msg__Image__fini(&msg->rgb);
  // depth
  sensor_msgs__msg__Image__fini(&msg->depth);
}

bool
realsense2_camera_msgs__msg__RGBD__are_equal(const realsense2_camera_msgs__msg__RGBD * lhs, const realsense2_camera_msgs__msg__RGBD * rhs)
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
  // rgb_camera_info
  if (!sensor_msgs__msg__CameraInfo__are_equal(
      &(lhs->rgb_camera_info), &(rhs->rgb_camera_info)))
  {
    return false;
  }
  // depth_camera_info
  if (!sensor_msgs__msg__CameraInfo__are_equal(
      &(lhs->depth_camera_info), &(rhs->depth_camera_info)))
  {
    return false;
  }
  // rgb
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->rgb), &(rhs->rgb)))
  {
    return false;
  }
  // depth
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->depth), &(rhs->depth)))
  {
    return false;
  }
  return true;
}

bool
realsense2_camera_msgs__msg__RGBD__copy(
  const realsense2_camera_msgs__msg__RGBD * input,
  realsense2_camera_msgs__msg__RGBD * output)
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
  // rgb_camera_info
  if (!sensor_msgs__msg__CameraInfo__copy(
      &(input->rgb_camera_info), &(output->rgb_camera_info)))
  {
    return false;
  }
  // depth_camera_info
  if (!sensor_msgs__msg__CameraInfo__copy(
      &(input->depth_camera_info), &(output->depth_camera_info)))
  {
    return false;
  }
  // rgb
  if (!sensor_msgs__msg__Image__copy(
      &(input->rgb), &(output->rgb)))
  {
    return false;
  }
  // depth
  if (!sensor_msgs__msg__Image__copy(
      &(input->depth), &(output->depth)))
  {
    return false;
  }
  return true;
}

realsense2_camera_msgs__msg__RGBD *
realsense2_camera_msgs__msg__RGBD__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__msg__RGBD * msg = (realsense2_camera_msgs__msg__RGBD *)allocator.allocate(sizeof(realsense2_camera_msgs__msg__RGBD), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(realsense2_camera_msgs__msg__RGBD));
  bool success = realsense2_camera_msgs__msg__RGBD__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
realsense2_camera_msgs__msg__RGBD__destroy(realsense2_camera_msgs__msg__RGBD * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    realsense2_camera_msgs__msg__RGBD__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
realsense2_camera_msgs__msg__RGBD__Sequence__init(realsense2_camera_msgs__msg__RGBD__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__msg__RGBD * data = NULL;

  if (size) {
    data = (realsense2_camera_msgs__msg__RGBD *)allocator.zero_allocate(size, sizeof(realsense2_camera_msgs__msg__RGBD), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = realsense2_camera_msgs__msg__RGBD__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        realsense2_camera_msgs__msg__RGBD__fini(&data[i - 1]);
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
realsense2_camera_msgs__msg__RGBD__Sequence__fini(realsense2_camera_msgs__msg__RGBD__Sequence * array)
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
      realsense2_camera_msgs__msg__RGBD__fini(&array->data[i]);
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

realsense2_camera_msgs__msg__RGBD__Sequence *
realsense2_camera_msgs__msg__RGBD__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__msg__RGBD__Sequence * array = (realsense2_camera_msgs__msg__RGBD__Sequence *)allocator.allocate(sizeof(realsense2_camera_msgs__msg__RGBD__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = realsense2_camera_msgs__msg__RGBD__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
realsense2_camera_msgs__msg__RGBD__Sequence__destroy(realsense2_camera_msgs__msg__RGBD__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    realsense2_camera_msgs__msg__RGBD__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
realsense2_camera_msgs__msg__RGBD__Sequence__are_equal(const realsense2_camera_msgs__msg__RGBD__Sequence * lhs, const realsense2_camera_msgs__msg__RGBD__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!realsense2_camera_msgs__msg__RGBD__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
realsense2_camera_msgs__msg__RGBD__Sequence__copy(
  const realsense2_camera_msgs__msg__RGBD__Sequence * input,
  realsense2_camera_msgs__msg__RGBD__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(realsense2_camera_msgs__msg__RGBD);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    realsense2_camera_msgs__msg__RGBD * data =
      (realsense2_camera_msgs__msg__RGBD *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!realsense2_camera_msgs__msg__RGBD__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          realsense2_camera_msgs__msg__RGBD__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!realsense2_camera_msgs__msg__RGBD__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
