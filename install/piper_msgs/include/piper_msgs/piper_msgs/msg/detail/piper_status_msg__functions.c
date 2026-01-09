// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
// generated code does not contain a copyright notice
#include "piper_msgs/msg/detail/piper_status_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
piper_msgs__msg__PiperStatusMsg__init(piper_msgs__msg__PiperStatusMsg * msg)
{
  if (!msg) {
    return false;
  }
  // ctrl_mode
  // arm_status
  // mode_feedback
  // teach_status
  // motion_status
  // trajectory_num
  // err_code
  // joint_1_angle_limit
  // joint_2_angle_limit
  // joint_3_angle_limit
  // joint_4_angle_limit
  // joint_5_angle_limit
  // joint_6_angle_limit
  // communication_status_joint_1
  // communication_status_joint_2
  // communication_status_joint_3
  // communication_status_joint_4
  // communication_status_joint_5
  // communication_status_joint_6
  return true;
}

void
piper_msgs__msg__PiperStatusMsg__fini(piper_msgs__msg__PiperStatusMsg * msg)
{
  if (!msg) {
    return;
  }
  // ctrl_mode
  // arm_status
  // mode_feedback
  // teach_status
  // motion_status
  // trajectory_num
  // err_code
  // joint_1_angle_limit
  // joint_2_angle_limit
  // joint_3_angle_limit
  // joint_4_angle_limit
  // joint_5_angle_limit
  // joint_6_angle_limit
  // communication_status_joint_1
  // communication_status_joint_2
  // communication_status_joint_3
  // communication_status_joint_4
  // communication_status_joint_5
  // communication_status_joint_6
}

bool
piper_msgs__msg__PiperStatusMsg__are_equal(const piper_msgs__msg__PiperStatusMsg * lhs, const piper_msgs__msg__PiperStatusMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ctrl_mode
  if (lhs->ctrl_mode != rhs->ctrl_mode) {
    return false;
  }
  // arm_status
  if (lhs->arm_status != rhs->arm_status) {
    return false;
  }
  // mode_feedback
  if (lhs->mode_feedback != rhs->mode_feedback) {
    return false;
  }
  // teach_status
  if (lhs->teach_status != rhs->teach_status) {
    return false;
  }
  // motion_status
  if (lhs->motion_status != rhs->motion_status) {
    return false;
  }
  // trajectory_num
  if (lhs->trajectory_num != rhs->trajectory_num) {
    return false;
  }
  // err_code
  if (lhs->err_code != rhs->err_code) {
    return false;
  }
  // joint_1_angle_limit
  if (lhs->joint_1_angle_limit != rhs->joint_1_angle_limit) {
    return false;
  }
  // joint_2_angle_limit
  if (lhs->joint_2_angle_limit != rhs->joint_2_angle_limit) {
    return false;
  }
  // joint_3_angle_limit
  if (lhs->joint_3_angle_limit != rhs->joint_3_angle_limit) {
    return false;
  }
  // joint_4_angle_limit
  if (lhs->joint_4_angle_limit != rhs->joint_4_angle_limit) {
    return false;
  }
  // joint_5_angle_limit
  if (lhs->joint_5_angle_limit != rhs->joint_5_angle_limit) {
    return false;
  }
  // joint_6_angle_limit
  if (lhs->joint_6_angle_limit != rhs->joint_6_angle_limit) {
    return false;
  }
  // communication_status_joint_1
  if (lhs->communication_status_joint_1 != rhs->communication_status_joint_1) {
    return false;
  }
  // communication_status_joint_2
  if (lhs->communication_status_joint_2 != rhs->communication_status_joint_2) {
    return false;
  }
  // communication_status_joint_3
  if (lhs->communication_status_joint_3 != rhs->communication_status_joint_3) {
    return false;
  }
  // communication_status_joint_4
  if (lhs->communication_status_joint_4 != rhs->communication_status_joint_4) {
    return false;
  }
  // communication_status_joint_5
  if (lhs->communication_status_joint_5 != rhs->communication_status_joint_5) {
    return false;
  }
  // communication_status_joint_6
  if (lhs->communication_status_joint_6 != rhs->communication_status_joint_6) {
    return false;
  }
  return true;
}

bool
piper_msgs__msg__PiperStatusMsg__copy(
  const piper_msgs__msg__PiperStatusMsg * input,
  piper_msgs__msg__PiperStatusMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // ctrl_mode
  output->ctrl_mode = input->ctrl_mode;
  // arm_status
  output->arm_status = input->arm_status;
  // mode_feedback
  output->mode_feedback = input->mode_feedback;
  // teach_status
  output->teach_status = input->teach_status;
  // motion_status
  output->motion_status = input->motion_status;
  // trajectory_num
  output->trajectory_num = input->trajectory_num;
  // err_code
  output->err_code = input->err_code;
  // joint_1_angle_limit
  output->joint_1_angle_limit = input->joint_1_angle_limit;
  // joint_2_angle_limit
  output->joint_2_angle_limit = input->joint_2_angle_limit;
  // joint_3_angle_limit
  output->joint_3_angle_limit = input->joint_3_angle_limit;
  // joint_4_angle_limit
  output->joint_4_angle_limit = input->joint_4_angle_limit;
  // joint_5_angle_limit
  output->joint_5_angle_limit = input->joint_5_angle_limit;
  // joint_6_angle_limit
  output->joint_6_angle_limit = input->joint_6_angle_limit;
  // communication_status_joint_1
  output->communication_status_joint_1 = input->communication_status_joint_1;
  // communication_status_joint_2
  output->communication_status_joint_2 = input->communication_status_joint_2;
  // communication_status_joint_3
  output->communication_status_joint_3 = input->communication_status_joint_3;
  // communication_status_joint_4
  output->communication_status_joint_4 = input->communication_status_joint_4;
  // communication_status_joint_5
  output->communication_status_joint_5 = input->communication_status_joint_5;
  // communication_status_joint_6
  output->communication_status_joint_6 = input->communication_status_joint_6;
  return true;
}

piper_msgs__msg__PiperStatusMsg *
piper_msgs__msg__PiperStatusMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__msg__PiperStatusMsg * msg = (piper_msgs__msg__PiperStatusMsg *)allocator.allocate(sizeof(piper_msgs__msg__PiperStatusMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(piper_msgs__msg__PiperStatusMsg));
  bool success = piper_msgs__msg__PiperStatusMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
piper_msgs__msg__PiperStatusMsg__destroy(piper_msgs__msg__PiperStatusMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    piper_msgs__msg__PiperStatusMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
piper_msgs__msg__PiperStatusMsg__Sequence__init(piper_msgs__msg__PiperStatusMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__msg__PiperStatusMsg * data = NULL;

  if (size) {
    data = (piper_msgs__msg__PiperStatusMsg *)allocator.zero_allocate(size, sizeof(piper_msgs__msg__PiperStatusMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = piper_msgs__msg__PiperStatusMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        piper_msgs__msg__PiperStatusMsg__fini(&data[i - 1]);
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
piper_msgs__msg__PiperStatusMsg__Sequence__fini(piper_msgs__msg__PiperStatusMsg__Sequence * array)
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
      piper_msgs__msg__PiperStatusMsg__fini(&array->data[i]);
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

piper_msgs__msg__PiperStatusMsg__Sequence *
piper_msgs__msg__PiperStatusMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__msg__PiperStatusMsg__Sequence * array = (piper_msgs__msg__PiperStatusMsg__Sequence *)allocator.allocate(sizeof(piper_msgs__msg__PiperStatusMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = piper_msgs__msg__PiperStatusMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
piper_msgs__msg__PiperStatusMsg__Sequence__destroy(piper_msgs__msg__PiperStatusMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    piper_msgs__msg__PiperStatusMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
piper_msgs__msg__PiperStatusMsg__Sequence__are_equal(const piper_msgs__msg__PiperStatusMsg__Sequence * lhs, const piper_msgs__msg__PiperStatusMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!piper_msgs__msg__PiperStatusMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
piper_msgs__msg__PiperStatusMsg__Sequence__copy(
  const piper_msgs__msg__PiperStatusMsg__Sequence * input,
  piper_msgs__msg__PiperStatusMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(piper_msgs__msg__PiperStatusMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    piper_msgs__msg__PiperStatusMsg * data =
      (piper_msgs__msg__PiperStatusMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!piper_msgs__msg__PiperStatusMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          piper_msgs__msg__PiperStatusMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!piper_msgs__msg__PiperStatusMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
