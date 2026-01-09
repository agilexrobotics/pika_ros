// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from data_msgs:msg/Gripper.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__GRIPPER__STRUCT_H_
#define DATA_MSGS__MSG__DETAIL__GRIPPER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Gripper in the package data_msgs.
typedef struct data_msgs__msg__Gripper
{
  std_msgs__msg__Header header;
  double angle;
  double distance;
  double effort;
  double velocity;
  bool enable;
  bool set_zero;
  bool error;
  double voltage;
  double driver_temp;
  double motor_temp;
  double bus_current;
  rosidl_runtime_c__String status;
} data_msgs__msg__Gripper;

// Struct for a sequence of data_msgs__msg__Gripper.
typedef struct data_msgs__msg__Gripper__Sequence
{
  data_msgs__msg__Gripper * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} data_msgs__msg__Gripper__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DATA_MSGS__MSG__DETAIL__GRIPPER__STRUCT_H_
