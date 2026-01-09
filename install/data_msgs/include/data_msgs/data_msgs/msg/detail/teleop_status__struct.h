// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from data_msgs:msg/TeleopStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__STRUCT_H_
#define DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topics'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TeleopStatus in the package data_msgs.
typedef struct data_msgs__msg__TeleopStatus
{
  rosidl_runtime_c__String__Sequence topics;
  bool fail;
  bool quit;
} data_msgs__msg__TeleopStatus;

// Struct for a sequence of data_msgs__msg__TeleopStatus.
typedef struct data_msgs__msg__TeleopStatus__Sequence
{
  data_msgs__msg__TeleopStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} data_msgs__msg__TeleopStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__STRUCT_H_
