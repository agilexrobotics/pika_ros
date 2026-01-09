// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from data_msgs:msg/CaptureStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__STRUCT_H_
#define DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__STRUCT_H_

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
// Member 'count_in_seconds'
// Member 'frequencies'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/CaptureStatus in the package data_msgs.
typedef struct data_msgs__msg__CaptureStatus
{
  rosidl_runtime_c__String__Sequence topics;
  rosidl_runtime_c__int32__Sequence count_in_seconds;
  rosidl_runtime_c__float__Sequence frequencies;
  bool fail;
  bool quit;
} data_msgs__msg__CaptureStatus;

// Struct for a sequence of data_msgs__msg__CaptureStatus.
typedef struct data_msgs__msg__CaptureStatus__Sequence
{
  data_msgs__msg__CaptureStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} data_msgs__msg__CaptureStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__STRUCT_H_
