// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from data_msgs:msg/LocalizationStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__STRUCT_H_
#define DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LocalizationStatus in the package data_msgs.
typedef struct data_msgs__msg__LocalizationStatus
{
  bool accurate;
} data_msgs__msg__LocalizationStatus;

// Struct for a sequence of data_msgs__msg__LocalizationStatus.
typedef struct data_msgs__msg__LocalizationStatus__Sequence
{
  data_msgs__msg__LocalizationStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} data_msgs__msg__LocalizationStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__STRUCT_H_
