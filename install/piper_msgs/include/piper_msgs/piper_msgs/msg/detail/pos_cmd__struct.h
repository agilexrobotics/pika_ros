// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:msg/PosCmd.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__MSG__DETAIL__POS_CMD__STRUCT_H_
#define PIPER_MSGS__MSG__DETAIL__POS_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PosCmd in the package piper_msgs.
typedef struct piper_msgs__msg__PosCmd
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  double gripper;
  int32_t mode1;
  int32_t mode2;
} piper_msgs__msg__PosCmd;

// Struct for a sequence of piper_msgs__msg__PosCmd.
typedef struct piper_msgs__msg__PosCmd__Sequence
{
  piper_msgs__msg__PosCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__msg__PosCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__MSG__DETAIL__POS_CMD__STRUCT_H_
