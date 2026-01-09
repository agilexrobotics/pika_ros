// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__STRUCT_H_
#define PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PiperStatusMsg in the package piper_msgs.
typedef struct piper_msgs__msg__PiperStatusMsg
{
  uint8_t ctrl_mode;
  uint8_t arm_status;
  uint8_t mode_feedback;
  uint8_t teach_status;
  uint8_t motion_status;
  uint8_t trajectory_num;
  int64_t err_code;
  bool joint_1_angle_limit;
  bool joint_2_angle_limit;
  bool joint_3_angle_limit;
  bool joint_4_angle_limit;
  bool joint_5_angle_limit;
  bool joint_6_angle_limit;
  bool communication_status_joint_1;
  bool communication_status_joint_2;
  bool communication_status_joint_3;
  bool communication_status_joint_4;
  bool communication_status_joint_5;
  bool communication_status_joint_6;
} piper_msgs__msg__PiperStatusMsg;

// Struct for a sequence of piper_msgs__msg__PiperStatusMsg.
typedef struct piper_msgs__msg__PiperStatusMsg__Sequence
{
  piper_msgs__msg__PiperStatusMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__msg__PiperStatusMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__STRUCT_H_
