// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:srv/Enable.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__ENABLE__STRUCT_H_
#define PIPER_MSGS__SRV__DETAIL__ENABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Enable in the package piper_msgs.
typedef struct piper_msgs__srv__Enable_Request
{
  /// 请求消息类型为bool
  bool enable_request;
} piper_msgs__srv__Enable_Request;

// Struct for a sequence of piper_msgs__srv__Enable_Request.
typedef struct piper_msgs__srv__Enable_Request__Sequence
{
  piper_msgs__srv__Enable_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__Enable_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Enable in the package piper_msgs.
typedef struct piper_msgs__srv__Enable_Response
{
  /// 响应消息类型为bool
  bool enable_response;
} piper_msgs__srv__Enable_Response;

// Struct for a sequence of piper_msgs__srv__Enable_Response.
typedef struct piper_msgs__srv__Enable_Response__Sequence
{
  piper_msgs__srv__Enable_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__Enable_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__SRV__DETAIL__ENABLE__STRUCT_H_
