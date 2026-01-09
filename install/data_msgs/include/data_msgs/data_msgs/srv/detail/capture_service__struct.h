// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from data_msgs:srv/CaptureService.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__STRUCT_H_
#define DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dataset_dir'
// Member 'instructions'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CaptureService in the package data_msgs.
typedef struct data_msgs__srv__CaptureService_Request
{
  bool start;
  bool end;
  int32_t episode_index;
  rosidl_runtime_c__String dataset_dir;
  rosidl_runtime_c__String instructions;
} data_msgs__srv__CaptureService_Request;

// Struct for a sequence of data_msgs__srv__CaptureService_Request.
typedef struct data_msgs__srv__CaptureService_Request__Sequence
{
  data_msgs__srv__CaptureService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} data_msgs__srv__CaptureService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CaptureService in the package data_msgs.
typedef struct data_msgs__srv__CaptureService_Response
{
  bool success;
  rosidl_runtime_c__String message;
} data_msgs__srv__CaptureService_Response;

// Struct for a sequence of data_msgs__srv__CaptureService_Response.
typedef struct data_msgs__srv__CaptureService_Response__Sequence
{
  data_msgs__srv__CaptureService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} data_msgs__srv__CaptureService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__STRUCT_H_
