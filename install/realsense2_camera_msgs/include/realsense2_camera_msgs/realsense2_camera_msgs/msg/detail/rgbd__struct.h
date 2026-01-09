// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from realsense2_camera_msgs:msg/RGBD.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__STRUCT_H_
#define REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__STRUCT_H_

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
// Member 'rgb_camera_info'
// Member 'depth_camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"
// Member 'rgb'
// Member 'depth'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/RGBD in the package realsense2_camera_msgs.
/**
  * RGBD Message
 */
typedef struct realsense2_camera_msgs__msg__RGBD
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__CameraInfo rgb_camera_info;
  sensor_msgs__msg__CameraInfo depth_camera_info;
  sensor_msgs__msg__Image rgb;
  sensor_msgs__msg__Image depth;
} realsense2_camera_msgs__msg__RGBD;

// Struct for a sequence of realsense2_camera_msgs__msg__RGBD.
typedef struct realsense2_camera_msgs__msg__RGBD__Sequence
{
  realsense2_camera_msgs__msg__RGBD * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} realsense2_camera_msgs__msg__RGBD__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__STRUCT_H_
