// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from realsense2_camera_msgs:msg/RGBD.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__FUNCTIONS_H_
#define REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "realsense2_camera_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "realsense2_camera_msgs/msg/detail/rgbd__struct.h"

/// Initialize msg/RGBD message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * realsense2_camera_msgs__msg__RGBD
 * )) before or use
 * realsense2_camera_msgs__msg__RGBD__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__msg__RGBD__init(realsense2_camera_msgs__msg__RGBD * msg);

/// Finalize msg/RGBD message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__msg__RGBD__fini(realsense2_camera_msgs__msg__RGBD * msg);

/// Create msg/RGBD message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * realsense2_camera_msgs__msg__RGBD__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
realsense2_camera_msgs__msg__RGBD *
realsense2_camera_msgs__msg__RGBD__create();

/// Destroy msg/RGBD message.
/**
 * It calls
 * realsense2_camera_msgs__msg__RGBD__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__msg__RGBD__destroy(realsense2_camera_msgs__msg__RGBD * msg);

/// Check for msg/RGBD message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__msg__RGBD__are_equal(const realsense2_camera_msgs__msg__RGBD * lhs, const realsense2_camera_msgs__msg__RGBD * rhs);

/// Copy a msg/RGBD message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__msg__RGBD__copy(
  const realsense2_camera_msgs__msg__RGBD * input,
  realsense2_camera_msgs__msg__RGBD * output);

/// Initialize array of msg/RGBD messages.
/**
 * It allocates the memory for the number of elements and calls
 * realsense2_camera_msgs__msg__RGBD__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__msg__RGBD__Sequence__init(realsense2_camera_msgs__msg__RGBD__Sequence * array, size_t size);

/// Finalize array of msg/RGBD messages.
/**
 * It calls
 * realsense2_camera_msgs__msg__RGBD__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__msg__RGBD__Sequence__fini(realsense2_camera_msgs__msg__RGBD__Sequence * array);

/// Create array of msg/RGBD messages.
/**
 * It allocates the memory for the array and calls
 * realsense2_camera_msgs__msg__RGBD__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
realsense2_camera_msgs__msg__RGBD__Sequence *
realsense2_camera_msgs__msg__RGBD__Sequence__create(size_t size);

/// Destroy array of msg/RGBD messages.
/**
 * It calls
 * realsense2_camera_msgs__msg__RGBD__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__msg__RGBD__Sequence__destroy(realsense2_camera_msgs__msg__RGBD__Sequence * array);

/// Check for msg/RGBD message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__msg__RGBD__Sequence__are_equal(const realsense2_camera_msgs__msg__RGBD__Sequence * lhs, const realsense2_camera_msgs__msg__RGBD__Sequence * rhs);

/// Copy an array of msg/RGBD messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__msg__RGBD__Sequence__copy(
  const realsense2_camera_msgs__msg__RGBD__Sequence * input,
  realsense2_camera_msgs__msg__RGBD__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__FUNCTIONS_H_
