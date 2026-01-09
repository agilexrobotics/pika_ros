// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from data_msgs:msg/CaptureStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "data_msgs/msg/detail/capture_status__rosidl_typesupport_introspection_c.h"
#include "data_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "data_msgs/msg/detail/capture_status__functions.h"
#include "data_msgs/msg/detail/capture_status__struct.h"


// Include directives for member types
// Member `topics`
#include "rosidl_runtime_c/string_functions.h"
// Member `count_in_seconds`
// Member `frequencies`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  data_msgs__msg__CaptureStatus__init(message_memory);
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_fini_function(void * message_memory)
{
  data_msgs__msg__CaptureStatus__fini(message_memory);
}

size_t data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__size_function__CaptureStatus__topics(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__topics(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__topics(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__fetch_function__CaptureStatus__topics(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__topics(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__assign_function__CaptureStatus__topics(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__topics(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__resize_function__CaptureStatus__topics(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__size_function__CaptureStatus__count_in_seconds(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__count_in_seconds(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__count_in_seconds(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__fetch_function__CaptureStatus__count_in_seconds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__count_in_seconds(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__assign_function__CaptureStatus__count_in_seconds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__count_in_seconds(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__resize_function__CaptureStatus__count_in_seconds(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__size_function__CaptureStatus__frequencies(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__frequencies(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__frequencies(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__fetch_function__CaptureStatus__frequencies(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__frequencies(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__assign_function__CaptureStatus__frequencies(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__frequencies(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__resize_function__CaptureStatus__frequencies(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_member_array[5] = {
  {
    "topics",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs__msg__CaptureStatus, topics),  // bytes offset in struct
    NULL,  // default value
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__size_function__CaptureStatus__topics,  // size() function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__topics,  // get_const(index) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__topics,  // get(index) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__fetch_function__CaptureStatus__topics,  // fetch(index, &value) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__assign_function__CaptureStatus__topics,  // assign(index, value) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__resize_function__CaptureStatus__topics  // resize(index) function pointer
  },
  {
    "count_in_seconds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs__msg__CaptureStatus, count_in_seconds),  // bytes offset in struct
    NULL,  // default value
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__size_function__CaptureStatus__count_in_seconds,  // size() function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__count_in_seconds,  // get_const(index) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__count_in_seconds,  // get(index) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__fetch_function__CaptureStatus__count_in_seconds,  // fetch(index, &value) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__assign_function__CaptureStatus__count_in_seconds,  // assign(index, value) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__resize_function__CaptureStatus__count_in_seconds  // resize(index) function pointer
  },
  {
    "frequencies",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs__msg__CaptureStatus, frequencies),  // bytes offset in struct
    NULL,  // default value
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__size_function__CaptureStatus__frequencies,  // size() function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_const_function__CaptureStatus__frequencies,  // get_const(index) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__get_function__CaptureStatus__frequencies,  // get(index) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__fetch_function__CaptureStatus__frequencies,  // fetch(index, &value) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__assign_function__CaptureStatus__frequencies,  // assign(index, value) function pointer
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__resize_function__CaptureStatus__frequencies  // resize(index) function pointer
  },
  {
    "fail",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs__msg__CaptureStatus, fail),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs__msg__CaptureStatus, quit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_members = {
  "data_msgs__msg",  // message namespace
  "CaptureStatus",  // message name
  5,  // number of fields
  sizeof(data_msgs__msg__CaptureStatus),
  data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_member_array,  // message members
  data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_type_support_handle = {
  0,
  &data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_data_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, data_msgs, msg, CaptureStatus)() {
  if (!data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_type_support_handle.typesupport_identifier) {
    data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &data_msgs__msg__CaptureStatus__rosidl_typesupport_introspection_c__CaptureStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
