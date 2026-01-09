// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from data_msgs:msg/ArmControlStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "data_msgs/msg/detail/arm_control_status__rosidl_typesupport_introspection_c.h"
#include "data_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "data_msgs/msg/detail/arm_control_status__functions.h"
#include "data_msgs/msg/detail/arm_control_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  data_msgs__msg__ArmControlStatus__init(message_memory);
}

void data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_fini_function(void * message_memory)
{
  data_msgs__msg__ArmControlStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_member_array[1] = {
  {
    "over_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs__msg__ArmControlStatus, over_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_members = {
  "data_msgs__msg",  // message namespace
  "ArmControlStatus",  // message name
  1,  // number of fields
  sizeof(data_msgs__msg__ArmControlStatus),
  data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_member_array,  // message members
  data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_type_support_handle = {
  0,
  &data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_data_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, data_msgs, msg, ArmControlStatus)() {
  if (!data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_type_support_handle.typesupport_identifier) {
    data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &data_msgs__msg__ArmControlStatus__rosidl_typesupport_introspection_c__ArmControlStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
