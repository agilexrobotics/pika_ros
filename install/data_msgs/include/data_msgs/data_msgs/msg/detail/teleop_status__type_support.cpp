// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from data_msgs:msg/TeleopStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "data_msgs/msg/detail/teleop_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace data_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TeleopStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) data_msgs::msg::TeleopStatus(_init);
}

void TeleopStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<data_msgs::msg::TeleopStatus *>(message_memory);
  typed_message->~TeleopStatus();
}

size_t size_function__TeleopStatus__topics(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TeleopStatus__topics(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__TeleopStatus__topics(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__TeleopStatus__topics(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__TeleopStatus__topics(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__TeleopStatus__topics(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__TeleopStatus__topics(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__TeleopStatus__topics(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TeleopStatus_message_member_array[3] = {
  {
    "topics",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs::msg::TeleopStatus, topics),  // bytes offset in struct
    nullptr,  // default value
    size_function__TeleopStatus__topics,  // size() function pointer
    get_const_function__TeleopStatus__topics,  // get_const(index) function pointer
    get_function__TeleopStatus__topics,  // get(index) function pointer
    fetch_function__TeleopStatus__topics,  // fetch(index, &value) function pointer
    assign_function__TeleopStatus__topics,  // assign(index, value) function pointer
    resize_function__TeleopStatus__topics  // resize(index) function pointer
  },
  {
    "fail",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs::msg::TeleopStatus, fail),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "quit",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(data_msgs::msg::TeleopStatus, quit),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TeleopStatus_message_members = {
  "data_msgs::msg",  // message namespace
  "TeleopStatus",  // message name
  3,  // number of fields
  sizeof(data_msgs::msg::TeleopStatus),
  TeleopStatus_message_member_array,  // message members
  TeleopStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  TeleopStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TeleopStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TeleopStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace data_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<data_msgs::msg::TeleopStatus>()
{
  return &::data_msgs::msg::rosidl_typesupport_introspection_cpp::TeleopStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, data_msgs, msg, TeleopStatus)() {
  return &::data_msgs::msg::rosidl_typesupport_introspection_cpp::TeleopStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
