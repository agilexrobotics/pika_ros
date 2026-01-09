// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from data_msgs:msg/ArmControlStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__ARM_CONTROL_STATUS__BUILDER_HPP_
#define DATA_MSGS__MSG__DETAIL__ARM_CONTROL_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "data_msgs/msg/detail/arm_control_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace data_msgs
{

namespace msg
{

namespace builder
{

class Init_ArmControlStatus_over_limit
{
public:
  Init_ArmControlStatus_over_limit()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::data_msgs::msg::ArmControlStatus over_limit(::data_msgs::msg::ArmControlStatus::_over_limit_type arg)
  {
    msg_.over_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::msg::ArmControlStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::msg::ArmControlStatus>()
{
  return data_msgs::msg::builder::Init_ArmControlStatus_over_limit();
}

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__ARM_CONTROL_STATUS__BUILDER_HPP_
