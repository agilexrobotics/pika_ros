// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from data_msgs:msg/LocalizationStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__BUILDER_HPP_
#define DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "data_msgs/msg/detail/localization_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace data_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalizationStatus_accurate
{
public:
  Init_LocalizationStatus_accurate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::data_msgs::msg::LocalizationStatus accurate(::data_msgs::msg::LocalizationStatus::_accurate_type arg)
  {
    msg_.accurate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::msg::LocalizationStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::msg::LocalizationStatus>()
{
  return data_msgs::msg::builder::Init_LocalizationStatus_accurate();
}

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__BUILDER_HPP_
