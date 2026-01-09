// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from data_msgs:msg/TeleopStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__BUILDER_HPP_
#define DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "data_msgs/msg/detail/teleop_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace data_msgs
{

namespace msg
{

namespace builder
{

class Init_TeleopStatus_quit
{
public:
  explicit Init_TeleopStatus_quit(::data_msgs::msg::TeleopStatus & msg)
  : msg_(msg)
  {}
  ::data_msgs::msg::TeleopStatus quit(::data_msgs::msg::TeleopStatus::_quit_type arg)
  {
    msg_.quit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::msg::TeleopStatus msg_;
};

class Init_TeleopStatus_fail
{
public:
  explicit Init_TeleopStatus_fail(::data_msgs::msg::TeleopStatus & msg)
  : msg_(msg)
  {}
  Init_TeleopStatus_quit fail(::data_msgs::msg::TeleopStatus::_fail_type arg)
  {
    msg_.fail = std::move(arg);
    return Init_TeleopStatus_quit(msg_);
  }

private:
  ::data_msgs::msg::TeleopStatus msg_;
};

class Init_TeleopStatus_topics
{
public:
  Init_TeleopStatus_topics()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TeleopStatus_fail topics(::data_msgs::msg::TeleopStatus::_topics_type arg)
  {
    msg_.topics = std::move(arg);
    return Init_TeleopStatus_fail(msg_);
  }

private:
  ::data_msgs::msg::TeleopStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::msg::TeleopStatus>()
{
  return data_msgs::msg::builder::Init_TeleopStatus_topics();
}

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__BUILDER_HPP_
