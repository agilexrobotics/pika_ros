// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from data_msgs:msg/CaptureStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__BUILDER_HPP_
#define DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "data_msgs/msg/detail/capture_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace data_msgs
{

namespace msg
{

namespace builder
{

class Init_CaptureStatus_quit
{
public:
  explicit Init_CaptureStatus_quit(::data_msgs::msg::CaptureStatus & msg)
  : msg_(msg)
  {}
  ::data_msgs::msg::CaptureStatus quit(::data_msgs::msg::CaptureStatus::_quit_type arg)
  {
    msg_.quit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::msg::CaptureStatus msg_;
};

class Init_CaptureStatus_fail
{
public:
  explicit Init_CaptureStatus_fail(::data_msgs::msg::CaptureStatus & msg)
  : msg_(msg)
  {}
  Init_CaptureStatus_quit fail(::data_msgs::msg::CaptureStatus::_fail_type arg)
  {
    msg_.fail = std::move(arg);
    return Init_CaptureStatus_quit(msg_);
  }

private:
  ::data_msgs::msg::CaptureStatus msg_;
};

class Init_CaptureStatus_frequencies
{
public:
  explicit Init_CaptureStatus_frequencies(::data_msgs::msg::CaptureStatus & msg)
  : msg_(msg)
  {}
  Init_CaptureStatus_fail frequencies(::data_msgs::msg::CaptureStatus::_frequencies_type arg)
  {
    msg_.frequencies = std::move(arg);
    return Init_CaptureStatus_fail(msg_);
  }

private:
  ::data_msgs::msg::CaptureStatus msg_;
};

class Init_CaptureStatus_count_in_seconds
{
public:
  explicit Init_CaptureStatus_count_in_seconds(::data_msgs::msg::CaptureStatus & msg)
  : msg_(msg)
  {}
  Init_CaptureStatus_frequencies count_in_seconds(::data_msgs::msg::CaptureStatus::_count_in_seconds_type arg)
  {
    msg_.count_in_seconds = std::move(arg);
    return Init_CaptureStatus_frequencies(msg_);
  }

private:
  ::data_msgs::msg::CaptureStatus msg_;
};

class Init_CaptureStatus_topics
{
public:
  Init_CaptureStatus_topics()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureStatus_count_in_seconds topics(::data_msgs::msg::CaptureStatus::_topics_type arg)
  {
    msg_.topics = std::move(arg);
    return Init_CaptureStatus_count_in_seconds(msg_);
  }

private:
  ::data_msgs::msg::CaptureStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::msg::CaptureStatus>()
{
  return data_msgs::msg::builder::Init_CaptureStatus_topics();
}

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__BUILDER_HPP_
