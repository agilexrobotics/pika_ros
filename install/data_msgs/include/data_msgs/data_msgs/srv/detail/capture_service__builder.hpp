// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from data_msgs:srv/CaptureService.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__BUILDER_HPP_
#define DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "data_msgs/srv/detail/capture_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace data_msgs
{

namespace srv
{

namespace builder
{

class Init_CaptureService_Request_instructions
{
public:
  explicit Init_CaptureService_Request_instructions(::data_msgs::srv::CaptureService_Request & msg)
  : msg_(msg)
  {}
  ::data_msgs::srv::CaptureService_Request instructions(::data_msgs::srv::CaptureService_Request::_instructions_type arg)
  {
    msg_.instructions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Request msg_;
};

class Init_CaptureService_Request_dataset_dir
{
public:
  explicit Init_CaptureService_Request_dataset_dir(::data_msgs::srv::CaptureService_Request & msg)
  : msg_(msg)
  {}
  Init_CaptureService_Request_instructions dataset_dir(::data_msgs::srv::CaptureService_Request::_dataset_dir_type arg)
  {
    msg_.dataset_dir = std::move(arg);
    return Init_CaptureService_Request_instructions(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Request msg_;
};

class Init_CaptureService_Request_episode_index
{
public:
  explicit Init_CaptureService_Request_episode_index(::data_msgs::srv::CaptureService_Request & msg)
  : msg_(msg)
  {}
  Init_CaptureService_Request_dataset_dir episode_index(::data_msgs::srv::CaptureService_Request::_episode_index_type arg)
  {
    msg_.episode_index = std::move(arg);
    return Init_CaptureService_Request_dataset_dir(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Request msg_;
};

class Init_CaptureService_Request_end
{
public:
  explicit Init_CaptureService_Request_end(::data_msgs::srv::CaptureService_Request & msg)
  : msg_(msg)
  {}
  Init_CaptureService_Request_episode_index end(::data_msgs::srv::CaptureService_Request::_end_type arg)
  {
    msg_.end = std::move(arg);
    return Init_CaptureService_Request_episode_index(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Request msg_;
};

class Init_CaptureService_Request_start
{
public:
  Init_CaptureService_Request_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureService_Request_end start(::data_msgs::srv::CaptureService_Request::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_CaptureService_Request_end(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::srv::CaptureService_Request>()
{
  return data_msgs::srv::builder::Init_CaptureService_Request_start();
}

}  // namespace data_msgs


namespace data_msgs
{

namespace srv
{

namespace builder
{

class Init_CaptureService_Response_message
{
public:
  explicit Init_CaptureService_Response_message(::data_msgs::srv::CaptureService_Response & msg)
  : msg_(msg)
  {}
  ::data_msgs::srv::CaptureService_Response message(::data_msgs::srv::CaptureService_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Response msg_;
};

class Init_CaptureService_Response_success
{
public:
  Init_CaptureService_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CaptureService_Response_message success(::data_msgs::srv::CaptureService_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CaptureService_Response_message(msg_);
  }

private:
  ::data_msgs::srv::CaptureService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::srv::CaptureService_Response>()
{
  return data_msgs::srv::builder::Init_CaptureService_Response_success();
}

}  // namespace data_msgs

#endif  // DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__BUILDER_HPP_
