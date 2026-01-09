// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from realsense2_camera_msgs:msg/RGBD.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__BUILDER_HPP_
#define REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "realsense2_camera_msgs/msg/detail/rgbd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace realsense2_camera_msgs
{

namespace msg
{

namespace builder
{

class Init_RGBD_depth
{
public:
  explicit Init_RGBD_depth(::realsense2_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  ::realsense2_camera_msgs::msg::RGBD depth(::realsense2_camera_msgs::msg::RGBD::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return std::move(msg_);
  }

private:
  ::realsense2_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_rgb
{
public:
  explicit Init_RGBD_rgb(::realsense2_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  Init_RGBD_depth rgb(::realsense2_camera_msgs::msg::RGBD::_rgb_type arg)
  {
    msg_.rgb = std::move(arg);
    return Init_RGBD_depth(msg_);
  }

private:
  ::realsense2_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_depth_camera_info
{
public:
  explicit Init_RGBD_depth_camera_info(::realsense2_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  Init_RGBD_rgb depth_camera_info(::realsense2_camera_msgs::msg::RGBD::_depth_camera_info_type arg)
  {
    msg_.depth_camera_info = std::move(arg);
    return Init_RGBD_rgb(msg_);
  }

private:
  ::realsense2_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_rgb_camera_info
{
public:
  explicit Init_RGBD_rgb_camera_info(::realsense2_camera_msgs::msg::RGBD & msg)
  : msg_(msg)
  {}
  Init_RGBD_depth_camera_info rgb_camera_info(::realsense2_camera_msgs::msg::RGBD::_rgb_camera_info_type arg)
  {
    msg_.rgb_camera_info = std::move(arg);
    return Init_RGBD_depth_camera_info(msg_);
  }

private:
  ::realsense2_camera_msgs::msg::RGBD msg_;
};

class Init_RGBD_header
{
public:
  Init_RGBD_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RGBD_rgb_camera_info header(::realsense2_camera_msgs::msg::RGBD::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RGBD_rgb_camera_info(msg_);
  }

private:
  ::realsense2_camera_msgs::msg::RGBD msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::realsense2_camera_msgs::msg::RGBD>()
{
  return realsense2_camera_msgs::msg::builder::Init_RGBD_header();
}

}  // namespace realsense2_camera_msgs

#endif  // REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__BUILDER_HPP_
