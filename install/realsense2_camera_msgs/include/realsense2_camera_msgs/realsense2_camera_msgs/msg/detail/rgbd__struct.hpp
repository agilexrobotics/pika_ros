// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from realsense2_camera_msgs:msg/RGBD.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__STRUCT_HPP_
#define REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'rgb_camera_info'
// Member 'depth_camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.hpp"
// Member 'rgb'
// Member 'depth'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__realsense2_camera_msgs__msg__RGBD __attribute__((deprecated))
#else
# define DEPRECATED__realsense2_camera_msgs__msg__RGBD __declspec(deprecated)
#endif

namespace realsense2_camera_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RGBD_
{
  using Type = RGBD_<ContainerAllocator>;

  explicit RGBD_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    rgb_camera_info(_init),
    depth_camera_info(_init),
    rgb(_init),
    depth(_init)
  {
    (void)_init;
  }

  explicit RGBD_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    rgb_camera_info(_alloc, _init),
    depth_camera_info(_alloc, _init),
    rgb(_alloc, _init),
    depth(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rgb_camera_info_type =
    sensor_msgs::msg::CameraInfo_<ContainerAllocator>;
  _rgb_camera_info_type rgb_camera_info;
  using _depth_camera_info_type =
    sensor_msgs::msg::CameraInfo_<ContainerAllocator>;
  _depth_camera_info_type depth_camera_info;
  using _rgb_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _rgb_type rgb;
  using _depth_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _depth_type depth;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rgb_camera_info(
    const sensor_msgs::msg::CameraInfo_<ContainerAllocator> & _arg)
  {
    this->rgb_camera_info = _arg;
    return *this;
  }
  Type & set__depth_camera_info(
    const sensor_msgs::msg::CameraInfo_<ContainerAllocator> & _arg)
  {
    this->depth_camera_info = _arg;
    return *this;
  }
  Type & set__rgb(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->rgb = _arg;
    return *this;
  }
  Type & set__depth(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->depth = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    realsense2_camera_msgs::msg::RGBD_<ContainerAllocator> *;
  using ConstRawPtr =
    const realsense2_camera_msgs::msg::RGBD_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      realsense2_camera_msgs::msg::RGBD_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      realsense2_camera_msgs::msg::RGBD_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__realsense2_camera_msgs__msg__RGBD
    std::shared_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__realsense2_camera_msgs__msg__RGBD
    std::shared_ptr<realsense2_camera_msgs::msg::RGBD_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RGBD_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rgb_camera_info != other.rgb_camera_info) {
      return false;
    }
    if (this->depth_camera_info != other.depth_camera_info) {
      return false;
    }
    if (this->rgb != other.rgb) {
      return false;
    }
    if (this->depth != other.depth) {
      return false;
    }
    return true;
  }
  bool operator!=(const RGBD_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RGBD_

// alias to use template instance with default allocator
using RGBD =
  realsense2_camera_msgs::msg::RGBD_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace realsense2_camera_msgs

#endif  // REALSENSE2_CAMERA_MSGS__MSG__DETAIL__RGBD__STRUCT_HPP_
