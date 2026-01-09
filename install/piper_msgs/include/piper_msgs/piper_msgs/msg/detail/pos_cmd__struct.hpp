// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from piper_msgs:msg/PosCmd.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__MSG__DETAIL__POS_CMD__STRUCT_HPP_
#define PIPER_MSGS__MSG__DETAIL__POS_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__piper_msgs__msg__PosCmd __attribute__((deprecated))
#else
# define DEPRECATED__piper_msgs__msg__PosCmd __declspec(deprecated)
#endif

namespace piper_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PosCmd_
{
  using Type = PosCmd_<ContainerAllocator>;

  explicit PosCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->gripper = 0.0;
      this->mode1 = 0l;
      this->mode2 = 0l;
    }
  }

  explicit PosCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->gripper = 0.0;
      this->mode1 = 0l;
      this->mode2 = 0l;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _gripper_type =
    double;
  _gripper_type gripper;
  using _mode1_type =
    int32_t;
  _mode1_type mode1;
  using _mode2_type =
    int32_t;
  _mode2_type mode2;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__gripper(
    const double & _arg)
  {
    this->gripper = _arg;
    return *this;
  }
  Type & set__mode1(
    const int32_t & _arg)
  {
    this->mode1 = _arg;
    return *this;
  }
  Type & set__mode2(
    const int32_t & _arg)
  {
    this->mode2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    piper_msgs::msg::PosCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const piper_msgs::msg::PosCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      piper_msgs::msg::PosCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      piper_msgs::msg::PosCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__piper_msgs__msg__PosCmd
    std::shared_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__piper_msgs__msg__PosCmd
    std::shared_ptr<piper_msgs::msg::PosCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PosCmd_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->gripper != other.gripper) {
      return false;
    }
    if (this->mode1 != other.mode1) {
      return false;
    }
    if (this->mode2 != other.mode2) {
      return false;
    }
    return true;
  }
  bool operator!=(const PosCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PosCmd_

// alias to use template instance with default allocator
using PosCmd =
  piper_msgs::msg::PosCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace piper_msgs

#endif  // PIPER_MSGS__MSG__DETAIL__POS_CMD__STRUCT_HPP_
