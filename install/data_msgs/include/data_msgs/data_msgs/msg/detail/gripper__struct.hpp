// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from data_msgs:msg/Gripper.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__GRIPPER__STRUCT_HPP_
#define DATA_MSGS__MSG__DETAIL__GRIPPER__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__data_msgs__msg__Gripper __attribute__((deprecated))
#else
# define DEPRECATED__data_msgs__msg__Gripper __declspec(deprecated)
#endif

namespace data_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gripper_
{
  using Type = Gripper_<ContainerAllocator>;

  explicit Gripper_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0;
      this->distance = 0.0;
      this->effort = 0.0;
      this->velocity = 0.0;
      this->enable = false;
      this->set_zero = false;
      this->error = false;
      this->voltage = 0.0;
      this->driver_temp = 0.0;
      this->motor_temp = 0.0;
      this->bus_current = 0.0;
      this->status = "";
    }
  }

  explicit Gripper_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0;
      this->distance = 0.0;
      this->effort = 0.0;
      this->velocity = 0.0;
      this->enable = false;
      this->set_zero = false;
      this->error = false;
      this->voltage = 0.0;
      this->driver_temp = 0.0;
      this->motor_temp = 0.0;
      this->bus_current = 0.0;
      this->status = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angle_type =
    double;
  _angle_type angle;
  using _distance_type =
    double;
  _distance_type distance;
  using _effort_type =
    double;
  _effort_type effort;
  using _velocity_type =
    double;
  _velocity_type velocity;
  using _enable_type =
    bool;
  _enable_type enable;
  using _set_zero_type =
    bool;
  _set_zero_type set_zero;
  using _error_type =
    bool;
  _error_type error;
  using _voltage_type =
    double;
  _voltage_type voltage;
  using _driver_temp_type =
    double;
  _driver_temp_type driver_temp;
  using _motor_temp_type =
    double;
  _motor_temp_type motor_temp;
  using _bus_current_type =
    double;
  _bus_current_type bus_current;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angle(
    const double & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__effort(
    const double & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__velocity(
    const double & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__enable(
    const bool & _arg)
  {
    this->enable = _arg;
    return *this;
  }
  Type & set__set_zero(
    const bool & _arg)
  {
    this->set_zero = _arg;
    return *this;
  }
  Type & set__error(
    const bool & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__voltage(
    const double & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__driver_temp(
    const double & _arg)
  {
    this->driver_temp = _arg;
    return *this;
  }
  Type & set__motor_temp(
    const double & _arg)
  {
    this->motor_temp = _arg;
    return *this;
  }
  Type & set__bus_current(
    const double & _arg)
  {
    this->bus_current = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    data_msgs::msg::Gripper_<ContainerAllocator> *;
  using ConstRawPtr =
    const data_msgs::msg::Gripper_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<data_msgs::msg::Gripper_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<data_msgs::msg::Gripper_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::Gripper_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::Gripper_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::Gripper_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::Gripper_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<data_msgs::msg::Gripper_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<data_msgs::msg::Gripper_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__data_msgs__msg__Gripper
    std::shared_ptr<data_msgs::msg::Gripper_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__data_msgs__msg__Gripper
    std::shared_ptr<data_msgs::msg::Gripper_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gripper_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->enable != other.enable) {
      return false;
    }
    if (this->set_zero != other.set_zero) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->driver_temp != other.driver_temp) {
      return false;
    }
    if (this->motor_temp != other.motor_temp) {
      return false;
    }
    if (this->bus_current != other.bus_current) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gripper_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gripper_

// alias to use template instance with default allocator
using Gripper =
  data_msgs::msg::Gripper_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__GRIPPER__STRUCT_HPP_
