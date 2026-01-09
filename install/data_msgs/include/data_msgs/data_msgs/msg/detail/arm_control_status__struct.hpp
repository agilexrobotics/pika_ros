// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from data_msgs:msg/ArmControlStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__ARM_CONTROL_STATUS__STRUCT_HPP_
#define DATA_MSGS__MSG__DETAIL__ARM_CONTROL_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__data_msgs__msg__ArmControlStatus __attribute__((deprecated))
#else
# define DEPRECATED__data_msgs__msg__ArmControlStatus __declspec(deprecated)
#endif

namespace data_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmControlStatus_
{
  using Type = ArmControlStatus_<ContainerAllocator>;

  explicit ArmControlStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->over_limit = false;
    }
  }

  explicit ArmControlStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->over_limit = false;
    }
  }

  // field types and members
  using _over_limit_type =
    bool;
  _over_limit_type over_limit;

  // setters for named parameter idiom
  Type & set__over_limit(
    const bool & _arg)
  {
    this->over_limit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    data_msgs::msg::ArmControlStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const data_msgs::msg::ArmControlStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::ArmControlStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::ArmControlStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__data_msgs__msg__ArmControlStatus
    std::shared_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__data_msgs__msg__ArmControlStatus
    std::shared_ptr<data_msgs::msg::ArmControlStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmControlStatus_ & other) const
  {
    if (this->over_limit != other.over_limit) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmControlStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmControlStatus_

// alias to use template instance with default allocator
using ArmControlStatus =
  data_msgs::msg::ArmControlStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__ARM_CONTROL_STATUS__STRUCT_HPP_
