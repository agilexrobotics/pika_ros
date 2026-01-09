// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from data_msgs:msg/TeleopStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__STRUCT_HPP_
#define DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__data_msgs__msg__TeleopStatus __attribute__((deprecated))
#else
# define DEPRECATED__data_msgs__msg__TeleopStatus __declspec(deprecated)
#endif

namespace data_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TeleopStatus_
{
  using Type = TeleopStatus_<ContainerAllocator>;

  explicit TeleopStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fail = false;
      this->quit = false;
    }
  }

  explicit TeleopStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fail = false;
      this->quit = false;
    }
  }

  // field types and members
  using _topics_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _topics_type topics;
  using _fail_type =
    bool;
  _fail_type fail;
  using _quit_type =
    bool;
  _quit_type quit;

  // setters for named parameter idiom
  Type & set__topics(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->topics = _arg;
    return *this;
  }
  Type & set__fail(
    const bool & _arg)
  {
    this->fail = _arg;
    return *this;
  }
  Type & set__quit(
    const bool & _arg)
  {
    this->quit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    data_msgs::msg::TeleopStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const data_msgs::msg::TeleopStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::TeleopStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::TeleopStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__data_msgs__msg__TeleopStatus
    std::shared_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__data_msgs__msg__TeleopStatus
    std::shared_ptr<data_msgs::msg::TeleopStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TeleopStatus_ & other) const
  {
    if (this->topics != other.topics) {
      return false;
    }
    if (this->fail != other.fail) {
      return false;
    }
    if (this->quit != other.quit) {
      return false;
    }
    return true;
  }
  bool operator!=(const TeleopStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TeleopStatus_

// alias to use template instance with default allocator
using TeleopStatus =
  data_msgs::msg::TeleopStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__TELEOP_STATUS__STRUCT_HPP_
