// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from data_msgs:msg/CaptureStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__STRUCT_HPP_
#define DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__data_msgs__msg__CaptureStatus __attribute__((deprecated))
#else
# define DEPRECATED__data_msgs__msg__CaptureStatus __declspec(deprecated)
#endif

namespace data_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CaptureStatus_
{
  using Type = CaptureStatus_<ContainerAllocator>;

  explicit CaptureStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fail = false;
      this->quit = false;
    }
  }

  explicit CaptureStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _count_in_seconds_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _count_in_seconds_type count_in_seconds;
  using _frequencies_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _frequencies_type frequencies;
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
  Type & set__count_in_seconds(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->count_in_seconds = _arg;
    return *this;
  }
  Type & set__frequencies(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->frequencies = _arg;
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
    data_msgs::msg::CaptureStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const data_msgs::msg::CaptureStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::CaptureStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      data_msgs::msg::CaptureStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__data_msgs__msg__CaptureStatus
    std::shared_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__data_msgs__msg__CaptureStatus
    std::shared_ptr<data_msgs::msg::CaptureStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CaptureStatus_ & other) const
  {
    if (this->topics != other.topics) {
      return false;
    }
    if (this->count_in_seconds != other.count_in_seconds) {
      return false;
    }
    if (this->frequencies != other.frequencies) {
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
  bool operator!=(const CaptureStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CaptureStatus_

// alias to use template instance with default allocator
using CaptureStatus =
  data_msgs::msg::CaptureStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__STRUCT_HPP_
