// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from data_msgs:srv/CaptureService.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__STRUCT_HPP_
#define DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__data_msgs__srv__CaptureService_Request __attribute__((deprecated))
#else
# define DEPRECATED__data_msgs__srv__CaptureService_Request __declspec(deprecated)
#endif

namespace data_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CaptureService_Request_
{
  using Type = CaptureService_Request_<ContainerAllocator>;

  explicit CaptureService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
      this->end = false;
      this->episode_index = 0l;
      this->dataset_dir = "";
      this->instructions = "";
    }
  }

  explicit CaptureService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dataset_dir(_alloc),
    instructions(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
      this->end = false;
      this->episode_index = 0l;
      this->dataset_dir = "";
      this->instructions = "";
    }
  }

  // field types and members
  using _start_type =
    bool;
  _start_type start;
  using _end_type =
    bool;
  _end_type end;
  using _episode_index_type =
    int32_t;
  _episode_index_type episode_index;
  using _dataset_dir_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dataset_dir_type dataset_dir;
  using _instructions_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _instructions_type instructions;

  // setters for named parameter idiom
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__end(
    const bool & _arg)
  {
    this->end = _arg;
    return *this;
  }
  Type & set__episode_index(
    const int32_t & _arg)
  {
    this->episode_index = _arg;
    return *this;
  }
  Type & set__dataset_dir(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dataset_dir = _arg;
    return *this;
  }
  Type & set__instructions(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->instructions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    data_msgs::srv::CaptureService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const data_msgs::srv::CaptureService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      data_msgs::srv::CaptureService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      data_msgs::srv::CaptureService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__data_msgs__srv__CaptureService_Request
    std::shared_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__data_msgs__srv__CaptureService_Request
    std::shared_ptr<data_msgs::srv::CaptureService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CaptureService_Request_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    if (this->end != other.end) {
      return false;
    }
    if (this->episode_index != other.episode_index) {
      return false;
    }
    if (this->dataset_dir != other.dataset_dir) {
      return false;
    }
    if (this->instructions != other.instructions) {
      return false;
    }
    return true;
  }
  bool operator!=(const CaptureService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CaptureService_Request_

// alias to use template instance with default allocator
using CaptureService_Request =
  data_msgs::srv::CaptureService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace data_msgs


#ifndef _WIN32
# define DEPRECATED__data_msgs__srv__CaptureService_Response __attribute__((deprecated))
#else
# define DEPRECATED__data_msgs__srv__CaptureService_Response __declspec(deprecated)
#endif

namespace data_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CaptureService_Response_
{
  using Type = CaptureService_Response_<ContainerAllocator>;

  explicit CaptureService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit CaptureService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    data_msgs::srv::CaptureService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const data_msgs::srv::CaptureService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      data_msgs::srv::CaptureService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      data_msgs::srv::CaptureService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__data_msgs__srv__CaptureService_Response
    std::shared_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__data_msgs__srv__CaptureService_Response
    std::shared_ptr<data_msgs::srv::CaptureService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CaptureService_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const CaptureService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CaptureService_Response_

// alias to use template instance with default allocator
using CaptureService_Response =
  data_msgs::srv::CaptureService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace data_msgs

namespace data_msgs
{

namespace srv
{

struct CaptureService
{
  using Request = data_msgs::srv::CaptureService_Request;
  using Response = data_msgs::srv::CaptureService_Response;
};

}  // namespace srv

}  // namespace data_msgs

#endif  // DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__STRUCT_HPP_
