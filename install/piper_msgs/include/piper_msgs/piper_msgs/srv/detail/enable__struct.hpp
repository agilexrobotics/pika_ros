// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from piper_msgs:srv/Enable.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__ENABLE__STRUCT_HPP_
#define PIPER_MSGS__SRV__DETAIL__ENABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__piper_msgs__srv__Enable_Request __attribute__((deprecated))
#else
# define DEPRECATED__piper_msgs__srv__Enable_Request __declspec(deprecated)
#endif

namespace piper_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Enable_Request_
{
  using Type = Enable_Request_<ContainerAllocator>;

  explicit Enable_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable_request = false;
    }
  }

  explicit Enable_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable_request = false;
    }
  }

  // field types and members
  using _enable_request_type =
    bool;
  _enable_request_type enable_request;

  // setters for named parameter idiom
  Type & set__enable_request(
    const bool & _arg)
  {
    this->enable_request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    piper_msgs::srv::Enable_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const piper_msgs::srv::Enable_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::Enable_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::Enable_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__piper_msgs__srv__Enable_Request
    std::shared_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__piper_msgs__srv__Enable_Request
    std::shared_ptr<piper_msgs::srv::Enable_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Enable_Request_ & other) const
  {
    if (this->enable_request != other.enable_request) {
      return false;
    }
    return true;
  }
  bool operator!=(const Enable_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Enable_Request_

// alias to use template instance with default allocator
using Enable_Request =
  piper_msgs::srv::Enable_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace piper_msgs


#ifndef _WIN32
# define DEPRECATED__piper_msgs__srv__Enable_Response __attribute__((deprecated))
#else
# define DEPRECATED__piper_msgs__srv__Enable_Response __declspec(deprecated)
#endif

namespace piper_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Enable_Response_
{
  using Type = Enable_Response_<ContainerAllocator>;

  explicit Enable_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable_response = false;
    }
  }

  explicit Enable_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable_response = false;
    }
  }

  // field types and members
  using _enable_response_type =
    bool;
  _enable_response_type enable_response;

  // setters for named parameter idiom
  Type & set__enable_response(
    const bool & _arg)
  {
    this->enable_response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    piper_msgs::srv::Enable_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const piper_msgs::srv::Enable_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::Enable_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::Enable_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__piper_msgs__srv__Enable_Response
    std::shared_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__piper_msgs__srv__Enable_Response
    std::shared_ptr<piper_msgs::srv::Enable_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Enable_Response_ & other) const
  {
    if (this->enable_response != other.enable_response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Enable_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Enable_Response_

// alias to use template instance with default allocator
using Enable_Response =
  piper_msgs::srv::Enable_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace piper_msgs

namespace piper_msgs
{

namespace srv
{

struct Enable
{
  using Request = piper_msgs::srv::Enable_Request;
  using Response = piper_msgs::srv::Enable_Response;
};

}  // namespace srv

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__ENABLE__STRUCT_HPP_
