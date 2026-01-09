// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__STRUCT_HPP_
#define PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__piper_msgs__msg__PiperStatusMsg __attribute__((deprecated))
#else
# define DEPRECATED__piper_msgs__msg__PiperStatusMsg __declspec(deprecated)
#endif

namespace piper_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PiperStatusMsg_
{
  using Type = PiperStatusMsg_<ContainerAllocator>;

  explicit PiperStatusMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ctrl_mode = 0;
      this->arm_status = 0;
      this->mode_feedback = 0;
      this->teach_status = 0;
      this->motion_status = 0;
      this->trajectory_num = 0;
      this->err_code = 0ll;
      this->joint_1_angle_limit = false;
      this->joint_2_angle_limit = false;
      this->joint_3_angle_limit = false;
      this->joint_4_angle_limit = false;
      this->joint_5_angle_limit = false;
      this->joint_6_angle_limit = false;
      this->communication_status_joint_1 = false;
      this->communication_status_joint_2 = false;
      this->communication_status_joint_3 = false;
      this->communication_status_joint_4 = false;
      this->communication_status_joint_5 = false;
      this->communication_status_joint_6 = false;
    }
  }

  explicit PiperStatusMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ctrl_mode = 0;
      this->arm_status = 0;
      this->mode_feedback = 0;
      this->teach_status = 0;
      this->motion_status = 0;
      this->trajectory_num = 0;
      this->err_code = 0ll;
      this->joint_1_angle_limit = false;
      this->joint_2_angle_limit = false;
      this->joint_3_angle_limit = false;
      this->joint_4_angle_limit = false;
      this->joint_5_angle_limit = false;
      this->joint_6_angle_limit = false;
      this->communication_status_joint_1 = false;
      this->communication_status_joint_2 = false;
      this->communication_status_joint_3 = false;
      this->communication_status_joint_4 = false;
      this->communication_status_joint_5 = false;
      this->communication_status_joint_6 = false;
    }
  }

  // field types and members
  using _ctrl_mode_type =
    uint8_t;
  _ctrl_mode_type ctrl_mode;
  using _arm_status_type =
    uint8_t;
  _arm_status_type arm_status;
  using _mode_feedback_type =
    uint8_t;
  _mode_feedback_type mode_feedback;
  using _teach_status_type =
    uint8_t;
  _teach_status_type teach_status;
  using _motion_status_type =
    uint8_t;
  _motion_status_type motion_status;
  using _trajectory_num_type =
    uint8_t;
  _trajectory_num_type trajectory_num;
  using _err_code_type =
    int64_t;
  _err_code_type err_code;
  using _joint_1_angle_limit_type =
    bool;
  _joint_1_angle_limit_type joint_1_angle_limit;
  using _joint_2_angle_limit_type =
    bool;
  _joint_2_angle_limit_type joint_2_angle_limit;
  using _joint_3_angle_limit_type =
    bool;
  _joint_3_angle_limit_type joint_3_angle_limit;
  using _joint_4_angle_limit_type =
    bool;
  _joint_4_angle_limit_type joint_4_angle_limit;
  using _joint_5_angle_limit_type =
    bool;
  _joint_5_angle_limit_type joint_5_angle_limit;
  using _joint_6_angle_limit_type =
    bool;
  _joint_6_angle_limit_type joint_6_angle_limit;
  using _communication_status_joint_1_type =
    bool;
  _communication_status_joint_1_type communication_status_joint_1;
  using _communication_status_joint_2_type =
    bool;
  _communication_status_joint_2_type communication_status_joint_2;
  using _communication_status_joint_3_type =
    bool;
  _communication_status_joint_3_type communication_status_joint_3;
  using _communication_status_joint_4_type =
    bool;
  _communication_status_joint_4_type communication_status_joint_4;
  using _communication_status_joint_5_type =
    bool;
  _communication_status_joint_5_type communication_status_joint_5;
  using _communication_status_joint_6_type =
    bool;
  _communication_status_joint_6_type communication_status_joint_6;

  // setters for named parameter idiom
  Type & set__ctrl_mode(
    const uint8_t & _arg)
  {
    this->ctrl_mode = _arg;
    return *this;
  }
  Type & set__arm_status(
    const uint8_t & _arg)
  {
    this->arm_status = _arg;
    return *this;
  }
  Type & set__mode_feedback(
    const uint8_t & _arg)
  {
    this->mode_feedback = _arg;
    return *this;
  }
  Type & set__teach_status(
    const uint8_t & _arg)
  {
    this->teach_status = _arg;
    return *this;
  }
  Type & set__motion_status(
    const uint8_t & _arg)
  {
    this->motion_status = _arg;
    return *this;
  }
  Type & set__trajectory_num(
    const uint8_t & _arg)
  {
    this->trajectory_num = _arg;
    return *this;
  }
  Type & set__err_code(
    const int64_t & _arg)
  {
    this->err_code = _arg;
    return *this;
  }
  Type & set__joint_1_angle_limit(
    const bool & _arg)
  {
    this->joint_1_angle_limit = _arg;
    return *this;
  }
  Type & set__joint_2_angle_limit(
    const bool & _arg)
  {
    this->joint_2_angle_limit = _arg;
    return *this;
  }
  Type & set__joint_3_angle_limit(
    const bool & _arg)
  {
    this->joint_3_angle_limit = _arg;
    return *this;
  }
  Type & set__joint_4_angle_limit(
    const bool & _arg)
  {
    this->joint_4_angle_limit = _arg;
    return *this;
  }
  Type & set__joint_5_angle_limit(
    const bool & _arg)
  {
    this->joint_5_angle_limit = _arg;
    return *this;
  }
  Type & set__joint_6_angle_limit(
    const bool & _arg)
  {
    this->joint_6_angle_limit = _arg;
    return *this;
  }
  Type & set__communication_status_joint_1(
    const bool & _arg)
  {
    this->communication_status_joint_1 = _arg;
    return *this;
  }
  Type & set__communication_status_joint_2(
    const bool & _arg)
  {
    this->communication_status_joint_2 = _arg;
    return *this;
  }
  Type & set__communication_status_joint_3(
    const bool & _arg)
  {
    this->communication_status_joint_3 = _arg;
    return *this;
  }
  Type & set__communication_status_joint_4(
    const bool & _arg)
  {
    this->communication_status_joint_4 = _arg;
    return *this;
  }
  Type & set__communication_status_joint_5(
    const bool & _arg)
  {
    this->communication_status_joint_5 = _arg;
    return *this;
  }
  Type & set__communication_status_joint_6(
    const bool & _arg)
  {
    this->communication_status_joint_6 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    piper_msgs::msg::PiperStatusMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const piper_msgs::msg::PiperStatusMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      piper_msgs::msg::PiperStatusMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      piper_msgs::msg::PiperStatusMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__piper_msgs__msg__PiperStatusMsg
    std::shared_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__piper_msgs__msg__PiperStatusMsg
    std::shared_ptr<piper_msgs::msg::PiperStatusMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PiperStatusMsg_ & other) const
  {
    if (this->ctrl_mode != other.ctrl_mode) {
      return false;
    }
    if (this->arm_status != other.arm_status) {
      return false;
    }
    if (this->mode_feedback != other.mode_feedback) {
      return false;
    }
    if (this->teach_status != other.teach_status) {
      return false;
    }
    if (this->motion_status != other.motion_status) {
      return false;
    }
    if (this->trajectory_num != other.trajectory_num) {
      return false;
    }
    if (this->err_code != other.err_code) {
      return false;
    }
    if (this->joint_1_angle_limit != other.joint_1_angle_limit) {
      return false;
    }
    if (this->joint_2_angle_limit != other.joint_2_angle_limit) {
      return false;
    }
    if (this->joint_3_angle_limit != other.joint_3_angle_limit) {
      return false;
    }
    if (this->joint_4_angle_limit != other.joint_4_angle_limit) {
      return false;
    }
    if (this->joint_5_angle_limit != other.joint_5_angle_limit) {
      return false;
    }
    if (this->joint_6_angle_limit != other.joint_6_angle_limit) {
      return false;
    }
    if (this->communication_status_joint_1 != other.communication_status_joint_1) {
      return false;
    }
    if (this->communication_status_joint_2 != other.communication_status_joint_2) {
      return false;
    }
    if (this->communication_status_joint_3 != other.communication_status_joint_3) {
      return false;
    }
    if (this->communication_status_joint_4 != other.communication_status_joint_4) {
      return false;
    }
    if (this->communication_status_joint_5 != other.communication_status_joint_5) {
      return false;
    }
    if (this->communication_status_joint_6 != other.communication_status_joint_6) {
      return false;
    }
    return true;
  }
  bool operator!=(const PiperStatusMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PiperStatusMsg_

// alias to use template instance with default allocator
using PiperStatusMsg =
  piper_msgs::msg::PiperStatusMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace piper_msgs

#endif  // PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__STRUCT_HPP_
