// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from data_msgs:msg/Gripper.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__GRIPPER__BUILDER_HPP_
#define DATA_MSGS__MSG__DETAIL__GRIPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "data_msgs/msg/detail/gripper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace data_msgs
{

namespace msg
{

namespace builder
{

class Init_Gripper_status
{
public:
  explicit Init_Gripper_status(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  ::data_msgs::msg::Gripper status(::data_msgs::msg::Gripper::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_bus_current
{
public:
  explicit Init_Gripper_bus_current(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_status bus_current(::data_msgs::msg::Gripper::_bus_current_type arg)
  {
    msg_.bus_current = std::move(arg);
    return Init_Gripper_status(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_motor_temp
{
public:
  explicit Init_Gripper_motor_temp(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_bus_current motor_temp(::data_msgs::msg::Gripper::_motor_temp_type arg)
  {
    msg_.motor_temp = std::move(arg);
    return Init_Gripper_bus_current(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_driver_temp
{
public:
  explicit Init_Gripper_driver_temp(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_motor_temp driver_temp(::data_msgs::msg::Gripper::_driver_temp_type arg)
  {
    msg_.driver_temp = std::move(arg);
    return Init_Gripper_motor_temp(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_voltage
{
public:
  explicit Init_Gripper_voltage(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_driver_temp voltage(::data_msgs::msg::Gripper::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_Gripper_driver_temp(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_error
{
public:
  explicit Init_Gripper_error(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_voltage error(::data_msgs::msg::Gripper::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_Gripper_voltage(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_set_zero
{
public:
  explicit Init_Gripper_set_zero(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_error set_zero(::data_msgs::msg::Gripper::_set_zero_type arg)
  {
    msg_.set_zero = std::move(arg);
    return Init_Gripper_error(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_enable
{
public:
  explicit Init_Gripper_enable(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_set_zero enable(::data_msgs::msg::Gripper::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return Init_Gripper_set_zero(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_velocity
{
public:
  explicit Init_Gripper_velocity(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_enable velocity(::data_msgs::msg::Gripper::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Gripper_enable(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_effort
{
public:
  explicit Init_Gripper_effort(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_velocity effort(::data_msgs::msg::Gripper::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return Init_Gripper_velocity(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_distance
{
public:
  explicit Init_Gripper_distance(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_effort distance(::data_msgs::msg::Gripper::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_Gripper_effort(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_angle
{
public:
  explicit Init_Gripper_angle(::data_msgs::msg::Gripper & msg)
  : msg_(msg)
  {}
  Init_Gripper_distance angle(::data_msgs::msg::Gripper::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_Gripper_distance(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

class Init_Gripper_header
{
public:
  Init_Gripper_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gripper_angle header(::data_msgs::msg::Gripper::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Gripper_angle(msg_);
  }

private:
  ::data_msgs::msg::Gripper msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::data_msgs::msg::Gripper>()
{
  return data_msgs::msg::builder::Init_Gripper_header();
}

}  // namespace data_msgs

#endif  // DATA_MSGS__MSG__DETAIL__GRIPPER__BUILDER_HPP_
