// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from data_msgs:msg/Gripper.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__GRIPPER__TRAITS_HPP_
#define DATA_MSGS__MSG__DETAIL__GRIPPER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "data_msgs/msg/detail/gripper__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace data_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Gripper & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: effort
  {
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: enable
  {
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << ", ";
  }

  // member: set_zero
  {
    out << "set_zero: ";
    rosidl_generator_traits::value_to_yaml(msg.set_zero, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: driver_temp
  {
    out << "driver_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_temp, out);
    out << ", ";
  }

  // member: motor_temp
  {
    out << "motor_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temp, out);
    out << ", ";
  }

  // member: bus_current
  {
    out << "bus_current: ";
    rosidl_generator_traits::value_to_yaml(msg.bus_current, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Gripper & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << "\n";
  }

  // member: set_zero
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "set_zero: ";
    rosidl_generator_traits::value_to_yaml(msg.set_zero, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: driver_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_temp, out);
    out << "\n";
  }

  // member: motor_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temp, out);
    out << "\n";
  }

  // member: bus_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bus_current: ";
    rosidl_generator_traits::value_to_yaml(msg.bus_current, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Gripper & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace data_msgs

namespace rosidl_generator_traits
{

[[deprecated("use data_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const data_msgs::msg::Gripper & msg,
  std::ostream & out, size_t indentation = 0)
{
  data_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use data_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const data_msgs::msg::Gripper & msg)
{
  return data_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<data_msgs::msg::Gripper>()
{
  return "data_msgs::msg::Gripper";
}

template<>
inline const char * name<data_msgs::msg::Gripper>()
{
  return "data_msgs/msg/Gripper";
}

template<>
struct has_fixed_size<data_msgs::msg::Gripper>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<data_msgs::msg::Gripper>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<data_msgs::msg::Gripper>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DATA_MSGS__MSG__DETAIL__GRIPPER__TRAITS_HPP_
