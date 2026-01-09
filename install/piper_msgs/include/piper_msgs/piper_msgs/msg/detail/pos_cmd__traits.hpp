// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from piper_msgs:msg/PosCmd.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__MSG__DETAIL__POS_CMD__TRAITS_HPP_
#define PIPER_MSGS__MSG__DETAIL__POS_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "piper_msgs/msg/detail/pos_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace piper_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PosCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: gripper
  {
    out << "gripper: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper, out);
    out << ", ";
  }

  // member: mode1
  {
    out << "mode1: ";
    rosidl_generator_traits::value_to_yaml(msg.mode1, out);
    out << ", ";
  }

  // member: mode2
  {
    out << "mode2: ";
    rosidl_generator_traits::value_to_yaml(msg.mode2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PosCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: gripper
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper, out);
    out << "\n";
  }

  // member: mode1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode1: ";
    rosidl_generator_traits::value_to_yaml(msg.mode1, out);
    out << "\n";
  }

  // member: mode2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode2: ";
    rosidl_generator_traits::value_to_yaml(msg.mode2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PosCmd & msg, bool use_flow_style = false)
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

}  // namespace piper_msgs

namespace rosidl_generator_traits
{

[[deprecated("use piper_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const piper_msgs::msg::PosCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  piper_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use piper_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const piper_msgs::msg::PosCmd & msg)
{
  return piper_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<piper_msgs::msg::PosCmd>()
{
  return "piper_msgs::msg::PosCmd";
}

template<>
inline const char * name<piper_msgs::msg::PosCmd>()
{
  return "piper_msgs/msg/PosCmd";
}

template<>
struct has_fixed_size<piper_msgs::msg::PosCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<piper_msgs::msg::PosCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<piper_msgs::msg::PosCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PIPER_MSGS__MSG__DETAIL__POS_CMD__TRAITS_HPP_
