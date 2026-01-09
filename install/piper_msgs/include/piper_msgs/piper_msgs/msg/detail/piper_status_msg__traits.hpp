// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__TRAITS_HPP_
#define PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "piper_msgs/msg/detail/piper_status_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace piper_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PiperStatusMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: ctrl_mode
  {
    out << "ctrl_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_mode, out);
    out << ", ";
  }

  // member: arm_status
  {
    out << "arm_status: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_status, out);
    out << ", ";
  }

  // member: mode_feedback
  {
    out << "mode_feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_feedback, out);
    out << ", ";
  }

  // member: teach_status
  {
    out << "teach_status: ";
    rosidl_generator_traits::value_to_yaml(msg.teach_status, out);
    out << ", ";
  }

  // member: motion_status
  {
    out << "motion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_status, out);
    out << ", ";
  }

  // member: trajectory_num
  {
    out << "trajectory_num: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_num, out);
    out << ", ";
  }

  // member: err_code
  {
    out << "err_code: ";
    rosidl_generator_traits::value_to_yaml(msg.err_code, out);
    out << ", ";
  }

  // member: joint_1_angle_limit
  {
    out << "joint_1_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_1_angle_limit, out);
    out << ", ";
  }

  // member: joint_2_angle_limit
  {
    out << "joint_2_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_2_angle_limit, out);
    out << ", ";
  }

  // member: joint_3_angle_limit
  {
    out << "joint_3_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_3_angle_limit, out);
    out << ", ";
  }

  // member: joint_4_angle_limit
  {
    out << "joint_4_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_4_angle_limit, out);
    out << ", ";
  }

  // member: joint_5_angle_limit
  {
    out << "joint_5_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_5_angle_limit, out);
    out << ", ";
  }

  // member: joint_6_angle_limit
  {
    out << "joint_6_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_6_angle_limit, out);
    out << ", ";
  }

  // member: communication_status_joint_1
  {
    out << "communication_status_joint_1: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_1, out);
    out << ", ";
  }

  // member: communication_status_joint_2
  {
    out << "communication_status_joint_2: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_2, out);
    out << ", ";
  }

  // member: communication_status_joint_3
  {
    out << "communication_status_joint_3: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_3, out);
    out << ", ";
  }

  // member: communication_status_joint_4
  {
    out << "communication_status_joint_4: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_4, out);
    out << ", ";
  }

  // member: communication_status_joint_5
  {
    out << "communication_status_joint_5: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_5, out);
    out << ", ";
  }

  // member: communication_status_joint_6
  {
    out << "communication_status_joint_6: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_6, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PiperStatusMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ctrl_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ctrl_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.ctrl_mode, out);
    out << "\n";
  }

  // member: arm_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "arm_status: ";
    rosidl_generator_traits::value_to_yaml(msg.arm_status, out);
    out << "\n";
  }

  // member: mode_feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode_feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_feedback, out);
    out << "\n";
  }

  // member: teach_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "teach_status: ";
    rosidl_generator_traits::value_to_yaml(msg.teach_status, out);
    out << "\n";
  }

  // member: motion_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motion_status: ";
    rosidl_generator_traits::value_to_yaml(msg.motion_status, out);
    out << "\n";
  }

  // member: trajectory_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_num: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_num, out);
    out << "\n";
  }

  // member: err_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_code: ";
    rosidl_generator_traits::value_to_yaml(msg.err_code, out);
    out << "\n";
  }

  // member: joint_1_angle_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_1_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_1_angle_limit, out);
    out << "\n";
  }

  // member: joint_2_angle_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_2_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_2_angle_limit, out);
    out << "\n";
  }

  // member: joint_3_angle_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_3_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_3_angle_limit, out);
    out << "\n";
  }

  // member: joint_4_angle_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_4_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_4_angle_limit, out);
    out << "\n";
  }

  // member: joint_5_angle_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_5_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_5_angle_limit, out);
    out << "\n";
  }

  // member: joint_6_angle_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_6_angle_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_6_angle_limit, out);
    out << "\n";
  }

  // member: communication_status_joint_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_status_joint_1: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_1, out);
    out << "\n";
  }

  // member: communication_status_joint_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_status_joint_2: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_2, out);
    out << "\n";
  }

  // member: communication_status_joint_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_status_joint_3: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_3, out);
    out << "\n";
  }

  // member: communication_status_joint_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_status_joint_4: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_4, out);
    out << "\n";
  }

  // member: communication_status_joint_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_status_joint_5: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_5, out);
    out << "\n";
  }

  // member: communication_status_joint_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "communication_status_joint_6: ";
    rosidl_generator_traits::value_to_yaml(msg.communication_status_joint_6, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PiperStatusMsg & msg, bool use_flow_style = false)
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
  const piper_msgs::msg::PiperStatusMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  piper_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use piper_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const piper_msgs::msg::PiperStatusMsg & msg)
{
  return piper_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<piper_msgs::msg::PiperStatusMsg>()
{
  return "piper_msgs::msg::PiperStatusMsg";
}

template<>
inline const char * name<piper_msgs::msg::PiperStatusMsg>()
{
  return "piper_msgs/msg/PiperStatusMsg";
}

template<>
struct has_fixed_size<piper_msgs::msg::PiperStatusMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<piper_msgs::msg::PiperStatusMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<piper_msgs::msg::PiperStatusMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PIPER_MSGS__MSG__DETAIL__PIPER_STATUS_MSG__TRAITS_HPP_
