// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from data_msgs:msg/CaptureStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__TRAITS_HPP_
#define DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "data_msgs/msg/detail/capture_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace data_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CaptureStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: topics
  {
    if (msg.topics.size() == 0) {
      out << "topics: []";
    } else {
      out << "topics: [";
      size_t pending_items = msg.topics.size();
      for (auto item : msg.topics) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: count_in_seconds
  {
    if (msg.count_in_seconds.size() == 0) {
      out << "count_in_seconds: []";
    } else {
      out << "count_in_seconds: [";
      size_t pending_items = msg.count_in_seconds.size();
      for (auto item : msg.count_in_seconds) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: frequencies
  {
    if (msg.frequencies.size() == 0) {
      out << "frequencies: []";
    } else {
      out << "frequencies: [";
      size_t pending_items = msg.frequencies.size();
      for (auto item : msg.frequencies) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: fail
  {
    out << "fail: ";
    rosidl_generator_traits::value_to_yaml(msg.fail, out);
    out << ", ";
  }

  // member: quit
  {
    out << "quit: ";
    rosidl_generator_traits::value_to_yaml(msg.quit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CaptureStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.topics.size() == 0) {
      out << "topics: []\n";
    } else {
      out << "topics:\n";
      for (auto item : msg.topics) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: count_in_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.count_in_seconds.size() == 0) {
      out << "count_in_seconds: []\n";
    } else {
      out << "count_in_seconds:\n";
      for (auto item : msg.count_in_seconds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: frequencies
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.frequencies.size() == 0) {
      out << "frequencies: []\n";
    } else {
      out << "frequencies:\n";
      for (auto item : msg.frequencies) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: fail
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fail: ";
    rosidl_generator_traits::value_to_yaml(msg.fail, out);
    out << "\n";
  }

  // member: quit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quit: ";
    rosidl_generator_traits::value_to_yaml(msg.quit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CaptureStatus & msg, bool use_flow_style = false)
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
  const data_msgs::msg::CaptureStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  data_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use data_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const data_msgs::msg::CaptureStatus & msg)
{
  return data_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<data_msgs::msg::CaptureStatus>()
{
  return "data_msgs::msg::CaptureStatus";
}

template<>
inline const char * name<data_msgs::msg::CaptureStatus>()
{
  return "data_msgs/msg/CaptureStatus";
}

template<>
struct has_fixed_size<data_msgs::msg::CaptureStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<data_msgs::msg::CaptureStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<data_msgs::msg::CaptureStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DATA_MSGS__MSG__DETAIL__CAPTURE_STATUS__TRAITS_HPP_
