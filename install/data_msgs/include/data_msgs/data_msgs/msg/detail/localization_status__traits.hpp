// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from data_msgs:msg/LocalizationStatus.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__TRAITS_HPP_
#define DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "data_msgs/msg/detail/localization_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace data_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalizationStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: accurate
  {
    out << "accurate: ";
    rosidl_generator_traits::value_to_yaml(msg.accurate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalizationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accurate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accurate: ";
    rosidl_generator_traits::value_to_yaml(msg.accurate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalizationStatus & msg, bool use_flow_style = false)
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
  const data_msgs::msg::LocalizationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  data_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use data_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const data_msgs::msg::LocalizationStatus & msg)
{
  return data_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<data_msgs::msg::LocalizationStatus>()
{
  return "data_msgs::msg::LocalizationStatus";
}

template<>
inline const char * name<data_msgs::msg::LocalizationStatus>()
{
  return "data_msgs/msg/LocalizationStatus";
}

template<>
struct has_fixed_size<data_msgs::msg::LocalizationStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<data_msgs::msg::LocalizationStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<data_msgs::msg::LocalizationStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DATA_MSGS__MSG__DETAIL__LOCALIZATION_STATUS__TRAITS_HPP_
