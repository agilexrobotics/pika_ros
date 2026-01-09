// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from piper_msgs:srv/Enable.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__ENABLE__TRAITS_HPP_
#define PIPER_MSGS__SRV__DETAIL__ENABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "piper_msgs/srv/detail/enable__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace piper_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Enable_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable_request
  {
    out << "enable_request: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Enable_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable_request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_request: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Enable_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace piper_msgs

namespace rosidl_generator_traits
{

[[deprecated("use piper_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const piper_msgs::srv::Enable_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  piper_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use piper_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const piper_msgs::srv::Enable_Request & msg)
{
  return piper_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<piper_msgs::srv::Enable_Request>()
{
  return "piper_msgs::srv::Enable_Request";
}

template<>
inline const char * name<piper_msgs::srv::Enable_Request>()
{
  return "piper_msgs/srv/Enable_Request";
}

template<>
struct has_fixed_size<piper_msgs::srv::Enable_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<piper_msgs::srv::Enable_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<piper_msgs::srv::Enable_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace piper_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Enable_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable_response
  {
    out << "enable_response: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Enable_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable_response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_response: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Enable_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace piper_msgs

namespace rosidl_generator_traits
{

[[deprecated("use piper_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const piper_msgs::srv::Enable_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  piper_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use piper_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const piper_msgs::srv::Enable_Response & msg)
{
  return piper_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<piper_msgs::srv::Enable_Response>()
{
  return "piper_msgs::srv::Enable_Response";
}

template<>
inline const char * name<piper_msgs::srv::Enable_Response>()
{
  return "piper_msgs/srv/Enable_Response";
}

template<>
struct has_fixed_size<piper_msgs::srv::Enable_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<piper_msgs::srv::Enable_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<piper_msgs::srv::Enable_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<piper_msgs::srv::Enable>()
{
  return "piper_msgs::srv::Enable";
}

template<>
inline const char * name<piper_msgs::srv::Enable>()
{
  return "piper_msgs/srv/Enable";
}

template<>
struct has_fixed_size<piper_msgs::srv::Enable>
  : std::integral_constant<
    bool,
    has_fixed_size<piper_msgs::srv::Enable_Request>::value &&
    has_fixed_size<piper_msgs::srv::Enable_Response>::value
  >
{
};

template<>
struct has_bounded_size<piper_msgs::srv::Enable>
  : std::integral_constant<
    bool,
    has_bounded_size<piper_msgs::srv::Enable_Request>::value &&
    has_bounded_size<piper_msgs::srv::Enable_Response>::value
  >
{
};

template<>
struct is_service<piper_msgs::srv::Enable>
  : std::true_type
{
};

template<>
struct is_service_request<piper_msgs::srv::Enable_Request>
  : std::true_type
{
};

template<>
struct is_service_response<piper_msgs::srv::Enable_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PIPER_MSGS__SRV__DETAIL__ENABLE__TRAITS_HPP_
