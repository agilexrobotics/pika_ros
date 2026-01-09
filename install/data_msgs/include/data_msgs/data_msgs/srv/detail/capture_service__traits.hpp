// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from data_msgs:srv/CaptureService.idl
// generated code does not contain a copyright notice

#ifndef DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__TRAITS_HPP_
#define DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "data_msgs/srv/detail/capture_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace data_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CaptureService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start
  {
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << ", ";
  }

  // member: end
  {
    out << "end: ";
    rosidl_generator_traits::value_to_yaml(msg.end, out);
    out << ", ";
  }

  // member: episode_index
  {
    out << "episode_index: ";
    rosidl_generator_traits::value_to_yaml(msg.episode_index, out);
    out << ", ";
  }

  // member: dataset_dir
  {
    out << "dataset_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset_dir, out);
    out << ", ";
  }

  // member: instructions
  {
    out << "instructions: ";
    rosidl_generator_traits::value_to_yaml(msg.instructions, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CaptureService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << "\n";
  }

  // member: end
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end: ";
    rosidl_generator_traits::value_to_yaml(msg.end, out);
    out << "\n";
  }

  // member: episode_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "episode_index: ";
    rosidl_generator_traits::value_to_yaml(msg.episode_index, out);
    out << "\n";
  }

  // member: dataset_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dataset_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset_dir, out);
    out << "\n";
  }

  // member: instructions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "instructions: ";
    rosidl_generator_traits::value_to_yaml(msg.instructions, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CaptureService_Request & msg, bool use_flow_style = false)
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

}  // namespace data_msgs

namespace rosidl_generator_traits
{

[[deprecated("use data_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const data_msgs::srv::CaptureService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  data_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use data_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const data_msgs::srv::CaptureService_Request & msg)
{
  return data_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<data_msgs::srv::CaptureService_Request>()
{
  return "data_msgs::srv::CaptureService_Request";
}

template<>
inline const char * name<data_msgs::srv::CaptureService_Request>()
{
  return "data_msgs/srv/CaptureService_Request";
}

template<>
struct has_fixed_size<data_msgs::srv::CaptureService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<data_msgs::srv::CaptureService_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<data_msgs::srv::CaptureService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace data_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const CaptureService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CaptureService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CaptureService_Response & msg, bool use_flow_style = false)
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

}  // namespace data_msgs

namespace rosidl_generator_traits
{

[[deprecated("use data_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const data_msgs::srv::CaptureService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  data_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use data_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const data_msgs::srv::CaptureService_Response & msg)
{
  return data_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<data_msgs::srv::CaptureService_Response>()
{
  return "data_msgs::srv::CaptureService_Response";
}

template<>
inline const char * name<data_msgs::srv::CaptureService_Response>()
{
  return "data_msgs/srv/CaptureService_Response";
}

template<>
struct has_fixed_size<data_msgs::srv::CaptureService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<data_msgs::srv::CaptureService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<data_msgs::srv::CaptureService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<data_msgs::srv::CaptureService>()
{
  return "data_msgs::srv::CaptureService";
}

template<>
inline const char * name<data_msgs::srv::CaptureService>()
{
  return "data_msgs/srv/CaptureService";
}

template<>
struct has_fixed_size<data_msgs::srv::CaptureService>
  : std::integral_constant<
    bool,
    has_fixed_size<data_msgs::srv::CaptureService_Request>::value &&
    has_fixed_size<data_msgs::srv::CaptureService_Response>::value
  >
{
};

template<>
struct has_bounded_size<data_msgs::srv::CaptureService>
  : std::integral_constant<
    bool,
    has_bounded_size<data_msgs::srv::CaptureService_Request>::value &&
    has_bounded_size<data_msgs::srv::CaptureService_Response>::value
  >
{
};

template<>
struct is_service<data_msgs::srv::CaptureService>
  : std::true_type
{
};

template<>
struct is_service_request<data_msgs::srv::CaptureService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<data_msgs::srv::CaptureService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DATA_MSGS__SRV__DETAIL__CAPTURE_SERVICE__TRAITS_HPP_
