// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:srv/GetOctomap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__TRAITS_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/srv/detail/get_octomap__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'bounding_box_origin'
// Member 'bounding_box_lengths'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace vox_nav_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetOctomap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: bounding_box_origin
  {
    out << "bounding_box_origin: ";
    to_flow_style_yaml(msg.bounding_box_origin, out);
    out << ", ";
  }

  // member: bounding_box_lengths
  {
    out << "bounding_box_lengths: ";
    to_flow_style_yaml(msg.bounding_box_lengths, out);
    out << ", ";
  }

  // member: leaf_size
  {
    out << "leaf_size: ";
    rosidl_generator_traits::value_to_yaml(msg.leaf_size, out);
    out << ", ";
  }

  // member: publish_octomap
  {
    out << "publish_octomap: ";
    rosidl_generator_traits::value_to_yaml(msg.publish_octomap, out);
    out << ", ";
  }

  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetOctomap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bounding_box_origin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bounding_box_origin:\n";
    to_block_style_yaml(msg.bounding_box_origin, out, indentation + 2);
  }

  // member: bounding_box_lengths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bounding_box_lengths:\n";
    to_block_style_yaml(msg.bounding_box_lengths, out, indentation + 2);
  }

  // member: leaf_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "leaf_size: ";
    rosidl_generator_traits::value_to_yaml(msg.leaf_size, out);
    out << "\n";
  }

  // member: publish_octomap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "publish_octomap: ";
    rosidl_generator_traits::value_to_yaml(msg.publish_octomap, out);
    out << "\n";
  }

  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetOctomap_Request & msg, bool use_flow_style = false)
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

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::srv::GetOctomap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::srv::GetOctomap_Request & msg)
{
  return vox_nav_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::srv::GetOctomap_Request>()
{
  return "vox_nav_msgs::srv::GetOctomap_Request";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetOctomap_Request>()
{
  return "vox_nav_msgs/srv/GetOctomap_Request";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetOctomap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetOctomap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vox_nav_msgs::srv::GetOctomap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'map'
#include "octomap_msgs/msg/detail/octomap__traits.hpp"

namespace vox_nav_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetOctomap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: map
  {
    out << "map: ";
    to_flow_style_yaml(msg.map, out);
    out << ", ";
  }

  // member: origin_latitude
  {
    out << "origin_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_latitude, out);
    out << ", ";
  }

  // member: origin_longitude
  {
    out << "origin_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_longitude, out);
    out << ", ";
  }

  // member: origin_altitude
  {
    out << "origin_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_altitude, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetOctomap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map:\n";
    to_block_style_yaml(msg.map, out, indentation + 2);
  }

  // member: origin_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_latitude, out);
    out << "\n";
  }

  // member: origin_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_longitude, out);
    out << "\n";
  }

  // member: origin_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_altitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetOctomap_Response & msg, bool use_flow_style = false)
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

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::srv::GetOctomap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::srv::GetOctomap_Response & msg)
{
  return vox_nav_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::srv::GetOctomap_Response>()
{
  return "vox_nav_msgs::srv::GetOctomap_Response";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetOctomap_Response>()
{
  return "vox_nav_msgs/srv/GetOctomap_Response";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetOctomap_Response>
  : std::integral_constant<bool, has_fixed_size<octomap_msgs::msg::Octomap>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetOctomap_Response>
  : std::integral_constant<bool, has_bounded_size<octomap_msgs::msg::Octomap>::value> {};

template<>
struct is_message<vox_nav_msgs::srv::GetOctomap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vox_nav_msgs::srv::GetOctomap>()
{
  return "vox_nav_msgs::srv::GetOctomap";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetOctomap>()
{
  return "vox_nav_msgs/srv/GetOctomap";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetOctomap>
  : std::integral_constant<
    bool,
    has_fixed_size<vox_nav_msgs::srv::GetOctomap_Request>::value &&
    has_fixed_size<vox_nav_msgs::srv::GetOctomap_Response>::value
  >
{
};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetOctomap>
  : std::integral_constant<
    bool,
    has_bounded_size<vox_nav_msgs::srv::GetOctomap_Request>::value &&
    has_bounded_size<vox_nav_msgs::srv::GetOctomap_Response>::value
  >
{
};

template<>
struct is_service<vox_nav_msgs::srv::GetOctomap>
  : std::true_type
{
};

template<>
struct is_service_request<vox_nav_msgs::srv::GetOctomap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vox_nav_msgs::srv::GetOctomap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__TRAITS_HPP_
