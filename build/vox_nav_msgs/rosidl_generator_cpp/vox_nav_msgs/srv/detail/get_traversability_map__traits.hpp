// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__TRAITS_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/srv/detail/get_traversability_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vox_nav_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTraversabilityMap_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTraversabilityMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTraversabilityMap_Request & msg, bool use_flow_style = false)
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
  const vox_nav_msgs::srv::GetTraversabilityMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::srv::GetTraversabilityMap_Request & msg)
{
  return vox_nav_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::srv::GetTraversabilityMap_Request>()
{
  return "vox_nav_msgs::srv::GetTraversabilityMap_Request";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetTraversabilityMap_Request>()
{
  return "vox_nav_msgs/srv/GetTraversabilityMap_Request";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetTraversabilityMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetTraversabilityMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vox_nav_msgs::srv::GetTraversabilityMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'original_octomap'
// Member 'collision_octomap'
// Member 'elevated_surfel_octomap'
#include "octomap_msgs/msg/detail/octomap__traits.hpp"
// Member 'elevated_surfel_poses'
#include "geometry_msgs/msg/detail/pose_array__traits.hpp"
// Member 'traversable_elevated_cloud'
// Member 'traversable_cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace vox_nav_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTraversabilityMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: original_octomap
  {
    out << "original_octomap: ";
    to_flow_style_yaml(msg.original_octomap, out);
    out << ", ";
  }

  // member: collision_octomap
  {
    out << "collision_octomap: ";
    to_flow_style_yaml(msg.collision_octomap, out);
    out << ", ";
  }

  // member: elevated_surfel_octomap
  {
    out << "elevated_surfel_octomap: ";
    to_flow_style_yaml(msg.elevated_surfel_octomap, out);
    out << ", ";
  }

  // member: elevated_surfel_poses
  {
    out << "elevated_surfel_poses: ";
    to_flow_style_yaml(msg.elevated_surfel_poses, out);
    out << ", ";
  }

  // member: traversable_elevated_cloud
  {
    out << "traversable_elevated_cloud: ";
    to_flow_style_yaml(msg.traversable_elevated_cloud, out);
    out << ", ";
  }

  // member: traversable_cloud
  {
    out << "traversable_cloud: ";
    to_flow_style_yaml(msg.traversable_cloud, out);
    out << ", ";
  }

  // member: is_valid
  {
    out << "is_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.is_valid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTraversabilityMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: original_octomap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "original_octomap:\n";
    to_block_style_yaml(msg.original_octomap, out, indentation + 2);
  }

  // member: collision_octomap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision_octomap:\n";
    to_block_style_yaml(msg.collision_octomap, out, indentation + 2);
  }

  // member: elevated_surfel_octomap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elevated_surfel_octomap:\n";
    to_block_style_yaml(msg.elevated_surfel_octomap, out, indentation + 2);
  }

  // member: elevated_surfel_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elevated_surfel_poses:\n";
    to_block_style_yaml(msg.elevated_surfel_poses, out, indentation + 2);
  }

  // member: traversable_elevated_cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traversable_elevated_cloud:\n";
    to_block_style_yaml(msg.traversable_elevated_cloud, out, indentation + 2);
  }

  // member: traversable_cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traversable_cloud:\n";
    to_block_style_yaml(msg.traversable_cloud, out, indentation + 2);
  }

  // member: is_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.is_valid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTraversabilityMap_Response & msg, bool use_flow_style = false)
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
  const vox_nav_msgs::srv::GetTraversabilityMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
{
  return vox_nav_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::srv::GetTraversabilityMap_Response>()
{
  return "vox_nav_msgs::srv::GetTraversabilityMap_Response";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetTraversabilityMap_Response>()
{
  return "vox_nav_msgs/srv/GetTraversabilityMap_Response";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetTraversabilityMap_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseArray>::value && has_fixed_size<octomap_msgs::msg::Octomap>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetTraversabilityMap_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseArray>::value && has_bounded_size<octomap_msgs::msg::Octomap>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct is_message<vox_nav_msgs::srv::GetTraversabilityMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vox_nav_msgs::srv::GetTraversabilityMap>()
{
  return "vox_nav_msgs::srv::GetTraversabilityMap";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetTraversabilityMap>()
{
  return "vox_nav_msgs/srv/GetTraversabilityMap";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetTraversabilityMap>
  : std::integral_constant<
    bool,
    has_fixed_size<vox_nav_msgs::srv::GetTraversabilityMap_Request>::value &&
    has_fixed_size<vox_nav_msgs::srv::GetTraversabilityMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetTraversabilityMap>
  : std::integral_constant<
    bool,
    has_bounded_size<vox_nav_msgs::srv::GetTraversabilityMap_Request>::value &&
    has_bounded_size<vox_nav_msgs::srv::GetTraversabilityMap_Response>::value
  >
{
};

template<>
struct is_service<vox_nav_msgs::srv::GetTraversabilityMap>
  : std::true_type
{
};

template<>
struct is_service_request<vox_nav_msgs::srv::GetTraversabilityMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vox_nav_msgs::srv::GetTraversabilityMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__TRAITS_HPP_
