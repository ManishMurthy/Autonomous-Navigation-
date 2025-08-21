// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:srv/GetOSMRoadTopologyMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__TRAITS_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/srv/detail/get_osm_road_topology_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vox_nav_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetOSMRoadTopologyMap_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetOSMRoadTopologyMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetOSMRoadTopologyMap_Request & msg, bool use_flow_style = false)
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
  const vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request & msg)
{
  return vox_nav_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>()
{
  return "vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>()
{
  return "vox_nav_msgs/srv/GetOSMRoadTopologyMap_Request";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'osm_road_topology'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace vox_nav_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetOSMRoadTopologyMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: osm_road_topology
  {
    out << "osm_road_topology: ";
    to_flow_style_yaml(msg.osm_road_topology, out);
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
  const GetOSMRoadTopologyMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: osm_road_topology
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "osm_road_topology:\n";
    to_block_style_yaml(msg.osm_road_topology, out, indentation + 2);
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

inline std::string to_yaml(const GetOSMRoadTopologyMap_Response & msg, bool use_flow_style = false)
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
  const vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response & msg)
{
  return vox_nav_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>()
{
  return "vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>()
{
  return "vox_nav_msgs/srv/GetOSMRoadTopologyMap_Response";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct is_message<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vox_nav_msgs::srv::GetOSMRoadTopologyMap>()
{
  return "vox_nav_msgs::srv::GetOSMRoadTopologyMap";
}

template<>
inline const char * name<vox_nav_msgs::srv::GetOSMRoadTopologyMap>()
{
  return "vox_nav_msgs/srv/GetOSMRoadTopologyMap";
}

template<>
struct has_fixed_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap>
  : std::integral_constant<
    bool,
    has_fixed_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>::value &&
    has_fixed_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap>
  : std::integral_constant<
    bool,
    has_bounded_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>::value &&
    has_bounded_size<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>::value
  >
{
};

template<>
struct is_service<vox_nav_msgs::srv::GetOSMRoadTopologyMap>
  : std::true_type
{
};

template<>
struct is_service_request<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__TRAITS_HPP_
