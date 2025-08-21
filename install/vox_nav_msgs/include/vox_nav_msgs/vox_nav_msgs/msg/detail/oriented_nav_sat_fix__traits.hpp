// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:msg/OrientedNavSatFix.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__TRAITS_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/msg/detail/oriented_nav_sat_fix__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "sensor_msgs/msg/detail/nav_sat_fix__traits.hpp"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace vox_nav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const OrientedNavSatFix & msg,
  std::ostream & out)
{
  out << "{";
  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: orientation
  {
    out << "orientation: ";
    to_flow_style_yaml(msg.orientation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OrientedNavSatFix & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orientation:\n";
    to_block_style_yaml(msg.orientation, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OrientedNavSatFix & msg, bool use_flow_style = false)
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

}  // namespace vox_nav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use vox_nav_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vox_nav_msgs::msg::OrientedNavSatFix & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::msg::OrientedNavSatFix & msg)
{
  return vox_nav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::msg::OrientedNavSatFix>()
{
  return "vox_nav_msgs::msg::OrientedNavSatFix";
}

template<>
inline const char * name<vox_nav_msgs::msg::OrientedNavSatFix>()
{
  return "vox_nav_msgs/msg/OrientedNavSatFix";
}

template<>
struct has_fixed_size<vox_nav_msgs::msg::OrientedNavSatFix>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<sensor_msgs::msg::NavSatFix>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::msg::OrientedNavSatFix>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<sensor_msgs::msg::NavSatFix>::value> {};

template<>
struct is_message<vox_nav_msgs::msg::OrientedNavSatFix>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__TRAITS_HPP_
