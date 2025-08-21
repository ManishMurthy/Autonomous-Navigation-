// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__OBJECT__TRAITS_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__OBJECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/msg/detail/object__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'accel'
#include "geometry_msgs/msg/detail/accel__traits.hpp"
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__traits.hpp"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__traits.hpp"
// Member 'cluster'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace vox_nav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Object & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: detection_level
  {
    out << "detection_level: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_level, out);
    out << ", ";
  }

  // member: object_classified
  {
    out << "object_classified: ";
    rosidl_generator_traits::value_to_yaml(msg.object_classified, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
    out << ", ";
  }

  // member: accel
  {
    out << "accel: ";
    to_flow_style_yaml(msg.accel, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: polygon
  {
    out << "polygon: ";
    to_flow_style_yaml(msg.polygon, out);
    out << ", ";
  }

  // member: shape
  {
    out << "shape: ";
    to_flow_style_yaml(msg.shape, out);
    out << ", ";
  }

  // member: classification_label
  {
    out << "classification_label: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_label, out);
    out << ", ";
  }

  // member: classification_probability
  {
    out << "classification_probability: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_probability, out);
    out << ", ";
  }

  // member: classification_age
  {
    out << "classification_age: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_age, out);
    out << ", ";
  }

  // member: is_dynamic
  {
    out << "is_dynamic: ";
    rosidl_generator_traits::value_to_yaml(msg.is_dynamic, out);
    out << ", ";
  }

  // member: cluster
  {
    out << "cluster: ";
    to_flow_style_yaml(msg.cluster, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Object & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: detection_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_level: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_level, out);
    out << "\n";
  }

  // member: object_classified
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_classified: ";
    rosidl_generator_traits::value_to_yaml(msg.object_classified, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }

  // member: accel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel:\n";
    to_block_style_yaml(msg.accel, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: polygon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "polygon:\n";
    to_block_style_yaml(msg.polygon, out, indentation + 2);
  }

  // member: shape
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shape:\n";
    to_block_style_yaml(msg.shape, out, indentation + 2);
  }

  // member: classification_label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "classification_label: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_label, out);
    out << "\n";
  }

  // member: classification_probability
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "classification_probability: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_probability, out);
    out << "\n";
  }

  // member: classification_age
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "classification_age: ";
    rosidl_generator_traits::value_to_yaml(msg.classification_age, out);
    out << "\n";
  }

  // member: is_dynamic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_dynamic: ";
    rosidl_generator_traits::value_to_yaml(msg.is_dynamic, out);
    out << "\n";
  }

  // member: cluster
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cluster:\n";
    to_block_style_yaml(msg.cluster, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Object & msg, bool use_flow_style = false)
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
  const vox_nav_msgs::msg::Object & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::msg::Object & msg)
{
  return vox_nav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::msg::Object>()
{
  return "vox_nav_msgs::msg::Object";
}

template<>
inline const char * name<vox_nav_msgs::msg::Object>()
{
  return "vox_nav_msgs/msg/Object";
}

template<>
struct has_fixed_size<vox_nav_msgs::msg::Object>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Accel>::value && has_fixed_size<geometry_msgs::msg::Polygon>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value && has_fixed_size<shape_msgs::msg::SolidPrimitive>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<vox_nav_msgs::msg::Object>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Accel>::value && has_bounded_size<geometry_msgs::msg::Polygon>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value && has_bounded_size<shape_msgs::msg::SolidPrimitive>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<vox_nav_msgs::msg::Object>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOX_NAV_MSGS__MSG__DETAIL__OBJECT__TRAITS_HPP_
