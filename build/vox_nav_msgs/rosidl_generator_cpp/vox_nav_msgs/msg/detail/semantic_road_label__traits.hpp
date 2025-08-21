// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vox_nav_msgs:msg/SemanticRoadLabel.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__TRAITS_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vox_nav_msgs/msg/detail/semantic_road_label__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vox_nav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SemanticRoadLabel & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SemanticRoadLabel & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SemanticRoadLabel & msg, bool use_flow_style = false)
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
  const vox_nav_msgs::msg::SemanticRoadLabel & msg,
  std::ostream & out, size_t indentation = 0)
{
  vox_nav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vox_nav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const vox_nav_msgs::msg::SemanticRoadLabel & msg)
{
  return vox_nav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vox_nav_msgs::msg::SemanticRoadLabel>()
{
  return "vox_nav_msgs::msg::SemanticRoadLabel";
}

template<>
inline const char * name<vox_nav_msgs::msg::SemanticRoadLabel>()
{
  return "vox_nav_msgs/msg/SemanticRoadLabel";
}

template<>
struct has_fixed_size<vox_nav_msgs::msg::SemanticRoadLabel>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vox_nav_msgs::msg::SemanticRoadLabel>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vox_nav_msgs::msg::SemanticRoadLabel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__TRAITS_HPP_
