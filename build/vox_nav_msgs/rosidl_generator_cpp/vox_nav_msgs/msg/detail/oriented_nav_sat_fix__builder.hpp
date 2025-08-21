// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:msg/OrientedNavSatFix.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__BUILDER_HPP_
#define VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/msg/detail/oriented_nav_sat_fix__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vox_nav_msgs
{

namespace msg
{

namespace builder
{

class Init_OrientedNavSatFix_orientation
{
public:
  explicit Init_OrientedNavSatFix_orientation(::vox_nav_msgs::msg::OrientedNavSatFix & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::msg::OrientedNavSatFix orientation(::vox_nav_msgs::msg::OrientedNavSatFix::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::msg::OrientedNavSatFix msg_;
};

class Init_OrientedNavSatFix_position
{
public:
  Init_OrientedNavSatFix_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrientedNavSatFix_orientation position(::vox_nav_msgs::msg::OrientedNavSatFix::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_OrientedNavSatFix_orientation(msg_);
  }

private:
  ::vox_nav_msgs::msg::OrientedNavSatFix msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::msg::OrientedNavSatFix>()
{
  return vox_nav_msgs::msg::builder::Init_OrientedNavSatFix_position();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__BUILDER_HPP_
