// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__BUILDER_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/srv/detail/get_traversability_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vox_nav_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::srv::GetTraversabilityMap_Request>()
{
  return ::vox_nav_msgs::srv::GetTraversabilityMap_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTraversabilityMap_Response_is_valid
{
public:
  explicit Init_GetTraversabilityMap_Response_is_valid(::vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response is_valid(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

class Init_GetTraversabilityMap_Response_traversable_cloud
{
public:
  explicit Init_GetTraversabilityMap_Response_traversable_cloud(::vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetTraversabilityMap_Response_is_valid traversable_cloud(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_traversable_cloud_type arg)
  {
    msg_.traversable_cloud = std::move(arg);
    return Init_GetTraversabilityMap_Response_is_valid(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

class Init_GetTraversabilityMap_Response_traversable_elevated_cloud
{
public:
  explicit Init_GetTraversabilityMap_Response_traversable_elevated_cloud(::vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetTraversabilityMap_Response_traversable_cloud traversable_elevated_cloud(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_traversable_elevated_cloud_type arg)
  {
    msg_.traversable_elevated_cloud = std::move(arg);
    return Init_GetTraversabilityMap_Response_traversable_cloud(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

class Init_GetTraversabilityMap_Response_elevated_surfel_poses
{
public:
  explicit Init_GetTraversabilityMap_Response_elevated_surfel_poses(::vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetTraversabilityMap_Response_traversable_elevated_cloud elevated_surfel_poses(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_elevated_surfel_poses_type arg)
  {
    msg_.elevated_surfel_poses = std::move(arg);
    return Init_GetTraversabilityMap_Response_traversable_elevated_cloud(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

class Init_GetTraversabilityMap_Response_elevated_surfel_octomap
{
public:
  explicit Init_GetTraversabilityMap_Response_elevated_surfel_octomap(::vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetTraversabilityMap_Response_elevated_surfel_poses elevated_surfel_octomap(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_elevated_surfel_octomap_type arg)
  {
    msg_.elevated_surfel_octomap = std::move(arg);
    return Init_GetTraversabilityMap_Response_elevated_surfel_poses(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

class Init_GetTraversabilityMap_Response_collision_octomap
{
public:
  explicit Init_GetTraversabilityMap_Response_collision_octomap(::vox_nav_msgs::srv::GetTraversabilityMap_Response & msg)
  : msg_(msg)
  {}
  Init_GetTraversabilityMap_Response_elevated_surfel_octomap collision_octomap(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_collision_octomap_type arg)
  {
    msg_.collision_octomap = std::move(arg);
    return Init_GetTraversabilityMap_Response_elevated_surfel_octomap(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

class Init_GetTraversabilityMap_Response_original_octomap
{
public:
  Init_GetTraversabilityMap_Response_original_octomap()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTraversabilityMap_Response_collision_octomap original_octomap(::vox_nav_msgs::srv::GetTraversabilityMap_Response::_original_octomap_type arg)
  {
    msg_.original_octomap = std::move(arg);
    return Init_GetTraversabilityMap_Response_collision_octomap(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetTraversabilityMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::srv::GetTraversabilityMap_Response>()
{
  return vox_nav_msgs::srv::builder::Init_GetTraversabilityMap_Response_original_octomap();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_TRAVERSABILITY_MAP__BUILDER_HPP_
