// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:srv/GetOctomap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__BUILDER_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/srv/detail/get_octomap__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vox_nav_msgs
{

namespace srv
{

namespace builder
{

class Init_GetOctomap_Request_filename
{
public:
  explicit Init_GetOctomap_Request_filename(::vox_nav_msgs::srv::GetOctomap_Request & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::srv::GetOctomap_Request filename(::vox_nav_msgs::srv::GetOctomap_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Request msg_;
};

class Init_GetOctomap_Request_publish_octomap
{
public:
  explicit Init_GetOctomap_Request_publish_octomap(::vox_nav_msgs::srv::GetOctomap_Request & msg)
  : msg_(msg)
  {}
  Init_GetOctomap_Request_filename publish_octomap(::vox_nav_msgs::srv::GetOctomap_Request::_publish_octomap_type arg)
  {
    msg_.publish_octomap = std::move(arg);
    return Init_GetOctomap_Request_filename(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Request msg_;
};

class Init_GetOctomap_Request_leaf_size
{
public:
  explicit Init_GetOctomap_Request_leaf_size(::vox_nav_msgs::srv::GetOctomap_Request & msg)
  : msg_(msg)
  {}
  Init_GetOctomap_Request_publish_octomap leaf_size(::vox_nav_msgs::srv::GetOctomap_Request::_leaf_size_type arg)
  {
    msg_.leaf_size = std::move(arg);
    return Init_GetOctomap_Request_publish_octomap(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Request msg_;
};

class Init_GetOctomap_Request_bounding_box_lengths
{
public:
  explicit Init_GetOctomap_Request_bounding_box_lengths(::vox_nav_msgs::srv::GetOctomap_Request & msg)
  : msg_(msg)
  {}
  Init_GetOctomap_Request_leaf_size bounding_box_lengths(::vox_nav_msgs::srv::GetOctomap_Request::_bounding_box_lengths_type arg)
  {
    msg_.bounding_box_lengths = std::move(arg);
    return Init_GetOctomap_Request_leaf_size(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Request msg_;
};

class Init_GetOctomap_Request_bounding_box_origin
{
public:
  Init_GetOctomap_Request_bounding_box_origin()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetOctomap_Request_bounding_box_lengths bounding_box_origin(::vox_nav_msgs::srv::GetOctomap_Request::_bounding_box_origin_type arg)
  {
    msg_.bounding_box_origin = std::move(arg);
    return Init_GetOctomap_Request_bounding_box_lengths(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::srv::GetOctomap_Request>()
{
  return vox_nav_msgs::srv::builder::Init_GetOctomap_Request_bounding_box_origin();
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace srv
{

namespace builder
{

class Init_GetOctomap_Response_origin_altitude
{
public:
  explicit Init_GetOctomap_Response_origin_altitude(::vox_nav_msgs::srv::GetOctomap_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::srv::GetOctomap_Response origin_altitude(::vox_nav_msgs::srv::GetOctomap_Response::_origin_altitude_type arg)
  {
    msg_.origin_altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Response msg_;
};

class Init_GetOctomap_Response_origin_longitude
{
public:
  explicit Init_GetOctomap_Response_origin_longitude(::vox_nav_msgs::srv::GetOctomap_Response & msg)
  : msg_(msg)
  {}
  Init_GetOctomap_Response_origin_altitude origin_longitude(::vox_nav_msgs::srv::GetOctomap_Response::_origin_longitude_type arg)
  {
    msg_.origin_longitude = std::move(arg);
    return Init_GetOctomap_Response_origin_altitude(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Response msg_;
};

class Init_GetOctomap_Response_origin_latitude
{
public:
  explicit Init_GetOctomap_Response_origin_latitude(::vox_nav_msgs::srv::GetOctomap_Response & msg)
  : msg_(msg)
  {}
  Init_GetOctomap_Response_origin_longitude origin_latitude(::vox_nav_msgs::srv::GetOctomap_Response::_origin_latitude_type arg)
  {
    msg_.origin_latitude = std::move(arg);
    return Init_GetOctomap_Response_origin_longitude(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Response msg_;
};

class Init_GetOctomap_Response_map
{
public:
  Init_GetOctomap_Response_map()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetOctomap_Response_origin_latitude map(::vox_nav_msgs::srv::GetOctomap_Response::_map_type arg)
  {
    msg_.map = std::move(arg);
    return Init_GetOctomap_Response_origin_latitude(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOctomap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::srv::GetOctomap_Response>()
{
  return vox_nav_msgs::srv::builder::Init_GetOctomap_Response_map();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OCTOMAP__BUILDER_HPP_
