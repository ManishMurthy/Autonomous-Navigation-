// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vox_nav_msgs:srv/GetOSMRoadTopologyMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__BUILDER_HPP_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vox_nav_msgs/srv/detail/get_osm_road_topology_map__struct.hpp"
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
auto build<::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request>()
{
  return ::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace vox_nav_msgs


namespace vox_nav_msgs
{

namespace srv
{

namespace builder
{

class Init_GetOSMRoadTopologyMap_Response_is_valid
{
public:
  explicit Init_GetOSMRoadTopologyMap_Response_is_valid(::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response & msg)
  : msg_(msg)
  {}
  ::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response is_valid(::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response::_is_valid_type arg)
  {
    msg_.is_valid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response msg_;
};

class Init_GetOSMRoadTopologyMap_Response_osm_road_topology
{
public:
  Init_GetOSMRoadTopologyMap_Response_osm_road_topology()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetOSMRoadTopologyMap_Response_is_valid osm_road_topology(::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response::_osm_road_topology_type arg)
  {
    msg_.osm_road_topology = std::move(arg);
    return Init_GetOSMRoadTopologyMap_Response_is_valid(msg_);
  }

private:
  ::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::vox_nav_msgs::srv::GetOSMRoadTopologyMap_Response>()
{
  return vox_nav_msgs::srv::builder::Init_GetOSMRoadTopologyMap_Response_osm_road_topology();
}

}  // namespace vox_nav_msgs

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__BUILDER_HPP_
