// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:srv/GetOSMRoadTopologyMap.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__STRUCT_H_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetOSMRoadTopologyMap in the package vox_nav_msgs.
typedef struct vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request
{
  uint8_t structure_needs_at_least_one_member;
} vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request;

// Struct for a sequence of vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request.
typedef struct vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request__Sequence
{
  vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__srv__GetOSMRoadTopologyMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'osm_road_topology'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in srv/GetOSMRoadTopologyMap in the package vox_nav_msgs.
typedef struct vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response
{
  sensor_msgs__msg__PointCloud2 osm_road_topology;
  bool is_valid;
} vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response;

// Struct for a sequence of vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response.
typedef struct vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response__Sequence
{
  vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__srv__GetOSMRoadTopologyMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_OSM_ROAD_TOPOLOGY_MAP__STRUCT_H_
