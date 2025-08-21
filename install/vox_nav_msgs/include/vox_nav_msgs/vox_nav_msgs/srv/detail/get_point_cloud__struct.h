// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:srv/GetPointCloud.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_POINT_CLOUD__STRUCT_H_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_POINT_CLOUD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bounding_box_origin'
// Member 'bounding_box_lengths'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'filename'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetPointCloud in the package vox_nav_msgs.
typedef struct vox_nav_msgs__srv__GetPointCloud_Request
{
  geometry_msgs__msg__Point bounding_box_origin;
  /// The 3 side lenghts of the axis-aligned bounding box
  geometry_msgs__msg__Point bounding_box_lengths;
  /// The leaf size or resolution of the octomap
  double leaf_size;
  /// Indicate if the generated octomap should be published.
  bool publish_pointcloud;
  /// The filename under which the octomap should be stored (only stored if set)
  rosidl_runtime_c__String filename;
} vox_nav_msgs__srv__GetPointCloud_Request;

// Struct for a sequence of vox_nav_msgs__srv__GetPointCloud_Request.
typedef struct vox_nav_msgs__srv__GetPointCloud_Request__Sequence
{
  vox_nav_msgs__srv__GetPointCloud_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__srv__GetPointCloud_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in srv/GetPointCloud in the package vox_nav_msgs.
typedef struct vox_nav_msgs__srv__GetPointCloud_Response
{
  sensor_msgs__msg__PointCloud2 cloud;
  /// The latitude of the gazebo coordinates origin
  double origin_latitude;
  /// The longitude of the gazebo coordinates origin
  double origin_longitude;
  /// The altitude of the gazebo coordinates origin
  double origin_altitude;
} vox_nav_msgs__srv__GetPointCloud_Response;

// Struct for a sequence of vox_nav_msgs__srv__GetPointCloud_Response.
typedef struct vox_nav_msgs__srv__GetPointCloud_Response__Sequence
{
  vox_nav_msgs__srv__GetPointCloud_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__srv__GetPointCloud_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_POINT_CLOUD__STRUCT_H_
