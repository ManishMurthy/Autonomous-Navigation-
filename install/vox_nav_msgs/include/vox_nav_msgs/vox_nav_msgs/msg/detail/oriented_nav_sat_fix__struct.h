// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:msg/OrientedNavSatFix.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__STRUCT_H_
#define VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"

/// Struct defined in msg/OrientedNavSatFix in the package vox_nav_msgs.
typedef struct vox_nav_msgs__msg__OrientedNavSatFix
{
  sensor_msgs__msg__NavSatFix position;
  geometry_msgs__msg__Quaternion orientation;
} vox_nav_msgs__msg__OrientedNavSatFix;

// Struct for a sequence of vox_nav_msgs__msg__OrientedNavSatFix.
typedef struct vox_nav_msgs__msg__OrientedNavSatFix__Sequence
{
  vox_nav_msgs__msg__OrientedNavSatFix * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__msg__OrientedNavSatFix__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__MSG__DETAIL__ORIENTED_NAV_SAT_FIX__STRUCT_H_
