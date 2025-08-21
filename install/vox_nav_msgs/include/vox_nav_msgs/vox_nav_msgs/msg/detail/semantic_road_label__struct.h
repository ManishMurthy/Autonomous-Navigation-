// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:msg/SemanticRoadLabel.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__STRUCT_H_
#define VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAJOR_ROADS_WHITE'.
static const char * const vox_nav_msgs__msg__SemanticRoadLabel__MAJOR_ROADS_WHITE = "255255255";

/// Constant 'MISC_ORANGE'.
static const char * const vox_nav_msgs__msg__SemanticRoadLabel__MISC_ORANGE = "255127000";

/// Constant 'MINOR_ROAD_MAROON'.
static const char * const vox_nav_msgs__msg__SemanticRoadLabel__MINOR_ROAD_MAROON = "127000000";

/// Constant 'FOOTWAY_OLIVE'.
static const char * const vox_nav_msgs__msg__SemanticRoadLabel__FOOTWAY_OLIVE = "127127000";

/// Constant 'TRACK_NAVY'.
static const char * const vox_nav_msgs__msg__SemanticRoadLabel__TRACK_NAVY = "000000127";

/// Struct defined in msg/SemanticRoadLabel in the package vox_nav_msgs.
typedef struct vox_nav_msgs__msg__SemanticRoadLabel
{
  uint8_t structure_needs_at_least_one_member;
} vox_nav_msgs__msg__SemanticRoadLabel;

// Struct for a sequence of vox_nav_msgs__msg__SemanticRoadLabel.
typedef struct vox_nav_msgs__msg__SemanticRoadLabel__Sequence
{
  vox_nav_msgs__msg__SemanticRoadLabel * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__msg__SemanticRoadLabel__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__MSG__DETAIL__SEMANTIC_ROAD_LABEL__STRUCT_H_
