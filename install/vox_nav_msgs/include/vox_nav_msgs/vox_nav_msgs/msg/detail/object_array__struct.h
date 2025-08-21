// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_
#define VOX_NAV_MSGS__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'objects'
#include "vox_nav_msgs/msg/detail/object__struct.h"

/// Struct defined in msg/ObjectArray in the package vox_nav_msgs.
typedef struct vox_nav_msgs__msg__ObjectArray
{
  std_msgs__msg__Header header;
  vox_nav_msgs__msg__Object__Sequence objects;
} vox_nav_msgs__msg__ObjectArray;

// Struct for a sequence of vox_nav_msgs__msg__ObjectArray.
typedef struct vox_nav_msgs__msg__ObjectArray__Sequence
{
  vox_nav_msgs__msg__ObjectArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__msg__ObjectArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_
