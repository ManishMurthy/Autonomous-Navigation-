// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_
#define VOX_NAV_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OBJECT_DETECTED'.
enum
{
  vox_nav_msgs__msg__Object__OBJECT_DETECTED = 0
};

/// Constant 'OBJECT_TRACKED'.
enum
{
  vox_nav_msgs__msg__Object__OBJECT_TRACKED = 1
};

/// Constant 'CLASSIFICATION_UNKNOWN'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_UNKNOWN = 0
};

/// Constant 'CLASSIFICATION_UNKNOWN_SMALL'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_UNKNOWN_SMALL = 1
};

/// Constant 'CLASSIFICATION_UNKNOWN_MEDIUM'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_UNKNOWN_MEDIUM = 2
};

/// Constant 'CLASSIFICATION_UNKNOWN_BIG'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_UNKNOWN_BIG = 3
};

/// Constant 'CLASSIFICATION_PEDESTRIAN'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_PEDESTRIAN = 4
};

/// Constant 'CLASSIFICATION_BIKE'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_BIKE = 5
};

/// Constant 'CLASSIFICATION_CAR'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_CAR = 6
};

/// Constant 'CLASSIFICATION_TRUCK'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_TRUCK = 7
};

/// Constant 'CLASSIFICATION_MOTORCYCLE'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_MOTORCYCLE = 8
};

/// Constant 'CLASSIFICATION_OTHER_VEHICLE'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_OTHER_VEHICLE = 9
};

/// Constant 'CLASSIFICATION_BARRIER'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_BARRIER = 10
};

/// Constant 'CLASSIFICATION_SIGN'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_SIGN = 11
};

/// Constant 'CLASSIFICATION_BUS'.
enum
{
  vox_nav_msgs__msg__Object__CLASSIFICATION_BUS = 12
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'accel'
#include "geometry_msgs/msg/detail/accel__struct.h"
// Member 'polygon'
#include "geometry_msgs/msg/detail/polygon__struct.h"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__struct.h"
// Member 'cluster'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in msg/Object in the package vox_nav_msgs.
typedef struct vox_nav_msgs__msg__Object
{
  /// This represents a detected or tracked object with reference coordinate frame and timestamp.
  std_msgs__msg__Header header;
  /// The id of the object (presumably from the detecting sensor).
  uint32_t id;
  /// A Detected object is one which has been seen in at least one scan/frame of a sensor.
  /// A Tracked object is one which has been correlated over multiple scans/frames of a sensor.
  /// An object which is detected can only be assumed to have valid pose and shape properties.
  /// An object which is tracked should also be assumed to have valid twist and accel properties.
  uint8_t detection_level;
  /// A Classified object is one which has been identified as a certain object type.
  /// Classified objects should have valid classification, classification_certainty, and classification_age properties.
  bool object_classified;
  /// The detected position and orientation of the object.
  geometry_msgs__msg__Pose pose;
  /// The detected linear and angular velocities of the object.
  geometry_msgs__msg__Twist twist;
  /// The detected linear and angular accelerations of the object.
  geometry_msgs__msg__Accel accel;
  float velocity;
  float heading;
  /// (OPTIONAL) The polygon defining the detection points at the outer edges of the object.
  geometry_msgs__msg__Polygon polygon;
  /// A shape conforming to the outer bounding edges of the object.
  shape_msgs__msg__SolidPrimitive shape;
  /// The type of classification given to this object.
  uint8_t classification_label;
  /// The certainty of the classification from the originating sensor.
  /// Higher value indicates greater certainty (MAX=255).
  /// It is recommended that a native sensor value be scaled to 0-255 for interoperability.
  float classification_probability;
  /// The number of scans/frames from the sensor that this object has been classified as the current classification.
  uint32_t classification_age;
  bool is_dynamic;
  sensor_msgs__msg__PointCloud2 cluster;
} vox_nav_msgs__msg__Object;

// Struct for a sequence of vox_nav_msgs__msg__Object.
typedef struct vox_nav_msgs__msg__Object__Sequence
{
  vox_nav_msgs__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__MSG__DETAIL__OBJECT__STRUCT_H_
