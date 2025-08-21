// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:action/NavigateThroughPoses.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_POSES__STRUCT_H_
#define VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_POSES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'poses'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'behavior_tree'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_Goal
{
  geometry_msgs__msg__PoseStamped__Sequence poses;
  rosidl_runtime_c__String behavior_tree;
} vox_nav_msgs__action__NavigateThroughPoses_Goal;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_Goal.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_Goal__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
#include "std_msgs/msg/detail/empty__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_Result
{
  std_msgs__msg__Empty result;
} vox_nav_msgs__action__NavigateThroughPoses_Result;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_Result.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_Result__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'navigation_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_Feedback
{
  geometry_msgs__msg__PoseStamped current_pose;
  builtin_interfaces__msg__Duration navigation_time;
  int16_t number_of_recoveries;
  float distance_remaining;
} vox_nav_msgs__action__NavigateThroughPoses_Feedback;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_Feedback.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_Feedback__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "vox_nav_msgs/action/detail/navigate_through_poses__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  vox_nav_msgs__action__NavigateThroughPoses_Goal goal;
} vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Request;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Request.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Response;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Response.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} vox_nav_msgs__action__NavigateThroughPoses_GetResult_Request;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_GetResult_Request.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "vox_nav_msgs/action/detail/navigate_through_poses__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_GetResult_Response
{
  int8_t status;
  vox_nav_msgs__action__NavigateThroughPoses_Result result;
} vox_nav_msgs__action__NavigateThroughPoses_GetResult_Response;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_GetResult_Response.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "vox_nav_msgs/action/detail/navigate_through_poses__struct.h"

/// Struct defined in action/NavigateThroughPoses in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  vox_nav_msgs__action__NavigateThroughPoses_Feedback feedback;
} vox_nav_msgs__action__NavigateThroughPoses_FeedbackMessage;

// Struct for a sequence of vox_nav_msgs__action__NavigateThroughPoses_FeedbackMessage.
typedef struct vox_nav_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence
{
  vox_nav_msgs__action__NavigateThroughPoses_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__NavigateThroughPoses_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__ACTION__DETAIL__NAVIGATE_THROUGH_POSES__STRUCT_H_
