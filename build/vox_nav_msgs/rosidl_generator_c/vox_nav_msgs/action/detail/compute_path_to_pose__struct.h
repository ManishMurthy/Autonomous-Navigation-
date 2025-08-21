// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:action/ComputePathToPose.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__ACTION__DETAIL__COMPUTE_PATH_TO_POSE__STRUCT_H_
#define VOX_NAV_MSGS__ACTION__DETAIL__COMPUTE_PATH_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'planner_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_Goal
{
  geometry_msgs__msg__PoseStamped pose;
  rosidl_runtime_c__String planner_id;
} vox_nav_msgs__action__ComputePathToPose_Goal;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_Goal.
typedef struct vox_nav_msgs__action__ComputePathToPose_Goal__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"
// Member 'planning_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_Result
{
  nav_msgs__msg__Path path;
  builtin_interfaces__msg__Duration planning_time;
} vox_nav_msgs__action__ComputePathToPose_Result;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_Result.
typedef struct vox_nav_msgs__action__ComputePathToPose_Result__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'elapsed_time'
// already included above
// #include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_Feedback
{
  builtin_interfaces__msg__Duration elapsed_time;
} vox_nav_msgs__action__ComputePathToPose_Feedback;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_Feedback.
typedef struct vox_nav_msgs__action__ComputePathToPose_Feedback__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "vox_nav_msgs/action/detail/compute_path_to_pose__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  vox_nav_msgs__action__ComputePathToPose_Goal goal;
} vox_nav_msgs__action__ComputePathToPose_SendGoal_Request;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_SendGoal_Request.
typedef struct vox_nav_msgs__action__ComputePathToPose_SendGoal_Request__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} vox_nav_msgs__action__ComputePathToPose_SendGoal_Response;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_SendGoal_Response.
typedef struct vox_nav_msgs__action__ComputePathToPose_SendGoal_Response__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} vox_nav_msgs__action__ComputePathToPose_GetResult_Request;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_GetResult_Request.
typedef struct vox_nav_msgs__action__ComputePathToPose_GetResult_Request__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "vox_nav_msgs/action/detail/compute_path_to_pose__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_GetResult_Response
{
  int8_t status;
  vox_nav_msgs__action__ComputePathToPose_Result result;
} vox_nav_msgs__action__ComputePathToPose_GetResult_Response;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_GetResult_Response.
typedef struct vox_nav_msgs__action__ComputePathToPose_GetResult_Response__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "vox_nav_msgs/action/detail/compute_path_to_pose__struct.h"

/// Struct defined in action/ComputePathToPose in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__ComputePathToPose_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  vox_nav_msgs__action__ComputePathToPose_Feedback feedback;
} vox_nav_msgs__action__ComputePathToPose_FeedbackMessage;

// Struct for a sequence of vox_nav_msgs__action__ComputePathToPose_FeedbackMessage.
typedef struct vox_nav_msgs__action__ComputePathToPose_FeedbackMessage__Sequence
{
  vox_nav_msgs__action__ComputePathToPose_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__ComputePathToPose_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__ACTION__DETAIL__COMPUTE_PATH_TO_POSE__STRUCT_H_
