// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vox_nav_msgs:action/FollowPath.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_
#define VOX_NAV_MSGS__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"
// Member 'controller_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_Goal
{
  nav_msgs__msg__Path path;
  rosidl_runtime_c__String controller_id;
} vox_nav_msgs__action__FollowPath_Goal;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_Goal.
typedef struct vox_nav_msgs__action__FollowPath_Goal__Sequence
{
  vox_nav_msgs__action__FollowPath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'total_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_Result
{
  builtin_interfaces__msg__Duration total_time;
} vox_nav_msgs__action__FollowPath_Result;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_Result.
typedef struct vox_nav_msgs__action__FollowPath_Result__Sequence
{
  vox_nav_msgs__action__FollowPath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'elapsed_time'
// already included above
// #include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_Feedback
{
  float distance_to_goal;
  float speed;
  builtin_interfaces__msg__Duration elapsed_time;
} vox_nav_msgs__action__FollowPath_Feedback;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_Feedback.
typedef struct vox_nav_msgs__action__FollowPath_Feedback__Sequence
{
  vox_nav_msgs__action__FollowPath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "vox_nav_msgs/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  vox_nav_msgs__action__FollowPath_Goal goal;
} vox_nav_msgs__action__FollowPath_SendGoal_Request;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_SendGoal_Request.
typedef struct vox_nav_msgs__action__FollowPath_SendGoal_Request__Sequence
{
  vox_nav_msgs__action__FollowPath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} vox_nav_msgs__action__FollowPath_SendGoal_Response;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_SendGoal_Response.
typedef struct vox_nav_msgs__action__FollowPath_SendGoal_Response__Sequence
{
  vox_nav_msgs__action__FollowPath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} vox_nav_msgs__action__FollowPath_GetResult_Request;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_GetResult_Request.
typedef struct vox_nav_msgs__action__FollowPath_GetResult_Request__Sequence
{
  vox_nav_msgs__action__FollowPath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "vox_nav_msgs/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_GetResult_Response
{
  int8_t status;
  vox_nav_msgs__action__FollowPath_Result result;
} vox_nav_msgs__action__FollowPath_GetResult_Response;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_GetResult_Response.
typedef struct vox_nav_msgs__action__FollowPath_GetResult_Response__Sequence
{
  vox_nav_msgs__action__FollowPath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "vox_nav_msgs/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package vox_nav_msgs.
typedef struct vox_nav_msgs__action__FollowPath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  vox_nav_msgs__action__FollowPath_Feedback feedback;
} vox_nav_msgs__action__FollowPath_FeedbackMessage;

// Struct for a sequence of vox_nav_msgs__action__FollowPath_FeedbackMessage.
typedef struct vox_nav_msgs__action__FollowPath_FeedbackMessage__Sequence
{
  vox_nav_msgs__action__FollowPath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vox_nav_msgs__action__FollowPath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_
