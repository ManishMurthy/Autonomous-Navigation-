// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
// generated code does not contain a copyright notice
#include "vox_nav_msgs/srv/detail/get_traversability_map__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vox_nav_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vox_nav_msgs/srv/detail/get_traversability_map__struct.h"
#include "vox_nav_msgs/srv/detail/get_traversability_map__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _GetTraversabilityMap_Request__ros_msg_type = vox_nav_msgs__srv__GetTraversabilityMap_Request;

static bool _GetTraversabilityMap_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetTraversabilityMap_Request__ros_msg_type * ros_message = static_cast<const _GetTraversabilityMap_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _GetTraversabilityMap_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetTraversabilityMap_Request__ros_msg_type * ros_message = static_cast<_GetTraversabilityMap_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vox_nav_msgs
size_t get_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetTraversabilityMap_Request__ros_msg_type * ros_message = static_cast<const _GetTraversabilityMap_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetTraversabilityMap_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vox_nav_msgs
size_t max_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vox_nav_msgs__srv__GetTraversabilityMap_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetTraversabilityMap_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetTraversabilityMap_Request = {
  "vox_nav_msgs::srv",
  "GetTraversabilityMap_Request",
  _GetTraversabilityMap_Request__cdr_serialize,
  _GetTraversabilityMap_Request__cdr_deserialize,
  _GetTraversabilityMap_Request__get_serialized_size,
  _GetTraversabilityMap_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetTraversabilityMap_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetTraversabilityMap_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vox_nav_msgs, srv, GetTraversabilityMap_Request)() {
  return &_GetTraversabilityMap_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "vox_nav_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_traversability_map__struct.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_traversability_map__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/pose_array__functions.h"  // elevated_surfel_poses
#include "octomap_msgs/msg/detail/octomap__functions.h"  // collision_octomap, elevated_surfel_octomap, original_octomap
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"  // traversable_cloud, traversable_elevated_cloud

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
size_t get_serialized_size_geometry_msgs__msg__PoseArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
size_t max_serialized_size_geometry_msgs__msg__PoseArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseArray)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
size_t get_serialized_size_octomap_msgs__msg__Octomap(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
size_t max_serialized_size_octomap_msgs__msg__Octomap(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
size_t get_serialized_size_sensor_msgs__msg__PointCloud2(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
size_t max_serialized_size_sensor_msgs__msg__PointCloud2(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vox_nav_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2)();


using _GetTraversabilityMap_Response__ros_msg_type = vox_nav_msgs__srv__GetTraversabilityMap_Response;

static bool _GetTraversabilityMap_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetTraversabilityMap_Response__ros_msg_type * ros_message = static_cast<const _GetTraversabilityMap_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: original_octomap
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->original_octomap, cdr))
    {
      return false;
    }
  }

  // Field name: collision_octomap
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->collision_octomap, cdr))
    {
      return false;
    }
  }

  // Field name: elevated_surfel_octomap
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->elevated_surfel_octomap, cdr))
    {
      return false;
    }
  }

  // Field name: elevated_surfel_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseArray
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->elevated_surfel_poses, cdr))
    {
      return false;
    }
  }

  // Field name: traversable_elevated_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->traversable_elevated_cloud, cdr))
    {
      return false;
    }
  }

  // Field name: traversable_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->traversable_cloud, cdr))
    {
      return false;
    }
  }

  // Field name: is_valid
  {
    cdr << (ros_message->is_valid ? true : false);
  }

  return true;
}

static bool _GetTraversabilityMap_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetTraversabilityMap_Response__ros_msg_type * ros_message = static_cast<_GetTraversabilityMap_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: original_octomap
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->original_octomap))
    {
      return false;
    }
  }

  // Field name: collision_octomap
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->collision_octomap))
    {
      return false;
    }
  }

  // Field name: elevated_surfel_octomap
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, octomap_msgs, msg, Octomap
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->elevated_surfel_octomap))
    {
      return false;
    }
  }

  // Field name: elevated_surfel_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseArray
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->elevated_surfel_poses))
    {
      return false;
    }
  }

  // Field name: traversable_elevated_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->traversable_elevated_cloud))
    {
      return false;
    }
  }

  // Field name: traversable_cloud
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->traversable_cloud))
    {
      return false;
    }
  }

  // Field name: is_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_valid = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vox_nav_msgs
size_t get_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetTraversabilityMap_Response__ros_msg_type * ros_message = static_cast<const _GetTraversabilityMap_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name original_octomap

  current_alignment += get_serialized_size_octomap_msgs__msg__Octomap(
    &(ros_message->original_octomap), current_alignment);
  // field.name collision_octomap

  current_alignment += get_serialized_size_octomap_msgs__msg__Octomap(
    &(ros_message->collision_octomap), current_alignment);
  // field.name elevated_surfel_octomap

  current_alignment += get_serialized_size_octomap_msgs__msg__Octomap(
    &(ros_message->elevated_surfel_octomap), current_alignment);
  // field.name elevated_surfel_poses

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseArray(
    &(ros_message->elevated_surfel_poses), current_alignment);
  // field.name traversable_elevated_cloud

  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->traversable_elevated_cloud), current_alignment);
  // field.name traversable_cloud

  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->traversable_cloud), current_alignment);
  // field.name is_valid
  {
    size_t item_size = sizeof(ros_message->is_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetTraversabilityMap_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vox_nav_msgs
size_t max_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: original_octomap
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_octomap_msgs__msg__Octomap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: collision_octomap
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_octomap_msgs__msg__Octomap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: elevated_surfel_octomap
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_octomap_msgs__msg__Octomap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: elevated_surfel_poses
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: traversable_elevated_cloud
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: traversable_cloud
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: is_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vox_nav_msgs__srv__GetTraversabilityMap_Response;
    is_plain =
      (
      offsetof(DataType, is_valid) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetTraversabilityMap_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vox_nav_msgs__srv__GetTraversabilityMap_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetTraversabilityMap_Response = {
  "vox_nav_msgs::srv",
  "GetTraversabilityMap_Response",
  _GetTraversabilityMap_Response__cdr_serialize,
  _GetTraversabilityMap_Response__cdr_deserialize,
  _GetTraversabilityMap_Response__get_serialized_size,
  _GetTraversabilityMap_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetTraversabilityMap_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetTraversabilityMap_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vox_nav_msgs, srv, GetTraversabilityMap_Response)() {
  return &_GetTraversabilityMap_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "vox_nav_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vox_nav_msgs/srv/get_traversability_map.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetTraversabilityMap__callbacks = {
  "vox_nav_msgs::srv",
  "GetTraversabilityMap",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vox_nav_msgs, srv, GetTraversabilityMap_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vox_nav_msgs, srv, GetTraversabilityMap_Response)(),
};

static rosidl_service_type_support_t GetTraversabilityMap__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetTraversabilityMap__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vox_nav_msgs, srv, GetTraversabilityMap)() {
  return &GetTraversabilityMap__handle;
}

#if defined(__cplusplus)
}
#endif
