// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
// generated code does not contain a copyright notice
#include "vox_nav_msgs/srv/detail/get_traversability_map__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vox_nav_msgs/srv/detail/get_traversability_map__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vox_nav_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
cdr_serialize(
  const vox_nav_msgs::srv::GetTraversabilityMap_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: structure_needs_at_least_one_member
  cdr << ros_message.structure_needs_at_least_one_member;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vox_nav_msgs::srv::GetTraversabilityMap_Request & ros_message)
{
  // Member: structure_needs_at_least_one_member
  cdr >> ros_message.structure_needs_at_least_one_member;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
get_serialized_size(
  const vox_nav_msgs::srv::GetTraversabilityMap_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message.structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
max_serialized_size_GetTraversabilityMap_Request(
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


  // Member: structure_needs_at_least_one_member
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
    using DataType = vox_nav_msgs::srv::GetTraversabilityMap_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GetTraversabilityMap_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vox_nav_msgs::srv::GetTraversabilityMap_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GetTraversabilityMap_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vox_nav_msgs::srv::GetTraversabilityMap_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GetTraversabilityMap_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vox_nav_msgs::srv::GetTraversabilityMap_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GetTraversabilityMap_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GetTraversabilityMap_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GetTraversabilityMap_Request__callbacks = {
  "vox_nav_msgs::srv",
  "GetTraversabilityMap_Request",
  _GetTraversabilityMap_Request__cdr_serialize,
  _GetTraversabilityMap_Request__cdr_deserialize,
  _GetTraversabilityMap_Request__get_serialized_size,
  _GetTraversabilityMap_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetTraversabilityMap_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GetTraversabilityMap_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace vox_nav_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vox_nav_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vox_nav_msgs::srv::GetTraversabilityMap_Request>()
{
  return &vox_nav_msgs::srv::typesupport_fastrtps_cpp::_GetTraversabilityMap_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vox_nav_msgs, srv, GetTraversabilityMap_Request)() {
  return &vox_nav_msgs::srv::typesupport_fastrtps_cpp::_GetTraversabilityMap_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace octomap_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const octomap_msgs::msg::Octomap &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  octomap_msgs::msg::Octomap &);
size_t get_serialized_size(
  const octomap_msgs::msg::Octomap &,
  size_t current_alignment);
size_t
max_serialized_size_Octomap(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace octomap_msgs

// functions for octomap_msgs::msg::Octomap already declared above

// functions for octomap_msgs::msg::Octomap already declared above

namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::PoseArray &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::PoseArray &);
size_t get_serialized_size(
  const geometry_msgs::msg::PoseArray &,
  size_t current_alignment);
size_t
max_serialized_size_PoseArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs

namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::PointCloud2 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::PointCloud2 &);
size_t get_serialized_size(
  const sensor_msgs::msg::PointCloud2 &,
  size_t current_alignment);
size_t
max_serialized_size_PointCloud2(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs

// functions for sensor_msgs::msg::PointCloud2 already declared above


namespace vox_nav_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
cdr_serialize(
  const vox_nav_msgs::srv::GetTraversabilityMap_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: original_octomap
  octomap_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.original_octomap,
    cdr);
  // Member: collision_octomap
  octomap_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.collision_octomap,
    cdr);
  // Member: elevated_surfel_octomap
  octomap_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.elevated_surfel_octomap,
    cdr);
  // Member: elevated_surfel_poses
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.elevated_surfel_poses,
    cdr);
  // Member: traversable_elevated_cloud
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.traversable_elevated_cloud,
    cdr);
  // Member: traversable_cloud
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.traversable_cloud,
    cdr);
  // Member: is_valid
  cdr << (ros_message.is_valid ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vox_nav_msgs::srv::GetTraversabilityMap_Response & ros_message)
{
  // Member: original_octomap
  octomap_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.original_octomap);

  // Member: collision_octomap
  octomap_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.collision_octomap);

  // Member: elevated_surfel_octomap
  octomap_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.elevated_surfel_octomap);

  // Member: elevated_surfel_poses
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.elevated_surfel_poses);

  // Member: traversable_elevated_cloud
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.traversable_elevated_cloud);

  // Member: traversable_cloud
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.traversable_cloud);

  // Member: is_valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_valid = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
get_serialized_size(
  const vox_nav_msgs::srv::GetTraversabilityMap_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: original_octomap

  current_alignment +=
    octomap_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.original_octomap, current_alignment);
  // Member: collision_octomap

  current_alignment +=
    octomap_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.collision_octomap, current_alignment);
  // Member: elevated_surfel_octomap

  current_alignment +=
    octomap_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.elevated_surfel_octomap, current_alignment);
  // Member: elevated_surfel_poses

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.elevated_surfel_poses, current_alignment);
  // Member: traversable_elevated_cloud

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.traversable_elevated_cloud, current_alignment);
  // Member: traversable_cloud

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.traversable_cloud, current_alignment);
  // Member: is_valid
  {
    size_t item_size = sizeof(ros_message.is_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vox_nav_msgs
max_serialized_size_GetTraversabilityMap_Response(
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


  // Member: original_octomap
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        octomap_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Octomap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: collision_octomap
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        octomap_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Octomap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: elevated_surfel_octomap
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        octomap_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Octomap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: elevated_surfel_poses
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PoseArray(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: traversable_elevated_cloud
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: traversable_cloud
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: is_valid
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
    using DataType = vox_nav_msgs::srv::GetTraversabilityMap_Response;
    is_plain =
      (
      offsetof(DataType, is_valid) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GetTraversabilityMap_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vox_nav_msgs::srv::GetTraversabilityMap_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GetTraversabilityMap_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vox_nav_msgs::srv::GetTraversabilityMap_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GetTraversabilityMap_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vox_nav_msgs::srv::GetTraversabilityMap_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GetTraversabilityMap_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GetTraversabilityMap_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GetTraversabilityMap_Response__callbacks = {
  "vox_nav_msgs::srv",
  "GetTraversabilityMap_Response",
  _GetTraversabilityMap_Response__cdr_serialize,
  _GetTraversabilityMap_Response__cdr_deserialize,
  _GetTraversabilityMap_Response__get_serialized_size,
  _GetTraversabilityMap_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetTraversabilityMap_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GetTraversabilityMap_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace vox_nav_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vox_nav_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<vox_nav_msgs::srv::GetTraversabilityMap_Response>()
{
  return &vox_nav_msgs::srv::typesupport_fastrtps_cpp::_GetTraversabilityMap_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vox_nav_msgs, srv, GetTraversabilityMap_Response)() {
  return &vox_nav_msgs::srv::typesupport_fastrtps_cpp::_GetTraversabilityMap_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace vox_nav_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _GetTraversabilityMap__callbacks = {
  "vox_nav_msgs::srv",
  "GetTraversabilityMap",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vox_nav_msgs, srv, GetTraversabilityMap_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vox_nav_msgs, srv, GetTraversabilityMap_Response)(),
};

static rosidl_service_type_support_t _GetTraversabilityMap__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GetTraversabilityMap__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace vox_nav_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vox_nav_msgs
const rosidl_service_type_support_t *
get_service_type_support_handle<vox_nav_msgs::srv::GetTraversabilityMap>()
{
  return &vox_nav_msgs::srv::typesupport_fastrtps_cpp::_GetTraversabilityMap__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vox_nav_msgs, srv, GetTraversabilityMap)() {
  return &vox_nav_msgs::srv::typesupport_fastrtps_cpp::_GetTraversabilityMap__handle;
}

#ifdef __cplusplus
}
#endif
