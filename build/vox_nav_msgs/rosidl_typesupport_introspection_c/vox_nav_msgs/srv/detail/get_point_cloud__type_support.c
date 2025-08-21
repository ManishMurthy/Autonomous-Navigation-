// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vox_nav_msgs:srv/GetPointCloud.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vox_nav_msgs/srv/detail/get_point_cloud__rosidl_typesupport_introspection_c.h"
#include "vox_nav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vox_nav_msgs/srv/detail/get_point_cloud__functions.h"
#include "vox_nav_msgs/srv/detail/get_point_cloud__struct.h"


// Include directives for member types
// Member `bounding_box_origin`
// Member `bounding_box_lengths`
#include "geometry_msgs/msg/point.h"
// Member `bounding_box_origin`
// Member `bounding_box_lengths`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `filename`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vox_nav_msgs__srv__GetPointCloud_Request__init(message_memory);
}

void vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_fini_function(void * message_memory)
{
  vox_nav_msgs__srv__GetPointCloud_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_member_array[5] = {
  {
    "bounding_box_origin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Request, bounding_box_origin),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_box_lengths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Request, bounding_box_lengths),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "leaf_size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Request, leaf_size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "publish_pointcloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Request, publish_pointcloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "filename",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Request, filename),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_members = {
  "vox_nav_msgs__srv",  // message namespace
  "GetPointCloud_Request",  // message name
  5,  // number of fields
  sizeof(vox_nav_msgs__srv__GetPointCloud_Request),
  vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_member_array,  // message members
  vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_type_support_handle = {
  0,
  &vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vox_nav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud_Request)() {
  vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_type_support_handle.typesupport_identifier) {
    vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vox_nav_msgs__srv__GetPointCloud_Request__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "vox_nav_msgs/srv/detail/get_point_cloud__rosidl_typesupport_introspection_c.h"
// already included above
// #include "vox_nav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_point_cloud__functions.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_point_cloud__struct.h"


// Include directives for member types
// Member `cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vox_nav_msgs__srv__GetPointCloud_Response__init(message_memory);
}

void vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_fini_function(void * message_memory)
{
  vox_nav_msgs__srv__GetPointCloud_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_member_array[4] = {
  {
    "cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Response, cloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "origin_latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Response, origin_latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "origin_longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Response, origin_longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "origin_altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__srv__GetPointCloud_Response, origin_altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_members = {
  "vox_nav_msgs__srv",  // message namespace
  "GetPointCloud_Response",  // message name
  4,  // number of fields
  sizeof(vox_nav_msgs__srv__GetPointCloud_Response),
  vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_member_array,  // message members
  vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_type_support_handle = {
  0,
  &vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vox_nav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud_Response)() {
  vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_type_support_handle.typesupport_identifier) {
    vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vox_nav_msgs__srv__GetPointCloud_Response__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "vox_nav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_point_cloud__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_members = {
  "vox_nav_msgs__srv",  // service namespace
  "GetPointCloud",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_Request_message_type_support_handle,
  NULL  // response message
  // vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_Response_message_type_support_handle
};

static rosidl_service_type_support_t vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_type_support_handle = {
  0,
  &vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vox_nav_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud)() {
  if (!vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_type_support_handle.typesupport_identifier) {
    vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, srv, GetPointCloud_Response)()->data;
  }

  return &vox_nav_msgs__srv__detail__get_point_cloud__rosidl_typesupport_introspection_c__GetPointCloud_service_type_support_handle;
}
