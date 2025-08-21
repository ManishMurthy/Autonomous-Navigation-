// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vox_nav_msgs:msg/SemanticRoadLabel.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vox_nav_msgs/msg/detail/semantic_road_label__rosidl_typesupport_introspection_c.h"
#include "vox_nav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vox_nav_msgs/msg/detail/semantic_road_label__functions.h"
#include "vox_nav_msgs/msg/detail/semantic_road_label__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vox_nav_msgs__msg__SemanticRoadLabel__init(message_memory);
}

void vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_fini_function(void * message_memory)
{
  vox_nav_msgs__msg__SemanticRoadLabel__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__msg__SemanticRoadLabel, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_members = {
  "vox_nav_msgs__msg",  // message namespace
  "SemanticRoadLabel",  // message name
  1,  // number of fields
  sizeof(vox_nav_msgs__msg__SemanticRoadLabel),
  vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_member_array,  // message members
  vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_init_function,  // function to initialize message memory (memory has to be allocated)
  vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_type_support_handle = {
  0,
  &vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vox_nav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, msg, SemanticRoadLabel)() {
  if (!vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_type_support_handle.typesupport_identifier) {
    vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vox_nav_msgs__msg__SemanticRoadLabel__rosidl_typesupport_introspection_c__SemanticRoadLabel_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
