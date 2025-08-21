// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vox_nav_msgs:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vox_nav_msgs/msg/detail/object_array__rosidl_typesupport_introspection_c.h"
#include "vox_nav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vox_nav_msgs/msg/detail/object_array__functions.h"
#include "vox_nav_msgs/msg/detail/object_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `objects`
#include "vox_nav_msgs/msg/object.h"
// Member `objects`
#include "vox_nav_msgs/msg/detail/object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vox_nav_msgs__msg__ObjectArray__init(message_memory);
}

void vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_fini_function(void * message_memory)
{
  vox_nav_msgs__msg__ObjectArray__fini(message_memory);
}

size_t vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__size_function__ObjectArray__objects(
  const void * untyped_member)
{
  const vox_nav_msgs__msg__Object__Sequence * member =
    (const vox_nav_msgs__msg__Object__Sequence *)(untyped_member);
  return member->size;
}

const void * vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__get_const_function__ObjectArray__objects(
  const void * untyped_member, size_t index)
{
  const vox_nav_msgs__msg__Object__Sequence * member =
    (const vox_nav_msgs__msg__Object__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__get_function__ObjectArray__objects(
  void * untyped_member, size_t index)
{
  vox_nav_msgs__msg__Object__Sequence * member =
    (vox_nav_msgs__msg__Object__Sequence *)(untyped_member);
  return &member->data[index];
}

void vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__fetch_function__ObjectArray__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const vox_nav_msgs__msg__Object * item =
    ((const vox_nav_msgs__msg__Object *)
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__get_const_function__ObjectArray__objects(untyped_member, index));
  vox_nav_msgs__msg__Object * value =
    (vox_nav_msgs__msg__Object *)(untyped_value);
  *value = *item;
}

void vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__assign_function__ObjectArray__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  vox_nav_msgs__msg__Object * item =
    ((vox_nav_msgs__msg__Object *)
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__get_function__ObjectArray__objects(untyped_member, index));
  const vox_nav_msgs__msg__Object * value =
    (const vox_nav_msgs__msg__Object *)(untyped_value);
  *item = *value;
}

bool vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__resize_function__ObjectArray__objects(
  void * untyped_member, size_t size)
{
  vox_nav_msgs__msg__Object__Sequence * member =
    (vox_nav_msgs__msg__Object__Sequence *)(untyped_member);
  vox_nav_msgs__msg__Object__Sequence__fini(member);
  return vox_nav_msgs__msg__Object__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__msg__ObjectArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vox_nav_msgs__msg__ObjectArray, objects),  // bytes offset in struct
    NULL,  // default value
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__size_function__ObjectArray__objects,  // size() function pointer
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__get_const_function__ObjectArray__objects,  // get_const(index) function pointer
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__get_function__ObjectArray__objects,  // get(index) function pointer
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__fetch_function__ObjectArray__objects,  // fetch(index, &value) function pointer
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__assign_function__ObjectArray__objects,  // assign(index, value) function pointer
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__resize_function__ObjectArray__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_members = {
  "vox_nav_msgs__msg",  // message namespace
  "ObjectArray",  // message name
  2,  // number of fields
  sizeof(vox_nav_msgs__msg__ObjectArray),
  vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array,  // message members
  vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_init_function,  // function to initialize message memory (memory has to be allocated)
  vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle = {
  0,
  &vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vox_nav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, msg, ObjectArray)() {
  vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vox_nav_msgs, msg, Object)();
  if (!vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle.typesupport_identifier) {
    vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vox_nav_msgs__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
