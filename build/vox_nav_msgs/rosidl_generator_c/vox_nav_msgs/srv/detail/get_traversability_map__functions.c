// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
// generated code does not contain a copyright notice
#include "vox_nav_msgs/srv/detail/get_traversability_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
vox_nav_msgs__srv__GetTraversabilityMap_Request__init(vox_nav_msgs__srv__GetTraversabilityMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Request__fini(vox_nav_msgs__srv__GetTraversabilityMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Request__are_equal(const vox_nav_msgs__srv__GetTraversabilityMap_Request * lhs, const vox_nav_msgs__srv__GetTraversabilityMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Request__copy(
  const vox_nav_msgs__srv__GetTraversabilityMap_Request * input,
  vox_nav_msgs__srv__GetTraversabilityMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

vox_nav_msgs__srv__GetTraversabilityMap_Request *
vox_nav_msgs__srv__GetTraversabilityMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetTraversabilityMap_Request * msg = (vox_nav_msgs__srv__GetTraversabilityMap_Request *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Request));
  bool success = vox_nav_msgs__srv__GetTraversabilityMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Request__destroy(vox_nav_msgs__srv__GetTraversabilityMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vox_nav_msgs__srv__GetTraversabilityMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__init(vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetTraversabilityMap_Request * data = NULL;

  if (size) {
    data = (vox_nav_msgs__srv__GetTraversabilityMap_Request *)allocator.zero_allocate(size, sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vox_nav_msgs__srv__GetTraversabilityMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vox_nav_msgs__srv__GetTraversabilityMap_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__fini(vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vox_nav_msgs__srv__GetTraversabilityMap_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence *
vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * array = (vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__destroy(vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__are_equal(const vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * lhs, const vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vox_nav_msgs__srv__GetTraversabilityMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence__copy(
  const vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * input,
  vox_nav_msgs__srv__GetTraversabilityMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vox_nav_msgs__srv__GetTraversabilityMap_Request * data =
      (vox_nav_msgs__srv__GetTraversabilityMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vox_nav_msgs__srv__GetTraversabilityMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vox_nav_msgs__srv__GetTraversabilityMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vox_nav_msgs__srv__GetTraversabilityMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `original_octomap`
// Member `collision_octomap`
// Member `elevated_surfel_octomap`
#include "octomap_msgs/msg/detail/octomap__functions.h"
// Member `elevated_surfel_poses`
#include "geometry_msgs/msg/detail/pose_array__functions.h"
// Member `traversable_elevated_cloud`
// Member `traversable_cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
vox_nav_msgs__srv__GetTraversabilityMap_Response__init(vox_nav_msgs__srv__GetTraversabilityMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // original_octomap
  if (!octomap_msgs__msg__Octomap__init(&msg->original_octomap)) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
    return false;
  }
  // collision_octomap
  if (!octomap_msgs__msg__Octomap__init(&msg->collision_octomap)) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
    return false;
  }
  // elevated_surfel_octomap
  if (!octomap_msgs__msg__Octomap__init(&msg->elevated_surfel_octomap)) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
    return false;
  }
  // elevated_surfel_poses
  if (!geometry_msgs__msg__PoseArray__init(&msg->elevated_surfel_poses)) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
    return false;
  }
  // traversable_elevated_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->traversable_elevated_cloud)) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
    return false;
  }
  // traversable_cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->traversable_cloud)) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
    return false;
  }
  // is_valid
  return true;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(vox_nav_msgs__srv__GetTraversabilityMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // original_octomap
  octomap_msgs__msg__Octomap__fini(&msg->original_octomap);
  // collision_octomap
  octomap_msgs__msg__Octomap__fini(&msg->collision_octomap);
  // elevated_surfel_octomap
  octomap_msgs__msg__Octomap__fini(&msg->elevated_surfel_octomap);
  // elevated_surfel_poses
  geometry_msgs__msg__PoseArray__fini(&msg->elevated_surfel_poses);
  // traversable_elevated_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->traversable_elevated_cloud);
  // traversable_cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->traversable_cloud);
  // is_valid
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Response__are_equal(const vox_nav_msgs__srv__GetTraversabilityMap_Response * lhs, const vox_nav_msgs__srv__GetTraversabilityMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // original_octomap
  if (!octomap_msgs__msg__Octomap__are_equal(
      &(lhs->original_octomap), &(rhs->original_octomap)))
  {
    return false;
  }
  // collision_octomap
  if (!octomap_msgs__msg__Octomap__are_equal(
      &(lhs->collision_octomap), &(rhs->collision_octomap)))
  {
    return false;
  }
  // elevated_surfel_octomap
  if (!octomap_msgs__msg__Octomap__are_equal(
      &(lhs->elevated_surfel_octomap), &(rhs->elevated_surfel_octomap)))
  {
    return false;
  }
  // elevated_surfel_poses
  if (!geometry_msgs__msg__PoseArray__are_equal(
      &(lhs->elevated_surfel_poses), &(rhs->elevated_surfel_poses)))
  {
    return false;
  }
  // traversable_elevated_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->traversable_elevated_cloud), &(rhs->traversable_elevated_cloud)))
  {
    return false;
  }
  // traversable_cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->traversable_cloud), &(rhs->traversable_cloud)))
  {
    return false;
  }
  // is_valid
  if (lhs->is_valid != rhs->is_valid) {
    return false;
  }
  return true;
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Response__copy(
  const vox_nav_msgs__srv__GetTraversabilityMap_Response * input,
  vox_nav_msgs__srv__GetTraversabilityMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // original_octomap
  if (!octomap_msgs__msg__Octomap__copy(
      &(input->original_octomap), &(output->original_octomap)))
  {
    return false;
  }
  // collision_octomap
  if (!octomap_msgs__msg__Octomap__copy(
      &(input->collision_octomap), &(output->collision_octomap)))
  {
    return false;
  }
  // elevated_surfel_octomap
  if (!octomap_msgs__msg__Octomap__copy(
      &(input->elevated_surfel_octomap), &(output->elevated_surfel_octomap)))
  {
    return false;
  }
  // elevated_surfel_poses
  if (!geometry_msgs__msg__PoseArray__copy(
      &(input->elevated_surfel_poses), &(output->elevated_surfel_poses)))
  {
    return false;
  }
  // traversable_elevated_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->traversable_elevated_cloud), &(output->traversable_elevated_cloud)))
  {
    return false;
  }
  // traversable_cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->traversable_cloud), &(output->traversable_cloud)))
  {
    return false;
  }
  // is_valid
  output->is_valid = input->is_valid;
  return true;
}

vox_nav_msgs__srv__GetTraversabilityMap_Response *
vox_nav_msgs__srv__GetTraversabilityMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetTraversabilityMap_Response * msg = (vox_nav_msgs__srv__GetTraversabilityMap_Response *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Response));
  bool success = vox_nav_msgs__srv__GetTraversabilityMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Response__destroy(vox_nav_msgs__srv__GetTraversabilityMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__init(vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetTraversabilityMap_Response * data = NULL;

  if (size) {
    data = (vox_nav_msgs__srv__GetTraversabilityMap_Response *)allocator.zero_allocate(size, sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vox_nav_msgs__srv__GetTraversabilityMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__fini(vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence *
vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * array = (vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__destroy(vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__are_equal(const vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * lhs, const vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vox_nav_msgs__srv__GetTraversabilityMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence__copy(
  const vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * input,
  vox_nav_msgs__srv__GetTraversabilityMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vox_nav_msgs__srv__GetTraversabilityMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vox_nav_msgs__srv__GetTraversabilityMap_Response * data =
      (vox_nav_msgs__srv__GetTraversabilityMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vox_nav_msgs__srv__GetTraversabilityMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vox_nav_msgs__srv__GetTraversabilityMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vox_nav_msgs__srv__GetTraversabilityMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
