// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vox_nav_msgs:msg/OrientedNavSatFix.idl
// generated code does not contain a copyright notice
#include "vox_nav_msgs/msg/detail/oriented_nav_sat_fix__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "sensor_msgs/msg/detail/nav_sat_fix__functions.h"
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"

bool
vox_nav_msgs__msg__OrientedNavSatFix__init(vox_nav_msgs__msg__OrientedNavSatFix * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!sensor_msgs__msg__NavSatFix__init(&msg->position)) {
    vox_nav_msgs__msg__OrientedNavSatFix__fini(msg);
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    vox_nav_msgs__msg__OrientedNavSatFix__fini(msg);
    return false;
  }
  return true;
}

void
vox_nav_msgs__msg__OrientedNavSatFix__fini(vox_nav_msgs__msg__OrientedNavSatFix * msg)
{
  if (!msg) {
    return;
  }
  // position
  sensor_msgs__msg__NavSatFix__fini(&msg->position);
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
}

bool
vox_nav_msgs__msg__OrientedNavSatFix__are_equal(const vox_nav_msgs__msg__OrientedNavSatFix * lhs, const vox_nav_msgs__msg__OrientedNavSatFix * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!sensor_msgs__msg__NavSatFix__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  return true;
}

bool
vox_nav_msgs__msg__OrientedNavSatFix__copy(
  const vox_nav_msgs__msg__OrientedNavSatFix * input,
  vox_nav_msgs__msg__OrientedNavSatFix * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!sensor_msgs__msg__NavSatFix__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  return true;
}

vox_nav_msgs__msg__OrientedNavSatFix *
vox_nav_msgs__msg__OrientedNavSatFix__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__msg__OrientedNavSatFix * msg = (vox_nav_msgs__msg__OrientedNavSatFix *)allocator.allocate(sizeof(vox_nav_msgs__msg__OrientedNavSatFix), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vox_nav_msgs__msg__OrientedNavSatFix));
  bool success = vox_nav_msgs__msg__OrientedNavSatFix__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vox_nav_msgs__msg__OrientedNavSatFix__destroy(vox_nav_msgs__msg__OrientedNavSatFix * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vox_nav_msgs__msg__OrientedNavSatFix__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vox_nav_msgs__msg__OrientedNavSatFix__Sequence__init(vox_nav_msgs__msg__OrientedNavSatFix__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__msg__OrientedNavSatFix * data = NULL;

  if (size) {
    data = (vox_nav_msgs__msg__OrientedNavSatFix *)allocator.zero_allocate(size, sizeof(vox_nav_msgs__msg__OrientedNavSatFix), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vox_nav_msgs__msg__OrientedNavSatFix__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vox_nav_msgs__msg__OrientedNavSatFix__fini(&data[i - 1]);
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
vox_nav_msgs__msg__OrientedNavSatFix__Sequence__fini(vox_nav_msgs__msg__OrientedNavSatFix__Sequence * array)
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
      vox_nav_msgs__msg__OrientedNavSatFix__fini(&array->data[i]);
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

vox_nav_msgs__msg__OrientedNavSatFix__Sequence *
vox_nav_msgs__msg__OrientedNavSatFix__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__msg__OrientedNavSatFix__Sequence * array = (vox_nav_msgs__msg__OrientedNavSatFix__Sequence *)allocator.allocate(sizeof(vox_nav_msgs__msg__OrientedNavSatFix__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vox_nav_msgs__msg__OrientedNavSatFix__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vox_nav_msgs__msg__OrientedNavSatFix__Sequence__destroy(vox_nav_msgs__msg__OrientedNavSatFix__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vox_nav_msgs__msg__OrientedNavSatFix__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vox_nav_msgs__msg__OrientedNavSatFix__Sequence__are_equal(const vox_nav_msgs__msg__OrientedNavSatFix__Sequence * lhs, const vox_nav_msgs__msg__OrientedNavSatFix__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vox_nav_msgs__msg__OrientedNavSatFix__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vox_nav_msgs__msg__OrientedNavSatFix__Sequence__copy(
  const vox_nav_msgs__msg__OrientedNavSatFix__Sequence * input,
  vox_nav_msgs__msg__OrientedNavSatFix__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vox_nav_msgs__msg__OrientedNavSatFix);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vox_nav_msgs__msg__OrientedNavSatFix * data =
      (vox_nav_msgs__msg__OrientedNavSatFix *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vox_nav_msgs__msg__OrientedNavSatFix__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vox_nav_msgs__msg__OrientedNavSatFix__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vox_nav_msgs__msg__OrientedNavSatFix__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
