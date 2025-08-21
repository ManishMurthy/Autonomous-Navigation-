// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vox_nav_msgs:msg/Object.idl
// generated code does not contain a copyright notice
#include "vox_nav_msgs/msg/detail/object__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `accel`
#include "geometry_msgs/msg/detail/accel__functions.h"
// Member `polygon`
#include "geometry_msgs/msg/detail/polygon__functions.h"
// Member `shape`
#include "shape_msgs/msg/detail/solid_primitive__functions.h"
// Member `cluster`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
vox_nav_msgs__msg__Object__init(vox_nav_msgs__msg__Object * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  // id
  // detection_level
  // object_classified
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  // accel
  if (!geometry_msgs__msg__Accel__init(&msg->accel)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  // velocity
  // heading
  // polygon
  if (!geometry_msgs__msg__Polygon__init(&msg->polygon)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__init(&msg->shape)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  // classification_label
  // classification_probability
  // classification_age
  // is_dynamic
  // cluster
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cluster)) {
    vox_nav_msgs__msg__Object__fini(msg);
    return false;
  }
  return true;
}

void
vox_nav_msgs__msg__Object__fini(vox_nav_msgs__msg__Object * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // detection_level
  // object_classified
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
  // accel
  geometry_msgs__msg__Accel__fini(&msg->accel);
  // velocity
  // heading
  // polygon
  geometry_msgs__msg__Polygon__fini(&msg->polygon);
  // shape
  shape_msgs__msg__SolidPrimitive__fini(&msg->shape);
  // classification_label
  // classification_probability
  // classification_age
  // is_dynamic
  // cluster
  sensor_msgs__msg__PointCloud2__fini(&msg->cluster);
}

bool
vox_nav_msgs__msg__Object__are_equal(const vox_nav_msgs__msg__Object * lhs, const vox_nav_msgs__msg__Object * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // detection_level
  if (lhs->detection_level != rhs->detection_level) {
    return false;
  }
  // object_classified
  if (lhs->object_classified != rhs->object_classified) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // accel
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->accel), &(rhs->accel)))
  {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // heading
  if (lhs->heading != rhs->heading) {
    return false;
  }
  // polygon
  if (!geometry_msgs__msg__Polygon__are_equal(
      &(lhs->polygon), &(rhs->polygon)))
  {
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__are_equal(
      &(lhs->shape), &(rhs->shape)))
  {
    return false;
  }
  // classification_label
  if (lhs->classification_label != rhs->classification_label) {
    return false;
  }
  // classification_probability
  if (lhs->classification_probability != rhs->classification_probability) {
    return false;
  }
  // classification_age
  if (lhs->classification_age != rhs->classification_age) {
    return false;
  }
  // is_dynamic
  if (lhs->is_dynamic != rhs->is_dynamic) {
    return false;
  }
  // cluster
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cluster), &(rhs->cluster)))
  {
    return false;
  }
  return true;
}

bool
vox_nav_msgs__msg__Object__copy(
  const vox_nav_msgs__msg__Object * input,
  vox_nav_msgs__msg__Object * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // detection_level
  output->detection_level = input->detection_level;
  // object_classified
  output->object_classified = input->object_classified;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // accel
  if (!geometry_msgs__msg__Accel__copy(
      &(input->accel), &(output->accel)))
  {
    return false;
  }
  // velocity
  output->velocity = input->velocity;
  // heading
  output->heading = input->heading;
  // polygon
  if (!geometry_msgs__msg__Polygon__copy(
      &(input->polygon), &(output->polygon)))
  {
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__copy(
      &(input->shape), &(output->shape)))
  {
    return false;
  }
  // classification_label
  output->classification_label = input->classification_label;
  // classification_probability
  output->classification_probability = input->classification_probability;
  // classification_age
  output->classification_age = input->classification_age;
  // is_dynamic
  output->is_dynamic = input->is_dynamic;
  // cluster
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cluster), &(output->cluster)))
  {
    return false;
  }
  return true;
}

vox_nav_msgs__msg__Object *
vox_nav_msgs__msg__Object__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__msg__Object * msg = (vox_nav_msgs__msg__Object *)allocator.allocate(sizeof(vox_nav_msgs__msg__Object), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vox_nav_msgs__msg__Object));
  bool success = vox_nav_msgs__msg__Object__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vox_nav_msgs__msg__Object__destroy(vox_nav_msgs__msg__Object * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vox_nav_msgs__msg__Object__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vox_nav_msgs__msg__Object__Sequence__init(vox_nav_msgs__msg__Object__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__msg__Object * data = NULL;

  if (size) {
    data = (vox_nav_msgs__msg__Object *)allocator.zero_allocate(size, sizeof(vox_nav_msgs__msg__Object), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vox_nav_msgs__msg__Object__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vox_nav_msgs__msg__Object__fini(&data[i - 1]);
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
vox_nav_msgs__msg__Object__Sequence__fini(vox_nav_msgs__msg__Object__Sequence * array)
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
      vox_nav_msgs__msg__Object__fini(&array->data[i]);
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

vox_nav_msgs__msg__Object__Sequence *
vox_nav_msgs__msg__Object__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__msg__Object__Sequence * array = (vox_nav_msgs__msg__Object__Sequence *)allocator.allocate(sizeof(vox_nav_msgs__msg__Object__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vox_nav_msgs__msg__Object__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vox_nav_msgs__msg__Object__Sequence__destroy(vox_nav_msgs__msg__Object__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vox_nav_msgs__msg__Object__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vox_nav_msgs__msg__Object__Sequence__are_equal(const vox_nav_msgs__msg__Object__Sequence * lhs, const vox_nav_msgs__msg__Object__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vox_nav_msgs__msg__Object__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vox_nav_msgs__msg__Object__Sequence__copy(
  const vox_nav_msgs__msg__Object__Sequence * input,
  vox_nav_msgs__msg__Object__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vox_nav_msgs__msg__Object);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vox_nav_msgs__msg__Object * data =
      (vox_nav_msgs__msg__Object *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vox_nav_msgs__msg__Object__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vox_nav_msgs__msg__Object__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vox_nav_msgs__msg__Object__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
