// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vox_nav_msgs:srv/GetPointCloud.idl
// generated code does not contain a copyright notice
#include "vox_nav_msgs/srv/detail/get_point_cloud__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `bounding_box_origin`
// Member `bounding_box_lengths`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `filename`
#include "rosidl_runtime_c/string_functions.h"

bool
vox_nav_msgs__srv__GetPointCloud_Request__init(vox_nav_msgs__srv__GetPointCloud_Request * msg)
{
  if (!msg) {
    return false;
  }
  // bounding_box_origin
  if (!geometry_msgs__msg__Point__init(&msg->bounding_box_origin)) {
    vox_nav_msgs__srv__GetPointCloud_Request__fini(msg);
    return false;
  }
  // bounding_box_lengths
  if (!geometry_msgs__msg__Point__init(&msg->bounding_box_lengths)) {
    vox_nav_msgs__srv__GetPointCloud_Request__fini(msg);
    return false;
  }
  // leaf_size
  // publish_pointcloud
  // filename
  if (!rosidl_runtime_c__String__init(&msg->filename)) {
    vox_nav_msgs__srv__GetPointCloud_Request__fini(msg);
    return false;
  }
  return true;
}

void
vox_nav_msgs__srv__GetPointCloud_Request__fini(vox_nav_msgs__srv__GetPointCloud_Request * msg)
{
  if (!msg) {
    return;
  }
  // bounding_box_origin
  geometry_msgs__msg__Point__fini(&msg->bounding_box_origin);
  // bounding_box_lengths
  geometry_msgs__msg__Point__fini(&msg->bounding_box_lengths);
  // leaf_size
  // publish_pointcloud
  // filename
  rosidl_runtime_c__String__fini(&msg->filename);
}

bool
vox_nav_msgs__srv__GetPointCloud_Request__are_equal(const vox_nav_msgs__srv__GetPointCloud_Request * lhs, const vox_nav_msgs__srv__GetPointCloud_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bounding_box_origin
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->bounding_box_origin), &(rhs->bounding_box_origin)))
  {
    return false;
  }
  // bounding_box_lengths
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->bounding_box_lengths), &(rhs->bounding_box_lengths)))
  {
    return false;
  }
  // leaf_size
  if (lhs->leaf_size != rhs->leaf_size) {
    return false;
  }
  // publish_pointcloud
  if (lhs->publish_pointcloud != rhs->publish_pointcloud) {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->filename), &(rhs->filename)))
  {
    return false;
  }
  return true;
}

bool
vox_nav_msgs__srv__GetPointCloud_Request__copy(
  const vox_nav_msgs__srv__GetPointCloud_Request * input,
  vox_nav_msgs__srv__GetPointCloud_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // bounding_box_origin
  if (!geometry_msgs__msg__Point__copy(
      &(input->bounding_box_origin), &(output->bounding_box_origin)))
  {
    return false;
  }
  // bounding_box_lengths
  if (!geometry_msgs__msg__Point__copy(
      &(input->bounding_box_lengths), &(output->bounding_box_lengths)))
  {
    return false;
  }
  // leaf_size
  output->leaf_size = input->leaf_size;
  // publish_pointcloud
  output->publish_pointcloud = input->publish_pointcloud;
  // filename
  if (!rosidl_runtime_c__String__copy(
      &(input->filename), &(output->filename)))
  {
    return false;
  }
  return true;
}

vox_nav_msgs__srv__GetPointCloud_Request *
vox_nav_msgs__srv__GetPointCloud_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetPointCloud_Request * msg = (vox_nav_msgs__srv__GetPointCloud_Request *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetPointCloud_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vox_nav_msgs__srv__GetPointCloud_Request));
  bool success = vox_nav_msgs__srv__GetPointCloud_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vox_nav_msgs__srv__GetPointCloud_Request__destroy(vox_nav_msgs__srv__GetPointCloud_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vox_nav_msgs__srv__GetPointCloud_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__init(vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetPointCloud_Request * data = NULL;

  if (size) {
    data = (vox_nav_msgs__srv__GetPointCloud_Request *)allocator.zero_allocate(size, sizeof(vox_nav_msgs__srv__GetPointCloud_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vox_nav_msgs__srv__GetPointCloud_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vox_nav_msgs__srv__GetPointCloud_Request__fini(&data[i - 1]);
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
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__fini(vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array)
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
      vox_nav_msgs__srv__GetPointCloud_Request__fini(&array->data[i]);
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

vox_nav_msgs__srv__GetPointCloud_Request__Sequence *
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array = (vox_nav_msgs__srv__GetPointCloud_Request__Sequence *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetPointCloud_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vox_nav_msgs__srv__GetPointCloud_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__destroy(vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vox_nav_msgs__srv__GetPointCloud_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__are_equal(const vox_nav_msgs__srv__GetPointCloud_Request__Sequence * lhs, const vox_nav_msgs__srv__GetPointCloud_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vox_nav_msgs__srv__GetPointCloud_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__copy(
  const vox_nav_msgs__srv__GetPointCloud_Request__Sequence * input,
  vox_nav_msgs__srv__GetPointCloud_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vox_nav_msgs__srv__GetPointCloud_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vox_nav_msgs__srv__GetPointCloud_Request * data =
      (vox_nav_msgs__srv__GetPointCloud_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vox_nav_msgs__srv__GetPointCloud_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vox_nav_msgs__srv__GetPointCloud_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vox_nav_msgs__srv__GetPointCloud_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
vox_nav_msgs__srv__GetPointCloud_Response__init(vox_nav_msgs__srv__GetPointCloud_Response * msg)
{
  if (!msg) {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud)) {
    vox_nav_msgs__srv__GetPointCloud_Response__fini(msg);
    return false;
  }
  // origin_latitude
  // origin_longitude
  // origin_altitude
  return true;
}

void
vox_nav_msgs__srv__GetPointCloud_Response__fini(vox_nav_msgs__srv__GetPointCloud_Response * msg)
{
  if (!msg) {
    return;
  }
  // cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud);
  // origin_latitude
  // origin_longitude
  // origin_altitude
}

bool
vox_nav_msgs__srv__GetPointCloud_Response__are_equal(const vox_nav_msgs__srv__GetPointCloud_Response * lhs, const vox_nav_msgs__srv__GetPointCloud_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud), &(rhs->cloud)))
  {
    return false;
  }
  // origin_latitude
  if (lhs->origin_latitude != rhs->origin_latitude) {
    return false;
  }
  // origin_longitude
  if (lhs->origin_longitude != rhs->origin_longitude) {
    return false;
  }
  // origin_altitude
  if (lhs->origin_altitude != rhs->origin_altitude) {
    return false;
  }
  return true;
}

bool
vox_nav_msgs__srv__GetPointCloud_Response__copy(
  const vox_nav_msgs__srv__GetPointCloud_Response * input,
  vox_nav_msgs__srv__GetPointCloud_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud), &(output->cloud)))
  {
    return false;
  }
  // origin_latitude
  output->origin_latitude = input->origin_latitude;
  // origin_longitude
  output->origin_longitude = input->origin_longitude;
  // origin_altitude
  output->origin_altitude = input->origin_altitude;
  return true;
}

vox_nav_msgs__srv__GetPointCloud_Response *
vox_nav_msgs__srv__GetPointCloud_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetPointCloud_Response * msg = (vox_nav_msgs__srv__GetPointCloud_Response *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetPointCloud_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vox_nav_msgs__srv__GetPointCloud_Response));
  bool success = vox_nav_msgs__srv__GetPointCloud_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vox_nav_msgs__srv__GetPointCloud_Response__destroy(vox_nav_msgs__srv__GetPointCloud_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vox_nav_msgs__srv__GetPointCloud_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__init(vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetPointCloud_Response * data = NULL;

  if (size) {
    data = (vox_nav_msgs__srv__GetPointCloud_Response *)allocator.zero_allocate(size, sizeof(vox_nav_msgs__srv__GetPointCloud_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vox_nav_msgs__srv__GetPointCloud_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vox_nav_msgs__srv__GetPointCloud_Response__fini(&data[i - 1]);
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
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__fini(vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array)
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
      vox_nav_msgs__srv__GetPointCloud_Response__fini(&array->data[i]);
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

vox_nav_msgs__srv__GetPointCloud_Response__Sequence *
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array = (vox_nav_msgs__srv__GetPointCloud_Response__Sequence *)allocator.allocate(sizeof(vox_nav_msgs__srv__GetPointCloud_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vox_nav_msgs__srv__GetPointCloud_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__destroy(vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vox_nav_msgs__srv__GetPointCloud_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__are_equal(const vox_nav_msgs__srv__GetPointCloud_Response__Sequence * lhs, const vox_nav_msgs__srv__GetPointCloud_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vox_nav_msgs__srv__GetPointCloud_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__copy(
  const vox_nav_msgs__srv__GetPointCloud_Response__Sequence * input,
  vox_nav_msgs__srv__GetPointCloud_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vox_nav_msgs__srv__GetPointCloud_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vox_nav_msgs__srv__GetPointCloud_Response * data =
      (vox_nav_msgs__srv__GetPointCloud_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vox_nav_msgs__srv__GetPointCloud_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vox_nav_msgs__srv__GetPointCloud_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vox_nav_msgs__srv__GetPointCloud_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
