// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vox_nav_msgs:srv/GetPointCloud.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "vox_nav_msgs/srv/detail/get_point_cloud__struct.h"
#include "vox_nav_msgs/srv/detail/get_point_cloud__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vox_nav_msgs__srv__get_point_cloud__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("vox_nav_msgs.srv._get_point_cloud.GetPointCloud_Request", full_classname_dest, 55) == 0);
  }
  vox_nav_msgs__srv__GetPointCloud_Request * ros_message = _ros_message;
  {  // bounding_box_origin
    PyObject * field = PyObject_GetAttrString(_pymsg, "bounding_box_origin");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->bounding_box_origin)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // bounding_box_lengths
    PyObject * field = PyObject_GetAttrString(_pymsg, "bounding_box_lengths");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__point__convert_from_py(field, &ros_message->bounding_box_lengths)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // leaf_size
    PyObject * field = PyObject_GetAttrString(_pymsg, "leaf_size");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->leaf_size = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // publish_pointcloud
    PyObject * field = PyObject_GetAttrString(_pymsg, "publish_pointcloud");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->publish_pointcloud = (Py_True == field);
    Py_DECREF(field);
  }
  {  // filename
    PyObject * field = PyObject_GetAttrString(_pymsg, "filename");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->filename, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vox_nav_msgs__srv__get_point_cloud__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetPointCloud_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vox_nav_msgs.srv._get_point_cloud");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetPointCloud_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vox_nav_msgs__srv__GetPointCloud_Request * ros_message = (vox_nav_msgs__srv__GetPointCloud_Request *)raw_ros_message;
  {  // bounding_box_origin
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->bounding_box_origin);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bounding_box_origin", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bounding_box_lengths
    PyObject * field = NULL;
    field = geometry_msgs__msg__point__convert_to_py(&ros_message->bounding_box_lengths);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "bounding_box_lengths", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // leaf_size
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->leaf_size);
    {
      int rc = PyObject_SetAttrString(_pymessage, "leaf_size", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // publish_pointcloud
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->publish_pointcloud ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "publish_pointcloud", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // filename
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->filename.data,
      strlen(ros_message->filename.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "filename", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_point_cloud__struct.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_point_cloud__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__point_cloud2__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__point_cloud2__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vox_nav_msgs__srv__get_point_cloud__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("vox_nav_msgs.srv._get_point_cloud.GetPointCloud_Response", full_classname_dest, 56) == 0);
  }
  vox_nav_msgs__srv__GetPointCloud_Response * ros_message = _ros_message;
  {  // cloud
    PyObject * field = PyObject_GetAttrString(_pymsg, "cloud");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__point_cloud2__convert_from_py(field, &ros_message->cloud)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // origin_latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "origin_latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->origin_latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // origin_longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "origin_longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->origin_longitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // origin_altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "origin_altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->origin_altitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vox_nav_msgs__srv__get_point_cloud__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetPointCloud_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vox_nav_msgs.srv._get_point_cloud");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetPointCloud_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vox_nav_msgs__srv__GetPointCloud_Response * ros_message = (vox_nav_msgs__srv__GetPointCloud_Response *)raw_ros_message;
  {  // cloud
    PyObject * field = NULL;
    field = sensor_msgs__msg__point_cloud2__convert_to_py(&ros_message->cloud);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "cloud", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // origin_latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->origin_latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "origin_latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // origin_longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->origin_longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "origin_longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // origin_altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->origin_altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "origin_altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
