// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
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
#include "vox_nav_msgs/srv/detail/get_traversability_map__struct.h"
#include "vox_nav_msgs/srv/detail/get_traversability_map__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vox_nav_msgs__srv__get_traversability_map__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[70];
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
    assert(strncmp("vox_nav_msgs.srv._get_traversability_map.GetTraversabilityMap_Request", full_classname_dest, 69) == 0);
  }
  vox_nav_msgs__srv__GetTraversabilityMap_Request * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vox_nav_msgs__srv__get_traversability_map__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetTraversabilityMap_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vox_nav_msgs.srv._get_traversability_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetTraversabilityMap_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

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
// #include "vox_nav_msgs/srv/detail/get_traversability_map__struct.h"
// already included above
// #include "vox_nav_msgs/srv/detail/get_traversability_map__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool octomap_msgs__msg__octomap__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * octomap_msgs__msg__octomap__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool octomap_msgs__msg__octomap__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * octomap_msgs__msg__octomap__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool octomap_msgs__msg__octomap__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * octomap_msgs__msg__octomap__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_array__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_array__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__point_cloud2__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__point_cloud2__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__point_cloud2__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__point_cloud2__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool vox_nav_msgs__srv__get_traversability_map__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[71];
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
    assert(strncmp("vox_nav_msgs.srv._get_traversability_map.GetTraversabilityMap_Response", full_classname_dest, 70) == 0);
  }
  vox_nav_msgs__srv__GetTraversabilityMap_Response * ros_message = _ros_message;
  {  // original_octomap
    PyObject * field = PyObject_GetAttrString(_pymsg, "original_octomap");
    if (!field) {
      return false;
    }
    if (!octomap_msgs__msg__octomap__convert_from_py(field, &ros_message->original_octomap)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // collision_octomap
    PyObject * field = PyObject_GetAttrString(_pymsg, "collision_octomap");
    if (!field) {
      return false;
    }
    if (!octomap_msgs__msg__octomap__convert_from_py(field, &ros_message->collision_octomap)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // elevated_surfel_octomap
    PyObject * field = PyObject_GetAttrString(_pymsg, "elevated_surfel_octomap");
    if (!field) {
      return false;
    }
    if (!octomap_msgs__msg__octomap__convert_from_py(field, &ros_message->elevated_surfel_octomap)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // elevated_surfel_poses
    PyObject * field = PyObject_GetAttrString(_pymsg, "elevated_surfel_poses");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_array__convert_from_py(field, &ros_message->elevated_surfel_poses)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // traversable_elevated_cloud
    PyObject * field = PyObject_GetAttrString(_pymsg, "traversable_elevated_cloud");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__point_cloud2__convert_from_py(field, &ros_message->traversable_elevated_cloud)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // traversable_cloud
    PyObject * field = PyObject_GetAttrString(_pymsg, "traversable_cloud");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__point_cloud2__convert_from_py(field, &ros_message->traversable_cloud)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // is_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_valid = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vox_nav_msgs__srv__get_traversability_map__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GetTraversabilityMap_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vox_nav_msgs.srv._get_traversability_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GetTraversabilityMap_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vox_nav_msgs__srv__GetTraversabilityMap_Response * ros_message = (vox_nav_msgs__srv__GetTraversabilityMap_Response *)raw_ros_message;
  {  // original_octomap
    PyObject * field = NULL;
    field = octomap_msgs__msg__octomap__convert_to_py(&ros_message->original_octomap);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "original_octomap", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // collision_octomap
    PyObject * field = NULL;
    field = octomap_msgs__msg__octomap__convert_to_py(&ros_message->collision_octomap);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "collision_octomap", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // elevated_surfel_octomap
    PyObject * field = NULL;
    field = octomap_msgs__msg__octomap__convert_to_py(&ros_message->elevated_surfel_octomap);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "elevated_surfel_octomap", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // elevated_surfel_poses
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_array__convert_to_py(&ros_message->elevated_surfel_poses);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "elevated_surfel_poses", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // traversable_elevated_cloud
    PyObject * field = NULL;
    field = sensor_msgs__msg__point_cloud2__convert_to_py(&ros_message->traversable_elevated_cloud);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "traversable_elevated_cloud", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // traversable_cloud
    PyObject * field = NULL;
    field = sensor_msgs__msg__point_cloud2__convert_to_py(&ros_message->traversable_cloud);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "traversable_cloud", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
