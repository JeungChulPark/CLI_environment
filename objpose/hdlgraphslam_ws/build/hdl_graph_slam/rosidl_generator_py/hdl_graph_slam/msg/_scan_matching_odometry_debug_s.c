// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
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
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__struct.h"
#include "hdl_graph_slam/msg/detail/scan_matching_odometry_debug__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool hdl_graph_slam__msg__scan_matching_odometry_debug__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[75];
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
    assert(strncmp("hdl_graph_slam.msg._scan_matching_odometry_debug.ScanMatchingOdometryDebug", full_classname_dest, 74) == 0);
  }
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // input_point_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "input_point_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->input_point_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // downsampled_point_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "downsampled_point_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->downsampled_point_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // registration_time_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "registration_time_ms");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->registration_time_ms = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // callback_time_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "callback_time_ms");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->callback_time_ms = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // odom_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "odom_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->odom_count = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // registration_triggered
    PyObject * field = PyObject_GetAttrString(_pymsg, "registration_triggered");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->registration_triggered = (Py_True == field);
    Py_DECREF(field);
  }
  {  // has_converged
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_converged");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_converged = (Py_True == field);
    Py_DECREF(field);
  }
  {  // matching_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "matching_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->matching_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hdl_graph_slam__msg__scan_matching_odometry_debug__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ScanMatchingOdometryDebug */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hdl_graph_slam.msg._scan_matching_odometry_debug");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ScanMatchingOdometryDebug");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hdl_graph_slam__msg__ScanMatchingOdometryDebug * ros_message = (hdl_graph_slam__msg__ScanMatchingOdometryDebug *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // input_point_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->input_point_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "input_point_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // downsampled_point_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->downsampled_point_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "downsampled_point_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // registration_time_ms
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->registration_time_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "registration_time_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // callback_time_ms
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->callback_time_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "callback_time_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // odom_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->odom_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "odom_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // registration_triggered
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->registration_triggered ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "registration_triggered", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // has_converged
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_converged ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_converged", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // matching_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->matching_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "matching_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
