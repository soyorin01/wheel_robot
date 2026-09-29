// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from wheel_robot:msg/ObstacleStatus.idl
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
#include "wheel_robot/msg/detail/obstacle_status__struct.h"
#include "wheel_robot/msg/detail/obstacle_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool wheel_robot__msg__obstacle_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("wheel_robot.msg._obstacle_status.ObstacleStatus", full_classname_dest, 47) == 0);
  }
  wheel_robot__msg__ObstacleStatus * ros_message = _ros_message;
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
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // left_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->left_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // center_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "center_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->center_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // right_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->right_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // left_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->left_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // center_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "center_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->center_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // right_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->right_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * wheel_robot__msg__obstacle_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ObstacleStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("wheel_robot.msg._obstacle_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ObstacleStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  wheel_robot__msg__ObstacleStatus * ros_message = (wheel_robot__msg__ObstacleStatus *)raw_ros_message;
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
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->left_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // center_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->center_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->right_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->left_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // center_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->center_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "center_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->right_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
