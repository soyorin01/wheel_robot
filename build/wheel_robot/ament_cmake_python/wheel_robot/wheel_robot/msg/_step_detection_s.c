// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from wheel_robot:msg/StepDetection.idl
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
#include "wheel_robot/msg/detail/step_detection__struct.h"
#include "wheel_robot/msg/detail/step_detection__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool wheel_robot__msg__step_detection__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("wheel_robot.msg._step_detection.StepDetection", full_classname_dest, 45) == 0);
  }
  wheel_robot__msg__StepDetection * ros_message = _ros_message;
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
  {  // detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // height
    PyObject * field = PyObject_GetAttrString(_pymsg, "height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pose_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pose_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // edge_angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "edge_angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->edge_angle_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lateral_offset_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "lateral_offset_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lateral_offset_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // confirm_frames
    PyObject * field = PyObject_GetAttrString(_pymsg, "confirm_frames");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->confirm_frames = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // required_confirm_frames
    PyObject * field = PyObject_GetAttrString(_pymsg, "required_confirm_frames");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->required_confirm_frames = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * wheel_robot__msg__step_detection__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StepDetection */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("wheel_robot.msg._step_detection");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StepDetection");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  wheel_robot__msg__StepDetection * ros_message = (wheel_robot__msg__StepDetection *)raw_ros_message;
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
  {  // detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pose_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // edge_angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->edge_angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "edge_angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lateral_offset_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lateral_offset_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lateral_offset_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confirm_frames
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->confirm_frames);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confirm_frames", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // required_confirm_frames
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->required_confirm_frames);
    {
      int rc = PyObject_SetAttrString(_pymessage, "required_confirm_frames", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
