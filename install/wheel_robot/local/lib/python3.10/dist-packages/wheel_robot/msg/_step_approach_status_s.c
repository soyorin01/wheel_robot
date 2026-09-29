// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from wheel_robot:msg/StepApproachStatus.idl
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
#include "wheel_robot/msg/detail/step_approach_status__struct.h"
#include "wheel_robot/msg/detail/step_approach_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool wheel_robot__msg__step_approach_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("wheel_robot.msg._step_approach_status.StepApproachStatus", full_classname_dest, 56) == 0);
  }
  wheel_robot__msg__StepApproachStatus * ros_message = _ros_message;
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
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // locked
    PyObject * field = PyObject_GetAttrString(_pymsg, "locked");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->locked = (Py_True == field);
    Py_DECREF(field);
  }
  {  // step_detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "step_detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->step_detected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // detected_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "detected_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->detected_distance = (float)PyFloat_AS_DOUBLE(field);
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
  {  // stop_distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "stop_distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->stop_distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_travel
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_travel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_travel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // traveled
    PyObject * field = PyObject_GetAttrString(_pymsg, "traveled");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->traveled = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // remaining
    PyObject * field = PyObject_GetAttrString(_pymsg, "remaining");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->remaining = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * wheel_robot__msg__step_approach_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StepApproachStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("wheel_robot.msg._step_approach_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StepApproachStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  wheel_robot__msg__StepApproachStatus * ros_message = (wheel_robot__msg__StepApproachStatus *)raw_ros_message;
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
  {  // state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->state.data,
      strlen(ros_message->state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // locked
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->locked ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "locked", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // step_detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->step_detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "step_detected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // detected_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->detected_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "detected_distance", field);
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
  {  // stop_distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->stop_distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stop_distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_travel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_travel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_travel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // traveled
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->traveled);
    {
      int rc = PyObject_SetAttrString(_pymessage, "traveled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // remaining
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->remaining);
    {
      int rc = PyObject_SetAttrString(_pymessage, "remaining", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
