// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_interfaces:msg/VelocityData.idl
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
#include "robot_interfaces/msg/detail/velocity_data__struct.h"
#include "robot_interfaces/msg/detail/velocity_data__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_interfaces__msg__velocity_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("robot_interfaces.msg._velocity_data.VelocityData", full_classname_dest, 48) == 0);
  }
  robot_interfaces__msg__VelocityData * ros_message = _ros_message;
  {  // angular_speed_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_speed_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_speed_left = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_speed_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_speed_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_speed_right = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // servo_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->servo_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_interfaces__msg__velocity_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VelocityData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_interfaces.msg._velocity_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VelocityData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_interfaces__msg__VelocityData * ros_message = (robot_interfaces__msg__VelocityData *)raw_ros_message;
  {  // angular_speed_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_speed_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_speed_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_speed_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_speed_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_speed_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // servo_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->servo_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
