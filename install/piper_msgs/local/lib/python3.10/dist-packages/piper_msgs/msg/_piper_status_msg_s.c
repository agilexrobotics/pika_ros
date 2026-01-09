// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
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
#include "piper_msgs/msg/detail/piper_status_msg__struct.h"
#include "piper_msgs/msg/detail/piper_status_msg__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool piper_msgs__msg__piper_status_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("piper_msgs.msg._piper_status_msg.PiperStatusMsg", full_classname_dest, 47) == 0);
  }
  piper_msgs__msg__PiperStatusMsg * ros_message = _ros_message;
  {  // ctrl_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "ctrl_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->ctrl_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // arm_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "arm_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->arm_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // mode_feedback
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode_feedback");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode_feedback = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // teach_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "teach_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->teach_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // motion_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "motion_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motion_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // trajectory_num
    PyObject * field = PyObject_GetAttrString(_pymsg, "trajectory_num");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->trajectory_num = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // err_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->err_code = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // joint_1_angle_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_1_angle_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->joint_1_angle_limit = (Py_True == field);
    Py_DECREF(field);
  }
  {  // joint_2_angle_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_2_angle_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->joint_2_angle_limit = (Py_True == field);
    Py_DECREF(field);
  }
  {  // joint_3_angle_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_3_angle_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->joint_3_angle_limit = (Py_True == field);
    Py_DECREF(field);
  }
  {  // joint_4_angle_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_4_angle_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->joint_4_angle_limit = (Py_True == field);
    Py_DECREF(field);
  }
  {  // joint_5_angle_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_5_angle_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->joint_5_angle_limit = (Py_True == field);
    Py_DECREF(field);
  }
  {  // joint_6_angle_limit
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_6_angle_limit");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->joint_6_angle_limit = (Py_True == field);
    Py_DECREF(field);
  }
  {  // communication_status_joint_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "communication_status_joint_1");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->communication_status_joint_1 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // communication_status_joint_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "communication_status_joint_2");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->communication_status_joint_2 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // communication_status_joint_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "communication_status_joint_3");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->communication_status_joint_3 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // communication_status_joint_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "communication_status_joint_4");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->communication_status_joint_4 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // communication_status_joint_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "communication_status_joint_5");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->communication_status_joint_5 = (Py_True == field);
    Py_DECREF(field);
  }
  {  // communication_status_joint_6
    PyObject * field = PyObject_GetAttrString(_pymsg, "communication_status_joint_6");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->communication_status_joint_6 = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * piper_msgs__msg__piper_status_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PiperStatusMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("piper_msgs.msg._piper_status_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PiperStatusMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  piper_msgs__msg__PiperStatusMsg * ros_message = (piper_msgs__msg__PiperStatusMsg *)raw_ros_message;
  {  // ctrl_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->ctrl_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ctrl_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arm_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->arm_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arm_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode_feedback
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mode_feedback);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode_feedback", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // teach_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->teach_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "teach_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motion_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->motion_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motion_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trajectory_num
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->trajectory_num);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trajectory_num", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_code
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->err_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_1_angle_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->joint_1_angle_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_1_angle_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_2_angle_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->joint_2_angle_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_2_angle_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_3_angle_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->joint_3_angle_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_3_angle_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_4_angle_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->joint_4_angle_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_4_angle_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_5_angle_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->joint_5_angle_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_5_angle_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_6_angle_limit
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->joint_6_angle_limit ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_6_angle_limit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // communication_status_joint_1
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->communication_status_joint_1 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "communication_status_joint_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // communication_status_joint_2
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->communication_status_joint_2 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "communication_status_joint_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // communication_status_joint_3
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->communication_status_joint_3 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "communication_status_joint_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // communication_status_joint_4
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->communication_status_joint_4 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "communication_status_joint_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // communication_status_joint_5
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->communication_status_joint_5 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "communication_status_joint_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // communication_status_joint_6
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->communication_status_joint_6 ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "communication_status_joint_6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
