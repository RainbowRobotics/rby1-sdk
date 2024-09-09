#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/robot_info.h"

namespace py = pybind11;
using namespace rb;

void bind_battery_info(py::module_& m) {
  py::class_<BatteryInfo>(m, "BatteryInfo")  //
      .def(py::init<>());
}

void bind_power_info(py::module_& m) {
  py::class_<PowerInfo>(m, "PowerInfo")  //
      .def(py::init<>())
      .def_readonly("name", &PowerInfo::name);
}

void bind_joint_info(py::module_& m) {
  py::class_<JointInfo>(m, "JointInfo")
      .def(py::init<>())
      .def_readonly("name", &JointInfo::name)
      .def_readonly("has_brake", &JointInfo::has_brake);
}

void bind_robot_info(py::module_& m) {
  py::class_<RobotInfo>(m, "RobotInfo")
      .def(py::init<>())
      .def_readonly("robot_version", &RobotInfo::robot_version)
      .def_readonly("sdk_commit_id", &RobotInfo::sdk_commit_id)
      .def_readonly("battery_info", &RobotInfo::battery_info)
      .def_readonly("power_infos", &RobotInfo::power_infos)
      .def_readonly("degree_of_freedom", &RobotInfo::degree_of_freedom)
      .def_readonly("joint_infos", &RobotInfo::joint_infos)
      .def_readonly("mobility_joint_idx", &RobotInfo::mobility_joint_idx)
      .def_readonly("body_joint_idx", &RobotInfo::body_joint_idx)
      .def_readonly("head_joint_idx", &RobotInfo::head_joint_idx);
}

void pybind11_robot_info(py::module_& m) {
  bind_battery_info(m);
  bind_power_info(m);
  bind_joint_info(m);
  bind_robot_info(m);
}