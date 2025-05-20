#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

#include "rby1-sdk/upc/device.h"
#include "rby1-sdk/upc/master_arm.h"

namespace py = pybind11;
using namespace rb;
using namespace rb::upc;
using namespace py::literals;

void bind_device(py::module_& m) {
  m.attr("GripperDeviceName") = kGripperDeviceName;
  m.attr("MasterArmDeviceName") = kMasterArmDeviceName;

  m.def("initialize_device", &InitializeDevice, "device_name"_a, "Initialize a USB device with the given name");
}

void bind_master_arm(py::module_& m) {
  auto ma_m = py::class_<MasterArm>(m, "MasterArm");

  py::class_<MasterArm::State>(ma_m, "State")
      .def(py::init<>())
      .def_readonly("q_joint", &MasterArm::State::q_joint)
      .def_readonly("qvel_joint", &MasterArm::State::qvel_joint)
      .def_readonly("torque_joint", &MasterArm::State::torque_joint)
      .def_readonly("gravity_term", &MasterArm::State::gravity_term)
      .def_readonly("operating_mode", &MasterArm::State::operating_mode)
      .def_readonly("button_right", &MasterArm::State::button_right)
      .def_readonly("button_left", &MasterArm::State::button_left)
      .def_readonly("T_right", &MasterArm::State::T_right)
      .def_readonly("T_left", &MasterArm::State::T_left);

  py::class_<MasterArm::ControlInput>(ma_m, "ControlInput")
      .def(py::init<>())
      .def_property(
          "target_operating_mode",
          [](MasterArm::ControlInput& self) -> Eigen::Vector<int, MasterArm::kDOF>& {
            return self.target_operating_mode;
          },
          [](MasterArm::ControlInput& self, const Eigen::Vector<int, MasterArm::kDOF>& mat) {
            self.target_operating_mode = mat;
          },
          py::return_value_policy::reference_internal)
      .def_property(
          "target_position",
          [](MasterArm::ControlInput& self) -> Eigen::Vector<double, MasterArm::kDOF>& {
            return self.target_position;
          },
          [](MasterArm::ControlInput& self, const Eigen::Vector<double, MasterArm::kDOF>& mat) {
            self.target_position = mat;
          },
          py::return_value_policy::reference_internal)
      .def_property(
          "target_torque",
          [](MasterArm::ControlInput& self) -> Eigen::Vector<double, MasterArm::kDOF>& {
            return self.target_torque;
          },
          [](MasterArm::ControlInput& self, const Eigen::Vector<double, MasterArm::kDOF>& mat) {
            self.target_torque = mat;
          },
          py::return_value_policy::reference_internal);

  ma_m  //
      .def_readonly_static("DOF", &MasterArm::kDOF)
      .def_readonly_static("DeviceCount", &MasterArm::kDeivceCount)
      .def_readonly_static("TorqueScaling", &MasterArm::kTorqueScaling)
      .def_readonly_static("MaximumTorque", &MasterArm::kMaximumTorque)
      .def_readonly_static("RightToolId", &MasterArm::kRightToolId)
      .def_readonly_static("LeftToolId", &MasterArm::kLeftToolId)
      .def(py::init<const std::string&>(), "dev_name"_a = kMasterArmDeviceName)
      .def("set_control_period", &MasterArm::SetControlPeriod, "control_period"_a)
      .def("set_model_path", &MasterArm::SetModelPath, "model_path"_a)
      .def("initialize", &MasterArm::Initialize, "verbose"_a = false, py::call_guard<py::gil_scoped_release>())
      .def("start_control", &MasterArm::StartControl, "control"_a, py::call_guard<py::gil_scoped_release>())
      .def("stop_control", &MasterArm::StopControl, py::call_guard<py::gil_scoped_release>());
}

void pybind11_upc(py::module_& m) {
  bind_device(m);
  bind_master_arm(m);
}