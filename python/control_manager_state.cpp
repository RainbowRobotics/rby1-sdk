#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iomanip>
#include <sstream>

#include "common.h"
#include "rby1-sdk/control_manager_state.h"

namespace py = pybind11;
using namespace rb;

void bind_control_manager_state(py::module_& m) {
  py::class_<ControlManagerState> cms(m, "ControlManagerState");

  py::enum_<ControlManagerState::State>(cms, "State")
      .value("Unknown", ControlManagerState::State::kUnknown)
      .value("Idle", ControlManagerState::State::kIdle)
      .value("Enabled", ControlManagerState::State::kEnabled)
      .value("MinorFault", ControlManagerState::State::kMinorFault)
      .value("MajorFault", ControlManagerState::State::kMajorFault);

  py::enum_<ControlManagerState::ControlState>(cms, "ControlState")
      .value("Unknown", ControlManagerState::ControlState::kUnknown)
      .value("Idle", ControlManagerState::ControlState::kIdle)
      .value("Executing", ControlManagerState::ControlState::kExecuting)
      .value("Switching", ControlManagerState::ControlState::kSwitching);

  cms.def(py::init<>())
      .def_readonly("state", &ControlManagerState::state)
      .def_readonly("time_scale", &ControlManagerState::time_scale)
      .def_readonly("control_state", &ControlManagerState::control_state)
      .def_readonly("enabled_joint_idx", &ControlManagerState::enabled_joint_idx)
      .def_readonly("unlimited_mode_enabled", &ControlManagerState::unlimited_mode_enabled)
      .def("__repr__", [](const ControlManagerState& self) {
        py::object np = py::module_::import("numpy");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)    //
           << "ControlManagerState("                               //
           << "state=" << to_string(self.state)                    //
           << ", time_scale=" << self.time_scale                   //
           << ", control_state=" << to_string(self.control_state)  //
           << ", enabled_joint_idx=";
        ss << "[";
        for (int i = 0; i < (int)self.enabled_joint_idx.size(); i++) {
          if (i != 0) {
            ss << ", ";
          }
          ss << self.enabled_joint_idx[i];
        }
        ss << "]";
        ss << ", unlimited_mode_enabled=" << (self.unlimited_mode_enabled ? "True" : "False")  //
           << ")";
        return ss.str();
      });
}

void pybind11_control_manager_state(py::module_& m) {
  bind_control_manager_state(m);
}