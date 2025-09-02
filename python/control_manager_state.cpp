#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iomanip>
#include <sstream>

#include "common.h"
#include "print_helper.h"
#include "rby1-sdk/control_manager_state.h"

namespace py = pybind11;
using namespace rb;
using rb::print::indent_continuation;
using rb::print::inline_obj;
using rb::print::np_array_to_string;
using rb::print::np_shape_dtype;
using rb::print::Style;

void bind_control_manager_state(py::module_& m) {
  py::class_<ControlManagerState> cms(m, "ControlManagerState", R"doc(
Control manager state information.

This class represents the current state of the robot's control manager,
including operational status, time scaling, and joint enablement.

Attributes
----------
state : State
    Current operational state of the control manager.
time_scale : float
    Time scaling factor for motion execution (0.0 to 1.0).
control_state : ControlState
    Current control execution state.
enabled_joint_idx : list[int]
    Indices of currently enabled joints.
unlimited_mode_enabled : bool
    Whether unlimited mode is currently enabled.
)doc");

  py::enum_<ControlManagerState::State>(cms, "State", R"doc(
Control manager operational state enumeration.

Defines the possible operational states of the control manager,
from normal operation to various fault conditions.

Members
-------
Unknown : int
    State is unknown or undefined.
Idle : int
    Control manager is idle and ready for commands.
Enabled : int
    Control manager is enabled and actively controlling.
MinorFault : int
    Minor fault condition detected.
MajorFault : int
    Major fault condition detected.
)doc")
      .value("Unknown", ControlManagerState::State::kUnknown, R"doc(
State is unknown or undefined.
)doc")
      .value("Idle", ControlManagerState::State::kIdle, R"doc(
Control manager is idle and ready for commands.
)doc")
      .value("Enabled", ControlManagerState::State::kEnabled, R"doc(
Control manager is enabled and actively controlling.
)doc")
      .value("MinorFault", ControlManagerState::State::kMinorFault, R"doc(
Minor fault condition detected.
)doc")
      .value("MajorFault", ControlManagerState::State::kMajorFault, R"doc(
Major fault condition detected.
)doc");

  py::enum_<ControlManagerState::ControlState>(cms, "ControlState", R"doc(
Control execution state enumeration.

Defines the current state of control execution within the control manager.

Members
-------
Unknown : int
    Control state is unknown or undefined.
Idle : int
    No control commands are being executed.
Executing : int
    Control commands are actively being executed.
Switching : int
    Control is switching between different modes or commands.
)doc")
      .value("Unknown", ControlManagerState::ControlState::kUnknown, R"doc(
Control state is unknown or undefined.
)doc")
      .value("Idle", ControlManagerState::ControlState::kIdle, R"doc(
No control commands are being executed.
)doc")
      .value("Executing", ControlManagerState::ControlState::kExecuting, R"doc(
Control commands are actively being executed.
)doc")
      .value("Switching", ControlManagerState::ControlState::kSwitching, R"doc(
Control is switching between different modes or commands.
)doc");

  cms.def(py::init<>(), R"doc(
Construct a ControlManagerState instance with default values.
)doc")
      .def_readonly("state", &ControlManagerState::state, R"doc(
Current operational state of the control manager.

Type
----
State
    Enumeration value indicating the operational state.
)doc")
      .def_readonly("time_scale", &ControlManagerState::time_scale, R"doc(
Time scaling factor for motion execution.

Type
----
float
    Time scaling factor from 0.0 (stopped) to 1.0 (normal speed).
)doc")
      .def_readonly("control_state", &ControlManagerState::control_state, R"doc(
Current control execution state.

Type
----
ControlState
    Enumeration value indicating the control execution state.
)doc")
      .def_readonly("enabled_joint_idx", &ControlManagerState::enabled_joint_idx, R"doc(
Indices of currently enabled joints.

Type
----
list[int]
    List of joint indices that are currently enabled for control.
)doc")
      .def_readonly("unlimited_mode_enabled", &ControlManagerState::unlimited_mode_enabled, R"doc(
Whether unlimited mode is currently enabled.

Type
----
bool
    ``True`` if unlimited mode is enabled, ``False`` otherwise.
)doc")
      .def("__repr__",
           [](const ControlManagerState& self) {
             using namespace rb::print;

             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "ControlManagerState(" << FIRST                                                        //
                 << "state=" << to_string(self.state) << SEP                                               //
                 << "time_scale=" << format_number(self.time_scale, Style::Repr) << SEP                    //
                 << "control_state=" << to_string(self.control_state) << SEP                               //
                 << "enabled_joint_idx=" << inline_obj(py::cast(self.enabled_joint_idx)) << SEP            //
                 << "unlimited_mode_enabled=" << (self.unlimited_mode_enabled ? "True" : "False") << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const ControlManagerState& self) {
        std::ostringstream out;
        out << "ControlManagerState("
            << "state=" << to_string(self.state) << ", "
            << "ctrl=" << to_string(self.control_state) << ", "
            << "time_scale=" << format_number(self.time_scale, Style::Str) << ", "
            << "enabled=" << self.enabled_joint_idx.size() << (self.unlimited_mode_enabled ? ", unlimited" : "") << ")";
        return out.str();
      });
}

void pybind11_control_manager_state(py::module_& m) {
  bind_control_manager_state(m);
}