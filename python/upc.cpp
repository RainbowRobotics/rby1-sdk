#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

#include "print_helper.h"
#include "rby1-sdk/upc/device.h"
#include "rby1-sdk/upc/leader_arm.h"

namespace py = pybind11;
using namespace rb;
using namespace rb::upc;
using namespace py::literals;

void bind_device(py::module_& m) {
  m.attr("GripperDeviceName") = kGripperDeviceName;
  m.attr("LeaderArmDeviceName") = kLeaderArmDeviceName;
  m.attr("MasterArmDeviceName") = kLeaderArmDeviceName;  // Deprecated alias
  m.attr("LegacyLeaderArmDeviceName") = kLegacyLeaderArmDeviceName;

  m.def("resolve_leader_arm_device_name", &ResolveLeaderArmDeviceName, R"doc(
resolve_leader_arm_device_name() -> str

Return the leader arm device path, falling back to the legacy path if needed.

Returns ``'/dev/rby1_leader_arm'`` if it exists, otherwise falls back to
``'/dev/rby1_master_arm'`` (with a warning printed to stderr). On Windows, or
when neither device is present, returns ``'/dev/rby1_leader_arm'``.

Returns
-------
str
    Resolved device path.
)doc");
  // Deprecated alias kept for backward compatibility with code that used the
  // ``MasterArm`` naming. Both names invoke the same resolver.
  m.def("resolve_master_arm_device_name", &ResolveLeaderArmDeviceName,
        "Deprecated alias for resolve_leader_arm_device_name().");

  m.def("initialize_device", &InitializeDevice, "device_name"_a, R"doc(
initialize_device(device_name)

Initialize a device with the given name.

Sets the latency timer of the device to 1.

Args:
    device_name (str): Name of the device to initialize (e.g., '/dev/ttyUSB0', '/dev/rby1_leader_arm').

Returns:
    bool: True if device initialized successfully, False otherwise.
)doc");
}

void bind_leader_arm(py::module_& m) {
  auto ma_m = py::class_<LeaderArm>(m, "LeaderArm", R"doc(
Leader arm control interface.

This class provides control interface for a leader arm device
with 14 degrees of freedom, including joint control, gravity compensation,
and button/trigger input handling.

Attributes
----------
DOF : int
    Number of degrees of freedom (14).
DeviceCount : int
    Total number of devices including tools (16).
TorqueScaling : float
    Torque scaling factor for gravity compensation (0.5).
MaximumTorque : float
    Maximum allowed torque in Nm (4.0).
RightToolId : int
    Device ID for right tool (0x80).
LeftToolId : int
    Device ID for left tool (0x81).
)doc");

  py::class_<LeaderArm::State>(ma_m, "State", R"doc(
Leader arm state information.

This class represents the current state of the leader arm
including joint positions, velocities, torques, and tool states.

Attributes
----------
q_joint : numpy.ndarray, shape (14,), dtype=float64
    Joint positions [rad].
qvel_joint : numpy.ndarray, shape (14,), dtype=float64
    Joint velocities [rad/s].
torque_joint : numpy.ndarray, shape (14,), dtype=float64
    Joint torques [Nm].
gravity_term : numpy.ndarray, shape (14,), dtype=float64
    Gravity compensation terms.
operating_mode : numpy.ndarray, shape (14,), dtype=int32
    Operating modes for each joint.
button_right : ButtonState
    Right tool button and trigger state.
button_left : ButtonState
    Left tool button and trigger state.
T_right : numpy.ndarray, shape (4, 4), dtype=float64
    Right tool transformation matrix (SE(3)) with respect to the leader arm base.
T_left : numpy.ndarray, shape (4, 4), dtype=float64
    Left tool transformation matrix (SE(3)) with respect to the leader arm base.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``State`` instance with default values.
)doc")
      .def_readonly("q_joint", &LeaderArm::State::q_joint)
      .def_readonly("qvel_joint", &LeaderArm::State::qvel_joint)
      .def_readonly("torque_joint", &LeaderArm::State::torque_joint)
      .def_readonly("gravity_term", &LeaderArm::State::gravity_term)
      .def_readonly("operating_mode", &LeaderArm::State::operating_mode)
      .def_readonly("target_position", &LeaderArm::State::target_position)
      .def_readonly("button_right", &LeaderArm::State::button_right)
      .def_readonly("button_left", &LeaderArm::State::button_left)
      .def_readonly("T_right", &LeaderArm::State::T_right)
      .def_readonly("T_left", &LeaderArm::State::T_left)
      .def("__repr__",
           [](const LeaderArm::State& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             const auto& print_array = [&](const std::string& name, const auto& array) {
               std::string k = name + "=";
               std::string v = np_array_to_string(py::cast(array), Style::Repr);
               ss << k << indent_continuation(v, (int)k.size());
             };

             ss << "State(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             print_array("q_joint", self.q_joint);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("qvel_joint", self.qvel_joint);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("torque_joint", self.torque_joint);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("gravity_term", self.gravity_term);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("operating_mode", self.operating_mode);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_position", self.target_position);
             ss << ",";
             ss << (ml ? "\n" : " ");

             ss << "button_right=" << inline_obj_one_line(py::cast(self.button_right)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "button_left=" << inline_obj_one_line(py::cast(self.button_left)) << ",";
             ss << (ml ? "\n" : " ");

             print_array("T_right", self.T_right);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("T_left", self.T_left);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const LeaderArm::State& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << "LeaderArm.State(q=" << np_array_to_string(py::cast(self.q_joint), Style::Str)
           << ", dq=" << np_array_to_string(py::cast(self.qvel_joint), Style::Str)
           << ", tau=" << np_array_to_string(py::cast(self.torque_joint), Style::Str)
           << ", right_btn=" << inline_obj_one_line(py::cast(self.button_right))
           << ", left_btn=" << inline_obj_one_line(py::cast(self.button_left)) << ")";
        return ss.str();
      });

  py::class_<LeaderArm::ControlInput>(ma_m, "ControlInput", R"doc(
Leader arm control input.

This class represents the control input for the leader arm
including target operating modes, positions, and torques.

Attributes
----------
target_operating_mode : numpy.ndarray, shape (14,), dtype=int32
    Target operating modes for each joint.
target_position : numpy.ndarray, shape (14,), dtype=float64
    Target positions for each joint [rad].
target_torque : numpy.ndarray, shape (14,), dtype=float64
    Target torques for each joint [Nm].
)doc")
      .def(py::init<>(), R"doc(
Construct a ``ControlInput`` instance with default values.
)doc")
      .def_property(
          "target_operating_mode",
          [](LeaderArm::ControlInput& self) -> Eigen::Vector<int, LeaderArm::kDOF>& {
            return self.target_operating_mode;
          },
          [](LeaderArm::ControlInput& self, const Eigen::Vector<int, LeaderArm::kDOF>& mat) {
            self.target_operating_mode = mat;
          },
          py::return_value_policy::reference_internal)
      .def_property(
          "target_position",
          [](LeaderArm::ControlInput& self) -> Eigen::Vector<double, LeaderArm::kDOF>& { return self.target_position; },
          [](LeaderArm::ControlInput& self, const Eigen::Vector<double, LeaderArm::kDOF>& mat) {
            self.target_position = mat;
          },
          py::return_value_policy::reference_internal)
      .def_property(
          "target_torque",
          [](LeaderArm::ControlInput& self) -> Eigen::Vector<double, LeaderArm::kDOF>& { return self.target_torque; },
          [](LeaderArm::ControlInput& self, const Eigen::Vector<double, LeaderArm::kDOF>& mat) {
            self.target_torque = mat;
          },
          py::return_value_policy::reference_internal);

  ma_m  //
      .def_readonly_static("DOF", &LeaderArm::kDOF)
      .def_readonly_static("DeviceCount", &LeaderArm::kDeviceCount)
      .def_readonly_static("TorqueScaling", &LeaderArm::kTorqueScaling)
      .def_readonly_static("MaximumTorque", &LeaderArm::kMaximumTorque)
      .def_readonly_static("RightToolId", &LeaderArm::kRightToolId)
      .def_readonly_static("LeftToolId", &LeaderArm::kLeftToolId)
      .def(py::init<const std::string&>(), "dev_name"_a = "", R"doc(
Construct a ``LeaderArm`` instance.

If no device name is provided, the constructor automatically resolves the device path:
first trying ``/dev/rby1_leader_arm``, then falling back to ``/dev/rby1_master_arm``.

Parameters
----------
dev_name : str, optional
    Device name. Default is auto-resolved.
)doc")
      .def("set_control_period", &LeaderArm::SetControlPeriod, "control_period"_a, R"doc(
Set the control update period.

Parameters
----------
control_period : float
    Control period in seconds.
)doc")
      .def("set_model_path", &LeaderArm::SetModelPath, "model_path"_a, R"doc(
Set the path to the URDF model file.

Parameters
----------
model_path : str
    Path to the URDF model file.
)doc")
      .def("set_torque_constant", &LeaderArm::SetTorqueConstant, "torque_constant"_a, R"doc(
Set torque constant.

Parameters
----------
torque_constant : numpy.ndarray (14, )

Parameters
----------
model_path : str
    Path to the URDF model file.
)doc")
      .def("initialize", &LeaderArm::Initialize, "verbose"_a = false, py::call_guard<py::gil_scoped_release>(), R"doc(
Initialize the leader arm and detect active devices.

Parameters
----------
verbose : bool, optional
    Whether to print verbose output. Default is False.

Returns
-------
list[int]
    List of active device IDs.
)doc")
      .def("start_control", &LeaderArm::StartControl, "control"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Start the control loop.

Parameters
----------
control : callable, optional
    Control callback function that takes State and returns ControlInput.
    If None, no control is applied.

Returns
-------
bool

Examples
--------
>>> leader_arm = rby.upc.LeaderArm()
>>> leader_arm.set_model_path("model.urdf") # path/to/leader_arm_model.urdf
>>> leader_arm.set_control_period(0.01)
>>> active_ids = leader_arm.initialize(verbose=True)
>>> if len(active_ids) != rby.upc.LeaderArm.DeviceCount:
...     print("Error: Mismatch in the number of devices detected for RBY Leader Arm.")
...     exit(1)
>>>
>>> def control(state: rby.upc.LeaderArm.State):
...     with np.printoptions(suppress=True, precision=3, linewidth=300):
...         print(f"--- {datetime.datetime.now().time()} ---")
...         print(f"q: {state.q_joint}")
...         print(f"g: {state.gravity_term}")
...         print(
...             f"right: {state.button_right.button}, left: {state.button_left.button}"
...         )
...     input = rby.upc.LeaderArm.ControlInput()
...     input.target_operating_mode.fill(rby.DynamixelBus.CurrentControlMode)
...     input.target_torque = state.gravity_term
...     return input
>>>
>>> leader_arm.start_control(control)
>>> time.sleep(100)
)doc")
      .def("stop_control", &LeaderArm::StopControl, "torque_disable"_a = false,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Stop the control loop.

Parameters
----------
torque_disable : bool, optional

Returns
-------
bool
)doc")
      .def("enable_torque", &LeaderArm::EnableTorque, py::call_guard<py::gil_scoped_release>(), R"doc(
Enable torque of motors
)doc")
      .def("disable_torque", &LeaderArm::DisableTorque, py::call_guard<py::gil_scoped_release>(), R"doc(
Disable torque of motors
)doc")
      .def("__repr__",
           [](const LeaderArm::ControlInput& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             const auto& print_array = [&](const std::string& name, const auto& array) {
               std::string k = name + "=";
               std::string v = np_array_to_string(py::cast(array), Style::Repr);
               ss << k << indent_continuation(v, (int)k.size());
             };

             ss << "ControlInput(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             print_array("target_operating_mode", self.target_operating_mode);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_position", self.target_position);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_torque", self.target_torque);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const LeaderArm::ControlInput& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << "LeaderArm.ControlInput(target_mode="
           << np_array_to_string(py::cast(self.target_operating_mode), Style::Str)
           << ", target_position=" << np_array_to_string(py::cast(self.target_position), Style::Str)
           << ", target_torque=" << np_array_to_string(py::cast(self.target_torque), Style::Str) << ")";
        return ss.str();
      });
}

void pybind11_upc(py::module_& m) {
  bind_device(m);
  bind_leader_arm(m);
  m.attr("MasterArm") = m.attr("LeaderArm");  // Deprecated alias
}