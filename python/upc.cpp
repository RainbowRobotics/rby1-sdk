#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

#include "print_helper.h"
#include "rby1-sdk/upc/device.h"
#include "rby1-sdk/upc/master_arm.h"

namespace py = pybind11;
using namespace rb;
using namespace rb::upc;
using namespace py::literals;

void bind_device(py::module_& m) {
  m.attr("GripperDeviceName") = kGripperDeviceName;
  m.attr("MasterArmDeviceName") = kMasterArmDeviceName;

  m.def("initialize_device", &InitializeDevice, "device_name"_a, R"doc(
initialize_device(device_name)

Initialize a device with the given name.

Sets the latency timer of the device to 1.

Args:
    device_name (str): Name of the device to initialize (e.g., '/dev/ttyUSB0', '/dev/rby1_master_arm').

Returns:
    bool: True if device initialized successfully, False otherwise.
)doc");
}

void bind_master_arm(py::module_& m) {
  auto ma_m = py::class_<MasterArm>(m, "MasterArm", R"doc(
Master arm control interface.

This class provides control interface for a master arm device
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

  py::class_<MasterArm::State>(ma_m, "State", R"doc(
Master arm state information.

This class represents the current state of the master arm
including joint positions, velocities, torques, and tool states.

Attributes
----------
q_joint : numpy.ndarray
    Joint positions, shape (14,), dtype=float64.
qvel_joint : numpy.ndarray
    Joint velocities, shape (14,), dtype=float64.
torque_joint : numpy.ndarray
    Joint torques, shape (14,), dtype=float64.
gravity_term : numpy.ndarray
    Gravity compensation terms, shape (14,), dtype=float64.
operating_mode : numpy.ndarray
    Operating modes for each joint, shape (14,), dtype=int32.
button_right : ButtonState
    Right tool button and trigger state.
button_left : ButtonState
    Left tool button and trigger state.
T_right : numpy.ndarray
    Right tool transformation matrix, shape (4, 4), dtype=float64.
T_left : numpy.ndarray
    Left tool transformation matrix, shape (4, 4), dtype=float64.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``State`` instance with default values.
)doc")
      .def_readonly("q_joint", &MasterArm::State::q_joint, R"doc(
Joint positions.

Type
----
numpy.ndarray
    Shape (14,), dtype=float64. Joint positions in radians.
)doc")
      .def_readonly("qvel_joint", &MasterArm::State::qvel_joint, R"doc(
Joint velocities.

Type
----
numpy.ndarray
    Shape (14,), dtype=float64. Joint velocities in rad/s.
)doc")
      .def_readonly("torque_joint", &MasterArm::State::torque_joint, R"doc(
Joint torques.

Type
----
numpy.ndarray
    Shape (14,), dtype=float64. Joint torques in Nm.
)doc")
      .def_readonly("gravity_term", &MasterArm::State::gravity_term, R"doc(
Gravity compensation terms.

Type
----
numpy.ndarray
    Shape (14,), dtype=float64. Gravity compensation torques in Nm.
)doc")
      .def_readonly("operating_mode", &MasterArm::State::operating_mode, R"doc(
Operating modes for each joint.

Type
----
numpy.ndarray
    Shape (14,), dtype=int32. Operating mode for each joint.
)doc")
      .def_readonly("button_right", &MasterArm::State::button_right, R"doc(
Right tool button and trigger state.

Type
----
ButtonState
    Button and trigger state for right tool.
)doc")
      .def_readonly("button_left", &MasterArm::State::button_left, R"doc(
Left tool button and trigger state.

Type
----
ButtonState
    Button and trigger state for left tool.
)doc")
      .def_readonly("T_right", &MasterArm::State::T_right, R"doc(
Right tool transformation matrix.

Type
----
numpy.ndarray
    Shape (4, 4), dtype=float64. Homogeneous transformation matrix.
)doc")
      .def_readonly("T_left", &MasterArm::State::T_left, R"doc(
Left tool transformation matrix.

Type
----
numpy.ndarray
    Shape (4, 4), dtype=float64. Homogeneous transformation matrix.
)doc")
      .def("__repr__",
           [](const MasterArm::State& self) {
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
      .def("__str__", [](const MasterArm::State& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << "MasterArm.State(q=" << np_array_to_string(py::cast(self.q_joint), Style::Str)
           << ", dq=" << np_array_to_string(py::cast(self.qvel_joint), Style::Str)
           << ", tau=" << np_array_to_string(py::cast(self.torque_joint), Style::Str)
           << ", right_btn=" << inline_obj_one_line(py::cast(self.button_right))
           << ", left_btn=" << inline_obj_one_line(py::cast(self.button_left)) << ")";
        return ss.str();
      });

  py::class_<MasterArm::ControlInput>(ma_m, "ControlInput", R"doc(
Master arm control input.

This class represents the control input for the master arm
including target operating modes, positions, and torques.

Attributes
----------
target_operating_mode : numpy.ndarray
    Target operating modes for each joint, shape (14,), dtype=int32.
target_position : numpy.ndarray
    Target positions for each joint, shape (14,), dtype=float64.
target_torque : numpy.ndarray
    Target torques for each joint, shape (14,), dtype=float64.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``ControlInput`` instance with default values.
)doc")
      .def_property(
          "target_operating_mode",
          [](MasterArm::ControlInput& self) -> Eigen::Vector<int, MasterArm::kDOF>& {
            return self.target_operating_mode;
          },
          [](MasterArm::ControlInput& self, const Eigen::Vector<int, MasterArm::kDOF>& mat) {
            self.target_operating_mode = mat;
          },
          py::return_value_policy::reference_internal, R"doc(
Target operating modes for each joint.

Type
----
numpy.ndarray
    Shape (14,), dtype=int32. Target operating mode for each joint.
)doc")
      .def_property(
          "target_position",
          [](MasterArm::ControlInput& self) -> Eigen::Vector<double, MasterArm::kDOF>& { return self.target_position; },
          [](MasterArm::ControlInput& self, const Eigen::Vector<double, MasterArm::kDOF>& mat) {
            self.target_position = mat;
          },
          py::return_value_policy::reference_internal, R"doc(
Target positions for each joint.

Type
----
numpy.ndarray
    Shape (14,), dtype=float64. Target position for each joint in radians.
)doc")
      .def_property(
          "target_torque",
          [](MasterArm::ControlInput& self) -> Eigen::Vector<double, MasterArm::kDOF>& { return self.target_torque; },
          [](MasterArm::ControlInput& self, const Eigen::Vector<double, MasterArm::kDOF>& mat) {
            self.target_torque = mat;
          },
          py::return_value_policy::reference_internal, R"doc(
Target torques for each joint.

Type
----
numpy.ndarray
    Shape (14,), dtype=float64. Target torque for each joint in Nm.
)doc");

  ma_m  //
      .def_readonly_static("DOF", &MasterArm::kDOF, R"doc(
Number of degrees of freedom.

Type
----
int
    Number of degrees of freedom (14).
)doc")
      .def_readonly_static("DeviceCount", &MasterArm::kDeivceCount, R"doc(
Total number of devices.

Type
----
int
    Total number of devices including tools (16).
)doc")
      .def_readonly_static("TorqueScaling", &MasterArm::kTorqueScaling, R"doc(
Torque scaling factor.

Type
----
float
    Torque scaling factor for gravity compensation (0.5).
)doc")
      .def_readonly_static("MaximumTorque", &MasterArm::kMaximumTorque, R"doc(
Maximum allowed torque.

Type
----
float
    Maximum allowed torque in Nm (4.0).
)doc")
      .def_readonly_static("RightToolId", &MasterArm::kRightToolId, R"doc(
Right tool device ID.

Type
----
int
    Device ID for right tool (0x80).
)doc")
      .def_readonly_static("LeftToolId", &MasterArm::kLeftToolId, R"doc(
Left tool device ID.

Type
----
int
    Device ID for left tool (0x81).
)doc")
      .def(py::init<const std::string&>(), "dev_name"_a = kMasterArmDeviceName, R"doc(
Construct a ``MasterArm`` instance.

Parameters
----------
dev_name : str, optional
    Device name. Default is ``/dev/rby1_master_arm``'.
)doc")
      .def("set_control_period", &MasterArm::SetControlPeriod, "control_period"_a, R"doc(
set_control_period(control_period)

Set the control update period.

Parameters
----------
control_period : float
    Control period in seconds.
)doc")
      .def("set_model_path", &MasterArm::SetModelPath, "model_path"_a, R"doc(
Set the path to the URDF model file.

Parameters
----------
model_path : str
    Path to the URDF model file.
)doc")
      .def("initialize", &MasterArm::Initialize, "verbose"_a = false, py::call_guard<py::gil_scoped_release>(), R"doc(
Initialize the master arm and detect active devices.

Parameters
----------
verbose : bool, optional
    Whether to print verbose output. Default is False.

Returns
-------
list[int]
    List of active device IDs.
)doc")
      .def("start_control", &MasterArm::StartControl, "control"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Start the control loop.

Parameters
----------
control : callable, optional
    Control callback function that takes State and returns ControlInput.
    If None, no control is applied.
)doc")
      .def("stop_control", &MasterArm::StopControl, py::call_guard<py::gil_scoped_release>(), R"doc(
Stop the control loop.
)doc")
      .def("__repr__",
           [](const MasterArm::ControlInput& self) {
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
      .def("__str__", [](const MasterArm::ControlInput& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << "MasterArm.ControlInput(target_mode="
           << np_array_to_string(py::cast(self.target_operating_mode), Style::Str)
           << ", target_position=" << np_array_to_string(py::cast(self.target_position), Style::Str)
           << ", target_torque=" << np_array_to_string(py::cast(self.target_torque), Style::Str) << ")";
        return ss.str();
      });
}

void pybind11_upc(py::module_& m) {
  bind_device(m);
  bind_master_arm(m);
}