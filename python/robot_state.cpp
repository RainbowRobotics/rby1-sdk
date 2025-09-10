#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iomanip>

#include "common.h"
#include "print_helper.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_state.h"

namespace py = pybind11;
using namespace rb;

void bind_system_stat(pybind11::module_& m) {
  py::class_<SystemStat>(m, "SystemStat", R"doc(
System statistics information.

Provides information about system performance including CPU and memory usage.

Attributes
----------
cpu_usage : float
    CPU usage percentage [0.0, 100.0].
memory_usage : float
    Memory usage percentage [0.0, 100.0].
uptime : float
    System uptime in seconds.
program_uptime : float
    Program uptime in seconds.
  )doc")
      .def(py::init<>(), R"doc(
      Construct a ``SystemStat`` instance.
)doc")
      .def_readonly("cpu_usage", &SystemStat::cpu_usage)
      .def_readonly("memory_usage", &SystemStat::memory_usage)
      .def_readonly("uptime", &SystemStat::uptime)
      .def_readonly("program_uptime", &SystemStat::program_uptime)
      .def("__repr__",
           [](const SystemStat& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;
             ss << "SystemStat(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "cpu_usage=" << format_number(self.cpu_usage, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "memory_usage=" << format_number(self.memory_usage, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "uptime=" << format_number(self.uptime, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "program_uptime=" << format_number(self.program_uptime, Style::Repr);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const SystemStat& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "SystemStat("
            << "CPU=" << format_number(self.cpu_usage, Style::Str) << "%, "
            << "MEM=" << format_number(self.memory_usage, Style::Str) << "%, "
            << "up=" << format_number(self.uptime, Style::Str) << "s, "
            << "prog=" << format_number(self.program_uptime, Style::Str) << "s)";
        return out.str();
      });
}

void bind_battery_state(pybind11::module_& m) {
  py::class_<BatteryState>(m, "BatteryState", R"doc(
Battery state information.

Provides information about battery status including voltage, current, and charge level.

Attributes
----------
voltage : float
    Battery voltage in V.
current : float
    Battery current [A].
level_percent : float
    Battery charge level percentage (0.0 to 100.0).
  )doc")
      .def(py::init<>(), R"doc(
      Construct a ``BatteryState`` instance.
)doc")
      .def_readonly("voltage", &BatteryState::voltage)
      .def_readonly("current", &BatteryState::current)
      .def_readonly("level_percent", &BatteryState::level_percent)
      .def("__repr__",
           [](const BatteryState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             ss << "BatteryState(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "voltage=" << format_number(self.voltage, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "current=" << format_number(self.current, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "level_percent=" << format_number(self.level_percent, Style::Repr);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const BatteryState& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "BatteryState(level=" << format_number(self.level_percent, Style::Str) << "%, "
            << "voltage=" << format_number(self.voltage, Style::Str) << "V, "
            << "current=" << format_number(self.current, Style::Str) << "A)";
        return out.str();
      });
}

void bind_power_state(pybind11::module_& m) {
  auto ps = py::class_<PowerState>(m, "PowerState", R"doc(
  Power state information.

  Provides information about power supply status and voltage.

  Attributes
  ----------
  state : State
      Current power state.
  voltage : float
      Power supply voltage in volts.
  )doc");

  py::enum_<PowerState::State>(ps, "State", R"doc(
Power state enumeration.

Defines the possible power states.

Members
-------
Unknown : int
    Power state is unknown.
PowerOff : int
    Power is off.
PowerOn : int
    Power is on.
)doc")
      .value("Unknown", PowerState::State::kUnknown)
      .value("PowerOff", PowerState::State::kPowerOff)
      .value("PowerOn", PowerState::State::kPowerOn);

  ps.def(py::init<>(), R"doc(
Construct a ``PowerState`` instance.
)doc")
      .def_readonly("state", &PowerState::state)
      .def_readonly("voltage", &PowerState::voltage)
      .def("__repr__",
           [](const PowerState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             ss << "PowerState(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "state=PowerState.State." << rb::to_string(self.state) << ",";
             ss << (ml ? "\n" : " ");

             ss << "voltage=" << format_number(self.voltage, Style::Repr);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const PowerState& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "PowerState(" << rb::to_string(self.state) << " @ " << format_number(self.voltage, Style::Str) << "V)";
        return out.str();
      });
}

void bind_emo_state(pybind11::module_& m) {
  auto ps = py::class_<EMOState>(m, "EMOState", R"doc(
Emergency stop button state.

Provides information about emergency stop button status.

Attributes
----------
state : State
    Current emergency stop button state.
)doc");

  py::enum_<EMOState::State>(ps, "State", R"doc(
Emergency stop button state enumeration.

Defines the possible emergency stop button states.

Members
-------
Released : int
    Emergency stop button is released.
Pressed : int
    Emergency stop button is pressed.
)doc")
      .value("Released", EMOState::State::kReleased)
      .value("Pressed", EMOState::State::kPressed);

  ps.def(py::init<>())
      .def_readonly("state", &EMOState::state)
      .def("__repr__",
           [](const EMOState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             ss << "EMOState(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "state=EMOState.State." << rb::to_string(self.state);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__",
           [](const EMOState& self) { return "EMOState(state=EMOState.State." + rb::to_string(self.state) + ")"; });
}

void bind_tool_flange(py::module_& m) {
  py::class_<ToolFlangeState>(m, "ToolFlangeState", R"doc(
Tool flange state information.

Provides information about tool flange sensors and outputs.

Attributes
----------
time_since_last_update : datetime.timedelta
    Time since last update as a Python timedelta.
gyro : numpy.ndarray, shape (3,), dtype=float64
    Gyroscope readings [rad/s].
acceleration : numpy.ndarray, shape (3,), dtype=float64
    Acceleration readings in m/s².
switch_A : bool
    Status of switch A.
output_voltage : float
    Output voltage in volts.
digital_input_A : bool
    Status of digital input A.
digital_input_B : bool
    Status of digital input B.
digital_output_A : bool
    Status of digital output A.
digital_output_B : bool
    Status of digital output B.
)doc")
      .def(py::init<>(), R"doc(
Construct a ToolFlangeState instance.
)doc")
      .def_property_readonly(
          "time_since_last_update",
          [](const ToolFlangeState& self) { return timespec_to_nanoseconds(self.time_since_last_update); })
      .def_readonly("gyro", &ToolFlangeState::gyro)
      .def_readonly("acceleration", &ToolFlangeState::acceleration)
      .def_readonly("switch_A", &ToolFlangeState::switch_A)
      .def_readonly("output_voltage", &ToolFlangeState::output_voltage)
      .def_readonly("digital_input_A", &ToolFlangeState::digital_input_A)
      .def_readonly("digital_input_B", &ToolFlangeState::digital_input_B)
      .def_readonly("digital_output_A", &ToolFlangeState::digital_output_A)
      .def_readonly("digital_output_B", &ToolFlangeState::digital_output_B)
      .def("__repr__",
           [](const ToolFlangeState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;
             const auto& print_array = [&](const std::string& name, const auto& array) {
               std::string k = name + "=";
               std::string v = np_array_to_string(py::cast(array), Style::Repr);
               ss << k << indent_continuation(v, (int)k.size());
             };

             ss << "ToolFlangeState(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             print_array("gyro", self.gyro);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("acceleration", self.acceleration);
             ss << ",";
             ss << (ml ? "\n" : " ");

             ss << "switch_A=" << (self.switch_A ? "True" : "False") << ",";
             ss << (ml ? "\n" : " ");

             ss << "output_voltage=" << format_number(self.output_voltage, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "digital_input_A=" << (self.digital_input_A ? "True" : "False") << ",";
             ss << (ml ? "\n" : " ");

             ss << "digital_input_B=" << (self.digital_input_B ? "True" : "False") << ",";
             ss << (ml ? "\n" : " ");

             ss << "digital_output_A=" << (self.digital_output_A ? "True" : "False") << ",";
             ss << (ml ? "\n" : " ");

             ss << "digital_output_B=" << (self.digital_output_B ? "True" : "False");
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const ToolFlangeState& self) {
        std::ostringstream ss;
        ss << "ToolFlangeState(swA=" << (self.switch_A ? "True" : "False") << ", Vout=" << self.output_voltage << "V"
           << ", inA/B=" << (self.digital_input_A ? "1" : "0") << "/" << (self.digital_input_B ? "1" : "0")
           << ", outA/B=" << (self.digital_output_A ? "1" : "0") << "/" << (self.digital_output_B ? "1" : "0") << ")";
        return ss.str();
      });
}

void bind_ft_sensor(py::module_& m) {
  py::class_<FTSensorData>(m, "FTSensorData", R"doc(
Force/Torque sensor data.

Provides information about force and torque measurements from a sensor.

Attributes
----------
time_since_last_update : datetime.timedelta
    Time since last update as a Python timedelta.
force : numpy.ndarray, shape (3,), dtype=float64
    Force readings in N.
torque : numpy.ndarray, shape (3,), dtype=float64
    Torque readings [Nm].
)doc")
      .def(py::init<>(), R"doc(
Construct a FTSensorData instance.
)doc")
      .def_property_readonly(
          "time_since_last_update",
          [](const FTSensorData& self) { return timespec_to_nanoseconds(self.time_since_last_update); })
      .def_readonly("force", &FTSensorData::force)
      .def_readonly("torque", &FTSensorData::torque)
      .def("__repr__",
           [](const FTSensorData& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;
             const auto& print_array = [&](const std::string& name, const auto& array) {
               std::string k = name + "=";
               std::string v = np_array_to_string(py::cast(array), Style::Repr);
               ss << k << indent_continuation(v, (int)k.size());
             };

             ss << "FTSensorData(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             print_array("force", self.force);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("torque", self.torque);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const FTSensorData& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << "FTSensorData(F=" << np_array_to_string(py::cast(self.force), Style::Str)
           << ", T=" << np_array_to_string(py::cast(self.torque), Style::Str) << ")";
        return ss.str();
      });
}

void bind_joint_state(py::module_& m) {
  auto js = py::class_<JointState>(m, "JointState", R"doc(
Joint state information.

Provides information about the state of a single joint, including its readiness,
FET state, run state, initialization state, motor type, motor state, power,
position, velocity, current, torque, and target values.

Attributes
----------
time_since_last_update : datetime.timedelta
    Time since last update as a Python timedelta.
is_ready : bool
    Indicates if the joint is ready to operate.
fet_state : FETState
    Current FET state of the joint.
run_state : RunState
    Current run state of the joint.
init_state : InitializationState
    Current initialization state of the joint.
motor_type : str
    Type of the joint's motor.
motor_state : str
    Current state of the joint's motor.
power_on : bool
    Indicates if the joint's power is on.
position : float
    Current position of the joint [rad].
velocity : float
    Current velocity of the joint [rad/s].
current : float
    Current current of the joint [A].
torque : float
    Current torque of the joint in Nm.
target_position : float
    Target position of the joint [rad].
target_velocity : float
    Target velocity of the joint [rad/s].
target_feedback_gain : float
    Target feedback gain of the joint.
target_feedforward_torque : float
    Target feedforward torque of the joint.
temperature : float
    Current temperature of the joint in °C.
)doc");

  py::enum_<JointState::FETState>(js, "FETState", R"doc(
FET (power stage) state.

Members
-------
Unknown : int
    State is unknown or not reported.
On : int
    Power stage is enabled.
Off : int
    Power stage is disabled.
)doc")
      .value("Unknown", JointState::FETState::kUnknown)
      .value("On", JointState::FETState::kOn)
      .value("Off", JointState::FETState::kOff);

  py::enum_<JointState::RunState>(js, "RunState", R"doc(
Current run state of the joint.

Members
-------
Unknown : int
    State is unknown or not reported.
ControlOn : int
    Closed-loop control is enabled (actively controlling).
ControlOff : int
    Closed-loop control is disabled.
)doc")
      .value("Unknown", JointState::RunState::kUnknown)
      .value("ControlOn", JointState::RunState::kControlOn)
      .value("ControlOff", JointState::RunState::kControlOff);

  py::enum_<JointState::InitializationState>(js, "InitializationState", R"doc(
Initialization state of the joint.

Members
-------
Unknown : int
    Initialization state is unknown.
Initialized : int
    Joint has been initialized/homed.
Uninitialized : int
    Joint has not been initialized/homed yet.
)doc")
      .value("Unknown", JointState::InitializationState::kUnknown)
      .value("Initialized", JointState::InitializationState::kInitialized)
      .value("Uninitialized", JointState::InitializationState::kUninitialized);

  js.def(py::init<>())
      .def_property_readonly(
          "time_since_last_update",
          [](const JointState& self) { return timespec_to_nanoseconds(self.time_since_last_update); })
      .def_readonly("is_ready", &JointState::is_ready)
      .def_readonly("fet_state", &JointState::fet_state)
      .def_readonly("run_state", &JointState::run_state)
      .def_readonly("init_state", &JointState::init_state)
      .def_readonly("motor_type", &JointState::motor_type)
      .def_readonly("motor_state", &JointState::motor_state)
      .def_readonly("power_on", &JointState::power_on)
      .def_readonly("position", &JointState::position)
      .def_readonly("velocity", &JointState::velocity)
      .def_readonly("current", &JointState::current)
      .def_readonly("torque", &JointState::torque)
      .def_readonly("target_position", &JointState::target_position)
      .def_readonly("target_velocity", &JointState::target_velocity)
      .def_readonly("target_feedback_gain", &JointState::target_feedback_gain)
      .def_readonly("target_feedforward_torque", &JointState::target_feedforward_torque)
      .def_readonly("temperature", &JointState::temperature)
      .def("__repr__",
           [](const JointState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;
             ss << "JointState(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "time_since_last_update=" << inline_obj(timespec_to_nanoseconds(self.time_since_last_update)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "is_ready=" << (self.is_ready ? "True" : "False") << ",";
             ss << (ml ? "\n" : " ");

             ss << "fet_state=JointState.FETState." << rb::to_string(self.fet_state) << ",";
             ss << (ml ? "\n" : " ");

             ss << "run_state=JointState.RunState." << rb::to_string(self.run_state) << ",";
             ss << (ml ? "\n" : " ");

             ss << "init_state=JointState.InitializationState." << rb::to_string(self.init_state) << ",";
             ss << (ml ? "\n" : " ");

             ss << "motor_type=" << self.motor_type << ",";
             ss << (ml ? "\n" : " ");

             ss << "motor_state=" << self.motor_state << ",";
             ss << (ml ? "\n" : " ");

             ss << "power_on=" << (self.power_on ? "True" : "False") << ",";
             ss << (ml ? "\n" : " ");

             ss << "position=" << format_number(self.position, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "velocity=" << format_number(self.velocity, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "current=" << format_number(self.current, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "torque=" << format_number(self.torque, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "target_position=" << format_number(self.target_position, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "target_velocity=" << format_number(self.target_velocity, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "target_feedback_gain=" << self.target_feedback_gain << ",";
             ss << (ml ? "\n" : " ");

             ss << "target_feedforward_torque=" << format_number(self.target_feedforward_torque, Style::Repr) << ",";
             ss << (ml ? "\n" : " ");

             ss << "temperature=" << self.temperature;
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const JointState& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << "JointState(position=" << format_number(self.position, Style::Str)
           << ", velocity=" << format_number(self.velocity, Style::Str)
           << ", torque=" << format_number(self.torque, Style::Str) << ", " << rb::to_string(self.run_state)
           << (self.is_ready ? ", ready" : ", not-ready") << ")";
        return ss.str();
      });
}

template <typename T>
void bind_robot_state(py::module_& m, const std::string& robot_state_name) {
  py::class_<RobotState<T>>(m, robot_state_name.c_str(), R"doc(
Robot state information.

Provides information about the overall state of the robot, including system
statistics, battery state, power states, emergency stop button states, joint
states, tool flange states, force/torque sensor data, and odometry.

Attributes
----------
timestamp : datetime.datetime
    Timestamp of the robot state (system clock).
system_stat : SystemStat
    System statistics (CPU, memory, uptimes).
battery_state : BatteryState
    Battery voltage/current/level.
power_states : list of PowerState
    Power supply states for each module.
emo_states : list of EMOState
    Emergency-stop button states for each module.
joint_states : list of JointState
    Per-joint status and measurements.

tool_flange_right : ToolFlangeState
    Tool flange sensor and IO state for the right arm.
tool_flange_left : ToolFlangeState
    Tool flange sensor and IO state for the left arm.
ft_sensor_right : FTSensorData
    Force/torque readings on the right arm.
ft_sensor_left : FTSensorData
    Force/torque readings on the left arm.

is_ready : numpy.ndarray of bool, shape (DOF,)
    Whether each joint is ready.
position : numpy.ndarray, shape (DOF,)
    Joint positions [rad].
velocity : numpy.ndarray, shape (DOF,)
    Joint velocities [rad/s].
current : numpy.ndarray, shape (DOF,)
    Joint currents [A].
torque : numpy.ndarray, shape (DOF,)
    Joint torques in Nm.

target_position : numpy.ndarray, shape (DOF,)
    Target joint positions [rad].
target_velocity : numpy.ndarray, shape (DOF,)
    Target joint velocities [rad/s].
target_feedback_gain : numpy.ndarray, shape (DOF,)
    Target feedback gains per joint. Range is [0, 10].
target_feedforward_torque : numpy.ndarray, shape (DOF,)
    Target feedforward torques per joint in Nm.

odometry : numpy.ndarray, shape (3, 3)
    Base pose as a 2D homogeneous transform (SE(2)): rotation (R) and translation (t).
center_of_mass : numpy.ndarray, shape (3,)
    Center of mass position in base frame in meters.

collisions : list of CollisionResult
    Detected collisions or nearest link pairs (links, closest points, signed distance).
temperature : numpy.ndarray, shape (DOF,)
    Joint temperatures in degrees Celsius.
gravity : numpy.ndarray, shape (DOF,)
    Gravity compensation torques per joint in Nm.

Notes
-----
Unless noted otherwise, joint-wise vectors are NumPy arrays with shape ``(DOF,)``,
where ``DOF = robot.model().robot_dof``.
``odometry`` uses a 2D homogeneous transform (SE(2)).
``signed distance`` is negative when penetrating.
)doc")
      .def(py::init<>(), R"doc(
Construct a RobotState instance.
)doc")
      .def_property_readonly(
          "timestamp", [](const RobotState<T>& self) { return timespec_to_time_point(self.timestamp); })
      .def_readonly("system_stat", &RobotState<T>::system_stat)
      .def_readonly("battery_state", &RobotState<T>::battery_state)
      .def_readonly("power_states", &RobotState<T>::power_states)
      .def_readonly("emo_states", &RobotState<T>::emo_states)
      .def_readonly("joint_states", &RobotState<T>::joint_states)
      .def_readonly("tool_flange_right", &RobotState<T>::tool_flange_right)
      .def_readonly("tool_flange_left", &RobotState<T>::tool_flange_left)
      .def_readonly("ft_sensor_right", &RobotState<T>::ft_sensor_right)
      .def_readonly("ft_sensor_left", &RobotState<T>::ft_sensor_left)
      .def_readonly("is_ready", &RobotState<T>::is_ready)
      .def_readonly("position", &RobotState<T>::position)
      .def_readonly("velocity", &RobotState<T>::velocity)
      .def_readonly("current", &RobotState<T>::current)
      .def_readonly("torque", &RobotState<T>::torque)
      .def_readonly("target_position", &RobotState<T>::target_position)
      .def_readonly("target_velocity", &RobotState<T>::target_velocity)
      .def_readonly("target_feedback_gain", &RobotState<T>::target_feedback_gain)
      .def_readonly("target_feedforward_torque", &RobotState<T>::target_feedforward_torque)
      .def_readonly("odometry", &RobotState<T>::odometry)
      .def_readonly("center_of_mass", &RobotState<T>::center_of_mass)
      .def_readonly("collisions", &RobotState<T>::collisions)
      .def_readonly("temperature", &RobotState<T>::temperature)
      .def_readonly("gravity", &RobotState<T>::gravity)
      .def("__repr__",
           [robot_state_name](const RobotState<T>& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             auto ts = timespec_to_time_point(self.timestamp);

             ReprStream ss;

             ss << robot_state_name << "(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "timestamp=" << py::repr(py::cast(ts)).template cast<std::string>() << ",";
             ss << (ml ? "\n" : " ");

             ss << "system_stat=" << inline_obj(self.system_stat) << ",";
             ss << (ml ? "\n" : " ");

             ss << "battery_state=" << inline_obj(self.battery_state) << ",";
             ss << (ml ? "\n" : " ");

             ss << "power_states=" << inline_obj(self.power_states) << ",";
             ss << (ml ? "\n" : " ");

             ss << "emo_states=" << inline_obj(self.emo_states) << ",";
             ss << (ml ? "\n" : " ");

             ss << "joint_states=" << inline_obj(self.joint_states) << ",";
             ss << (ml ? "\n" : " ");

             ss << "tool_flange_right=" << inline_obj(self.tool_flange_right) << ",";
             ss << (ml ? "\n" : " ");

             ss << "tool_flange_left=" << inline_obj(self.tool_flange_left) << ",";
             ss << (ml ? "\n" : " ");

             ss << "ft_sensor_right=" << inline_obj(self.ft_sensor_right) << ",";
             ss << (ml ? "\n" : " ");

             ss << "ft_sensor_left=" << inline_obj(self.ft_sensor_left) << ",";
             ss << (ml ? "\n" : " ");

             const auto& print_array = [&](const std::string& name, const auto& array) {
               std::string k = name + "=";
               std::string v = np_array_to_string(py::cast(array), Style::Repr);
               ss << k << indent_continuation(v, (int)k.size());
             };

             print_array("position", self.position);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("velocity", self.velocity);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("current", self.current);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("torque", self.torque);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_position", self.target_position);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_velocity", self.target_velocity);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_feedback_gain", self.target_feedback_gain);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("target_feedforward_torque", self.target_feedforward_torque);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("odometry", self.odometry);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("center_of_mass", self.center_of_mass);
             ss << ",";
             ss << (ml ? "\n" : " ");

             ss << "collisions=" << inline_obj(self.collisions) << ",";
             ss << (ml ? "\n" : " ");

             print_array("temperature", self.temperature);
             ss << ",";
             ss << (ml ? "\n" : " ");

             print_array("gravity", self.gravity);
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [robot_state_name](const RobotState<T>& self) {
        using namespace rb::print;

        auto ts = timespec_to_time_point(self.timestamp);
        Eigen::Index ready = 0;
        for (Eigen::Index i = 0; i < self.is_ready.size(); ++i) {
          if (self.is_ready[i]) {
            ++ready;
          }
        }
        const Eigen::Index total = self.is_ready.size();

        std::ostringstream ss;
        ss << robot_state_name << "(t=" << py::str(py::cast(ts)).cast<std::string>() << ", ready=" << ready << "/"
           << total << ", q=" << np_array_to_string(py::cast(self.position), Style::Str)
           << ", dq=" << np_array_to_string(py::cast(self.velocity), Style::Str)
           << ", tau=" << np_array_to_string(py::cast(self.torque), Style::Str)
           << ", collisions=" << self.collisions.size() << ")";
        return ss.str();
      });
}

void pybind11_robot_state(py::module_& m) {
  bind_system_stat(m);
  bind_battery_state(m);
  bind_power_state(m);
  bind_emo_state(m);
  bind_joint_state(m);
  bind_tool_flange(m);
  bind_ft_sensor(m);

  bind_robot_state<y1_model::A>(m, "RobotState_A");
  bind_robot_state<y1_model::T5>(m, "RobotState_T5");
  bind_robot_state<y1_model::M>(m, "RobotState_M");
  bind_robot_state<y1_model::UB>(m, "RobotState_UB");
}
