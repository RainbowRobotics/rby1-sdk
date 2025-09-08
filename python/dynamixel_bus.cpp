#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <iomanip>

#include "common.h"
#include "print_helper.h"
#include "rby1-sdk/base/dynamixel_bus.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void pybind11_dynamixel_bus(py::module_& m) {
  auto bus_m = py::class_<DynamixelBus>(m, "DynamixelBus", R"doc(
Dynamixel bus communication interface.

This class provides low-level communication with Dynamixel servos
including motor control, PID tuning, and status monitoring.

Attributes
----------
ProtocolVersion : float
    Dynamixel protocol version (2.0).
DefaultBaudrate : int
    Default communication baudrate (2000000).
AddrTorqueEnable : int
    Memory address for torque enable/disable (64).
AddrPresentCurrent : int
    Memory address for current reading (126).
AddrPresentVelocity : int
    Memory address for velocity reading (128).
AddrPresentPosition : int
    Memory address for position reading (132).
AddrGoalCurrent : int
    Memory address for current goal setting (102).
AddrGoalPosition : int
    Memory address for position goal setting (104).
AddrOperatingMode : int
    Memory address for operating mode setting (11).
AddrPresentButtonState : int
    Memory address for button state reading (12).
AddrGoalVibrationLevel : int
    Memory address for vibration level setting (13).
AddrPositionPGain : int
    Memory address for position P gain (80).
AddrPositionIGain : int
    Memory address for position I gain (81).
AddrPositionDGain : int
    Memory address for position D gain (82).
TorqueEnable : int
    Value to enable torque (1).
TorqueDisable : int
    Value to disable torque (0).
CurrentControlMode : int
    Operating mode for current control (0).
CurrentBasedPositionControlMode : int
    Operating mode for current-based position control (5).
AddrCurrentTemperature : int
    Memory address for temperature reading (146).
)doc");

  py::class_<DynamixelBus::ButtonState>(bus_m, "ButtonState", R"doc(
Button state information for gripper devices.

This class represents the state of buttons and triggers
on gripper or tool devices.

Attributes
----------
button : int
    Button state: 0 (released) or 1 (pressed).
trigger : int
    Trigger value ranging from 0 to 255.
)doc")
      .def(py::init<>(), R"doc(
Construct a ButtonState instance with default values.
)doc")
      .def_readonly("button", &DynamixelBus::ButtonState::button)
      .def_readonly("trigger", &DynamixelBus::ButtonState::trigger)
      .def("__repr__",
           [](const DynamixelBus::ButtonState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "ButtonState(" << FIRST             //
                 << "button=" << self.button << SEP     //
                 << "trigger=" << self.trigger << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const DynamixelBus::ButtonState& self) {
        std::ostringstream out;
        out << "btn=" << self.button << ", trg=" << self.trigger;
        return out.str();
      });

  py::class_<DynamixelBus::MotorState>(bus_m, "MotorState", R"doc(
Motor state information.

This class represents the current state of a Dynamixel motor
including position, velocity, current, and temperature.

Attributes
----------
torque_enable : bool
    Whether torque is currently enabled.
position : float
    Current position [rad].
velocity : float
    Current velocity [rad/s].
current : float
    Current current [A].
torque : float
    Current torque [Nm].
temperature : int
    Current temperature in degrees Celsius.
)doc")
      .def(py::init<>(), R"doc(
Construct a MotorState instance with default values.
)doc")
      .def_readonly("torque_enable", &DynamixelBus::MotorState::torque_enable)
      .def_readonly("position", &DynamixelBus::MotorState::position)
      .def_readonly("velocity", &DynamixelBus::MotorState::velocity)
      .def_readonly("current", &DynamixelBus::MotorState::current)
      .def_readonly("torque", &DynamixelBus::MotorState::torque)
      .def_readonly("temperature", &DynamixelBus::MotorState::temperature)
      .def("__repr__",
           [](const DynamixelBus::MotorState& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "MotorState(" << FIRST                                                  //
                 << "torque_enable=" << (self.torque_enable ? "True" : "False") << SEP      //
                 << "position=" << format_number(self.position, Style::Repr) << SEP         //
                 << "velocity=" << format_number(self.velocity, Style::Repr) << SEP         //
                 << "current=" << format_number(self.current, Style::Repr) << SEP           //
                 << "torque=" << format_number(self.torque, Style::Repr) << SEP             //
                 << "temperature=" << format_number(self.temperature, Style::Repr) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const DynamixelBus::MotorState& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "MotorState(q=" << format_number(self.position, Style::Str)
            << ", dq=" << format_number(self.velocity, Style::Str) << ", I=" << format_number(self.current, Style::Str)
            << ", tau=" << format_number(self.torque, Style::Str)
            << ", T=" << format_number(self.temperature, Style::Str) << "C"
            << ", enabled=" << (self.torque_enable ? "True" : "False") << ")";
        return out.str();
      });

  py::class_<DynamixelBus::PIDGain>(bus_m, "PIDGain", R"doc(
PID gain parameters for position control.

This class represents the proportional, integral, and derivative
gains used for position control of Dynamixel motors.

Attributes
----------
p_gain : int
    Proportional gain (0-65535).
i_gain : int
    Integral gain (0-65535).
d_gain : int
    Derivative gain (0-65535).
)doc")
      .def(py::init<>(), R"doc(
Construct a PIDGain instance with default values.
)doc")
      .def_readwrite("p_gain", &DynamixelBus::PIDGain::p_gain)
      .def_readwrite("i_gain", &DynamixelBus::PIDGain::i_gain)
      .def_readwrite("d_gain", &DynamixelBus::PIDGain::d_gain);

  bus_m  //
      .def_readonly_static("ProtocolVersion", &DynamixelBus::kProtocolVersion, "")
      .def_readonly_static("DefaultBaudrate", &DynamixelBus::kDefaultBaudrate, "")
      .def_readonly_static("AddrTorqueEnable", &DynamixelBus::kAddrTorqueEnable, "")
      .def_readonly_static("AddrPresentCurrent", &DynamixelBus::kAddrPresentCurrent, "")
      .def_readonly_static("AddrPresentVelocity", &DynamixelBus::kAddrPresentVelocity, "")
      .def_readonly_static("AddrPresentPosition", &DynamixelBus::kAddrPresentPosition, "")
      .def_readonly_static("AddrGoalCurrent", &DynamixelBus::kAddrGoalCurrent, "")
      .def_readonly_static("AddrGoalPosition", &DynamixelBus::kAddrGoalPosition, "")
      .def_readonly_static("AddrOperatingMode", &DynamixelBus::kAddrOperatingMode, "")
      .def_readonly_static("AddrPresentButtonState", &DynamixelBus::kAddrPresentButtonState, "")
      .def_readonly_static("AddrGoalVibrationLevel", &DynamixelBus::kAddrGoalVibrationLevel, "")
      .def_readonly_static("AddrPositionPGain", &DynamixelBus::kAddrPositionPGain, "")
      .def_readonly_static("AddrPositionIGain", &DynamixelBus::kAddrPositionIGain, "")
      .def_readonly_static("AddrPositionDGain", &DynamixelBus::kAddrPositionDGain, "")
      .def_readonly_static("TorqueEnable", &DynamixelBus::kTorqueEnable, "")
      .def_readonly_static("TorqueDisable", &DynamixelBus::kTorqueDisable, "")
      .def_readonly_static("CurrentControlMode", &DynamixelBus::kCurrentControlMode, "")
      .def_readonly_static("CurrentBasedPositionControlMode", &DynamixelBus::kCurrentBasedPositionControlMode, "")
      .def_readonly_static("AddrCurrentTemperature", &DynamixelBus::kAddrCurrentTemperature, "")

      .def(py::init<const std::string&>(), "dev_name"_a, R"doc(
Construct a DynamixelBus instance.

Parameters
----------
dev_name : str
    Device name (e.g., "/dev/ttyUSB0" on Linux).
)doc")
      .def("set_torque_constant", &DynamixelBus::SetTorqueConstant, "torque_constant"_a, R"doc(
Set torque constants for motors.

Parameters
----------
torque_constant : list[float]
    List of torque constants for each motor.
)doc")
      .def("open_port", &DynamixelBus::OpenPort, R"doc(
Open the communication port.

Returns
-------
bool
    True if port opened successfully, False otherwise.
)doc")
      .def("set_baud_rate", &DynamixelBus::SetBaudRate, "baudrate"_a, R"doc(
Set the communication baudrate.

Parameters
----------
baudrate : int
    Baudrate value (e.g., 2000000).

Returns
-------
bool
    True if baudrate set successfully, False otherwise.
)doc")
      .def("ping", &DynamixelBus::Ping, "id"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Ping a motor to check if it's responding.

Parameters
----------
id : int
    Motor ID to ping.

Returns
-------
bool
    True if motor responds, False otherwise.
)doc")
      .def("read_button_status", &DynamixelBus::ReadButtonStatus, py::call_guard<py::gil_scoped_release>(), R"doc(
Read button status from a device.

Parameters
----------
id : int
    Device ID to read from.

Returns
-------
tuple[int, ButtonState] or None
    Tuple of (id, button_state) if successful, None otherwise.
)doc")
      .def("send_torque_enable", &DynamixelBus::SendTorqueEnable, "id"_a, "onoff"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Enable or disable torque for a motor.

Parameters
----------
id : int
    Motor ID.
onoff : int
    Torque state: 1 (enable) or 0 (disable).
)doc")
      .def("set_position_p_gain", &DynamixelBus::SetPositionPGain, "id"_a, "p_gain"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Set position P gain for a motor.

Parameters
----------
id : int
    Motor ID.
p_gain : int
    P gain value (0-65535).
)doc")
      .def("set_position_i_gain", &DynamixelBus::SetPositionIGain, "id"_a, "i_gain"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Set position I gain for a motor.

Parameters
----------
id : int
    Motor ID.
i_gain : int
    I gain value (0-65535).
)doc")
      .def("set_position_d_gain", &DynamixelBus::SetPositionDGain, "id"_a, "d_gain"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Set position D gain for a motor.

Parameters
----------
id : int
    Motor ID.
d_gain : int
    D gain value (0-65535).
)doc")
      .def("set_position_pid_gain",
           py::overload_cast<int, std::optional<uint16_t>, std::optional<uint16_t>, std::optional<uint16_t>>(
               &DynamixelBus::SetPositionPIDGain),
           "id"_a, "p_gain"_a, "i_gain"_a, "d_gain"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Set position PID gains for a motor.

Parameters
----------
id : int
    Motor ID.
p_gain : int or None, optional
    P gain value (0-65535). If None, current value is preserved.
i_gain : int or None, optional
    I gain value (0-65535). If None, current value is preserved.
d_gain : int or None, optional
    D gain value (0-65535). If None, current value is preserved.
)doc")
      .def("set_position_pid_gain",
           py::overload_cast<int, const DynamixelBus::PIDGain&>(&DynamixelBus::SetPositionPIDGain), "id"_a,
           "pid_gain"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Set position PID gains for a motor using PIDGain struct.

Parameters
----------
id : int
    Motor ID.
pid_gain : PIDGain
    PIDGain struct containing P, I, and D values.
)doc")
      .def("get_position_p_gain", &DynamixelBus::GetPositionPGain, "id"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Get position P gain for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
int or None
    P gain value if successful, None otherwise.
)doc")
      .def("get_position_i_gain", &DynamixelBus::GetPositionIGain, "id"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Get position I gain for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
int or None
    I gain value if successful, None otherwise.
)doc")
      .def("get_position_d_gain", &DynamixelBus::GetPositionDGain, "id"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Get position D gain for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
int or None
    D gain value if successful, None otherwise.
)doc")
      .def("get_position_pid_gain", &DynamixelBus::GetPositionPIDGain, "id"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Get position PID gains for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
PIDGain or None
    PIDGain struct if successful, None otherwise.
)doc")
      .def("read_torque_enable", &DynamixelBus::ReadTorqueEnable, "id"_a, R"doc(
Read torque enable state for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
int or None
    Torque state (1=enabled, 0=disabled) if successful, None otherwise.
)doc")
      .def("read_encoder", &DynamixelBus::ReadEncoder, "id"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Read encoder position for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
float or None
    Encoder position in radians if successful, None otherwise.
)doc")
      .def("send_goal_position", &DynamixelBus::SendGoalPosition, "id"_a, "goal_position"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Send goal position to a motor.

Parameters
----------
id : int
    Motor ID.
goal_position : int
    Goal position in encoder units.
)doc")
      .def("read_operating_mode", &DynamixelBus::ReadOperatingMode, "id"_a, "use_cache"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Read operating mode for a motor.

Parameters
----------
id : int
    Motor ID.
use_cache : bool, optional
    Whether to use cached value. Default is False.

Returns
-------
int or None
    Operating mode value if successful, None otherwise.
)doc")
      .def("send_operating_mode", &DynamixelBus::SendOperatingMode, "id"_a, "operating_mode"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Send operating mode to a motor.

Parameters
----------
id : int
    Motor ID.
mode : int
    Operating mode value.

Returns
-------
bool
    True if successful, False otherwise.
)doc")
      .def("send_torque", &DynamixelBus::SendTorque, "id"_a, "torque"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Send torque command to a motor.

Parameters
----------
id : int
    Motor ID.
joint_torque : float
    Torque value [Nm].
)doc")
      .def("send_current", &DynamixelBus::SendCurrent, "id"_a, "current"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Send current command to a motor.

Parameters
----------
id : int
    Motor ID.
current : float
    Current value [A].
)doc")
      .def("read_temperature", &DynamixelBus::ReadTemperature, "id"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Read temperature for a motor.

Parameters
----------
id : int
    Motor ID.

Returns
-------
int or None
    Temperature in degrees Celsius if successful, None otherwise.
)doc")
      .def("group_fast_sync_read", &DynamixelBus::GroupFastSyncRead, "ids"_a, "addr"_a, "len"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform fast synchronous read for multiple motors.

Parameters
----------
ids : list[int]
    List of motor IDs.
addr : int
    Memory address to read from.
len : int
    Number of bytes to read.

Returns
-------
list[tuple[int, int]] or None
    List of (id, value) tuples if successful, None otherwise.
)doc")
      .def("group_fast_sync_read_encoder", &DynamixelBus::GroupFastSyncReadEncoder, "ids"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform fast synchronous read of encoder values.

Parameters
----------
ids : list[int]
    List of motor IDs.

Returns
-------
list[tuple[int, float]] or None
    List of (id, encoder_value) tuples if successful, None otherwise.
)doc")
      .def("group_fast_sync_read_operating_mode", &DynamixelBus::GroupFastSyncReadOperatingMode, "ids"_a, "use_cache"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform fast synchronous read of operating modes.

Parameters
----------
ids : list[int]
    List of motor IDs.
use_cache : bool, optional
    Whether to use cached values. Default is False.

Returns
-------
list[tuple[int, int]] or None
    List of (id, operating_mode) tuples if successful, None otherwise.
)doc")
      .def("group_fast_sync_read_torque_enable", &DynamixelBus::GroupFastSyncReadTorqueEnable, "ids"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform fast synchronous read of torque enable states.

Parameters
----------
ids : list[int]
    List of motor IDs.

Returns
-------
list[tuple[int, int]] or None
    List of (id, torque_enable) tuples if successful, None otherwise.
)doc")
      .def("get_motor_states", &DynamixelBus::GetMotorStates, "ids"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Get motor states for multiple motors.

Parameters
----------
ids : list[int]
    List of motor IDs.

Returns
-------
list[tuple[int, MotorState]] or None
    List of (id, motor_state) tuples if successful, None otherwise.
)doc")
      .def("group_sync_write_torque_enable",
           py::overload_cast<const std::vector<std::pair<int, int>>&>(&DynamixelBus::GroupSyncWriteTorqueEnable),
           "id_and_enable_vector"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Perform synchronous write of torque enable states.

Parameters
----------
id_and_enable_vector : list[tuple[int, int]]
    List of (id, enable) tuples.
)doc")
      .def("group_sync_write_torque_enable",
           py::overload_cast<const std::vector<int>&, int>(&DynamixelBus::GroupSyncWriteTorqueEnable), "ids"_a,
           "enable"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Perform synchronous write of torque enable states.

Parameters
----------
ids : list[int]
    List of motor IDs.
enable : int
    Enable value (1=enabled, 0=disabled).
)doc")
      .def("group_sync_write_operating_mode", &DynamixelBus::GroupSyncWriteOperatingMode, "id_and_mode_vector"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform synchronous write of operating modes.

Parameters
----------
id_and_mode_vector : list[tuple[int, int]]
    List of (id, mode) tuples.
)doc")
      .def("group_sync_write_send_position", &DynamixelBus::GroupSyncWriteSendPosition, "id_and_position_vector"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform synchronous write of goal positions.

Parameters
----------
id_and_position_vector : list[tuple[int, int]]
    List of (id, position) tuples.
)doc")
      .def("group_sync_write_send_torque", &DynamixelBus::GroupSyncWriteSendTorque, "id_and_torque_vector"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Perform synchronous write of torque commands.

Parameters
----------
id_and_torque_vector : list[tuple[int, float]]
    List of (id, torque) tuples.
)doc")
      .def("send_vibration", &DynamixelBus::SendVibration, "id"_a, "vibration"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Send vibration command to a device.

Parameters
----------
id : int
    Device ID.
level : int
    Vibration level (0-255).
)doc");
}