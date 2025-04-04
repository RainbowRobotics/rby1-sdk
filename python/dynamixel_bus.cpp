#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

#include "rby1-sdk/base/dynamixel_bus.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void pybind11_dynamixel_bus(py::module_& m) {
  auto bus_m = py::class_<DynamixelBus>(m, "DynamixelBus");

  py::class_<DynamixelBus::ButtonState>(bus_m, "ButtonState")
      .def(py::init<>())
      .def_readonly("button", &DynamixelBus::ButtonState::button)
      .def_readonly("trigger", &DynamixelBus::ButtonState::trigger);

  py::class_<DynamixelBus::MotorState>(bus_m, "MotorState")
      .def(py::init<>())
      .def_readonly("torque_enable", &DynamixelBus::MotorState::torque_enable)
      .def_readonly("position", &DynamixelBus::MotorState::position)
      .def_readonly("velocity", &DynamixelBus::MotorState::velocity)
      .def_readonly("current", &DynamixelBus::MotorState::current)
      .def_readonly("torque", &DynamixelBus::MotorState::torque)
      .def_readonly("temperature", &DynamixelBus::MotorState::temperature);

  py::class_<DynamixelBus::PIDGain>(bus_m, "PIDGain")
      .def(py::init<>())
      .def_readwrite("p_gain", &DynamixelBus::PIDGain::p_gain)
      .def_readwrite("i_gain", &DynamixelBus::PIDGain::i_gain)
      .def_readwrite("d_gain", &DynamixelBus::PIDGain::d_gain);

  bus_m  //
      .def_readonly_static("ProtocolVersion", &DynamixelBus::kProtocolVersion)
      .def_readonly_static("DefaultBaudrate", &DynamixelBus::kDefaultBaudrate)
      .def_readonly_static("AddrTorqueEnable", &DynamixelBus::kAddrTorqueEnable)
      .def_readonly_static("AddrPresentCurrent", &DynamixelBus::kAddrPresentCurrent)
      .def_readonly_static("AddrPresentVelocity", &DynamixelBus::kAddrPresentVelocity)
      .def_readonly_static("AddrPresentPosition", &DynamixelBus::kAddrPresentPosition)
      .def_readonly_static("AddrGoalCurrent", &DynamixelBus::kAddrGoalCurrent)
      .def_readonly_static("AddrGoalPosition", &DynamixelBus::kAddrGoalPosition)
      .def_readonly_static("AddrOperatingMode", &DynamixelBus::kAddrOperatingMode)
      .def_readonly_static("AddrPresentButtonState", &DynamixelBus::kAddrPresentButtonState)
      .def_readonly_static("AddrGoalVibrationLevel", &DynamixelBus::kAddrGoalVibrationLevel)
      .def_readonly_static("AddrPositionPGain", &DynamixelBus::kAddrPositionPGain)
      .def_readonly_static("AddrPositionIGain", &DynamixelBus::kAddrPositionIGain)
      .def_readonly_static("AddrPositionDGain", &DynamixelBus::kAddrPositionDGain)
      .def_readonly_static("TorqueEnable", &DynamixelBus::kTorqueEnable)
      .def_readonly_static("TorqueDisable", &DynamixelBus::kTorqueDisable)
      .def_readonly_static("CurrentControlMode", &DynamixelBus::kCurrentControlMode)
      .def_readonly_static("CurrentBasedPositionControlMode", &DynamixelBus::kCurrentBasedPositionControlMode)
      .def_readonly_static("AddrCurrentTemperature", &DynamixelBus::kAddrCurrentTemperature)

      .def(py::init<const std::string&>(), "dev_name"_a)
      .def("set_torque_constant", &DynamixelBus::SetTorqueConstant)
      .def("open_port", &DynamixelBus::OpenPort)
      .def("set_baud_rate", &DynamixelBus::SetBaudRate)
      .def("ping", &DynamixelBus::Ping, py::call_guard<py::gil_scoped_release>())
      .def("read_button_status", &DynamixelBus::ReadButtonStatus, py::call_guard<py::gil_scoped_release>())
      .def("send_torque_enable", &DynamixelBus::SendTorqueEnable, py::call_guard<py::gil_scoped_release>())
      .def("set_position_p_gain", &DynamixelBus::SetPositionPGain, py::call_guard<py::gil_scoped_release>())
      .def("set_position_i_gain", &DynamixelBus::SetPositionIGain, py::call_guard<py::gil_scoped_release>())
      .def("set_position_d_gain", &DynamixelBus::SetPositionDGain, py::call_guard<py::gil_scoped_release>())
      .def("set_position_pid_gain",
           py::overload_cast<int, std::optional<uint16_t>, std::optional<uint16_t>, std::optional<uint16_t>>(
               &DynamixelBus::SetPositionPIDGain), py::call_guard<py::gil_scoped_release>())
      .def("set_position_pid_gain",
           py::overload_cast<int, const DynamixelBus::PIDGain&>(&DynamixelBus::SetPositionPIDGain), py::call_guard<py::gil_scoped_release>())
      .def("get_position_p_gain", &DynamixelBus::GetPositionPGain, py::call_guard<py::gil_scoped_release>())
      .def("get_position_i_gain", &DynamixelBus::GetPositionIGain, py::call_guard<py::gil_scoped_release>())
      .def("get_position_d_gain", &DynamixelBus::GetPositionDGain, py::call_guard<py::gil_scoped_release>())
      .def("get_position_pid_gain", &DynamixelBus::GetPositionPIDGain, py::call_guard<py::gil_scoped_release>())
      .def("read_torque_enable", &DynamixelBus::ReadTorqueEnable)
      .def("read_encoder", &DynamixelBus::ReadEncoder, py::call_guard<py::gil_scoped_release>())
      .def("send_goal_position", &DynamixelBus::SendGoalPosition, py::call_guard<py::gil_scoped_release>())
      .def("read_operating_mode", &DynamixelBus::ReadOperatingMode, py::call_guard<py::gil_scoped_release>())
      .def("send_operating_mode", &DynamixelBus::SendOperatingMode, py::call_guard<py::gil_scoped_release>())
      .def("send_torque", &DynamixelBus::SendTorque, py::call_guard<py::gil_scoped_release>())
      .def("send_current", &DynamixelBus::SendCurrent, py::call_guard<py::gil_scoped_release>())
      .def("read_temperature", &DynamixelBus::ReadTemperature, py::call_guard<py::gil_scoped_release>())
      .def("group_fast_sync_read", &DynamixelBus::GroupFastSyncRead, py::call_guard<py::gil_scoped_release>())
      .def("group_fast_sync_read_encoder", &DynamixelBus::GroupFastSyncReadEncoder, py::call_guard<py::gil_scoped_release>())
      .def("group_fast_sync_read_operating_mode", &DynamixelBus::GroupFastSyncReadOperatingMode, py::call_guard<py::gil_scoped_release>())
      .def("group_fast_sync_read_torque_enable", &DynamixelBus::GroupFastSyncReadTorqueEnable, py::call_guard<py::gil_scoped_release>())
      .def("get_motor_states", &DynamixelBus::GetMotorStates, py::call_guard<py::gil_scoped_release>())
      .def("group_sync_write_torque_enable",
           py::overload_cast<const std::vector<std::pair<int, int>>&>(&DynamixelBus::GroupSyncWriteTorqueEnable), py::call_guard<py::gil_scoped_release>())
      .def("group_sync_write_torque_enable",
           py::overload_cast<const std::vector<int>&, int>(&DynamixelBus::GroupSyncWriteTorqueEnable), py::call_guard<py::gil_scoped_release>())
      .def("group_sync_write_operating_mode", &DynamixelBus::GroupSyncWriteOperatingMode, py::call_guard<py::gil_scoped_release>())
      .def("group_sync_write_send_position", &DynamixelBus::GroupSyncWriteSendPosition, py::call_guard<py::gil_scoped_release>())
      .def("group_sync_write_send_torque", &DynamixelBus::GroupSyncWriteSendTorque, py::call_guard<py::gil_scoped_release>())
      .def("send_vibration", &DynamixelBus::SendVibration, py::call_guard<py::gil_scoped_release>());
}