#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "common.h"

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot_state.h"

namespace py = pybind11;
using namespace rb;

void bind_system_stat(pybind11::module_& m) {
  py::class_<SystemStat>(m, "SystemStat")
      .def(py::init<>())
      .def_readonly("cpu_usage", &SystemStat::cpu_usage)
      .def_readonly("memory_usage", &SystemStat::memory_usage)
      .def_readonly("uptime", &SystemStat::uptime)
      .def_readonly("program_uptime", &SystemStat::program_uptime);
}

void bind_battery_state(pybind11::module_& m) {
  py::class_<BatteryState>(m, "BatteryState")
      .def(py::init<>())
      .def_readonly("voltage", &BatteryState::voltage)
      .def_readonly("current", &BatteryState::current);
}

void bind_power_state(pybind11::module_& m) {
  auto ps = py::class_<PowerState>(m, "PowerState");

  py::enum_<PowerState::State>(ps, "State")
      .value("Unknown", PowerState::State::kUnknown)
      .value("PowerOff", PowerState::State::kPowerOff)
      .value("PowerOn", PowerState::State::kPowerOn);

  ps.def(py::init<>())                            //
      .def_readonly("state", &PowerState::state)  //
      .def_readonly("voltage", &PowerState::voltage);
}

void bind_emo_state(pybind11::module_& m) {
  auto ps = py::class_<EMOState>(m, "EMOState");

  py::enum_<EMOState::State>(ps, "State")
      .value("Released", EMOState::State::kReleased)
      .value("Pressed", EMOState::State::kPressed);

  ps.def(py::init<>())  //
      .def_readonly("state", &EMOState::state);
}

void bind_tool_flange(py::module_& m) {
  py::class_<ToolFlangeState>(m, "ToolFlangeState")
      .def(py::init<>())
      .def_property_readonly(
          "time_since_last_update",
          [](const ToolFlangeState& self) { return timespec_to_nanoseconds(self.time_since_last_update); })
      .def_readonly("gyro", &ToolFlangeState::gyro)
      .def_readonly("acceleration", &ToolFlangeState::acceleration)
      .def_readonly("switch_A", &ToolFlangeState::switch_A)
      .def_readonly("output_voltage", &ToolFlangeState::output_voltage);
}

void bind_ft_sensor(py::module_& m) {
  py::class_<FTSensorData>(m, "FTSensorData")
      .def(py::init<>())
      .def_property_readonly(
          "time_since_last_update",
          [](const FTSensorData& self) { return timespec_to_nanoseconds(self.time_since_last_update); })
      .def_readonly("force", &FTSensorData::force)
      .def_readonly("torque", &FTSensorData::torque);
}

void bind_joint_state(py::module_& m) {
  auto js = py::class_<JointState>(m, "JointState");

  py::enum_<JointState::FETState>(js, "FETState")
      .value("Unknown", JointState::FETState::kUnknown)
      .value("On", JointState::FETState::kOn)
      .value("Off", JointState::FETState::kOff);

  py::enum_<JointState::RunState>(js, "RunState")
      .value("Unknown", JointState::RunState::kUnknown)
      .value("ControlOn", JointState::RunState::kControlOn)
      .value("ControlOff", JointState::RunState::kControlOff);

  py::enum_<JointState::InitializationState>(js, "InitializationState")
      .value("Unknown", JointState::InitializationState::kUnknown)
      .value("kInitialized", JointState::InitializationState::kInitialized)
      .value("kUninitialized", JointState::InitializationState::kUninitialized);

  js.def(py::init<>())  //
      .def_property_readonly(
          "time_since_last_update",
          [](const JointState& self) { return timespec_to_nanoseconds(self.time_since_last_update); })  //
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
      .def_readonly("target_feedforward_torque", &JointState::target_feedforward_torque);
}

template <typename T>
void bind_robot_state(py::module_& m, const std::string& robot_state_name) {
  py::class_<RobotState<T>>(m, robot_state_name.c_str())
      .def(py::init<>())
      .def_property_readonly("timestamp",
                             [](const RobotState<T>& self) { return timespec_to_time_point(self.timestamp); })
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
      .def_readonly("collisions", &RobotState<T>::collisions);
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
}
