#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iomanip>

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
      .def_readonly("program_uptime", &SystemStat::program_uptime)
      .def("__repr__", [](const SystemStat& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "SystemStat("                                      //
           << "cpu_usage=" << self.cpu_usage                     //
           << ", memory_usage=" << self.memory_usage             //
           << ", uptime=" << self.uptime                         //
           << ", program_uptime=" << self.program_uptime << ")";
        return ss.str();
      });
}

void bind_battery_state(pybind11::module_& m) {
  py::class_<BatteryState>(m, "BatteryState")
      .def(py::init<>())
      .def_readonly("voltage", &BatteryState::voltage)
      .def_readonly("current", &BatteryState::current)
      .def_readonly("level_percent", &BatteryState::level_percent)
      .def("__repr__", [](const BatteryState& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "BatteryState("                                    //
           << "voltage=" << self.voltage                         //
           << ", current=" << self.current                       //
           << ", level_percent=" << self.level_percent << ")";
        return ss.str();
      });
}

void bind_power_state(pybind11::module_& m) {
  auto ps = py::class_<PowerState>(m, "PowerState");

  py::enum_<PowerState::State>(ps, "State")
      .value("Unknown", PowerState::State::kUnknown)
      .value("PowerOff", PowerState::State::kPowerOff)
      .value("PowerOn", PowerState::State::kPowerOn);

  ps.def(py::init<>())                            //
      .def_readonly("state", &PowerState::state)  //
      .def_readonly("voltage", &PowerState::voltage)
      .def("__repr__", [](const PowerState& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "PowerState("                                      //
           << "state=" << rb::to_string(self.state)              //
           << ", voltage=" << self.voltage << ")";
        return ss.str();
      });
}

void bind_emo_state(pybind11::module_& m) {
  auto ps = py::class_<EMOState>(m, "EMOState");

  py::enum_<EMOState::State>(ps, "State")
      .value("Released", EMOState::State::kReleased)
      .value("Pressed", EMOState::State::kPressed);

  ps.def(py::init<>())  //
      .def_readonly("state", &EMOState::state)
      .def("__repr__", [](const EMOState& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "EMOState("                                        //
           << "state=" << rb::to_string(self.state) << ")";
        return ss.str();
      });
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
      .def_readonly("output_voltage", &ToolFlangeState::output_voltage)
      .def("__repr__", [](const ToolFlangeState& self) {
        py::object np = py::module_::import("numpy");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "ToolFlangeState("                                 //
           << "time_since_last_update=" << self.time_since_last_update.tv_sec << "." << std::setw(9)
           << std::setfill('0') << self.time_since_last_update.tv_nsec                     //
           << ", gyro=" << np.attr("array2string")(self.gyro).cast<std::string>()          //
           << ", acceleration=" << np.attr("array2string")(self.gyro).cast<std::string>()  //
           << ", switch_A=" << (self.switch_A ? "True" : "False")                          //
           << ", output_voltage=" << self.output_voltage                                   //
           << ")";
        return ss.str();
      });
}

void bind_ft_sensor(py::module_& m) {
  py::class_<FTSensorData>(m, "FTSensorData")
      .def(py::init<>())
      .def_property_readonly(
          "time_since_last_update",
          [](const FTSensorData& self) { return timespec_to_nanoseconds(self.time_since_last_update); })
      .def_readonly("force", &FTSensorData::force)
      .def_readonly("torque", &FTSensorData::torque)
      .def("__repr__", [](const FTSensorData& self) {
        py::object np = py::module_::import("numpy");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "FTSensorData("                                    //
           << "time_since_last_update=" << self.time_since_last_update.tv_sec << "." << std::setw(9)
           << std::setfill('0') << self.time_since_last_update.tv_nsec                 //
           << ", force=" << np.attr("array2string")(self.force).cast<std::string>()    //
           << ", torque=" << np.attr("array2string")(self.torque).cast<std::string>()  //
           << ")";
        return ss.str();
      });
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
      .def_readonly("target_feedforward_torque", &JointState::target_feedforward_torque)
      .def("__repr__", [](const JointState& self) {
        py::object np = py::module_::import("numpy");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "JointState("                                      //
           << "time_since_last_update=" << self.time_since_last_update.tv_sec << "." << std::setw(9)
           << std::setfill('0') << self.time_since_last_update.tv_nsec          //
           << ", is_ready=" << (self.is_ready ? "True" : "False")               //
           << ", position=" << self.position                                    //
           << ", velocity=" << self.velocity                                    //
           << ", current=" << self.current                                      //
           << ", torque=" << self.torque                                        //
           << ", target_position=" << self.target_position                      //
           << ", target_velocity=" << self.target_velocity                      //
           << ", target_feedback_gain=" << self.target_feedback_gain            //
           << ", target_feedforward_torque=" << self.target_feedforward_torque  //
           << ")";
        return ss.str();
      });
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
      .def_readonly("collisions", &RobotState<T>::collisions)
      .def("__repr__", [](const RobotState<T>& self) {
        auto timestamp = timespec_to_time_point(self.timestamp);
        py::object np = py::module_::import("numpy");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)                                     //
           << "RobotState("                                                                         //
           << "timestamp=" << static_cast<std::string>(py::repr(py::cast(timestamp)))               //
           << ", is_ready=" << np.attr("array2string")(self.is_ready).template cast<std::string>()  //
           << ", position=" << np.attr("array2string")(self.position).template cast<std::string>()  //
           << ", velocity=" << np.attr("array2string")(self.velocity).template cast<std::string>()  //
           << ", current=" << np.attr("array2string")(self.current).template cast<std::string>()    //
           << ", torque=" << np.attr("array2string")(self.torque).template cast<std::string>()      //
           << ", num_collisions=" << self.collisions.size()                                         //
           << ")";
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
}
