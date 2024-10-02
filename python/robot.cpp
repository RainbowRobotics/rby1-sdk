#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

template <typename T>
void bind_robot_command_handler(py::module_& m, const std::string& handler_name) {
  py::class_<RobotCommandHandler<T>>(m, handler_name.c_str())
      .def("is_done", &RobotCommandHandler<T>::IsDone)
      .def("wait",
           [](RobotCommandHandler<T>& self) {
             while (true) {
               {
                 py::gil_scoped_release release;
                 if (self.WaitFor(10 /* msec */)) {
                   return;
                 }
               }

               if (PyErr_CheckSignals() != 0) {
                 throw py::error_already_set();
               }
             }
           })
      .def("wait_for", &RobotCommandHandler<T>::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>())
      .def("cancel", &RobotCommandHandler<T>::Cancel, py::call_guard<py::gil_scoped_release>())
      .def("get",
           [](RobotCommandHandler<T>& self) {
             while (true) {
               {
                 py::gil_scoped_release release;
                 if (self.WaitFor(10 /* msec */)) {
                   return self.Get();
                 }
               }

               if (PyErr_CheckSignals() != 0) {
                 throw py::error_already_set();
               }
             }
           })
      .def("get_status", &RobotCommandHandler<T>::GetStatus);
}

template <typename T>
void bind_robot_command_stream_handler(py::module_& m, const std::string& handler_name) {
  py::class_<RobotCommandStreamHandler<T>>(m, handler_name.c_str())
      .def("is_done", &RobotCommandStreamHandler<T>::IsDone)
      .def("wait",
           [](RobotCommandStreamHandler<T>& self) {
             while (true) {
               {
                 py::gil_scoped_release release;
                 if (self.WaitFor(10 /* msec */)) {
                   return;
                 }
               }

               if (PyErr_CheckSignals() != 0) {
                 throw py::error_already_set();
               }
             }
           })
      .def("wait_for", &RobotCommandStreamHandler<T>::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>())
      .def("send_command", &RobotCommandStreamHandler<T>::SendCommand, "builder"_a, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>())
      .def("request_feedback", &RobotCommandStreamHandler<T>::RequestFeedback, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>())
      .def("cancel", &RobotCommandStreamHandler<T>::Cancel, py::call_guard<py::gil_scoped_release>());
}

template <typename T>
void bind_control_state(py::module_& m, const std::string& handler_name) {
  py::class_<ControlState<T>>(m, handler_name.c_str())
      .def(py::init<>())
      .def_readonly("t", &ControlState<T>::t)
      .def_readonly("is_ready", &ControlState<T>::is_ready)
      .def_readonly("position", &ControlState<T>::position)
      .def_readonly("velocity", &ControlState<T>::velocity)
      .def_readonly("current", &ControlState<T>::current)
      .def_readonly("torque", &ControlState<T>::torque);
}

template <typename T>
void bind_control_input(py::module_& m, const std::string& handler_name) {
  py::class_<ControlInput<T>>(m, handler_name.c_str())
      .def(py::init<>())
      .def_property(
          "mode", [](ControlInput<T>& self) -> Eigen::Vector<bool, T::kRobotDOF>& { return self.mode; },
          [](ControlInput<T>& self, const Eigen::Vector<bool, T::kRobotDOF>& mat) { self.mode = mat; },
          py::return_value_policy::reference_internal)
      .def_property(
          "target", [](ControlInput<T>& self) -> Eigen::Vector<double, T::kRobotDOF>& { return self.target; },
          [](ControlInput<T>& self, const Eigen::Vector<double, T::kRobotDOF>& mat) { self.target = mat; },
          py::return_value_policy::reference_internal)
      .def_property(
          "feedback_gain",
          [](ControlInput<T>& self) -> Eigen::Vector<unsigned int, T::kRobotDOF>& { return self.feedback_gain; },
          [](ControlInput<T>& self, const Eigen::Vector<unsigned int, T::kRobotDOF>& mat) { self.feedback_gain = mat; },
          py::return_value_policy::reference_internal)
      .def_property(
          "feedforward_torque",
          [](ControlInput<T>& self) -> Eigen::Vector<double, T::kRobotDOF>& { return self.feedforward_torque; },
          [](ControlInput<T>& self, const Eigen::Vector<double, T::kRobotDOF>& mat) { self.feedforward_torque = mat; },
          py::return_value_policy::reference_internal)
      .def_readwrite("finish", &ControlInput<T>::finish);
}

template <typename T>
void bind_robot(py::module_& m, const std::string& robot_name) {
  bind_robot_command_handler<T>(m, robot_name + "_CommandHandler");
  bind_robot_command_stream_handler<T>(m, robot_name + "_CommandStreamHandler");
  bind_control_state<T>(m, robot_name + "_ControlState");
  bind_control_input<T>(m, robot_name + "_ControlInput");

  py::class_<Robot<T>, std::shared_ptr<Robot<T>>>(m, robot_name.c_str())
      .def_static("create", &Robot<T>::Create)
      .def("connect", &Robot<T>::Connect, "max_retries"_a = 5, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>())
      .def("disconnect", &Robot<T>::Disconnect, py::call_guard<py::gil_scoped_release>())
      .def("is_connected", &Robot<T>::IsConnected)
      .def("get_robot_info", &Robot<T>::GetRobotInfo)
      .def("get_time_scale", &Robot<T>::GetTimeScale)
      .def("set_time_scale", &Robot<T>::SetTimeScale, "time_scale"_a)
      .def("power_on", &Robot<T>::PowerOn, "dev_name"_a, py::call_guard<py::gil_scoped_release>())
      .def("power_off", &Robot<T>::PowerOff, "dev_name"_a, py::call_guard<py::gil_scoped_release>())
      .def("is_power_on", &Robot<T>::IsPowerOn, "dev_name"_a)
      .def("servo_on", &Robot<T>::ServoOn, "dev_name"_a)
      .def("is_servo_on", &Robot<T>::IsServoOn, "dev_name"_a)
      .def("enable_control_manager", &Robot<T>::EnableControlManager, py::arg("unlimited_mode_enabled") = false,
           py::call_guard<py::gil_scoped_release>())
      .def("disable_control_manager", &Robot<T>::DisableControlManager, py::call_guard<py::gil_scoped_release>())
      .def("reset_fault_control_manager", &Robot<T>::ResetFaultControlManager, py::call_guard<py::gil_scoped_release>())
      .def("set_tool_flange_output_voltage", &Robot<T>::SetToolFlangeOutputVoltage,
           py::call_guard<py::gil_scoped_release>())
      .def("start_state_update", &Robot<T>::StartStateUpdate, "cb"_a, "rate"_a)
      .def("stop_state_update", &Robot<T>::StopStateUpdate, py::call_guard<py::gil_scoped_release>())
      .def("start_log_stream", &Robot<T>::StartLogStream, "cb"_a, "rate"_a)
      .def("stop_log_stream", &Robot<T>::StopLogStream)
      .def("get_state", &Robot<T>::GetState, py::call_guard<py::gil_scoped_release>())
      .def("get_last_log", &Robot<T>::GetLastLog, "count"_a)
      .def("get_control_manager_state", &Robot<T>::GetControlManagerState)
      .def("send_command", &Robot<T>::SendCommand, "builder"_a, "priority"_a = 1)
      .def("create_command_stream", &Robot<T>::CreateCommandStream, "priority"_a = 1)
      .def(
          "control",
          [](Robot<T>& self, std::function<ControlInput<T>(const ControlState<T>&)> control, int port, int priority) {
            return self.Control(
                [=](const ControlState<T>& state) {
                  py::gil_scoped_acquire acquire;
                  return control(state);
                },
                port, priority);
          },
          "control"_a, "port"_a = 0, "priority"_a = 1, py::call_guard<py::gil_scoped_release>())
      .def("reset_odometry", &Robot<T>::ResetOdometry, "angle"_a, "position"_a)
      .def("get_parameter_list", &Robot<T>::GetParameterList)
      .def("set_parameter", &Robot<T>::SetParameter, "name"_a, "value"_a)
      .def("get_parameter", &Robot<T>::GetParameter, "name"_a)
      .def("reset_parameter_to_default", &Robot<T>::ResetParameterToDefault, "name"_a)
      .def("reset_all_parameters_to_default", &Robot<T>::ResetAllParametersToDefault)
      .def("get_robot_model", &Robot<T>::GetRobotModel)
      .def("import_robot_model", &Robot<T>::ImportRobotModel, "name"_a, "model"_a)
      .def("sync_time", &Robot<T>::SyncTime)
      .def("has_established_time_sync", &Robot<T>::HasEstablishedTimeSync)
      .def("start_time_sync", &Robot<T>::StartTimeSync, "period_sec"_a)
      .def("stop_time_sync", &Robot<T>::StopTimeSync, py::call_guard<py::gil_scoped_release>())
      .def("get_dynamics", &Robot<T>::GetDynamics, "urdf_model"_a = "");
}

void pybind11_robot(py::module_& m) {
  bind_robot<y1_model::A>(m, "Robot_A");
}