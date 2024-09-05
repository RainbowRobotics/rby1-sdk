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
      .def("wait", &RobotCommandHandler<T>::Wait, py::call_guard<py::gil_scoped_release>())
      .def("wait_for", &RobotCommandHandler<T>::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>())
      .def("cancel", &RobotCommandHandler<T>::Cancel, py::call_guard<py::gil_scoped_release>())
      .def("get", &RobotCommandHandler<T>::Get, py::call_guard<py::gil_scoped_release>())
      .def("get_status", &RobotCommandHandler<T>::GetStatus);
}

template <typename T>
void bind_robot_command_stream_handler(py::module_& m, const std::string& handler_name) {
  py::class_<RobotCommandStreamHandler<T>>(m, handler_name.c_str())
      .def("is_done", &RobotCommandStreamHandler<T>::IsDone)
      .def("wait", &RobotCommandStreamHandler<T>::Wait, py::call_guard<py::gil_scoped_release>())
      .def("wait_for", &RobotCommandStreamHandler<T>::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>())
      .def("send_command", &RobotCommandStreamHandler<T>::SendCommand, "builder"_a, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>())
      .def("request_feedback", &RobotCommandStreamHandler<T>::RequestFeedback, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>())
      .def("cancel", &RobotCommandStreamHandler<T>::Cancel, py::call_guard<py::gil_scoped_release>());
}

template <typename T>
void bind_robot(py::module_& m, const std::string& robot_name) {
  bind_robot_command_handler<T>(m, robot_name + "_CommandHandler");
  bind_robot_command_stream_handler<T>(m, robot_name + "_CommandStreamHandler");

  py::class_<Robot<T>, std::shared_ptr<Robot<T>>>(m, robot_name.c_str())
      .def_static("create", &Robot<T>::Create)
      .def("connect", &Robot<T>::Connect, "max_retries"_a = 5, "timeout_ms"_a = 1000)
      .def("disconnect", &Robot<T>::Disconnect)
      .def("is_connected", &Robot<T>::IsConnected)
      .def("get_robot_info", &Robot<T>::GetRobotInfo)
      .def("power_on", &Robot<T>::PowerOn, "dev_name"_a)
      .def("power_off", &Robot<T>::PowerOff, "dev_name"_a)
      .def("is_power_on", &Robot<T>::IsPowerOn, "dev_name"_a)
      .def("servo_on", &Robot<T>::ServoOn, "dev_name"_a)
      .def("is_servo_on", &Robot<T>::IsServoOn, "dev_name"_a)
      .def("enable_control_manager", &Robot<T>::EnableControlManager)
      .def("disable_control_manager", &Robot<T>::DisableControlManager)
      .def("reset_fault_control_manager", &Robot<T>::ResetFaultControlManager)
      .def("set_tool_flange_output_voltage", &Robot<T>::SetToolFlangeOutputVoltage)
      .def("start_state_update", &Robot<T>::StartStateUpdate, "cb"_a, "rate"_a)
      .def("stop_state_update", &Robot<T>::StopStateUpdate)
      .def("start_log_stream", &Robot<T>::StartLogStream, "cb"_a, "rate"_a)
      .def("stop_log_stream", &Robot<T>::StopLogStream)
      .def("get_state", &Robot<T>::GetState)
      .def("get_last_log", &Robot<T>::GetLastLog, "count"_a)
      .def("get_control_manager_state", &Robot<T>::GetControlManagerState)
      .def("send_command", &Robot<T>::SendCommand, "builder"_a, "priority"_a)
      .def("create_command_stream", &Robot<T>::CreateCommandStream, "priority"_a)
      .def("reset_odometry", &Robot<T>::ResetOdometry, "angle"_a, "position"_a)
      .def("set_parameter", &Robot<T>::SetParameter, "name"_a, "value"_a)
      .def("get_parameter", &Robot<T>::GetParameter, "name"_a);
}

void pybind11_robot(py::module_& m) {
  bind_robot<y1_model::A>(m, "Robot_A");
}