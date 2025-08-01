#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

#include "model.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void bind_pid_gain(pybind11::module_& m) {
  py::class_<PIDGain>(m, "PIDGain")
      .def(py::init<>())
      .def(py::init<uint16_t, uint16_t, uint16_t>())
      .def_readonly("p_gain", &PIDGain::p_gain)
      .def_readonly("i_gain", &PIDGain::i_gain)
      .def_readonly("d_gain", &PIDGain::d_gain)
      .def("__repr__", [](const PIDGain& self) {
        std::stringstream ss;
        ss << "PIDGain("                  //
           << "p_gain=" << self.p_gain    //
           << ", i_gain=" << self.i_gain  //
           << ", d_gain=" << self.d_gain << ")";
        return ss.str();
      });
}

void bind_color(pybind11::module_& m) {
  py::class_<Color>(m, "Color")
      .def(py::init<>())
      .def(py::init<uint8_t, uint8_t, uint8_t>(), "r"_a, "g"_a, "b"_a)
      .def_readwrite("r", &Color::r)
      .def_readwrite("g", &Color::g)
      .def_readwrite("b", &Color::b);
}

void bind_serial(py::module_& m) {
  py::class_<SerialDevice>(m, "SerialDevice")
      .def(py::init<>())
      .def_readonly("path", &SerialDevice::path)
      .def_readonly("description", &SerialDevice::description)
      .def("__repr__", [](const SerialDevice& self) {
        std::stringstream ss;
        ss << "SerialDevice(path='" << self.path << "', description='" << self.description << "')";
        return ss.str();
      });

  py::class_<SerialStream>(m, "SerialStream")
      .def("connect", &SerialStream::Connect, "verbose"_a)
      .def("disconnect", &SerialStream::Disconnect)
      .def("wait",
           [](SerialStream& self) {
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
      .def("wait_for", &SerialStream::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>())
      .def("is_opened", &SerialStream::IsOpened)
      .def("is_cancelled", &SerialStream::IsCancelled)
      .def("is_done", &SerialStream::IsDone)
      .def("set_read_callback",
           [](SerialStream& self, std::function<void(py::bytes)> cb) {
             self.SetReadCallback([=](const std::string& data) {
               py::gil_scoped_acquire gil;
               cb(py::bytes(data));
             });
           })
      .def("write", py::overload_cast<const std::string&>(&SerialStream::Write), "data"_a,
           py::call_guard<py::gil_scoped_release>())
      .def("write", py::overload_cast<const char*>(&SerialStream::Write), "data"_a,
           py::call_guard<py::gil_scoped_release>())
      .def("write", py::overload_cast<const char*, int>(&SerialStream::Write), "data"_a, "n"_a,
           py::call_guard<py::gil_scoped_release>())
      .def("write_byte", &SerialStream::WriteByte, "ch"_a, py::call_guard<py::gil_scoped_release>());
}

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
      .def_static("model", []() { return PyModel<T>(); })
      .def_static("create", &Robot<T>::Create, "address"_a)
      .def(
          "connect",
          [](Robot<T>& self, int max_retries, int timeout_ms) {
            return self.Connect(max_retries, timeout_ms, []() {
              py::gil_scoped_acquire gil;
              if (PyErr_CheckSignals() != 0) {
                throw py::error_already_set();
              }
              return true;
            });
          },
          "max_retries"_a = 5, "timeout_ms"_a = 1000, py::call_guard<py::gil_scoped_release>(),
          R"(
connect(max_retries=5, timeout_ms=1000)

Attempts to establish a connection with the robot.

Parameters
----------
max_retries : int, optional
    Maximum number of retries to connect. Default is 5.
timeout_ms : int, optional
    Timeout in milliseconds for each connection attempt. Default is 1000.

Returns
-------
bool
    True if the connection was successful, false otherwise.
        )")
      .def("disconnect", &Robot<T>::Disconnect, py::call_guard<py::gil_scoped_release>(), R"(
disconnect()

Disconnects from the robot.
        )")
      .def("is_connected", &Robot<T>::IsConnected, R"(
is_connected()

Checks whether the robot is currently connected.

Returns
-------
bool
    True if the robot is connected to the robot, False otherwise.
      )")
      .def("get_robot_info", &Robot<T>::GetRobotInfo, R"(
get_robot_info()

Retrieves static information about the robot, such as model name, SDK version, and joint configuration.

Returns
-------
RobotInfo
    Structured metadata including joint details, device names, and robot model information.
      )")
      .def("get_time_scale", &Robot<T>::GetTimeScale)
      .def("set_time_scale", &Robot<T>::SetTimeScale, "time_scale"_a)
      .def("power_on", &Robot<T>::PowerOn, "dev_name"_a, py::call_guard<py::gil_scoped_release>())
      .def("power_off", &Robot<T>::PowerOff, "dev_name"_a, py::call_guard<py::gil_scoped_release>())
      .def("is_power_on", &Robot<T>::IsPowerOn, "dev_name"_a)
      .def("servo_on", &Robot<T>::ServoOn, "dev_name"_a, py::call_guard<py::gil_scoped_release>())
      .def("is_servo_on", &Robot<T>::IsServoOn, "dev_name"_a)
      .def("servo_off", &Robot<T>::ServoOff, "dev_name"_a)
      .def("break_engage", &Robot<T>::BreakEngage, "dev_name"_a)
      .def("break_release", &Robot<T>::BreakRelease, "dev_name"_a)
      .def("home_offset_reset", &Robot<T>::HomeOffsetReset, "dev_name"_a)
      .def("set_preset_position", &Robot<T>::SetPresetPosition, "joint_name"_a)
      .def("enable_control_manager", &Robot<T>::EnableControlManager, py::arg("unlimited_mode_enabled") = false,
           py::call_guard<py::gil_scoped_release>())
      .def("disable_control_manager", &Robot<T>::DisableControlManager, py::call_guard<py::gil_scoped_release>())
      .def("reset_fault_control_manager", &Robot<T>::ResetFaultControlManager, py::call_guard<py::gil_scoped_release>())
      .def("cancel_control", &Robot<T>::CancelControl, py::call_guard<py::gil_scoped_release>())
      .def("set_tool_flange_output_voltage", &Robot<T>::SetToolFlangeOutputVoltage,
           py::call_guard<py::gil_scoped_release>())
      .def("set_tool_flange_digital_output", &Robot<T>::SetToolFlangeDigitalOutput, "name"_a, "channel"_a, "duty"_a,
           py::call_guard<py::gil_scoped_release>())
      .def("set_tool_flange_digital_output_dual", &Robot<T>::SetToolFlangeDigitalOutputDual, "name"_a, "duty_0"_a,
           "duty_1"_a, py::call_guard<py::gil_scoped_release>())
      .def(
          "start_state_update",
          [](Robot<T>& self, py::function& cb, double rate) {
            pybind11::module inspect_module = pybind11::module::import("inspect");
            pybind11::object result = inspect_module.attr("signature")(cb).attr("parameters");
            auto num_params = pybind11::len(result);

            if (num_params == 1) {
              self.StartStateUpdate(py::cast<const std::function<void(const RobotState<T>&)>>(cb), rate);
            } else if (num_params == 2) {
              self.StartStateUpdate(
                  py::cast<const std::function<void(const RobotState<T>&, const ControlManagerState&)>>(cb), rate);
            } else {
              throw std::invalid_argument("The number of arguments of 'cb' can be one or two.");
            }
          },
          "cb"_a, "rate"_a)
      .def("stop_state_update", &Robot<T>::StopStateUpdate, py::call_guard<py::gil_scoped_release>())
      .def("start_log_stream", &Robot<T>::StartLogStream, "cb"_a, "rate"_a)
      .def("stop_log_stream", &Robot<T>::StopLogStream)
      .def("get_state", &Robot<T>::GetState, py::call_guard<py::gil_scoped_release>())
      .def("get_last_log", &Robot<T>::GetLastLog, "count"_a)
      .def("get_fault_log_list", &Robot<T>::GetFaultLogList)
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
      .def("set_parameter", &Robot<T>::SetParameter, "name"_a, "value"_a, "write_db"_a = true)
      .def("get_parameter", &Robot<T>::GetParameter, "name"_a)
      .def(
          "reset_parameter_to_default",
          [](Robot<T>& self, const std::string& name) {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "reset_parameter_to_default() is deprecated, use factory_reset_parameter() instead.", 1);
            return self.ResetParameterToDefault(name);
          },
          "name"_a)
      .def("reset_all_parameters_to_default",
           [](Robot<T>& self) {
             PyErr_WarnEx(
                 PyExc_DeprecationWarning,
                 "reset_all_parameters_to_default() is deprecated, use factory_reset_all_parameters() instead.", 1);
             self.ResetAllParametersToDefault();
           })
      .def("reset_parameter", &Robot<T>::ResetParameter, "name"_a)
      .def("reset_all_parameters", &Robot<T>::ResetAllParameters)
      .def("factory_reset_parameter", &Robot<T>::FactoryResetParameter, "name"_a)
      .def("factory_reset_all_parameters", &Robot<T>::FactoryResetAllParameters)
      .def("get_robot_model", &Robot<T>::GetRobotModel)
      .def("import_robot_model", &Robot<T>::ImportRobotModel, "name"_a, "model"_a)
      .def("sync_time", &Robot<T>::SyncTime)
      .def("has_established_time_sync", &Robot<T>::HasEstablishedTimeSync)
      .def("start_time_sync", &Robot<T>::StartTimeSync, "period_sec"_a)
      .def("stop_time_sync", &Robot<T>::StopTimeSync, py::call_guard<py::gil_scoped_release>())
      .def("get_dynamics", &Robot<T>::GetDynamics, "urdf_model"_a = "")
      .def("set_led_color", &Robot<T>::SetLEDColor, "color"_a, "duration"_a = 1, "transition_time"_a = 0,
           "blinking"_a = false, "blinking_freq"_a = 1)
      .def("get_system_time",
           [](Robot<T>& self) {
             const auto& [ts, tz_string, local_time_string] = self.GetSystemTime();

             py::module_ datetime = py::module_::import("datetime");
             py::module_ zoneinfo = py::module_::import("zoneinfo");

             double epoch_seconds = ts.tv_sec + static_cast<double>(ts.tv_nsec) / 1000000000.0;
             py::object dt =
                 datetime.attr("datetime").attr("fromtimestamp")(epoch_seconds, datetime.attr("timezone").attr("utc"));
             py::object tzinfo = zoneinfo.attr("ZoneInfo")(tz_string);
             dt = dt.attr("astimezone")(tzinfo);

             return std::pair(dt, local_time_string);
           })
      .def(
          "set_system_time",
          [](Robot<T>& self, py::object dt) {
            py::module_ datetime_mod = py::module_::import("datetime");
            py::object datetime_type = datetime_mod.attr("datetime");

            if (!py::isinstance(dt, datetime_type)) {
              throw std::runtime_error("Argument must be a datetime.datetime object");
            }

            std::string timezone_string;

            py::object tzinfo = dt.attr("tzinfo");
            if (tzinfo.is_none()) {
              dt = dt.attr("astimezone")();  // Naive datetime
            }

            tzinfo = dt.attr("tzinfo");
            if (py::hasattr(tzinfo, "key")) {
              // zoneinfo.ZoneInfo
              timezone_string = tzinfo.attr("key").cast<std::string>();
            } else {
              py::object dt_tzname = dt.attr("tzname")();
              if (!dt_tzname.is_none()) {
                timezone_string = dt_tzname.cast<std::string>();
              } else {
                timezone_string = "";
              }
            }
            double epoch_seconds = dt.attr("timestamp")().cast<double>();

            struct timespec ts;
            ts.tv_sec = static_cast<time_t>(epoch_seconds);
            double fractional = epoch_seconds - static_cast<double>(ts.tv_sec);
            ts.tv_nsec = static_cast<long>(fractional * 1000000000L);

            return self.SetSystemTime(ts, timezone_string.empty() ? std::nullopt : std::make_optional(timezone_string));
          },
          "datetime"_a)
      .def(
          "set_system_time",
          [](Robot<T>& self, py::object dt, const std::string& time_zone) {
            py::module_ datetime_mod = py::module_::import("datetime");
            py::object datetime_type = datetime_mod.attr("datetime");

            if (!py::isinstance(dt, datetime_type)) {
              throw std::runtime_error("Argument must be a datetime.datetime object");
            }
            double epoch_seconds = dt.attr("timestamp")().cast<double>();

            struct timespec ts;
            ts.tv_sec = static_cast<time_t>(epoch_seconds);
            double fractional = epoch_seconds - static_cast<double>(ts.tv_sec);
            ts.tv_nsec = static_cast<long>(fractional * 1000000000L);

            return self.SetSystemTime(ts, time_zone.empty() ? std::nullopt : std::make_optional(time_zone));
          },
          "datetime"_a, "time_zone"_a)
      .def("set_battery_level", &Robot<T>::SetBatteryLevel)
      .def("set_battery_config", &Robot<T>::SetBatteryConfig)
      .def("reset_battery_config", &Robot<T>::ResetBatteryConfig)
      .def("wait_for_control_ready", &Robot<T>::WaitForControlReady, "timeout_ms"_a)

      .def("reset_network_setting", &Robot<T>::ResetNetworkSetting, py::call_guard<py::gil_scoped_release>())
      .def("scan_wifi", &Robot<T>::ScanWifi, py::call_guard<py::gil_scoped_release>())
      .def("connect_wifi", &Robot<T>::ConnectWifi, "ssid"_a, "password"_a = "", "use_dhcp"_a = true,
           "ip_address"_a = "", "gateway"_a = "", "dns"_a = std::vector<std::string>{},
           py::call_guard<py::gil_scoped_release>())
      .def("disconnect_wifi", &Robot<T>::DisconnectWifi, py::call_guard<py::gil_scoped_release>())
      .def("get_wifi_status", &Robot<T>::GetWifiStatus, py::call_guard<py::gil_scoped_release>())

      .def("get_serial_device_list", &Robot<T>::GetSerialDeviceList, py::call_guard<py::gil_scoped_release>())
      .def("open_serial_stream", &Robot<T>::OpenSerialStream, "device_path"_a, "baudrate"_a, "bytesize"_a = 8,
           "parity"_a = 'N', "stopbits"_a = 1)
      .def(
          "download_file",
          [](Robot<T>& self, const std::string& path, py::object py_file_like) {
            return self.DownloadFileToCallback(
                path, [&](const char* data, size_t size) { py_file_like.attr("write")(py::bytes(data, size)); });
          },
          "path"_a, "file_like"_a, R"(
download_file(path, file_like)

Downloads a file from the robot and writes it into a file-like object.

Parameters
----------
path : str
    Relative path to the file on the robot's file system.
    The base directory is defined by the `RBY1_HOME` environment variable (default: `/root/.rby1`).
file_like : file-like object
    Any object with a `.write(bytes)` method (e.g., `io.BytesIO`, a real file).

Returns
-------
bool
    True if the file was downloaded successfully.

Raises
------
RuntimeError
    If the gRPC call fails (e.g., network issue, file not found, internal error).
           )")

      .def("set_position_p_gain", &Robot<T>::SetPositionPGain, "dev_name"_a, "p_gain"_a)
      .def("set_position_i_gain", &Robot<T>::SetPositionIGain, "dev_name"_a, "i_gain"_a)
      .def("set_position_d_gain", &Robot<T>::SetPositionDGain, "dev_name"_a, "d_gain"_a)
      .def("set_position_pid_gain",
           static_cast<bool (Robot<T>::*)(const std::string&, uint16_t, uint16_t, uint16_t) const>(
               &Robot<T>::SetPositionPIDGain),
           "dev_name"_a, "p_gain"_a, "i_gain"_a, "d_gain"_a)
      .def("set_position_pid_gain",
           static_cast<bool (Robot<T>::*)(const std::string&, const rb::PIDGain&) const>(&Robot<T>::SetPositionPIDGain),
           "dev_name"_a, "pid_gain"_a)

      .def("get_torso_position_pid_gains", &Robot<T>::GetTorsoPositionPIDGains)
      .def("get_right_arm_position_pid_gains", &Robot<T>::GetRightArmPositionPIDGains)
      .def("get_left_arm_position_pid_gains", &Robot<T>::GetLeftArmPositionPIDGains)
      .def("get_head_position_pid_gains", &Robot<T>::GetHeadPositionPIDGains)
      .def("get_position_pid_gain", &Robot<T>::GetPositionPIDGain, "dev_name"_a);
}

void pybind11_robot(py::module_& m) {
  bind_pid_gain(m);
  bind_color(m);
  bind_serial(m);
  bind_robot<y1_model::A>(m, "Robot_A");
  bind_robot<y1_model::T5>(m, "Robot_T5");
  bind_robot<y1_model::M>(m, "Robot_M");
  bind_robot<y1_model::UB>(m, "Robot_UB");
}