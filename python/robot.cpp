#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

#include "model.h"
#include "print_helper.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;
using rb::print::indent_continuation;
using rb::print::inline_obj;
using rb::print::np_array_to_string;
using rb::print::np_shape_dtype;
using rb::print::Style;

void bind_pid_gain(pybind11::module_& m) {
  py::class_<PIDGain>(m, "PIDGain", R"doc(
PID gain configuration.

Represents proportional, integral, and derivative gains for PID control.

Attributes
----------
p_gain : int
    Proportional gain value.
i_gain : int
    Integral gain value.
d_gain : int
    Derivative gain value.
)doc")
      .def(py::init<>(), R"doc(
Construct a PIDGain instance with default values.
)doc")
      .def(py::init<uint16_t, uint16_t, uint16_t>(), R"doc(
Construct a PIDGain instance with specified values.

Parameters
----------
p_gain : int
    Proportional gain value.
i_gain : int
    Integral gain value.
d_gain : int
    Derivative gain value.
)doc")
      .def_readonly("p_gain", &PIDGain::p_gain)
      .def_readonly("i_gain", &PIDGain::i_gain)
      .def_readonly("d_gain", &PIDGain::d_gain)
      .def("__repr__",
           [](const PIDGain& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "PIDGain(" << FIRST               //
                 << "p_gain=" << self.p_gain << SEP   //
                 << "i_gain=" << self.i_gain << SEP   //
                 << "d_gain=" << self.d_gain << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const PIDGain& self) {
        std::ostringstream ss;
        ss << "PIDGain(p_gain=" << self.p_gain  //
           << ", i_gain=" << self.i_gain        //
           << ", d_gain=" << self.d_gain << ")";
        return ss.str();
      });
}

void bind_color(pybind11::module_& m) {
  py::class_<Color>(m, "Color", R"doc(
RGB color representation.

Represents a color using red, green, and blue components.

Attributes
----------
r : int
    Red component [0, 255].
g : int
    Green component [0, 255].
b : int
    Blue component [0, 255].
)doc")
      .def(py::init<>(), R"doc(
Construct a Color instance with default values (0, 0, 0).
)doc")
      .def(py::init<uint8_t, uint8_t, uint8_t>(), "r"_a, "g"_a, "b"_a, R"doc(
Construct a Color instance with specified RGB values.

Parameters
----------
r : int
    Red component [0, 255].
g : int
    Green component [0, 255].
b : int
    Blue component [0, 255].
)doc")
      .def_readwrite("r", &Color::r)
      .def_readwrite("g", &Color::g)
      .def_readwrite("b", &Color::b)
      .def("__repr__",
           [](const Color& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "Color(" << FIRST                         //
                 << "r=" << static_cast<int>(self.r) << SEP   //
                 << "g=" << static_cast<int>(self.g) << SEP   //
                 << "b=" << static_cast<int>(self.b) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const Color& self) {
        std::ostringstream ss;
        ss << "Color(r=" << static_cast<int>(self.r) << ", g=" << static_cast<int>(self.g)
           << ", b=" << static_cast<int>(self.b) << ")";
        return ss.str();
      });
}

void bind_serial(py::module_& m) {
  py::class_<SerialDevice>(m, "SerialDevice", R"doc(
Serial device information.

Represents information about a serial device.

Attributes
----------
path : str
    Device path (e.g., "/dev/ttyUSB0").
description : str
    Device description.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``SerialDevice`` instance.
)doc")
      .def_readonly("path", &SerialDevice::path)
      .def_readonly("description", &SerialDevice::description)
      .def("__repr__",
           [](const SerialDevice& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "SerialDevice(" << FIRST                                          //
                 << "path=" << inline_obj(py::cast(self.path)) << SEP                 //
                 << "description=" << inline_obj(py::cast(self.description)) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const SerialDevice& self) {
        std::ostringstream out;
        out << "SerialDevice(" << self.path << " - " << self.description << ")";
        return out.str();
      });

  py::class_<SerialStream>(m, "SerialStream", R"doc(
Serial communication stream.

Provides serial communication functionality with read/write operations
and callback support.

Attributes
----------
is_opened : bool
    Whether the stream is currently open.
is_cancelled : bool
    Whether the stream operation was cancelled.
is_done : bool
    Whether the stream operation is complete.
)doc")
      .def("connect", &SerialStream::Connect, "verbose"_a, R"doc(
Connect to the serial device.

Parameters
----------
verbose : bool
    Whether to enable verbose output.

Returns
-------
bool
    True if connection successful, False otherwise.
)doc")
      .def("disconnect", &SerialStream::Disconnect, R"doc(
Disconnect from the serial device.
)doc")
      .def(
          "wait",
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
          },
          R"doc(
Wait for the stream operation to complete.

This method blocks until the operation is done or cancelled.
)doc")
      .def("wait_for", &SerialStream::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Wait for the stream operation with timeout.

Parameters
----------
timeout_ms : int
    Timeout in milliseconds.

Returns
-------
bool
    ``True`` if operation completed, ``False`` if timeout.
)doc")
      .def("is_opened", &SerialStream::IsOpened, R"doc(
is_opened()

Check if the stream is currently open.

Returns
-------
bool
    True if stream is open, False otherwise.
)doc")
      .def("is_cancelled", &SerialStream::IsCancelled, R"doc(
Check if the stream operation was cancelled.

Returns
-------
bool
    True if operation was cancelled, False otherwise.
)doc")
      .def("is_done", &SerialStream::IsDone, R"doc(
is_done()

Check if the stream operation is complete.

Returns
-------
bool
    True if operation is complete, False otherwise.
)doc")
      .def(
          "set_read_callback",
          [](SerialStream& self, std::function<void(py::bytes)> cb) {
            self.SetReadCallback([=](const std::string& data) {
              py::gil_scoped_acquire gil;
              cb(py::bytes(data));
            });
          },
          R"doc(
Set a callback function for read operations.

Parameters
----------
cb : callable
    Callback function that receives bytes data.

Examples
--------
>>> dev = robot.open_serial_stream(device_path, baudrate)
>>> connected = dev.connect(verbose=True)
>>> if not connected:
...     print("Failed to connect")
...     exit(1)
>>> print("Listening for incoming data...\n")
>>> def print_hex(data):
...     hex_string = "".join(f"{int(ch):02X}" for ch in data)
...     print(f"<< {hex_string}", flush=True)
>>> dev.set_read_callback(print_hex)
)doc")
      .def("write", py::overload_cast<const std::string&>(&SerialStream::Write), "data"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Write character data to the serial stream.

Parameters
----------
data : str
    Character data to write.
)doc")
      .def("write", py::overload_cast<const char*>(&SerialStream::Write), "data"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Write character data with specified length to the serial stream.

Parameters
----------
data : str
    Character data to write.
n : int
    Number of characters to write.
)doc")
      .def("write", py::overload_cast<const char*, int>(&SerialStream::Write), "data"_a, "n"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Write character data with specified length to the serial stream.

Parameters
----------
data : str
    Character data to write.
n : int
    Number of characters to write.
)doc")
      .def("write_byte", &SerialStream::WriteByte, "ch"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Write a single byte to the serial stream.

Parameters
----------
ch : int
    Byte value to write [0, 255].
)doc")
      .def("__repr__",
           [](const SerialStream& self) {
             const bool ml = rb::print::use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "SerialStream(" << FIRST                                           //
                 << "is_opened=" << (self.IsOpened() ? "True" : "False") << SEP        //
                 << "is_cancelled=" << (self.IsCancelled() ? "True" : "False") << SEP  //
                 << "is_done=" << (self.IsDone() ? "True" : "False") << LAST           //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const SerialStream& self) {
        std::ostringstream out;
        out << "SerialStream(opened=" << (self.IsOpened() ? "True" : "False")
            << ", cancelled=" << (self.IsCancelled() ? "True" : "False")
            << ", done=" << (self.IsDone() ? "True" : "False") << ")";
        return out.str();
      });
}

template <typename T>
void bind_robot_command_handler(py::module_& m) {
  const std::string model = std::string(T::kModelName);
  const std::string robot_name = std::string("Robot_") + model;
  const std::string handler_name = robot_name + "_CommandHandler";

  std::stringstream ss;
  ss << "Robot (model: ``Model_" << model << "``) command handler.\n";
  ss << R"doc(
Handles robot command execution and provides status monitoring.
)doc";

  py::class_<RobotCommandHandler<T>>(m, handler_name.c_str(), ss.str().c_str())
      .def("is_done", &RobotCommandHandler<T>::IsDone, R"doc(
Check if the command execution is complete.

Returns
-------
bool
    ``True`` if command is complete, ``False`` otherwise.
)doc")
      .def(
          "wait",
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
          },
          R"doc(
Wait for the command execution to complete.

This method blocks until the command is done or cancelled.
)doc")
      .def("wait_for", &RobotCommandHandler<T>::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Wait for the command execution with timeout.

Parameters
----------
timeout_ms : int
    Timeout in milliseconds.

Returns
-------
bool
    ``True`` if command completed, ``False`` if timeout.
)doc")
      .def("cancel", &RobotCommandHandler<T>::Cancel, py::call_guard<py::gil_scoped_release>(), R"doc(
Cancel the command execution.
)doc")
      .def(
          "get",
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
          },
          R"doc(
Wait for the command execution and get feedback

Returns
-------
RobotCommandFeedback
    Current feedback information.
)doc")
      .def("get_status", &RobotCommandHandler<T>::GetStatus, R"doc(
Get gRPC status
)doc")
      .def("__repr__",
           [](const RobotCommandHandler<T>& self) {
             const bool ml = rb::print::use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "RobotCommandHandler(" << FIRST                           //
                 << "is_done=" << (self.IsDone() ? "True" : "False") << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const RobotCommandHandler<T>& self) {
        std::ostringstream out;
        out << "RobotCommandHandler(done=" << (self.IsDone() ? "True" : "False") << ")";
        return out.str();
      });
}

template <typename T>
void bind_robot_command_stream_handler(py::module_& m) {
  const std::string model = std::string(T::kModelName);
  const std::string robot_name = std::string("Robot_") + model;
  const std::string handler_name = robot_name + "_CommandStreamHandler";

  std::stringstream ss;
  ss << "Robot (model: ``Model_" << model << "``) command stream handler.\n";
  ss << R"doc(
Handles robot command execution through streaming with send and feedback capabilities.

Attributes
----------
is_done : bool
    Whether the command execution is complete.
)doc";

  py::class_<RobotCommandStreamHandler<T>>(m, handler_name.c_str(), ss.str().c_str())
      .def("is_done", &RobotCommandStreamHandler<T>::IsDone, R"doc(
Check if the command execution is complete.

Returns
-------
bool
    True if command is complete, False otherwise.
)doc")
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
      .def("wait_for", &RobotCommandStreamHandler<T>::WaitFor, "timeout_ms"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Wait for the command execution to complete.

This method blocks until the command is done or cancelled.
)doc")
      .def("send_command", &RobotCommandStreamHandler<T>::SendCommand, "builder"_a, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Send a command through the stream.

Parameters
----------
builder : CommandBuilder
    Command builder to send.
timeout_ms : int, optional
    Timeout in milliseconds. Default is 1000.

Returns
-------
RobotCommandFeedback
    Current feedback information.
)doc")
      .def("request_feedback", &RobotCommandStreamHandler<T>::RequestFeedback, "timeout_ms"_a = 1000,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Request feedback from the robot.

Parameters
----------
timeout_ms : int, optional
    Timeout in milliseconds. Default is 1000.

Returns
-------
RobotCommandFeedback
    Current feedback information.
)doc")
      .def("cancel", &RobotCommandStreamHandler<T>::Cancel, py::call_guard<py::gil_scoped_release>(), R"doc(
Cancel the current command execution.
)doc")
      .def("__repr__",
           [](const RobotCommandStreamHandler<T>& self) {
             const bool ml = rb::print::use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "RobotCommandStreamHandler(" << FIRST                     //
                 << "is_done=" << (self.IsDone() ? "True" : "False") << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const RobotCommandStreamHandler<T>& self) {
        std::ostringstream out;
        out << "RobotCommandStreamHandler(done=" << (self.IsDone() ? "True" : "False") << ")";
        return out.str();
      });
}

template <typename T>
void bind_control_state(py::module_& m) {
  const std::string model = std::string(T::kModelName);
  const std::string robot_name = std::string("Robot_") + model;
  const std::string handler_name = robot_name + "_ControlState";

  std::stringstream ss;
  ss << "Robot (model: ``Model_" << model << "``) control state.\n";
  ss << R"doc(
Represents the current state of robot real-time control including position, velocity, and torque.

Attributes
----------
t : float
    Current time in seconds.
is_ready : numpy.ndarray
    Whether the joint is ready for control.
position : numpy.ndarray, shape (DOF,)
    Current joint positions [rad].
velocity : numpy.ndarray, shape (DOF,)
    Current joint velocities [rad/s].
current : numpy.ndarray, shape (DOF,)
    Current joint currents [A].
torque : numpy.ndarray, shape (DOF,)
    Current joint torques [Nm].
)doc";

  py::class_<ControlState<T>>(m, handler_name.c_str(), ss.str().c_str())
      .def_readonly("t", &ControlState<T>::t)
      .def_readonly("is_ready", &ControlState<T>::is_ready)
      .def_readonly("position", &ControlState<T>::position)
      .def_readonly("velocity", &ControlState<T>::velocity)
      .def_readonly("current", &ControlState<T>::current)
      .def_readonly("torque", &ControlState<T>::torque)
      .def("__repr__",
           [handler_name](const ControlState<T>& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n" : ", ";
             const char* LAST = ml ? "\n" : "";

             Eigen::Index ready = 0;
             for (Eigen::Index i = 0; i < self.is_ready.size(); ++i) {
               if (self.is_ready[i]) {
                 ++ready;
               }
             }
             const Eigen::Index total = self.is_ready.size();

             std::ostringstream out;
             out << handler_name << "(" << FIRST << (ml ? "  t=" : "t=") << format_number(self.t, Style::Repr) << SEP
                 << (ml ? "  ready=" : "ready=") << ready << "/" << total << SEP;

             {
               std::string k = ml ? "  position=" : "position=";
               std::string v = np_array_to_string(py::cast(self.position), Style::Repr);
               out << k << indent_continuation(v, (int)k.size()) << SEP;
             }
             {
               std::string k = ml ? "  velocity=" : "velocity=";
               std::string v = np_array_to_string(py::cast(self.velocity), Style::Repr);
               out << k << indent_continuation(v, (int)k.size()) << SEP;
             }
             {
               std::string k = ml ? "  current=" : "current=";
               std::string v = np_array_to_string(py::cast(self.current), Style::Repr);
               out << k << indent_continuation(v, (int)k.size()) << SEP;
             }
             {
               std::string k = ml ? "  torque=" : "torque=";
               std::string v = np_array_to_string(py::cast(self.torque), Style::Repr);
               out << k << indent_continuation(v, (int)k.size());
             }

             out << LAST << ")";
             return out.str();
           })
      .def("__str__", [handler_name](const ControlState<T>& self) {
        using namespace rb::print;
        Eigen::Index ready = 0;
        for (Eigen::Index i = 0; i < self.is_ready.size(); ++i)
          if (self.is_ready[i])
            ++ready;
        const Eigen::Index total = self.is_ready.size();

        std::ostringstream ss;
        ss << handler_name << "(t=" << format_number(self.t, Style::Str) << ", ready=" << ready << "/" << total
           << ", q=" << np_array_to_string(py::cast(self.position), Style::Str)
           << ", dq=" << np_array_to_string(py::cast(self.velocity), Style::Str)
           << ", tau=" << np_array_to_string(py::cast(self.torque), Style::Str) << ")";
        return ss.str();
      });
}

template <typename T>
void bind_control_input(py::module_& m) {
  const std::string model = std::string(T::kModelName);
  const std::string robot_name = std::string("Robot_") + model;
  const std::string handler_name = robot_name + "_ControlInput";

  std::stringstream ss;
  ss << "Robot (model: ``Model_" << model << "``) control input.\n";
  ss << R"doc(
Represents control input parameters for robot real-time control including mode, target, and gains.

Attributes
----------
mode : numpy.ndarray
    Control mode for each joint (boolean array).
target : numpy.ndarray
    Target positions for each joint [rad].
feedback_gain : numpy.ndarray
    Feedback gains for each joint.
feedforward_torque : numpy.ndarray
    Feedforward torque for each joint [Nm].
finish : bool
    Whether to finish the current control operation.
)doc";

  py::class_<ControlInput<T>>(m, handler_name.c_str(), ss.str().c_str())
      .def(py::init<>(), R"doc(
Construct a ``ControlInput`` instance.

Attributes
----------
mode : numpy.ndarray, shape (DOF,)
    Control mode for each joint (boolean array).
target : numpy.ndarray, shape (DOF,)
    Target positions for each joint [rad].
feedback_gain : numpy.ndarray, shape (DOF,)
    Feedback gains for each joint.
feedforward_torque : numpy.ndarray, shape (DOF,)
    Feedforward torque for each joint [Nm].
finish : bool
    Whether to finish the current control operation.
)doc")
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
      .def_readwrite("finish", &ControlInput<T>::finish)
      .def("__repr__",
           [handler_name](const ControlInput<T>& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << handler_name << "(" << FIRST                                                       //
                 << "mode=" << np_shape_dtype(py::cast(self.mode)) << SEP                              //
                 << "target=" << np_shape_dtype(py::cast(self.target)) << SEP                          //
                 << "feedback_gain=" << np_shape_dtype(py::cast(self.feedback_gain)) << SEP            //
                 << "feedforward_torque=" << np_shape_dtype(py::cast(self.feedforward_torque)) << SEP  //
                 << "finish=" << (self.finish ? "True" : "False") << LAST                              //
                 << ")";
             return out.str();
           })
      .def("__str__", [handler_name](const ControlInput<T>& self) {
        using namespace rb::print;
        std::ostringstream ss;
        ss << handler_name << "(mode=" << np_shape_dtype(py::cast(self.mode))
           << ", target=" << np_shape_dtype(py::cast(self.target)) << ", finish=" << (self.finish ? "True" : "False")
           << ")";
        return ss.str();
      });
}

template <typename T>
void bind_robot(py::module_& m) {
  const std::string model = std::string(T::kModelName);
  const std::string robot_name = std::string("Robot_") + model;
  bind_robot_command_handler<T>(m);
  bind_robot_command_stream_handler<T>(m);
  bind_control_state<T>(m);
  bind_control_input<T>(m);

  std::stringstream ss;
  ss << "Robot (model: ``Model_" << model << "``) control interface.\n";
  ss << R"doc(
Provides high-level control interface for robot operations including
connection management, power control, and command execution.

Attributes
----------
model : )doc";
  ss << "Model_" << T::kModelName << R"doc(
    Robot model configuration.
)doc";

  std::stringstream model_doc_ss;
  model_doc_ss << R"doc(
Get the robot model configuration.

Returns
-------
)doc";
  model_doc_ss << "Model_" << T::kModelName << R"doc(
    Robot model configuration object.
)doc";

  py::class_<Robot<T>, std::shared_ptr<Robot<T>>>(m, robot_name.c_str(), ss.str().c_str())
      .def_static(
          "model", []() { return PyModel<T>(); }, model_doc_ss.str().c_str())
      .def_static("create", &Robot<T>::Create, "address"_a, R"doc(
Create a robot instance.

Parameters
----------
address : str
    Robot network address, e.g. "192.168.1.100:50051".

Returns
-------
Robot
    Robot instance.

Examples
--------
>>> import rby1_sdk
>>> robot = rby.Robot_A.create("192.168.30.1:50051")
>>> robot.connect()
>>> if robot.is_connected():
...     print("Robot connected successfully")
...     robot_info = robot.get_robot_info()
...     print(f"Robot model: {robot.model().model_name}")
...     # Note: Model A has mobile base (2-DOF) + torso (6-DOF) + right arm (7-DOF) + left arm (7-DOF) + head (2-DOF)
...     # Total joints: 2 + 6 + 7 + 7 + 2 = 24 DOF for model A
)doc")
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
          "max_retries"_a = 5, "timeout_ms"_a = 1000, py::call_guard<py::gil_scoped_release>(), R"doc(
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
    ``True`` if the connection was successful, ``False`` otherwise.

Examples
--------
>>> robot = rby1_sdk.create_robot("192.168.1.100", "a")
>>> if robot.connect():
...     print("Connected to robot")
... else:
...     print("Failed to connect")
>>> # Default: max_retries=5, timeout_ms=1000
>>> # Custom: robot.connect(max_retries=10, timeout_ms=2000)
)doc")
      .def("disconnect", &Robot<T>::Disconnect, py::call_guard<py::gil_scoped_release>(), R"doc(
Disconnects from the robot.
)doc")
      .def("is_connected", &Robot<T>::IsConnected, R"doc(
Checks whether the robot is currently connected.

Returns
-------
bool
    ``True`` if the robot is connected to the robot, ``False`` otherwise.
)doc")
      .def("get_robot_info", &Robot<T>::GetRobotInfo, R"doc(
Retrieves static information about the robot, such as model name, SDK version, and joint configuration.

Returns
-------
RobotInfo
    Structured metadata including joint details, device names, and robot model information.

Examples
--------
>>> # Get robot information
>>> robot_info = robot.get_robot_info()
>>> print(f"SDK version: {robot_info.sdk_version}")
>>> print(f"Robot model name: {robot_info.robot_model_name}")
>>> print(f"Robot model version: {robot_info.robot_model_version}")
>>> print(f"Joint count: {len(robot_info.joint_infos)}")
>>> # Access joint information
>>> for joint in robot_info.joint_infos:
...     print(f"Joint: {joint.name}, Brake: {joint.has_brake}, Product name: {joint.product_name}, Firmware version: {joint.firmware_version}")
)doc")
      .def("get_time_scale", &Robot<T>::GetTimeScale, R"doc(
Get the current time scale.

Returns
-------
float
    Current time scale value.

Examples
--------
>>> # Get current time scale
>>> current_scale = robot.get_time_scale()
>>> print(f"Current time scale: {current_scale}")
>>> # Check if robot is running at full speed
>>> if current_scale == 1.0:
...     print("Robot running at full speed")
>>> # Check if robot is slowed down
>>> elif current_scale < 1.0:
...     print(f"Robot running at {current_scale * 100}% speed")
)doc")
      .def("set_time_scale", &Robot<T>::SetTimeScale, "time_scale"_a, R"doc(
Set the time scale for motion execution.

Parameters
----------
time_scale : float
    Time scale value (0.0 to 1.0).

Examples
--------
>>> # Set time scale for motion execution
>>> robot.set_time_scale(1.0)   # Normal speed (100%)
>>> robot.set_time_scale(0.5)   # Half speed (50%)
>>> robot.set_time_scale(0.25)  # Quarter speed (25%)
>>> # Range: 0.0 (stopped) to 1.0 (full speed)
>>> # Useful for testing and safety
)doc")
      .def("power_on", &Robot<T>::PowerOn, "dev_name"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Power on a device.

Parameters
----------
dev_name : str
    Device name to power on. Supports regex patterns.

Examples
--------
>>> robot.connect()
>>> if not robot.is_power_on(".*"):
...     robot.power_on(".*")
...     print("Robot powered on")
>>> # Pattern ".*" powers on all devices
>>> # Specific device: robot.power_on("12v"), robot.power_on("24v"), etc.
)doc")
      .def("power_off", &Robot<T>::PowerOff, "dev_name"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Power off a device.

Parameters
----------
dev_name : str
    Device name to power off. Supports regex patterns.
)doc")
      .def("is_power_on", &Robot<T>::IsPowerOn, "dev_name"_a, R"doc(
Check if a device is powered on.

Parameters
----------
dev_name : str
    Device name to check. Supports regex patterns.

Returns
-------
bool
    ``True`` if device is powered on, ``False`` otherwise.
)doc")
      .def("servo_on", &Robot<T>::ServoOn, "dev_name"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Enable servo control for a device.

Parameters
----------
dev_name : str
    Device name to enable servo control. Supports regex patterns.

Examples
--------
>>> robot.connect()
>>> if not robot.is_servo_on(".*"):
...     robot.servo_on(".*")  # Servo on all devices
...     print("All devices servoed on")
>>> # Specific device: robot.servo_on("^torso_.*"), robot.servo_on("^right_arm_.*"), etc.
)doc")
      .def("is_servo_on", &Robot<T>::IsServoOn, "dev_name"_a, R"doc(
Check if servo control is enabled for a device.

Parameters
----------
dev_name : str
    Device name to check. Supports regex patterns.

Returns
-------
bool
    ``True`` if servo control is enabled, ``False`` otherwise.
)doc")
      .def("servo_off", &Robot<T>::ServoOff, "dev_name"_a, R"doc(
Disable servo control for a device.

Parameters
----------
dev_name : str
    Device name to disable servo control. Supports regex patterns.

Examples
--------
>>> # Disable servo control for safety
>>> robot.servo_off(".*")  # All motor drivers
>>> # Specific device: robot.servo_off("^torso_.*")
)doc")
      .def("break_engage", &Robot<T>::BreakEngage, "dev_name"_a, R"doc(

Engage the brake for a device.

Parameters
----------
dev_name : str
    Device name (joint name) to engage brake. Supports regex patterns.

Examples
--------
>>> # Engage brake for manitance
>>> robot.break_engage("right_arm_0")
)doc")
      .def("break_release", &Robot<T>::BreakRelease, "dev_name"_a, R"doc(
break_release(dev_name)

Release the brake for a device.

Parameters
----------
dev_name : str
    Device name (joint name) to release brake. Supports regex patterns.

Examples
--------
>>> # Release brake to allow movement
>>> robot.break_release("right_arm_0")
)doc")
      .def("home_offset_reset", &Robot<T>::HomeOffsetReset, "dev_name"_a, R"doc(
Reset home offset for a device.

Parameters
----------
dev_name : str
    Device name to reset home offset. Supports regex patterns.
)doc")
      .def("set_preset_position", &Robot<T>::SetPresetPosition, "joint_name"_a, R"doc(
Set preset position for a joint (only available for PVL-based motors).

Parameters
----------
joint_name : str
    Joint name to set preset position.

Examples
--------
>>> # Set preset position for specific joints
>>> robot.set_preset_position("right_arm_6")  # Right arm joint 6
>>> # Preset position is used as reference for home offset
)doc")
      .def("enable_control_manager", &Robot<T>::EnableControlManager, py::arg("unlimited_mode_enabled") = false,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Enable the control manager.

Parameters
----------
unlimited_mode_enabled : bool, optional
    Whether to enable unlimited mode. Default is False.

Examples
--------
>>> # Enable control manager for robot control
>>> robot.enable_control_manager()
>>> # Enable with unlimited mode (use with caution)
>>> robot.enable_control_manager(unlimited_mode_enabled=True)
>>> # Must be called before sending commands
)doc")
      .def("disable_control_manager", &Robot<T>::DisableControlManager, py::call_guard<py::gil_scoped_release>(), R"doc(
Disable the control manager.
)doc")
      .def("reset_fault_control_manager", &Robot<T>::ResetFaultControlManager, py::call_guard<py::gil_scoped_release>(),
           R"doc(
Reset fault in the control manager.
)doc")
      .def("cancel_control", &Robot<T>::CancelControl, py::call_guard<py::gil_scoped_release>(), R"doc(
Cancel current control operation.
)doc")
      .def("set_tool_flange_output_voltage", &Robot<T>::SetToolFlangeOutputVoltage,
           py::call_guard<py::gil_scoped_release>(), "name"_a, "voltage"_a, R"doc(
Set tool flange output voltage.

Parameters
----------
name : str
    Tool flange name `"right"` or `"left"`.

voltage : int
    Output voltage in volts.

Examples
--------
>>> # Set tool flange output voltage
>>> robot.set_tool_flange_output_voltage("right", 12)  # 12V output
>>> robot.set_tool_flange_output_voltage("left", 24)  # 24V output
>>> # Common voltages: 12V, 24V
>>> # Use 0V to turn off output
>>> # Note: This method sets voltage for both tool flanges (right and left)
)doc")
      .def("set_tool_flange_digital_output", &Robot<T>::SetToolFlangeDigitalOutput, "name"_a, "channel"_a, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
Set the digital output state of a specific channel on the tool flange
of the specified arm.

Parameters
----------
name : str
    Arm identifier. Must be either `"right"` or `"left"`,
    indicating the tool flange of the right or left arm.
channel : int
    Digital output channel index.
state : bool
    Desired state of the digital output (``True`` = ON, ``False`` = OFF).

Returns
-------
bool
    ``True`` if the command was successfully sent, ``False`` otherwise.

Examples
--------
>>> # Turn on channel 0 on the right arm tool flange
>>> robot.set_tool_flange_digital_output("right", 0, True)
)doc")

      .def("set_tool_flange_digital_output_dual", &Robot<T>::SetToolFlangeDigitalOutputDual, "name"_a, "state_0"_a,
           "state_1"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
Set the digital output states of two channels on the tool flange
of the specified arm simultaneously.

Parameters
----------
name : str
    Arm identifier. Must be either `"right"` or `"left"`,
    indicating the tool flange of the right or left arm.
state_0 : bool
    Desired state for digital output channel 0 (True = ON, False = OFF).
state_1 : bool
    Desired state for digital output channel 1 (True = ON, False = OFF).

Returns
-------
bool
    ``True`` if the command was successfully sent, ``False`` otherwise.

Examples
--------
>>> # Turn on channel 0 and turn off channel 1 on the left arm tool flange
>>> robot.set_tool_flange_digital_output_dual("left", True, False)
)doc")
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
          "cb"_a, "rate"_a, R"doc(
Start state update callback.

Parameters
----------
cb : callable
    Callback function with 1 or 2 parameters:
    - cb(robot_state) or cb(robot_state, control_manager_state)
rate : float
    Update rate in Hz.

Examples
--------
>>> def state_callback(robot_state):
...     print(f"Timestamp: {robot_state.timestamp}, Joint positions: {robot_state.position}")
>>> # Start state updates at 100 Hz
>>> robot.start_state_update(state_callback, 100.0)
>>> # Stop when done: robot.stop_state_update()
)doc")
      .def("stop_state_update", &Robot<T>::StopStateUpdate, py::call_guard<py::gil_scoped_release>(), R"doc(
Stop state update callback.
)doc")
      .def("start_log_stream", &Robot<T>::StartLogStream, "cb"_a, "rate"_a, R"doc(
Start log stream callback.

Parameters
----------
cb : callable
    Callback function for log updates.
rate : float
    Update rate in Hz.

Examples
--------
>>> def log_callback(log_entries):
...     for log in log_entries:
...         print(f"{log.timestamp} - {log.level} - {log.message}")
>>> # Start log stream at 10 Hz
>>> robot.start_log_stream(log_callback, 10.0)
>>> # Stop when done: robot.stop_log_stream()
)doc")
      .def("stop_log_stream", &Robot<T>::StopLogStream, R"doc(
Stop log stream callback.
)doc")
      .def("get_state", &Robot<T>::GetState, py::call_guard<py::gil_scoped_release>(), R"doc(
Get current robot state.

Returns
-------
RobotState
    Current robot state.

Examples
--------
>>> robot_state = robot.get_state()
>>> model = robot.model()
>>> print(f"Joint positions: {robot_state.position}")
>>> print(f"Right arm position: {robot_state.position[model.right_arm_idx]}")
>>> # Note: Joint positions include mobile base (2-DOF) + torso (6-DOF) + right arm (7-DOF) + left arm (7-DOF) + head (2-DOF)
>>> # Total: 24 DOF for model A, 23 DOF for model M, 26 DOF for model M, 18 DOF for model UB
)doc")
      .def("get_last_log", &Robot<T>::GetLastLog, "count"_a, R"doc(
Get last log entries.

Parameters
----------
count : int
    Number of log entries to retrieve.

Returns
-------
list
    List of log entries.
)doc")
      .def("get_fault_log_list", &Robot<T>::GetFaultLogList, R"doc(
Get fault log list.

Returns
-------
list
    List of fault log entries.
)doc")
      .def("get_control_manager_state", &Robot<T>::GetControlManagerState, R"doc(
Get control manager state.

Returns
-------
ControlManagerState
    Current control manager state.

Examples
--------
>>> # Get control manager state
>>> ctrl_state = robot.get_control_manager_state()
>>> print(f"Control Manager state: {ctrl_state.state}")
>>> print(f"Control state: {ctrl_state.control_state}")
>>> # Check if control manager is enabled
>>> if ctrl_state.state == rby1_sdk.ControlManagerState.State.Enabled:
...     print("Control manager is enabled")
)doc")
      .def("send_command", &Robot<T>::SendCommand, "builder"_a, "priority"_a = 1, R"doc(
Send a command to the robot.

Parameters
----------
builder : CommandBuilder
    Command builder to send.
priority : int, optional
    Command priority. Default is 1.

Returns
-------
RobotCommandHandler
    Command handler for monitoring execution.

Examples
--------
>>> from rby1_sdk import RobotCommandBuilder, ComponentBasedCommandBuilder
>>> from rby1_sdk import BodyComponentBasedCommandBuilder, JointPositionCommandBuilder
>>> # Create command with method chaining (standard pattern)
>>> command = RobotCommandBuilder().set_command(
...     ComponentBasedCommandBuilder().set_body_command(
...         BodyComponentBasedCommandBuilder()
...             .set_torso_command(
...                 JointPositionCommandBuilder()
...                     .set_position([0, 0, 0, 0, 0, 0])  # 6-DOF torso
...                     .set_minimum_time(2.0)
...             )
...     )
... )
>>> handler = robot.send_command(command)
>>> handler.wait()  # Wait for completion
>>> # This pattern is used in most examples: examples/python/07_impedance_control.py, 09_demo_motion.py, etc.
)doc")
      .def("create_command_stream", &Robot<T>::CreateCommandStream, "priority"_a = 1, R"doc(
Create a command stream for continuous command sending.

Parameters
----------
priority : int, optional
    Command priority. Default is 1.

Returns
-------
RobotCommandStreamHandler
    Command stream handler.

Examples
--------
>>> # Create command stream for continuous operation
>>> stream_handler = robot.create_command_stream(priority=1)
>>> # Send commands continuously
>>> for i in range(10):
...     command_builder = create_command(i)
...     stream_handler.send_command(command_builder, timeout_ms=1000)
...     time.sleep(1)
>>> # Through the handler, you can terminate control or check the current state
>>> stream_handler.cancel()
>>> # See examples/python/cartesian_command_stream.py for complete usage
)doc")
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
          "control"_a, "port"_a = 0, "priority"_a = 1, py::call_guard<py::gil_scoped_release>(), R"doc(
Start a **blocking** real-time control loop using a custom control callback.

This function runs a UDP-based real-time loop that repeatedly calls the user-provided
``control(ControlState) -> ControlInput``. The loop exits when the callback returns a
``ControlInput`` with ``finish=True``, or when an error/abort occurs. The function
returns ``True`` only if it finished via ``finish=True``; otherwise ``False``.

Parameters
----------
control : callable
    A callable that accepts a ``ControlState`` and returns a ``ControlInput``.
    The callback is invoked for each received RT state packet.
port : int, optional
    UDP port to bind for the RT server. Use ``0`` to let the OS choose an available port
    (recommended). Default is ``0``.
priority : int, optional
    Command priority sent with the RT control request. Default is ``1``.

Returns
-------
bool
    ``True`` if the loop terminated because the callback set ``finish=True`` in
    ``ControlInput``. ``False`` if the loop ended due to an error or abort
    (e.g., UDP bind failure, connection issue).

Notes
-----
- **Blocking:** This call blocks until the control loop ends. There is no handler for
  external cancellation. Design the callback to set ``finish=True`` when you want to stop.
- **GIL behavior:** The function releases the Python GIL around I/O and reacquires it
  only when invoking your Python callback, allowing other Python threads to run between
  callbacks.
- **Performance:** Keep the callback lightweight—avoid long computations or blocking I/O
  inside the callback to maintain RT responsiveness.
- **State fields:** The provided ``ControlState`` includes timestamp ``t``, and arrays 
  such as ``position``, ``velocity``, ``current``, and ``torque`` for all DoFs of the robot.
- **Errors:** If the UDP server fails to bind (e.g., port unavailable), the function
  returns ``False`` immediately (and prints the error to ``stderr``).

Examples
--------
>>> import numpy as np
>>> 
>>> # Stop after ~2 seconds using state.t
>>> t0 = {"t": None}
>>> def control_fn(state):
...     if t0["t"] is None:
...         t0["t"] = state.t
...     finish = (state.t - t0["t"]) >= 2.0
...     i = ControlInput()
...     i.mode.fill(rby.ControlMode.Position)
...     i.target = np.zeros_like(state.position)   # hold home
...     i.feedback_gain.fill(10)                   # simple P gain
...     i.feedforward_torque.fill(0)               # no torque feedforward
...     i.finish = finish
...     return i
>>> ok = robot.control(control_fn)
>>> print(ok)   # True if finished via finish=True
)doc")
      .def("reset_odometry", &Robot<T>::ResetOdometry, "angle"_a, "position"_a, R"doc(
reset_odometry(angle, position)

Reset odometry to specified values.

Parameters
----------
angle : float
    New angle [rad].
position : numpy.ndarray
    New position [m].

Examples
--------
>>> import numpy as np
>>> # Reset odometry to origin
>>> robot.reset_odometry(angle=0.0, position=np.array([0.0, 0.0]))
>>> # Reset to specific position and orientation
>>> robot.reset_odometry(angle=np.pi/2, position=np.array([1.0, 2.0]))
>>> # Useful for mobile base navigation
>>> # Position is [x, y] in meters, angle is yaw in rad
)doc")
      .def("get_parameter_list", &Robot<T>::GetParameterList, R"doc(
Get list of available parameters.

Returns
-------
list
    List of parameter names.

Examples
--------
>>> # Get all available parameters
>>> param_list = robot.get_parameter_list()
>>> print(f"Total parameters: {len(param_list)}")
>>> # List common parameter categories
>>> for param in param_list:
...     if param[0].startswith("default."):
...         print(f"Default param: {param}")
...     elif param[0].startswith("joint_position_command."):
...         print(f"Joint position param: {param}")
>>> # Common categories: default.*, joint_position_command.*, cartesian_command.*
)doc")
      .def("set_parameter", &Robot<T>::SetParameter, "name"_a, "value"_a, "write_db"_a = true, R"doc(
Set a robot parameter.

Parameters
----------
name : str
    Parameter name.
value : any
    Parameter value.
write_db : bool, optional
    Whether to write to database. Default is True.

Examples
--------
>>> # Set acceleration limit scaling
>>> robot.set_parameter("default.acceleration_limit_scaling", "0.8")
>>> # Set joint position command cutoff frequency
>>> robot.set_parameter("joint_position_command.cutoff_frequency", "5")
>>> # Set hotspot SSID name
>>> robot.set_parameter("hotspot.ssid", "\"RBY1_HOTSPOT\"") # For str value, use escaped quotes
>>> # Set without writing to database (Return ``False`` if matching parameter not found)
>>> robot.set_parameter("temp.parameter", "value", write_db=False)
False
)doc")
      .def("get_parameter", &Robot<T>::GetParameter, "name"_a, R"doc(
Retrieve a robot parameter.

Parameters
----------
name : str
    Name of the parameter to retrieve.

Returns
-------
any
    Parameter value.
    Returned as a JSON value: numeric values are returned as-is, strings are enclosed in quotes ("...").

Examples
--------
>>> # Retrieve current parameter values
>>> acc_scaling = robot.get_parameter("default.acceleration_limit_scaling")
>>> cutoff_freq = robot.get_parameter("joint_position_command.cutoff_frequency")
>>> model_name = robot.get_parameter("robot_model_name")
>>> print(f"Acceleration scaling: {acc_scaling}")
>>> print(f"Cutoff frequency: {cutoff_freq}")
>>> print(f"Model name: {model_name}")
)doc")
      .def(
          "reset_parameter_to_default",
          [](Robot<T>& self, const std::string& name) {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "reset_parameter_to_default() is deprecated, use factory_reset_parameter() instead.", 1);
            return self.ResetParameterToDefault(name);
          },
          "name"_a, R"doc(
Reset a parameter to its default value (deprecated).

Parameters
----------
name : str
    Parameter name.

.. deprecated:: 0.6.0
   Use :meth:`factory_reset_parameter` instead.

Note
----
This method is deprecated. Use factory_reset_parameter() instead.
)doc")
      .def(
          "reset_all_parameters_to_default",
          [](Robot<T>& self) {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "reset_all_parameters_to_default() is deprecated, use factory_reset_all_parameters() instead.",
                         1);
            self.ResetAllParametersToDefault();
          },
          R"doc(
Reset all parameters to their default values (deprecated).

.. deprecated:: 0.6.0
   Use :meth:`factory_reset_all_parameters` instead.

Note
----
This method is deprecated. Use factory_reset_all_parameters() instead.
)doc")
      .def("reset_parameter", &Robot<T>::ResetParameter, "name"_a, R"doc(
Reset a parameter to its current value.

Parameters
----------
name : str
    Parameter name.
)doc")
      .def("reset_all_parameters", &Robot<T>::ResetAllParameters, R"doc(
Reset all parameters to their current values.
)doc")
      .def("factory_reset_parameter", &Robot<T>::FactoryResetParameter, "name"_a, R"doc(
Factory reset a parameter to its default value.

Parameters
----------
name : str
    Parameter name.
)doc")
      .def("factory_reset_all_parameters", &Robot<T>::FactoryResetAllParameters, R"doc(
Factory reset all parameters to their default values.
)doc")
      .def("get_robot_model", &Robot<T>::GetRobotModel, R"doc(
Get the current robot URDF model.

Returns
-------
RobotModel
    Current robot model.
)doc")
      .def("import_robot_model", &Robot<T>::ImportRobotModel, "name"_a, "model"_a, R"doc(
Import a robot model.

Parameters
----------
name : str
    Model name.
model : RobotModel
    Robot model to import.
)doc")
      .def("sync_time", &Robot<T>::SyncTime, R"doc(
Synchronizes the timestamp of ``RobotState`` to UPC time instead of UTC. 
Useful when synchronizing multiple devices to UPC time.
)doc")
      .def("has_established_time_sync", &Robot<T>::HasEstablishedTimeSync, R"doc(
Check if time synchronization is established.

Returns
-------
bool
    True if time sync is established, False otherwise.
)doc")
      .def("start_time_sync", &Robot<T>::StartTimeSync, "period_sec"_a, R"doc(
Start time synchronization.

Parameters
----------
period_sec : float
    Synchronization period in seconds.
)doc")
      .def("stop_time_sync", &Robot<T>::StopTimeSync, py::call_guard<py::gil_scoped_release>(), R"doc(
Stop time synchronization.
)doc")
      .def("get_dynamics", &Robot<T>::GetDynamics, "urdf_model"_a = "", R"doc(
Get robot dynamics model.

Parameters
----------
urdf_model : str, optional
    URDF model path. Default is empty string.

Returns
-------
dyn.Robot
    Robot dynamics model.

Examples
--------
>>> # Get dynamics model with the model from connected robot
>>> dynamics_robot = robot.get_dynamics()
>>> # Get dynamics model with custom URDF
>>> dynamics_robot = robot.get_dynamics("/path/to/custom.urdf")
>>> # Useful for advanced control algorithms
)doc")
      .def("set_led_color", &Robot<T>::SetLEDColor, "color"_a, "duration"_a = 1, "transition_time"_a = 0,
           "blinking"_a = false, "blinking_freq"_a = 1, R"doc(
Set LED color.

Parameters
----------
color : Color
    LED color.
duration : float, optional
    Duration in seconds. Default is 1.
transition_time : float, optional
    Transition time in seconds. Default is 0.
blinking : bool, optional
    Whether to blink. Default is False.
blinking_freq : float, optional
    Blinking frequency in Hz. Default is 1.

Examples
--------
>>> from rby1_sdk import Color
>>> # Set LED to red for 5 seconds
>>> robot.set_led_color(Color(255, 0, 0), duration=5.0)
>>> # Set LED to green with 1 second transition
>>> robot.set_led_color(Color(0, 255, 0), transition_time=1.0)
>>> # Set LED to blue with blinking
>>> robot.set_led_color(Color(0, 0, 255), blinking=True, blinking_freq=2.0)
>>> # Set LED to white with custom duration and transition
>>> robot.set_led_color(Color(255, 255, 255), duration=10.0, transition_time=2.0)
)doc")
      .def(
          "get_system_time",
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
          },
          R"doc(
Get robot system time.

Returns
-------
tuple
    (datetime, str): Robot system time and local time string.
)doc")
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
              dt = dt.attr("astimezone")();
            }

            tzinfo = dt.attr("tzinfo");
            if (py::hasattr(tzinfo, "key")) {
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
          "datetime"_a, R"doc(
Set robot system time.

Parameters
----------
datetime : datetime.datetime
    New system time.
)doc")
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
          "datetime"_a, "time_zone"_a, R"doc(
Set robot system time with timezone.

Parameters
----------
datetime : datetime.datetime
    New system time.
time_zone : str
    Timezone string.
)doc")
      .def("set_battery_level", &Robot<T>::SetBatteryLevel, R"doc(
Set battery level.

Parameters
----------
level : float
    Battery level percentage (0.0 to 100.0).

Examples
--------
>>> # Set battery level for testing
>>> robot.set_battery_level(100.0)  # Full battery
>>> robot.set_battery_level(50.0)   # Half battery
>>> robot.set_battery_level(25.0)   # Quarter battery
>>> robot.set_battery_level(0.0)    # Empty battery
>>> # Range: 0.0 to 100.0
>>> # Useful for using custom battery
)doc")
      .def("set_battery_config", &Robot<T>::SetBatteryConfig, "cutoff_voltage"_a, "fully_charged_voltage"_a,
           "coefficients"_a, R"doc(
Set battery configuration.
)doc")
      .def("reset_battery_config", &Robot<T>::ResetBatteryConfig, R"doc(
Reset battery configuration to default.
)doc")
      .def("wait_for_control_ready", &Robot<T>::WaitForControlReady, "timeout_ms"_a, R"doc(
Wait until the robot is ready to accept control commands.

Parameters
----------
timeout_ms : int
    Timeout in milliseconds.

Returns
-------
bool
    ``True`` if the robot is ready to accept control (you can now set control).
    ``False`` if the timeout expired before the robot became ready.

Examples
--------
>>> # Wait for control to be ready
>>> if robot.wait_for_control_ready(timeout_ms=5000):
...     print("Control is ready, you can now set control commands.")
... else:
...     print("Timeout: control is not ready.")
>>> # Call after servo_on() and before sending control commands
)doc")

      .def("reset_network_setting", &Robot<T>::ResetNetworkSetting, py::call_guard<py::gil_scoped_release>(), R"doc(
Reset network settings to default.
)doc")
      .def("scan_wifi", &Robot<T>::ScanWifi, py::call_guard<py::gil_scoped_release>(), R"doc(
Scan for available WiFi networks.

Returns
-------
list
    List of available WiFi networks.

Examples
--------
>>> # Scan for available WiFi networks
>>> networks = robot.scan_wifi()
>>> print(f"Found {len(networks)} networks")
>>> # List network information
>>> for network in networks:
...     print(f"SSID: {network.ssid}, Signal: {network.signal_strength}")
>>> # Connect to a network
>>> if networks:
...     robot.connect_wifi(networks[0].ssid, "password")
)doc")
      .def("connect_wifi", &Robot<T>::ConnectWifi, "ssid"_a, "password"_a = "", "use_dhcp"_a = true,
           "ip_address"_a = "", "gateway"_a = "", "dns"_a = std::vector<std::string>{},
           py::call_guard<py::gil_scoped_release>(), R"doc(
Connect to WiFi network.

Parameters
----------
ssid : str
    WiFi network name.
password : str, optional
    WiFi password. Default is empty string.
use_dhcp : bool, optional
    Use DHCP. Default is True.
ip_address : str, optional
    Static IP address. Default is empty string.
gateway : str, optional
    Gateway address. Default is empty string.
dns : list[str], optional
    DNS servers. Default is empty string.

Examples
--------
>>> # Connect using DHCP (automatic IP)
>>> robot.connect_wifi("MyWiFi", "mypassword123")
>>> # Connect with static IP configuration
>>> robot.connect_wifi("MyWiFi", "mypassword123", use_dhcp=False,
...                    ip_address="192.168.1.100/24", gateway="192.168.1.1")
>>> # Connect to open network
>>> robot.connect_wifi("OpenNetwork")
>>> # Check connection status
>>> status = robot.get_wifi_status()
>>> print(f"Connected: {status.connected}, SSID: {status.ssid}")
)doc")
      .def("disconnect_wifi", &Robot<T>::DisconnectWifi, py::call_guard<py::gil_scoped_release>(), R"doc(
Disconnect from WiFi network.
)doc")
      .def("get_wifi_status", &Robot<T>::GetWifiStatus, py::call_guard<py::gil_scoped_release>(), R"doc(
Get WiFi connection status.

Returns
-------
WifiStatus
    Current WiFi status.
)doc")

      .def("get_serial_device_list", &Robot<T>::GetSerialDeviceList, py::call_guard<py::gil_scoped_release>(), R"doc(
Get list of available serial devices on the robot.

Returns
-------
list
    List of available serial devices.

Examples
--------
>>> # Get available serial devices
>>> devices = robot.get_serial_device_list()
>>> print(f"Found {len(devices)} serial devices")
>>> # List device information
>>> for device in devices:
...     print(f"Path: {device.path}, Description: {device.description}")
>>> # Open serial stream to first device
>>> if devices:
...     stream = robot.open_serial_stream(devices[0].path, 115200)
>>> # Common device paths: /dev/ttyUSB0, /dev/ttyACM0, COM1, COM2
)doc")
      .def("open_serial_stream", &Robot<T>::OpenSerialStream, "device_path"_a, "baudrate"_a, "bytesize"_a = 8,
           "parity"_a = 'N', "stopbits"_a = 1, R"doc(
Open a serial stream.

Parameters
----------
device_path : str
    Path to serial device.
baudrate : int
    Baud rate.
bytesize : int, optional
    Data bits. Default is 8.
parity : str, optional
    Parity setting. Default is 'N'.
stopbits : int, optional
    Stop bits. Default is 1.

Returns
-------
SerialStream
    Serial stream object.

Examples
--------
>>> # Open serial stream with default settings
>>> stream = robot.open_serial_stream("/dev/ttyUSB0", 115200)
>>> # Open with custom settings
>>> stream = robot.open_serial_stream("/dev/ttyUSB1", 9600, bytesize=7, parity='E', stopbits=2)
>>> # Common baudrates: 9600, 19200, 38400, 57600, 115200
>>> # Parity: 'N' (none), 'E' (even), 'O' (odd)
>>> # Stop bits: 1, 2
)doc")
      .def(
          "download_file",
          [](Robot<T>& self, const std::string& path, py::object py_file_like) {
            return self.DownloadFileToCallback(
                path, [&](const char* data, size_t size) { py_file_like.attr("write")(py::bytes(data, size)); });
          },
          "path"_a, "file_like"_a, R"(
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
    ``True`` if the file was downloaded successfully.

Raises
------
RuntimeError
    If the gRPC call fails (e.g., network issue, file not found, internal error).
           )")

      .def("set_position_p_gain", &Robot<T>::SetPositionPGain, "dev_name"_a, "p_gain"_a, R"doc(
Set position P gain for a device.

Parameters
----------
dev_name : str
    Device name.
p_gain : int
    P gain value.

Examples
--------
>>> # Set P gain for specific devices
>>> robot.set_position_p_gain("torso_5", 220)
)doc")
      .def("set_position_i_gain", &Robot<T>::SetPositionIGain, "dev_name"_a, "i_gain"_a, R"doc(
Set position I gain for a device.

Parameters
----------
dev_name : str
    Device name.
i_gain : int
    I gain value.

Examples
--------
>>> # Set I gain for specific devices
>>> robot.set_position_i_gain("torso_5", 40)
)doc")
      .def("set_position_d_gain", &Robot<T>::SetPositionDGain, "dev_name"_a, "d_gain"_a, R"doc(
Set position D gain for a device.

Parameters
----------
dev_name : str
    Device name.
d_gain : int
    D gain value.

Examples
--------
>>> # Set D gain for specific devices
>>> robot.set_position_d_gain("torso_5", 400)
)doc")
      .def("set_position_pid_gain",
           static_cast<bool (Robot<T>::*)(const std::string&, uint16_t, uint16_t, uint16_t) const>(
               &Robot<T>::SetPositionPIDGain),
           "dev_name"_a, "p_gain"_a, "i_gain"_a, "d_gain"_a, R"doc(
Set position PID gains for a device.

Parameters
----------
dev_name : str
    Device name.
p_gain : int
    P gain value.
i_gain : int
    I gain value.
d_gain : int
    D gain value.

Returns
-------
bool
    True if successful, False otherwise.

Examples
--------
>>> # Set all PID gains at once for a device
>>> robot.set_position_pid_gain("right_arm_0", 80, 15, 200)
>>> robot.set_position_pid_gain("right_arm_1", 80, 15, 200)
>>> robot.set_position_pid_gain("right_arm_2", 80, 15, 200)
>>> robot.set_position_pid_gain("right_arm_3", 35, 5, 80)
>>> robot.set_position_pid_gain("right_arm_4", 30, 5, 70)
>>> robot.set_position_pid_gain("right_arm_5", 30, 5, 70)
>>> robot.set_position_pid_gain("right_arm_6", 100, 5, 120)
)doc")
      .def("set_position_pid_gain",
           static_cast<bool (Robot<T>::*)(const std::string&, const rb::PIDGain&) const>(&Robot<T>::SetPositionPIDGain),
           "dev_name"_a, "pid_gain"_a, R"doc(
Set position PID gains for a device using PIDGain object.

Parameters
----------
dev_name : str
    Device name.
pid_gain : PIDGain
    PID gain object.

Returns
-------
bool
    True if successful, False otherwise.
)doc")

      .def("get_torso_position_pid_gains", &Robot<T>::GetTorsoPositionPIDGains, R"doc(
Get torso position PID gains.

Returns
-------
PIDGain
    Torso position PID gains.

Examples
--------
>>> # Get current PID gains for torso
>>> torso_pid = robot.get_torso_position_pid_gains()[0]  # Get first (and only) torso device
>>> print(f"Torso P: {torso_pid.p_gain}, I: {torso_pid.i_gain}, D: {torso_pid.d_gain}")
>>> right_arm_pid = robot.get_right_arm_position_pid_gains()[0]
>>> print(f"Right arm: {right_arm_pid}")
)doc")
      .def("get_right_arm_position_pid_gains", &Robot<T>::GetRightArmPositionPIDGains, R"doc(
Get right arm position PID gains.

Returns
-------
PIDGain
    Right arm position PID gains.
)doc")
      .def("get_left_arm_position_pid_gains", &Robot<T>::GetLeftArmPositionPIDGains, R"doc(
Get left arm position PID gains.

Returns
-------
PIDGain
    Left arm position PID gains.
)doc")
      .def("get_head_position_pid_gains", &Robot<T>::GetHeadPositionPIDGains, R"doc(
Get head position PID gains.

Returns
-------
PIDGain
    Head position PID gains.
)doc")
      .def("get_position_pid_gain", &Robot<T>::GetPositionPIDGain, "dev_name"_a, R"doc(
Get position PID gains for a device.

Parameters
----------
dev_name : str
    Device name.

Returns
-------
PIDGain
    Position PID gains for the device.

Examples
--------
>>> # Get PID gains for specific device
>>> torso_pid = robot.get_position_pid_gain("torso_5")
>>> print(f"Torso 5 P: {torso_pid.p_gain}, I: {torso_pid.i_gain}, D: {torso_pid.d_gain}")
>>> # Get PID gains for arm
>>> arm_pid = robot.get_position_pid_gain("right_arm_0")
>>> print(f"Right arm 0 P: {arm_pid.p_gain}")
>>> devices = ["torso_5", "right_arm_0", "left_arm_1", "head_0"]
>>> for device in devices:
...     pid = robot.get_position_pid_gain(device)
...     print(f"{device}: P={pid.p_gain}, I={pid.i_gain}, D={pid.d_gain}")
)doc")
      .def("__repr__",
           [robot_name](const Robot<T>& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             PyModel<T> model;
             std::ostringstream out;
             out << robot_name << "(" << FIRST                                                      //
                 << "connected=" << (self.IsConnected() ? "True" : "False") << SEP                  //
                 << "time_scale=" << format_number(self.GetTimeScale(), Style::Repr) << SEP         //
                 << "model_name=" << inline_obj_one_line(py::cast(model.get_model_name())) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [robot_name](const Robot<T>& self) {
        PyModel<T> model;
        std::ostringstream out;
        out << robot_name << "(connected=" << (self.IsConnected() ? "True" : "False")
            << ", time_scale=" << rb::print::format_number(self.GetTimeScale(), rb::print::Style::Str)
            << ", model=" << model.get_model_name() << ")";
        return out.str();
      });
}

void pybind11_robot(py::module_& m) {
  bind_pid_gain(m);
  bind_color(m);
  bind_serial(m);
  bind_robot<y1_model::A>(m);
  bind_robot<y1_model::T5>(m);
  bind_robot<y1_model::M>(m);
  bind_robot<y1_model::UB>(m);
}