#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <sstream>

#include "common.h"
#include "print_helper.h"
#include "rby1-sdk/log.h"

namespace py = pybind11;
using namespace rb;
using rb::print::indent_continuation;
using rb::print::inline_obj;
using rb::print::np_array_to_string;
using rb::print::np_shape_dtype;
using rb::print::Style;

void bind_log(py::module_& m) {
  auto log = py::class_<Log>(m, "Log", R"doc(
Robot log entry with timestamp and message.

This class represents a single log entry from the robot system,
including timestamps, log level, and message content.

Attributes
----------
timestamp : datetime.datetime
    Timestamp when the log was created (client system time).
robot_system_timestamp : datetime.datetime
    Timestamp when the log was created (robot system time).
level : Level
    Log level indicating the severity of the message.
message : str
    The log message content.
)doc");

  py::enum_<Log::Level>(log, "Level", R"doc(
Log level enumeration.

Defines the severity levels for log messages, from least to most severe.

Members
-------
Trace : int
    Trace level for detailed debugging information.
Debug : int
    Debug level for debugging information.
Info : int
    Info level for general information messages.
Warn : int
    Warning level for warning messages.
Error : int
    Error level for error messages.
Critical : int
    Critical level for critical error messages.
)doc")
      .value("Trace", Log::Level::kTrace, R"doc(
Trace level for detailed debugging information.
)doc")
      .value("Debug", Log::Level::kDebug, R"doc(
Debug level for debugging information.
)doc")
      .value("Info", Log::Level::kInfo, R"doc(
Info level for general information messages.
)doc")
      .value("Warn", Log::Level::kWarn, R"doc(
Warning level for warning messages.
)doc")
      .value("Error", Log::Level::kError, R"doc(
Error level for error messages.
)doc")
      .value("Critical", Log::Level::kCritical, R"doc(
Critical level for critical error messages.
)doc");

  log.def(py::init<>(), R"doc(
Construct a Log instance with default values.
)doc")
      .def_property_readonly(
          "timestamp", [](const Log& self) { return timespec_to_time_point(self.timestamp); }, R"doc(
Timestamp when the log was created (client system time).

Type
----
datetime.datetime
    Client system timestamp when the log entry was created.
)doc")
      .def_property_readonly(
          "robot_system_timestamp", [](const Log& self) { return timespec_to_time_point(self.robot_system_timestamp); },
          R"doc(
Timestamp when the log was created (robot system time).

Type
----
datetime.datetime
    Robot system timestamp when the log entry was created.
)doc")
      .def_readonly("level", &Log::level, R"doc(
Log level indicating the severity of the message.

Type
----
Level
    Enumeration value indicating the log severity level.
)doc")
      .def_readonly("message", &Log::message, R"doc(
The log message content.

Type
----
str
    Human-readable log message describing the event or condition.
)doc")
      .def("__repr__",
           [](const Log& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             auto ts_client = timespec_to_time_point(self.timestamp);
             auto ts_robot = timespec_to_time_point(self.robot_system_timestamp);

             ReprStream ss;

             const auto& print_array = [&](const std::string& name, const auto& array) {
               std::string k = name + "=";
               std::string v = np_array_to_string(py::cast(array), Style::Repr);
               ss << k << indent_continuation(v, (int)k.size());
             };

             ss << "Log(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "timestamp=" << inline_obj(py::cast(ts_client)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "robot_system_timestamp=" << inline_obj(py::cast(ts_robot)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "level=" << to_string(self.level) << ",";
             ss << (ml ? "\n" : " ");

             ss << "message=" << inline_obj(py::cast(self.message));
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const Log& self) {
        auto ts_client = timespec_to_time_point(self.timestamp);
        std::ostringstream out;
        out << "[" << to_string(self.level) << "] " << py::str(py::cast(ts_client)).cast<std::string>() << " - "
            << self.message;
        return out.str();
      });
}

void pybind11_log(py::module_& m) {
  bind_log(m);
}