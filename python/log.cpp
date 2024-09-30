#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <sstream>

#include "common.h"
#include "rby1-sdk/log.h"

namespace py = pybind11;
using namespace rb;

void bind_log(py::module_& m) {
  auto log = py::class_<Log>(m, "Log");

  py::enum_<Log::Level>(log, "Level")
      .value("Trace", Log::Level::kTrace)
      .value("Debug", Log::Level::kDebug)
      .value("Info", Log::Level::kInfo)
      .value("Warn", Log::Level::kWarn)
      .value("Error", Log::Level::kError)
      .value("Critical", Log::Level::kCritical);

  log.def(py::init<>())
      .def_property_readonly("timestamp", [](const Log& self) { return timespec_to_time_point(self.timestamp); })
      .def_property_readonly("robot_system_timestamp",
                             [](const Log& self) { return timespec_to_time_point(self.robot_system_timestamp); })
      .def_readonly("level", &Log::level)
      .def_readonly("message", &Log::message)
      .def("__repr__", [](const Log& self) {
        auto timestamp = timespec_to_time_point(self.timestamp);
        auto robot_system_timestamp = timespec_to_time_point(self.robot_system_timestamp);
        std::stringstream ss;
        ss << "Log("                                                                                               //
           << "timestamp=" << static_cast<std::string>(py::repr(py::cast(timestamp)))                              //
           << ", robot_system_timestamp=" << static_cast<std::string>(py::repr(py::cast(robot_system_timestamp)))  //
           << ", level=" << to_string(self.level)                                                                  //
           << ", message='" << self.message << "'"                                                                 //
           << ")";
        return ss.str();
      });
}

void pybind11_log(py::module_& m) {
  bind_log(m);
}