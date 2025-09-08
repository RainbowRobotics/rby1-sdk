//
// Created by keunjun on 24. 8. 25.
//

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <chrono>
#include <ctime>

#include <iostream>

namespace py = pybind11;

std::chrono::system_clock::time_point timespec_to_time_point(const timespec& ts) {
  auto t = std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(
      std::chrono::nanoseconds(((uint64_t)ts.tv_sec * 1000000000) + (uint64_t)ts.tv_nsec));
  return std::chrono::system_clock::time_point(t);
}

std::chrono::nanoseconds timespec_to_nanoseconds(const timespec& ts) {
  return std::chrono::nanoseconds((uint64_t)(std::chrono::seconds(ts.tv_sec).count() * 1e9) +
                                  std::chrono::nanoseconds(ts.tv_nsec).count());
}

void pybind11_timespec(py::module_& m) {
  py::class_<timespec>(m, "timespec", R"doc(
POSIX timespec structure for high-resolution time representation.

This class represents time with nanosecond precision, commonly used
in system calls and high-resolution timing operations.

Attributes
----------
tv_sec : int
    Seconds since the Epoch (January 1, 1970, 00:00:00 UTC).
tv_nsec : int
    Nanoseconds component (0-999,999,999).
)doc")
      .def(py::init<>(), R"doc(
Construct a timespec with zero values.
)doc")
      .def_property(
          "tv_sec", [](struct timespec& ts) { return ts.tv_sec; },
          [](struct timespec& ts, int sec) { ts.tv_sec = sec; })
      .def_property(
          "tv_nsec", [](struct timespec& ts) { return ts.tv_nsec; },
          [](struct timespec& ts, int nsec) { ts.tv_nsec = nsec; });
}