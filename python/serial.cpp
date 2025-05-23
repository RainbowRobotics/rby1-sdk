#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "rby1-sdk/serial/types.h"

namespace py = pybind11;
using namespace rb;

void bind_serial_types(py::module_& m) {
  py::class_<SerialDevice>(m, "SerialDevice")
      .def(py::init<>())
      .def_readonly("path", &SerialDevice::path)
      .def_readonly("description", &SerialDevice::description)
      .def("__repr__", [](const SerialDevice& self) {
        std::stringstream ss;
        ss << "SerialDevice(path='" << self.path << "', description='" << self.description << "')";
        return ss.str();
      });
}

void pybind11_serial(py::module_& m) {
  bind_serial_types(m);
}