#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "rby1-sdk/net/types.h"

namespace py = pybind11;
using namespace rb;

void bind_types(py::module_& m) {

  py::class_<WifiNetwork>(m, "WifiNetwork")
      .def(py::init<>())
      .def_readonly("ssid", &WifiNetwork::ssid)
      .def_readonly("signal_strength", &WifiNetwork::signal_strength)
      .def_readonly("secured", &WifiNetwork::secured)
      .def("__repr__", [](const WifiNetwork& self) {
        std::stringstream ss;
        ss << "WifiNetwork(ssid='" << self.ssid << "', signal_strength=" << self.signal_strength
           << ", secured=" << (self.secured ? "True" : "False") << ")";
        return ss.str();
      });

  py::class_<WifiStatus>(m, "WifiStatus")
      .def(py::init<>())
      .def_readonly("ssid", &WifiStatus::ssid)
      .def_readonly("ip_address", &WifiStatus::ip_address)
      .def_readonly("gateway", &WifiStatus::gateway)
      .def_readonly("dns", &WifiStatus::dns)
      .def_readonly("connected", &WifiStatus::connected)
      .def("__repr__", [](const WifiStatus& self) {
        const auto& vector_to_string = [](const std::vector<std::string>& vec) {
          std::stringstream ss;
          ss << "[";
          for (size_t i = 0; i < vec.size(); ++i) {
            if (i > 0)
              ss << ", ";
            ss << "'" << vec[i] << "'";
          }
          ss << "]";
          return ss.str();
        };

        std::stringstream ss;
        ss << "WifiStatus(ssid='" << self.ssid << "', ip_address='" << self.ip_address << "', gateway='" << self.gateway
           << "', dns='" << vector_to_string(self.dns) << "', connected=" << (self.connected ? "True" : "False") << ")";
        return ss.str();
      });
}

void pybind11_net(py::module_& m) {
  bind_types(m);
}