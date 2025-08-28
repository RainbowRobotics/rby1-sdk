#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

#include "print_helper.h"
#include "rby1-sdk/net/types.h"

namespace py = pybind11;
using namespace rb;
using rb::print::indent_continuation;
using rb::print::inline_obj;
using rb::print::np_array_to_string;
using rb::print::np_shape_dtype;
using rb::print::Style;

void bind_net_types(py::module_& m) {
  py::class_<WifiNetwork>(m, "WifiNetwork", R"doc(
WiFi network information.

This class represents information about a detected WiFi network
including its SSID, signal strength, and security status.

Attributes
----------
ssid : str
    Service Set Identifier (network name).
signal_strength : int
    Signal strength in dBm (decibels relative to milliwatts).
    Higher values indicate stronger signals.
secured : bool
    Whether the network requires a password for connection.
)doc")
      .def(py::init<>(), R"doc(
  Construct a WifiNetwork instance with default values.
      )doc")
      .def_readonly("ssid", &WifiNetwork::ssid, R"doc(
Service Set Identifier (network name).

Type
----
str
    The name of the WiFi network.
)doc")
      .def_readonly("signal_strength", &WifiNetwork::signal_strength, R"doc(
Signal strength in dBm.

Type
----
int
    Signal strength in decibels relative to milliwatts.
    Typical range: -100 (very weak) to -30 (very strong).
      )doc")
      .def_readonly("secured", &WifiNetwork::secured, R"doc(
Network security status.

Type
----
bool
    True if the network requires a password, False otherwise.
)doc")
      .def("__repr__",
           [](const WifiNetwork& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "WifiNetwork(" << FIRST                                  //
                 << "ssid=" << inline_obj(py::cast(self.ssid)) << SEP        //
                 << "signal_strength=" << self.signal_strength << SEP        //
                 << "secured=" << (self.secured ? "True" : "False") << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const WifiNetwork& self) {
        std::ostringstream out;
        out << "WifiNetwork(ssid=" << self.ssid << ", signal=" << self.signal_strength << "dBm"
            << ", secured=" << (self.secured ? "True" : "False") << ")";
        return out.str();
      });

  py::class_<WifiStatus>(m, "WifiStatus", R"doc(
Current WiFi connection status.

This class represents the current status of a WiFi connection
including connection details and network configuration.

Attributes
----------
ssid : str
    Service Set Identifier of the connected network.
ip_address : str
    IP address assigned to the device.
gateway : str
    Gateway IP address for the network.
dns : list[str]
    List of DNS server IP addresses.
connected : bool
    Whether the device is currently connected to WiFi.
)doc")
      .def(py::init<>(), R"doc(
Construct a WifiStatus instance with default values.
)doc")
      .def_readonly("ssid", &WifiStatus::ssid, R"doc(
Service Set Identifier of the connected network.

Type
----
str
    The name of the currently connected WiFi network.
)doc")
      .def_readonly("ip_address", &WifiStatus::ip_address, R"doc(
IP address assigned to the device.

Type
----
str
    IP address in dotted decimal format (e.g., "192.168.1.100").
)doc")
      .def_readonly("gateway", &WifiStatus::gateway, R"doc(
Gateway IP address for the network.

Type
----
str
    Gateway IP address in dotted decimal format.
)doc")
      .def_readonly("dns", &WifiStatus::dns, R"doc(
List of DNS server IP addresses.

Type
----
list[str]
    List of DNS server IP addresses in dotted decimal format.
)doc")
      .def_readonly("connected", &WifiStatus::connected, R"doc(
WiFi connection status.

Type
----
bool
    True if connected to WiFi, False otherwise.
)doc")
      .def("__repr__",
           [](const WifiStatus& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "WifiStatus(" << FIRST                                         //
                 << "ssid=" << inline_obj(py::cast(self.ssid)) << SEP              //
                 << "ip_address=" << inline_obj(py::cast(self.ip_address)) << SEP  //
                 << "gateway=" << inline_obj(py::cast(self.gateway)) << SEP        //
                 << "dns=" << inline_obj(py::cast(self.dns)) << SEP                //
                 << "connected=" << (self.connected ? "True" : "False") << LAST    //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const WifiStatus& self) {
        std::ostringstream out;
        out << "WifiStatus(ssid=" << self.ssid << ", ip=" << self.ip_address << ", gw=" << self.gateway
            << ", connected=" << (self.connected ? "True" : "False") << ")";
        return out.str();
      });
}

void pybind11_net(py::module_& m) {
  bind_net_types(m);
}