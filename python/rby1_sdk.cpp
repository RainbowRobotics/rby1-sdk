#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <string>

#include "rby1-sdk/math/liegroup.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

#include "model.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void pybind11_math(py::module_& m);
void pybind11_net(py::module_& m);
void pybind11_dynamics(py::module_& m);
void pybind11_dynamixel_bus(py::module_& m);
void pybind11_upc(py::module_& m);
void pybind11_robot_state(py::module_& m);
void pybind11_model(py::module_& m);
void pybind11_robot_command_builder(py::module_& m);
void pybind11_robot_info(py::module_& m);
void pybind11_log(py::module_& m);
void pybind11_robot_command_feedback(py::module_& m);
void pybind11_robot(py::module_& m);
void pybind11_control_manager_state(py::module_& m);

py::object create_robot(const std::string& address, const py::object& model) {
  const auto& is_equal = [](const std::string& str1, const std::string& str2) {
    if (str1.length() == str2.length()) {
      return std::equal(str1.begin(), str1.end(), str2.begin(),
                        [](char c1, char c2) { return std::tolower(c1) == std::tolower(c2); });
    }
    return false;
  };

  std::string model_name = "";

  if (py::isinstance<py::str>(model)) {
    model_name = model.cast<std::string>();
  } else {
    model_name = model.attr("model_name").cast<std::string>();
  }

  if (is_equal(model_name, y1_model::A::kModelName.data())) {
    return py::cast(rb::Robot<y1_model::A>::Create(address));
  } else if (is_equal(model_name, y1_model::T5::kModelName.data())) {
    return py::cast(rb::Robot<y1_model::T5>::Create(address));
  } else if (is_equal(model_name, y1_model::M::kModelName.data())) {
    return py::cast(rb::Robot<y1_model::M>::Create(address));
  } else if (is_equal(model_name, y1_model::UB::kModelName.data())) {
    return py::cast(rb::Robot<y1_model::UB>::Create(address));
  }

  throw std::runtime_error("Unknown model name: " + std::string(model_name));
}

PYBIND11_MODULE(_bindings, m) {
  m.doc() = R"doc(
RBY1-SDK Python bindings.

This module provides high-level APIs for RB-Y1 communication, control, modeling, etc.
It includes mathematical operations, dynamics calculations, robot control,
and various utility functions for working with RB-Y1 robots.

Submodules
----------
math : Mathematical operations and Lie group utilities
dynamics : Robot dynamics calculations
upc : Utilities for controlling and communicating with devices on the user PC
)doc";
  m.attr("__version__") = RBY1_SDK_VERSION;

  pybind11_net(m);

  auto math_m = m.def_submodule("math", R"doc(
Math module for RB-Y1.

Provides mathematical operations including Lie group operations,
transformations, and other mathematical utilities.
)doc");
  pybind11_math(math_m);

  auto dyn_m = m.def_submodule("dynamics", R"doc(
Robot dynamics and kinematics module.

Provides robot dynamics calculations including inertia,
joint dynamics, and link properties.
)doc");
  pybind11_dynamics(dyn_m);

  pybind11_dynamixel_bus(m);

  auto upc_m = m.def_submodule("upc", R"doc(
Module for controlling and communicating with devices.

Provides utilities for controlling master arm and other devices.
)doc");
  pybind11_upc(upc_m);

  pybind11_robot_state(m);
  pybind11_control_manager_state(m);
  pybind11_model(m);
  pybind11_robot_command_builder(m);
  pybind11_robot_info(m);
  pybind11_log(m);
  pybind11_robot_command_feedback(m);
  pybind11_robot(m);

  m.def("_create_robot", &create_robot, "address"_a, "model"_a = "a", R"doc(
_create_robot(address, model='a')

Create a robot instance based on the provided model and address.

Parameters
----------
address : str
    Network address of the robot, e.g. "192.168.1.100:50051".
model : str or Model, default='a'
    Robot model specification. Can be a string identifier or model object.

Returns
-------
Robot
    Configured robot instance for the specified model.

Raises
------
RuntimeError
    If the model name is unknown or invalid.

See Also
--------
create_robot_a : Create a Robot A model instance.
create_robot_t5 : Create a Robot T5 model instance.
create_robot_m : Create a Robot M model instance.
create_robot_ub : Create a Robot UB model instance.

Examples
--------
>>> import rby1_sdk as rby
>>> robot = rby._create_robot("192.168.1.100:50051", "a")

>>> robot = rby._create_robot("192.168.1.100:50051", model_a)
)doc");

  m.def(
      "create_robot_a", [](const std::string& address) { return Robot<y1_model::A>::Create(address); }, "address"_a,
      R"doc(
create_robot_a(address)

Create a Robot A model instance.

Parameters
----------
address : str
    Network address of the robot, e.g. "192.168.1.100:50051".

Returns
-------
Robot
    Configured Robot A instance.

Examples
--------
>>> from rby1_sdk import create_robot_a
>>> robot = create_robot_a("192.168.1.100:50051")
)doc");
  m.def(
      "create_robot_t5", [](const std::string& address) { return Robot<y1_model::T5>::Create(address); }, "address"_a,
      R"doc(
create_robot_t5(address)

Create a Robot T5 model instance.

Parameters
----------
address : str
    Network address of the robot, e.g. "192.168.1.100:50051".

Returns
-------
Robot
    Configured Robot T5 instance.

Examples
--------
>>> from rby1_sdk import create_robot_t5
>>> robot = create_robot_t5("192.168.1.100:50051")
)doc");
  m.def(
      "create_robot_m", [](const std::string& address) { return Robot<y1_model::M>::Create(address); }, "address"_a,
      R"doc(
create_robot_m(address)

Create a Robot M model instance.

Parameters
----------
address : str
    Network address of the robot, e.g. "192.168.1.100:50051".

Returns
-------
Robot
    Configured Robot M instance.

Examples
--------
>>> from rby1_sdk import create_robot_m
>>> robot = create_robot_m("192.168.1.100:50051")
)doc");
  m.def(
      "create_robot_ub", [](const std::string& address) { return Robot<y1_model::UB>::Create(address); }, "address"_a,
      R"doc(
create_robot_ub(address)

Create a Robot UB model instance.

Parameters
----------
address : str
    Network address of the robot, e.g. "192.168.1.100:50051".

Returns
-------
Robot
    Configured Robot UB instance.

Examples
--------
>>> from rby1_sdk import create_robot_ub
>>> robot = create_robot_ub("192.168.1.100:50051")
)doc");
}