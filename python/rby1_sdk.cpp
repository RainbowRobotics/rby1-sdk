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

void pybind11_dynamics(py::module_& m);

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
  }

  throw std::runtime_error("Unknown model name: " + std::string(model_name));
}

PYBIND11_MODULE(_bindings, m) {
  m.doc() = "RBY1-SDK";  // optional module docstring

  auto dyn_m = m.def_submodule("dynamics", "Dynamics module for rby1");
  pybind11_dynamics(dyn_m);

  pybind11_robot_state(m);
  pybind11_control_manager_state(m);
  pybind11_model(m);
  pybind11_robot_command_builder(m);
  pybind11_robot_info(m);
  pybind11_log(m);
  pybind11_robot_command_feedback(m);
  pybind11_robot(m);

  m.def("create_robot", &create_robot, "Creates a robot based on the provided model and address.", py::arg("address"),
        py::arg("model") = "a");

  m.def(
      "create_robot_a",  //
      [](const std::string& address) { return Robot<y1_model::A>::Create(address); }, "address"_a);
  m.def(
      "create_robot_t5",  //
      [](const std::string& address) { return Robot<y1_model::T5>::Create(address); }, "address"_a);
  m.def(
      "create_robot_m",  //
      [](const std::string& address) { return Robot<y1_model::M>::Create(address); }, "address"_a);
}