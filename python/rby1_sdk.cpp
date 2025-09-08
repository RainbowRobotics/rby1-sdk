#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <string>

#include "rby1-sdk/math/liegroup.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

#include "model.h"
#include "print_helper.h"

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
    return py::cast(Robot<y1_model::A>::Create(address));
  } else if (is_equal(model_name, y1_model::T5::kModelName.data())) {
    return py::cast(Robot<y1_model::T5>::Create(address));
  } else if (is_equal(model_name, y1_model::M::kModelName.data())) {
    return py::cast(Robot<y1_model::M>::Create(address));
  } else if (is_equal(model_name, y1_model::UB::kModelName.data())) {
    return py::cast(Robot<y1_model::UB>::Create(address));
  }

  throw std::runtime_error("Unknown model name: " + std::string(model_name));
}

void pybind11_print_options(py::module_& m) {
  m.def(
      "set_printoptions",
      [](std::optional<std::string> array_mode, std::optional<int> linewidth, std::optional<int> precision,
         std::optional<int> edgeitems, std::optional<int> threshold, std::optional<bool> suppress_small,
         std::optional<bool> multiline_repr, std::optional<std::string> float_style, std::optional<std::string> sign,
         std::optional<bool> trim_trailing_zeros, std::optional<std::string> scope) {
        print::set_printoptions(array_mode, linewidth, precision, edgeitems, threshold, suppress_small, multiline_repr,
                                float_style, sign, trim_trailing_zeros, scope);
      },
      R"pbdoc(
Set global or thread-local rby1_sdk print options.

Controls how SDK objects and NumPy arrays are rendered by `__repr__`/`__str__`.
`array_mode='numpy'` delegates array printing entirely to NumPy's global
printoptions. `preview`/`full` make the SDK pass its options explicitly.

`multiline_repr` toggles **`__repr__` only**; by convention `__str__` is kept one line.

Parameters
----------
array_mode : {'numpy', 'preview', 'full'}, optional
    - 'numpy'  : Fully delegate arrays to NumPy's global printoptions.
    - 'preview': Use SDK preview policy (threshold/edgeitems/linewidth/precision/sign/…).
    - 'full'   : Print full arrays (ignore threshold), other options still apply.
linewidth : int, optional
    Target maximum line width for arrays (passed to NumPy when not 'numpy').
precision : int, optional
    Decimal precision for scalars and arrays.
edgeitems : int, optional
    Number of leading/trailing items to show when summarizing arrays.
threshold : int, optional
    Total number of array elements that triggers summarization (ellipsis).
suppress_small : bool, optional
    If True, very small floats are printed as 0 in arrays (when not 'numpy').
multiline_repr : bool, optional
    If True, top-level `__repr__` of complex SDK objects uses multi-line, field-per-line
    formatting. Nested objects should be rendered one-line where reasonable.
float_style : {'auto', 'default', 'fixed', 'scientific'}, optional
    Floating formatting style for scalars and (when not 'numpy') arrays.
sign : {'auto', 'minus', 'plus', 'space'}, optional
    Sign policy. 'space' shows a leading space for positive numbers.
trim_trailing_zeros : bool, optional
    If True, trailing zeros after the decimal point are trimmed (1.200 -> '1.2').
scope : {'thread', 'global'}, optional
    - 'thread' (default): apply to the current Python thread (thread-local stack top).
    - 'global'          : update process-wide defaults.

Notes
-----
- Options are maintained on a thread-local stack. Repeated calls update the top-of-stack
  for the current thread; using scope='global' updates process-wide defaults.
- For temporary changes, prefer the context manager `printoptions(...)` below.

Examples
--------
>>> import rby1_sdk as rby
>>> rby.set_printoptions(array_mode='numpy', precision=4)  # arrays follow NumPy global config; scalars use precision=4
>>> with rby.printoptions(array_mode='preview', precision=2, multiline_repr=True):
...     print(obj)        # one-line **str** with precision=2
...     print(repr(obj))  # multi-line **repr** (since multiline_repr=True)
      )pbdoc",
      "array_mode"_a = py::none(), "linewidth"_a = py::none(), "precision"_a = py::none(), "edgeitems"_a = py::none(),
      "threshold"_a = py::none(), "suppress_small"_a = py::none(), "multiline_repr"_a = py::none(),
      "float_style"_a = py::none(), "sign"_a = py::none(), "trim_trailing_zeros"_a = py::none(), "scope"_a = "thread");

  py::class_<print::PrintConfigGuard>(m, "printoptions",
                                      R"pbdoc(
Context manager for temporary print options.

Creates a thread-local scoped override of printing behavior for SDK objects and
NumPy arrays. Upon exit, previous options are restored.

`array_mode='numpy'` delegates arrays to NumPy's global printoptions; scalar formatting
and `multiline_repr` still apply as configured.

Examples
--------
>>> import rby1_sdk as sdk
>>> with sdk.printoptions(array_mode='preview', precision=3, multiline_repr=False):
...     print(obj)        # one-line **str**, precision=3
...     print(repr(obj))  # one-line **repr** (multiline_repr=False)
>>> with sdk.printoptions(array_mode='full', multiline_repr=True):
...     print(repr(obj))  # multi-line repr with full arrays
      )pbdoc")
      .def(py::init<std::optional<std::string>, std::optional<int>, std::optional<int>, std::optional<int>,
                    std::optional<int>, std::optional<bool>, std::optional<bool>,
                    std::optional<std::string>,  // float_style
                    std::optional<std::string>,  // sign
                    std::optional<bool>          // trim_trailing_zeros
                    >(),
           R"pbdoc(
__init__(array_mode=None, linewidth=None, precision=None, edgeitems=None, threshold=None,
         suppress_small=None, multiline_repr=None, float_style=None, sign=None,
         trim_trailing_zeros=None)

All parameters are optional; only those provided are overridden in this context.
           )pbdoc",
           "array_mode"_a = py::none(), "linewidth"_a = py::none(), "precision"_a = py::none(),
           "edgeitems"_a = py::none(), "threshold"_a = py::none(), "suppress_small"_a = py::none(),
           "multiline_repr"_a = py::none(), "float_style"_a = py::none(), "sign"_a = py::none(),
           "trim_trailing_zeros"_a = py::none())
      .def(
          "__enter__",
          [](print::PrintConfigGuard& g) {
            g.enter();
            return &g;
          },
          R"pbdoc(Return self and activate the context.)pbdoc")
      .def(
          "__exit__",
          [](print::PrintConfigGuard& g, py::object, py::object, py::object) {
            g.exit();
            return false;
          },
          R"pbdoc(Restore previous options and propagate exceptions.)pbdoc");
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

  pybind11_print_options(m);

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
Create a ``Robot_A`` instance.

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
Create a ``Robot_T5`` instance.

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
Create a ``Robot_M`` instance.

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
Create a ``Robot_UB`` instance.

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