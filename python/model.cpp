#include "model.h"

template <typename T>
void bind_model(py::module_& m, const std::string& model_name) {
  py::class_<PyModel<T>>(m, model_name.c_str(), R"doc(
Robot model configuration class.

This class provides access to robot model specifications including
joint configurations, degrees of freedom, and component indices.

Attributes
----------
model_name : str
    Name identifier for the robot model.
robot_dof : int
    Total degrees of freedom for the robot.
robot_joint_names : list[str]
    Names of all robot joints in order of their indices.
mobility_idx : list[int]
    Indices of mobility components (e.g., wheels).
body_idx : list[int]
    Indices of main body components (excluding mobility and head).
head_idx : list[int]
    Indices of head components.
right_arm_idx : list[int]
    Indices of right arm components.
left_arm_idx : list[int]
    Indices of left arm components.
torso_idx : list[int]
    Indices of torso components.
velocity_estimation_required_idx : list[int]
    Indices of joints requiring velocity estimation.
control_period : float
    Control update period in seconds.
)doc")
      .def(py::init<>(), R"doc(
Construct a model instance.
)doc")
      .def_property_readonly("model_name", &PyModel<T>::get_model_name, R"doc(
Get the model name identifier.

Returns
-------
str
    Model name (e.g., "A", "T5", "M", "UB").
)doc")
      .def_property_readonly("robot_dof", &PyModel<T>::get_robot_dof, R"doc(
Get the total degrees of freedom.

Returns
-------
int
    Total number of joints in the robot.
)doc")
      .def_property_readonly("robot_joint_names", &PyModel<T>::get_robot_joint_names, R"doc(
Get the names of all robot joints.

Returns
-------
list[str]
    List of joint names in order of their indices.
)doc")
      .def_property_readonly("mobility_idx", &PyModel<T>::get_mobility_idx, R"doc(
Get indices of mobility components.

Returns
-------
list[int]
    Indices of wheels or other mobility joints.
)doc")
      .def_property_readonly("body_idx", &PyModel<T>::get_body_idx, R"doc(
Get indices of body components.

Returns
-------
list[int]
    Indices of main body joints (excluding mobility and head).
)doc")
      .def_property_readonly("head_idx", &PyModel<T>::get_head_idx, R"doc(
Get indices of head components.

Returns
-------
list[int]
    Indices of head joints.
)doc")
      .def_property_readonly("right_arm_idx", &PyModel<T>::get_right_arm_idx, R"doc(
Get indices of right arm components.

Returns
-------
list[int]
    Indices of right arm joints.
)doc")
      .def_property_readonly("left_arm_idx", &PyModel<T>::get_left_arm_idx, R"doc(
Get indices of left arm components.

Returns
-------
list[int]
    Indices of left arm joints.
)doc")
      .def_property_readonly("torso_idx", &PyModel<T>::get_torso_idx, R"doc(
Get indices of torso components.

Returns
-------
list[int]
    Indices of torso joints.
)doc")
      .def_property_readonly("velocity_estimation_required_idx", &PyModel<T>::get_velocity_estimation_required_idx, R"doc(
Get indices of joints requiring velocity estimation.

Returns
-------
list[int]
    Indices of joints where velocity estimation is essential.
)doc")
      .def_property_readonly("control_period", &PyModel<T>::get_control_period, R"doc(
Get the control update period.

Returns
-------
float
    Control period in seconds (typically 0.002).
)doc");
}

void pybind11_model(py::module_& m) {
  bind_model<y1_model::A>(m, "Model_A");
  bind_model<y1_model::T5>(m, "Model_T5");
  bind_model<y1_model::M>(m, "Model_M");
  bind_model<y1_model::UB>(m, "Model_UB");
}