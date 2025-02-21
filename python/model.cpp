#include "model.h"

template <typename T>
void bind_model(py::module_& m, const std::string& model_name) {
  py::class_<PyModel<T>>(m, model_name.c_str())
      .def(py::init<>())
      .def_property_readonly("model_name", &PyModel<T>::get_model_name)
      .def_property_readonly("robot_dof", &PyModel<T>::get_robot_dof)
      .def_property_readonly("robot_joint_names", &PyModel<T>::get_robot_joint_names)
      .def_property_readonly("mobility_idx", &PyModel<T>::get_mobility_idx)
      .def_property_readonly("body_idx", &PyModel<T>::get_body_idx)
      .def_property_readonly("head_idx", &PyModel<T>::get_head_idx)
      .def_property_readonly("right_arm_idx", &PyModel<T>::get_right_arm_idx)
      .def_property_readonly("left_arm_idx", &PyModel<T>::get_left_arm_idx)
      .def_property_readonly("torso_idx", &PyModel<T>::get_torso_idx)
      .def_property_readonly("velocity_estimation_required_idx", &PyModel<T>::get_velocity_estimation_required_idx)
      .def_property_readonly("control_period", &PyModel<T>::get_control_period);
}

void pybind11_model(py::module_& m) {
  bind_model<y1_model::A>(m, "Model_A");
  bind_model<y1_model::T5>(m, "Model_T5");
  bind_model<y1_model::M>(m, "Model_M");
}