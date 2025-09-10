#include "model.h"

#include "print_helper.h"

namespace {
template <class T>
inline std::string build_model_class_doc() {
  using namespace rb::print;
  std::ostringstream s;
  s << "Robot model configuration (``" << T::kModelName << "``).\n\n"
    << "This class provides access to robot model specifications including\n"
    << "joint configurations, degrees of freedom, and component indices.\n\n"
    << "Attributes\n"
    << "----------\n"
    << "model_name : str\n"
    << "    \'" << T::kModelName << "\'\n\n"
    << "    Name identifier for the robot model.\n"
    << "robot_dof : int\n"
    << "    " << T::kRobotDOF << "\n\n"
    << "    Total degrees of freedom.\n"
    << "robot_joint_names : list[str]\n"
    << "    " << py_list(T::kRobotJointNames) << "\n\n"
    << "    List of joint names in index order.\n"
    << "mobility_idx : list[int]\n"
    << "    " << py_list(T::kMobilityIdx) << "\n\n"
    << "    Indices of mobility components (e.g., wheels).\n"
    << "body_idx : list[int]\n"
    << "    " << py_list(T::kBodyIdx) << "\n\n"
    << "    Indices of main body components (excluding mobility and head).\n"
    << "head_idx : list[int]\n"
    << "    " << py_list(T::kHeadIdx) << "\n\n"
    << "    Indices of head components.\n"
    << "right_arm_idx : list[int]\n"
    << "    " << py_list(T::kRightArmIdx) << "\n\n"
    << "    Indices of right arm components.\n"
    << "left_arm_idx : list[int]\n"
    << "    " << py_list(T::kLeftArmIdx) << "\n\n"
    << "    Indices of left arm components.\n"
    << "torso_idx : list[int]\n"
    << "    " << py_list(T::kTorsoIdx) << "\n\n"
    << "    Indices of torso components.\n"
    << "velocity_estimation_required_idx : list[int]\n"
    << "    " << py_list(T::kVelocityEstimationRequiredIdx) << "\n\n"
    << "    Indices of joints requiring velocity estimation.\n"
    << "control_period : float\n"
    << "    " << to_fixed(T::kControlPeriod, 6) << "\n\n"
    << "    Control update period in seconds.\n";
  return s.str();
}

template <class T>
inline std::string doc_model_name() {
  return std::string("\'") + std::string(T::kModelName) + "\'\n\nName identifier for the robot model.";
}

template <class T>
inline std::string doc_robot_dof() {
  std::ostringstream os;
  os << T::kRobotDOF << "\n\nTotal degrees of freedom.";
  return os.str();
}

template <class T>
inline std::string doc_robot_joint_names() {
  return rb::print::py_list(T::kRobotJointNames) + "\n\nList of joint names in index order.";
}

template <class T>
inline std::string doc_mobility_idx() {
  return rb::print::py_list(T::kMobilityIdx) + "\n\nIndices of mobility components (e.g., wheels).";
}

template <class T>
inline std::string doc_body_idx() {
  return rb::print::py_list(T::kBodyIdx) + "\n\nIndices of main body components (excluding mobility and head).";
}

template <class T>
inline std::string doc_head_idx() {
  return rb::print::py_list(T::kHeadIdx) + "\n\nIndices of head components.";
}

template <class T>
inline std::string doc_right_arm_idx() {
  return rb::print::py_list(T::kRightArmIdx) + "\n\nIndices of right arm components.";
}

template <class T>
inline std::string doc_left_arm_idx() {
  return rb::print::py_list(T::kLeftArmIdx) + "\n\nIndices of left arm components.";
}

template <class T>
inline std::string doc_torso_idx() {
  return rb::print::py_list(T::kTorsoIdx) + "\n\nIndices of torso components.";
}

template <class T>
inline std::string doc_velreq_idx() {
  return rb::print::py_list(T::kVelocityEstimationRequiredIdx) + "\n\nIndices of joints where velocity estimation is essential.";
}

template <class T>
inline std::string doc_control_period() {
  return rb::print::to_fixed(T::kControlPeriod, 6) + std::string("\n\nControl period in seconds.");
}
}  // namespace

template <typename T>
void bind_model(py::module_& m, const std::string& model_name) {
  using M = PyModel<T>;

  static const std::string kDocClass = build_model_class_doc<T>();
  static const std::string kDocName = doc_model_name<T>();
  static const std::string kDocDof = doc_robot_dof<T>();
  static const std::string kDocJoints = doc_robot_joint_names<T>();
  static const std::string kDocMob = doc_mobility_idx<T>();
  static const std::string kDocBody = doc_body_idx<T>();
  static const std::string kDocHead = doc_head_idx<T>();
  static const std::string kDocRarm = doc_right_arm_idx<T>();
  static const std::string kDocLarm = doc_left_arm_idx<T>();
  static const std::string kDocTorso = doc_torso_idx<T>();
  static const std::string kDocVelReq = doc_velreq_idx<T>();
  static const std::string kDocCtrlPeriod = doc_control_period<T>();

  py::class_<M>(m, model_name.c_str(), kDocClass.c_str())
      .def(py::init<>(), "Construct a model instance.")
      .def_property_readonly("model_name", &M::get_model_name, kDocName.c_str())
      .def_property_readonly("robot_dof", &M::get_robot_dof, kDocDof.c_str())
      .def_property_readonly("robot_joint_names", &M::get_robot_joint_names, kDocJoints.c_str())
      .def_property_readonly("mobility_idx", &M::get_mobility_idx, kDocMob.c_str())
      .def_property_readonly("body_idx", &M::get_body_idx, kDocBody.c_str())
      .def_property_readonly("head_idx", &M::get_head_idx, kDocHead.c_str())
      .def_property_readonly("right_arm_idx", &M::get_right_arm_idx, kDocRarm.c_str())
      .def_property_readonly("left_arm_idx", &M::get_left_arm_idx, kDocLarm.c_str())
      .def_property_readonly("torso_idx", &M::get_torso_idx, kDocTorso.c_str())
      .def_property_readonly("velocity_estimation_required_idx", &M::get_velocity_estimation_required_idx,
                             kDocVelReq.c_str())
      .def_property_readonly("control_period", &M::get_control_period, kDocCtrlPeriod.c_str())
      .def("__repr__",
           [model_name](const M& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             const auto& names = self.get_robot_joint_names();
             const auto& mob = self.get_mobility_idx();
             const auto& body = self.get_body_idx();
             const auto& head = self.get_head_idx();
             const auto& rarm = self.get_right_arm_idx();
             const auto& larm = self.get_left_arm_idx();
             const auto& torso = self.get_torso_idx();
             const auto& velreq = self.get_velocity_estimation_required_idx();

             std::ostringstream out;
             out << model_name << "(" << FIRST                                                          //
                 << "model_name=" << inline_obj(py::cast(self.get_model_name())) << SEP                 //
                 << "robot_dof=" << self.get_robot_dof() << SEP                                         //
                 << "robot_joint_names=" << inline_obj(py::cast(names)) << SEP                          //
                 << "mobility_idx=" << inline_obj(py::cast(mob)) << SEP                                 //
                 << "body_idx=" << inline_obj(py::cast(body)) << SEP                                    //
                 << "head_idx=" << inline_obj(py::cast(head)) << SEP                                    //
                 << "right_arm_idx=" << inline_obj(py::cast(rarm)) << SEP                               //
                 << "left_arm_idx=" << inline_obj(py::cast(larm)) << SEP                                //
                 << "torso_idx=" << inline_obj(py::cast(torso)) << SEP                                  //
                 << "velocity_estimation_required_idx=" << inline_obj(py::cast(velreq)) << SEP          //
                 << "control_period=" << format_number(self.get_control_period(), Style::Repr) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [model_name](const PyModel<T>& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << model_name << "(" << self.get_model_name() << ", dof=" << self.get_robot_dof()
            << ", joints=" << self.get_robot_joint_names().size() << ", body=" << self.get_body_idx().size()
            << ", head=" << self.get_head_idx().size() << ", R=" << self.get_right_arm_idx().size()
            << ", L=" << self.get_left_arm_idx().size() << ", torso=" << self.get_torso_idx().size()
            << ", dt=" << format_number(self.get_control_period(), Style::Str) << "s)";
        return out.str();
      });
}

void pybind11_model(py::module_& m) {
  bind_model<y1_model::A>(m, "Model_A");
  bind_model<y1_model::T5>(m, "Model_T5");
  bind_model<y1_model::M>(m, "Model_M");
  bind_model<y1_model::UB>(m, "Model_UB");
}