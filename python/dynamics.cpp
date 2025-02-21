#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <iomanip>
#include <utility>

#include "common.h"
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/dynamics/state.h"
#include "rby1-sdk/model.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

class PyGeom : public dyn::Geom {
 public:
  using dyn::Geom::Geom;

  dyn::GeomType GetType() const override { PYBIND11_OVERRIDE_PURE(dyn::GeomType, dyn::Geom, GetType); }

  std::optional<dyn::CollisionResult> ComputeMinimumDistance(const math::SE3::MatrixType& T, const Geom& other_geom,
                                                             const math::SE3::MatrixType& other_T) const override {
    PYBIND11_OVERRIDE_PURE(std::optional<dyn::CollisionResult>, dyn::Geom, ComputeMinimumDistance, T, other_geom,
                           other_T);
  }

  using dyn::Geom::GetColaffinity;
  using dyn::Geom::GetColtype;
};

void bind_collision_result(py::module& m) {
  py::class_<dyn::CollisionResult>(m, "CollisionResult")
      .def(py::init<>())
      .def_readonly("link1", &dyn::CollisionResult::link1)
      .def_readonly("link2", &dyn::CollisionResult::link2)
      .def_readonly("position1", &dyn::CollisionResult::position1)
      .def_readonly("position2", &dyn::CollisionResult::position2)
      .def_readonly("distance", &dyn::CollisionResult::distance)
      .def("__repr__", [](const dyn::CollisionResult& self) {
        py::object np = py::module_::import("numpy");
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)                              //
           << "CollisionResult("                                                             //
           << "link1='" << self.link1 << "'"                                                 //
           << ", link2=" << self.link2 << "'"                                                //
           << ", position1=" << np.attr("array2string")(self.position1).cast<std::string>()  //
           << ", position2=" << np.attr("array2string")(self.position2).cast<std::string>()  //
           << ", distance=" << self.distance                                                 //
           << ")";
        return ss.str();
      });
}

void bind_geom(py::module_& m) {
  py::enum_<dyn::GeomType>(m, "GeomType").value("Capsule", dyn::GeomType::kCapsule);

  py::class_<dyn::Geom, PyGeom, std::shared_ptr<dyn::Geom>>(m, "Geom")
      .def(py::init<unsigned int, unsigned int>())
      .def("get_type", &dyn::Geom::GetType)
      .def("get_coltype", &dyn::Geom::GetColtype)
      .def("get_colaffinity", &dyn::Geom::GetColaffinity)
      .def("compute_minimum_distance", &dyn::Geom::ComputeMinimumDistance, "T"_a, "other_geom"_a, "other_T"_a)
      .def("filter", &dyn::Geom::Filter, "other_geom"_a);

  py::class_<dyn::GeomCapsule, std::shared_ptr<dyn::GeomCapsule>>(m, "GeomCapsule")
      .def(py::init<double, double, unsigned int, unsigned int>())
      .def(py::init<Eigen::Vector3d, Eigen::Vector3d, double, unsigned int, unsigned int>())
      .def("get_start_point", &dyn::GeomCapsule::GetStartPoint)
      .def("get_end_point", &dyn::GeomCapsule::GetEndPoint)
      .def("get_radius", &dyn::GeomCapsule::GetRadius);
}

void bind_collision(py::module_& m) {
  py::class_<dyn::Collision, std::shared_ptr<dyn::Collision>>(m, "Collision")
      .def(py::init<std::string>())
      .def("set_origin", &dyn::Collision::SetOrigin, "T"_a)
      .def("get_origin", &dyn::Collision::GetOrigin)
      .def("add_geom", &dyn::Collision::AddGeom, "geom"_a)
      .def("get_geoms",
           static_cast<std::vector<std::shared_ptr<dyn::Geom>> (dyn::Collision::*)()>(&dyn::Collision::GetGeoms))
      .def("get_geoms", static_cast<const std::vector<std::shared_ptr<dyn::Geom>>& (dyn::Collision::*)() const>(
                            &dyn::Collision::GetGeoms));
}

void bind_link_joint(py::module_& m) {
  py::class_<dyn::Joint, std::shared_ptr<dyn::Joint>> joint(m, "Joint");

  py::class_<dyn::Link, std::shared_ptr<dyn::Link>>(m, "Link")
      .def(py::init(&dyn::Link::Make), "name"_a, "I"_a = dyn::Inertial::I(1.))
      .def("get_name", &dyn::Link::GetName)
      .def("get_parent_joint",
           [](dyn::Link& self) -> std::shared_ptr<dyn::Joint> {
             auto ptr = self.GetParentJoint();
             if (ptr.expired()) {
               return nullptr;
             }
             return ptr.lock();
           })
      .def("get_child_joint_list",
           static_cast<std::vector<std::shared_ptr<dyn::Joint>> (dyn::Link::*)()>(&dyn::Link::GetChildJointList))
      .def("get_child_joint_list",
           static_cast<const std::vector<std::shared_ptr<dyn::Joint>>& (dyn::Link::*)() const>(
               &dyn::Link::GetChildJointList),
           py::return_value_policy::reference_internal)
      .def("add_collision", &dyn::Link::AddCollision)
      .def("get_collisions",
           static_cast<std::vector<std::shared_ptr<dyn::Collision>> (dyn::Link::*)()>(&dyn::Link::GetCollisions))
      .def("get_collisions",
           static_cast<const std::vector<std::shared_ptr<dyn::Collision>>& (dyn::Link::*)() const>(
               &dyn::Link::GetCollisions),
           py::return_value_policy::reference_internal);

  joint.def(py::init(&dyn::Joint::Make), "name"_a, "S"_a)
      .def(py::init(&dyn::Joint::MakeRevoluteJoint), "name"_a, "T"_a = math::SE3::Identity(),
           "axis"_a = Eigen::Vector3d{0, 0, 1})
      .def(py::init(&dyn::Joint::MakePrismaticJoint), "name"_a, "T"_a = math::SE3::Identity(),
           "axis"_a = Eigen::Vector3d{0, 0, 1})
      .def(py::init(&dyn::Joint::MakeFixedJoint), "name"_a)
      .def("get_name", &dyn::Joint::GetName)
      .def("connect_links", &dyn::Joint::ConnectLinks, "parent_link"_a, "child_link"_a,
           "T_pj"_a = math::SE3::Identity(), "T_jc"_a = math::SE3::Identity())
      .def("disconnect", &dyn::Joint::Disconnect)
      .def("set_limit_q", &dyn::Joint::SetLimitQ, "lower"_a, "upper"_a)
      .def("set_limit_qdot", &dyn::Joint::SetLimitQdot, "lower"_a, "upper"_a)
      .def("set_limit_qddot", &dyn::Joint::SetLimitQddot, "lower"_a, "upper"_a)
      .def("get_limit_q_lower", &dyn::Joint::GetLimitQLower)
      .def("get_limit_q_upper", &dyn::Joint::GetLimitQUpper)
      .def("get_limit_qdot_lower", &dyn::Joint::GetLimitQdotLower)
      .def("get_limit_qdot_upper", &dyn::Joint::GetLimitQdotUpper)
      .def("get_limit_qddot_lower", &dyn::Joint::GetLimitQddotLower)
      .def("get_limit_qddot_upper", &dyn::Joint::GetLimitQddotUpper)
      .def("set_limit_q_lower", &dyn::Joint::SetLimitQLower, "val"_a)
      .def("set_limit_q_upper", &dyn::Joint::SetLimitQUpper, "val"_a)
      .def("set_limit_qdot_lower", &dyn::Joint::SetLimitQdotLower, "val"_a)
      .def("set_limit_qdot_upper", &dyn::Joint::SetLimitQdotUpper, "val"_a)
      .def("set_limit_qddot_lower", &dyn::Joint::SetLimitQddotLower, "val"_a)
      .def("set_limit_qddot_upper", &dyn::Joint::SetLimitQddotUpper, "val"_a)
      .def("get_parent_link",
           [](dyn::Joint& self) -> std::shared_ptr<dyn::Link> {
             auto ptr = self.GetParentLink();
             if (ptr.expired()) {
               return nullptr;
             }
             return ptr.lock();
           })
      .def("get_child_link", static_cast<std::shared_ptr<dyn::Link> (dyn::Joint::*)()>(&dyn::Joint::GetChildLink))
      .def("get_child_link",
           static_cast<std::shared_ptr<const dyn::Link> (dyn::Joint::*)() const>(&dyn::Joint::GetChildLink))
      .def("is_fixed", &dyn::Joint::IsFixed);
}

void bind_mobile_base(py::module_& m) {
  py::enum_<dyn::MobileBaseType>(m, "MobileBaseType")
      .value("None", dyn::MobileBaseType::kNone)
      .value("Differential", dyn::MobileBaseType::kDifferential)
      .value("Mecanum", dyn::MobileBaseType::kMecanum);

  py::class_<dyn::MobileBase>(m, "MobileBase")
      .def_readonly("type", &dyn::MobileBase::type)
      .def_readonly("T", &dyn::MobileBase::T)
      .def_readonly("joints", &dyn::MobileBase::joints)
      .def_readonly("params", &dyn::MobileBase::params);

  py::class_<dyn::MobileBaseDifferential, dyn::MobileBase>(m, "MobileBaseDifferential")
      .def_readonly("right_wheel_idx", &dyn::MobileBaseDifferential::right_wheel_idx)
      .def_readonly("left_wheel_idx", &dyn::MobileBaseDifferential::left_wheel_idx)
      .def_readonly("wheel_base", &dyn::MobileBaseDifferential::wheel_base)
      .def_readonly("wheel_radius", &dyn::MobileBaseDifferential::wheel_radius);
}

void bind_robot_configuration(py::module_& m) {
  py::class_<dyn::RobotConfiguration>(m, "RobotConfiguration")
      .def_readonly("name", &dyn::RobotConfiguration::name)
      .def_readonly("base_link", &dyn::RobotConfiguration::base_link)
      .def_readonly("mobile_base", &dyn::RobotConfiguration::mobile_base);
}

template <int DOF>
void bind_robot(py::module_& m) {
  std::stringstream ss;
  ss << "Robot";

  if constexpr (DOF > 0) {
    ss << "_" << DOF;
  }

  py::class_<dyn::Robot<DOF>, std::shared_ptr<dyn::Robot<DOF>>>(m, ss.str().c_str())
      .def(py::init<const dyn::RobotConfiguration&>(), "robot_configuration"_a)
      .def("get_base", &dyn::Robot<DOF>::GetBase)
      .def("get_link_names", &dyn::Robot<DOF>::GetLinkNames)
      .def("get_joint_names", &dyn::Robot<DOF>::GetJointNames)
      .def("get_link",
           static_cast<std::shared_ptr<dyn::Link> (dyn::Robot<DOF>::*)(const std::string&) const>(
               &dyn::Robot<DOF>::GetLink),
           "name"_a)
      .def("get_link",
           static_cast<std::shared_ptr<dyn::Link> (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, int) const>(
               &dyn::Robot<DOF>::GetLink),
           "state"_a, "index"_a)
      .def(
          "make_state",
          [](dyn::Robot<DOF>& self, const std::vector<std::string>& link_names,
             const std::vector<std::string>& joint_names) { return self.MakeState(link_names, joint_names); },
          "link_names"_a, "joint_names"_a)
      .def("get_dof", &dyn::Robot<DOF>::GetDOF)
      .def("get_number_of_joints", &dyn::Robot<DOF>::GetNumberOfJoints)
      .def("compute_forward_kinematics", &dyn::Robot<DOF>::ComputeForwardKinematics, "state"_a)
      .def("compute_diff_forward_kinematics", &dyn::Robot<DOF>::ComputeDiffForwardKinematics, "state"_a)
      .def("compute_2nd_diff_forward_kinematics", &dyn::Robot<DOF>::Compute2ndDiffForwardKinematics, "state"_a)
      .def("compute_inverse_dynamics", &dyn::Robot<DOF>::ComputeInverseDynamics, "state"_a)
      .def("compute_gravity_term", &dyn::Robot<DOF>::ComputeGravityTerm, "state"_a)
      .def("compute_mass_matrix", &dyn::Robot<DOF>::ComputeMassMatrix, "state"_a)
      .def("compute_reflective_inertia", &dyn::Robot<DOF>::ComputeReflectiveInertia, "state"_a, "from"_a, "to"_a)
      .def("compute_transformation", &dyn::Robot<DOF>::ComputeTransformation, "state"_a, "from"_a, "to"_a)
      .def("compute_body_velocity", &dyn::Robot<DOF>::ComputeBodyVelocity, "state"_a, "from"_a, "to"_a)
      .def("compute_space_jacobian", &dyn::Robot<DOF>::ComputeSpaceJacobian, "state"_a, "from"_a, "to"_a)
      .def("compute_body_jacobian", &dyn::Robot<DOF>::ComputeBodyJacobian, "state"_a, "from"_a, "to"_a)
      .def("compute_mass", &dyn::Robot<DOF>::ComputeMass, "state"_a, "target_link"_a)
      .def("compute_center_of_mass",
           static_cast<Eigen::Vector3d (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, unsigned int,
                                                            unsigned int)>(&dyn::Robot<DOF>::ComputeCenterOfMass),
           "state"_a, "ref_link"_a, "target_link"_a)
      .def("compute_center_of_mass",
           static_cast<Eigen::Vector3d (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, unsigned int,
                                                            const std::vector<unsigned int>&)>(
               &dyn::Robot<DOF>::ComputeCenterOfMass),
           "state"_a, "ref_link"_a, "target_links"_a)
      .def("compute_center_of_mass_jacobian",
           static_cast<Eigen::Matrix<double, 3, DOF> (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>,
                                                                          unsigned int, unsigned int)>(
               &dyn::Robot<DOF>::ComputeCenterOfMassJacobian),
           "state"_a, "ref_link"_a, "target_link"_a)
      .def("compute_total_inertial", &dyn::Robot<DOF>::ComputeTotalInertial, "state"_a, "ref_link"_a)
      .def("compute_center_of_mass",
           static_cast<Eigen::Vector3d (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, unsigned int)>(
               &dyn::Robot<DOF>::ComputeCenterOfMass),
           "state"_a, "ref_link"_a)
      .def("compute_center_of_mass_jacobian",
           static_cast<Eigen::Matrix<double, 3, DOF> (dyn::Robot<DOF>::*)(
               std::shared_ptr<dyn::State<DOF>>, unsigned int)>(&dyn::Robot<DOF>::ComputeCenterOfMassJacobian),
           "state"_a, "ref_link"_a)
      .def("detect_collisions_or_nearest_links", &dyn::Robot<DOF>::DetectCollisionsOrNearestLinks, "state"_a,
           "collision_threshold"_a = 0)
      .def("get_limit_q_lower", &dyn::Robot<DOF>::GetLimitQLower, "state"_a)
      .def("get_limit_q_upper", &dyn::Robot<DOF>::GetLimitQUpper, "state"_a)
      .def("get_limit_qdot_lower", &dyn::Robot<DOF>::GetLimitQdotLower, "state"_a)
      .def("get_limit_qdot_upper", &dyn::Robot<DOF>::GetLimitQdotUpper, "state"_a)
      .def("get_limit_qddot_lower", &dyn::Robot<DOF>::GetLimitQddotLower, "state"_a)
      .def("get_limit_qddot_upper", &dyn::Robot<DOF>::GetLimitQddotUpper, "state"_a)
      .def("get_joint_property", &dyn::Robot<DOF>::GetJointProperty, "state"_a, "getter"_a)
      .def("compute_mobility_inverse_diff_kinematics",
           static_cast<void (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, const Eigen::Vector2d&, double)>(
               &dyn::Robot<DOF>::ComputeMobilityInverseDiffKinematics),
           "state"_a, "linear_velocity"_a, "angular_velocity"_a)
      .def("compute_mobility_inverse_diff_kinematics",
           static_cast<void (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, const math::se2v::MatrixType&)>(
               &dyn::Robot<DOF>::ComputeMobilityInverseDiffKinematics),
           "state"_a, "body_velocity"_a)
      .def("compute_mobility_diff_kinematics", &dyn::Robot<DOF>::ComputeMobilityDiffKinematics, "state"_a)
      .def_static("count_joints", &dyn::Robot<DOF>::CountJoints, "base_link"_a, "include_fixed"_a = false);
}

template <int DOF>
void bind_state(py::module_& m) {
  std::stringstream ss;
  ss << "State";

  if constexpr (DOF > 0) {
    ss << "_" << DOF;
  }

  py::class_<dyn::State<DOF>, std::shared_ptr<dyn::State<DOF>>>(m, ss.str().c_str())
      .def("get_base_link_idx", &dyn::State<DOF>::GetBaseLinkIdx)
      .def(
          "set_q",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> q) { self.SetQ(q); }, "q"_a)
      .def("get_q", &dyn::State<DOF>::GetQ)
      .def(
          "set_qdot",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> qdot) { self.SetQdot(qdot); }, "qdot"_a)
      .def("get_qdot", &dyn::State<DOF>::GetQdot)
      .def(
          "set_qddot",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> qddot) { self.SetQddot(qddot); }, "qddot"_a)
      .def("get_qddot", &dyn::State<DOF>::GetQddot)
      .def(
          "set_tau",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> tau) { self.SetTau(tau); }, "tau"_a)
      .def("get_tau", &dyn::State<DOF>::GetTau)
      .def(
          "set_V0",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> V0) { self.SetV0(V0); }, "V0"_a)
      .def(
          "set_Vdot0",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::Vector<double, 6>> Vdot0) { self.SetVdot0(Vdot0); },
          "Vdot0"_a)
      .def(
          "set_gravity",  //
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::Vector<double, 6>> gravity) { self.SetGravity(gravity); },
          "gravity"_a)
      .def("get_joint_names", &dyn::State<DOF>::GetJointNames)
      .def("get_link_names", &dyn::State<DOF>::GetLinkNames);
}

void pybind11_dynamics(py::module_& m) {
  bind_collision_result(m);
  bind_geom(m);
  bind_collision(m);

  bind_link_joint(m);
  bind_mobile_base(m);
  bind_robot_configuration(m);

  bind_state<-1>(m);
  bind_robot<-1>(m);

  bind_state<y1_model::A::kRobotDOF>(m);
  bind_robot<y1_model::A::kRobotDOF>(m);

  bind_state<y1_model::T5::kRobotDOF>(m);
  bind_robot<y1_model::T5::kRobotDOF>(m);

  bind_state<y1_model::M::kRobotDOF>(m);
  bind_robot<y1_model::M::kRobotDOF>(m);

  m.def("load_robot_from_urdf_data", &dyn::LoadRobotFromURDFData, "model"_a, "base_link_name"_a);

  m.def("load_robot_from_urdf", &dyn::LoadRobotFromURDF, "path"_a, "base_link_name"_a);
}