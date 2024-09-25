#include <chrono>
#include <iostream>
#include "rby1-sdk/dynamics/robot.h"

using namespace rb::dyn;

int main() {
  // auto robot = std::make_shared<Robot<26>>(LoadFromURDF(PATH "/sample_model.urdf", "mobile_base_V2"));
  // auto state = robot->MakeState({"mobile_base_V2", "Link_6_Hip_Yaw"}, {});

  auto robot = std::make_shared<Robot<22>>(LoadRobotFromURDF(PATH "/sample_model.urdf", "base"));
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>({"base", "Link_6_Hip_Yaw"}, {});

  for (const auto& collision : robot->GetLink("Link_6_Hip_Yaw")->GetCollisions()) {
    for (const auto& geom : collision->GetGeoms()) {
      std::cout << "type: " << to_string(geom->GetType()) << std::endl;

      if (geom->GetType() == GeomType::kCapsule) {
        auto g = std::static_pointer_cast<GeomCapsule>(geom);
        std::cout << "sp: " << g->GetStartPoint().transpose() << std::endl;
        std::cout << "ep: " << g->GetEndPoint().transpose() << std::endl;
        std::cout << "radius: " << g->GetRadius() << std::endl;
      }
    }
  }

  for (const auto& collision : robot->GetLink(state, 1)->GetCollisions()) {
    for (const auto& geom : collision->GetGeoms()) {
      std::cout << "type: " << to_string(geom->GetType()) << std::endl;

      if (geom->GetType() == GeomType::kCapsule) {
        auto g = std::static_pointer_cast<GeomCapsule>(geom);
        std::cout << "sp: " << g->GetStartPoint().transpose() << std::endl;
        std::cout << "ep: " << g->GetEndPoint().transpose() << std::endl;
        std::cout << "radius: " << g->GetRadius() << std::endl;
      }
    }
  }
  //  state->SetQ(Eigen::Vector<double, 26>::Random().eval());
  //  std::cout << state->GetQ().transpose() << std::endl;

  std::cout << "Joint names : " << std::endl;
  const auto& joint_names = state->GetJointNames();
  for (const auto& name : joint_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;

  std::cout << "Link names : " << std::endl;
  const auto& link_names = state->GetLinkNames();
  for (const auto& name : link_names) {
    std::cout << name << " ";
  }
  std::cout << std::endl;
  std::cout << std::endl;

  std::cout << "GetLimitQLower()" << std::endl;
  std::cout << robot->GetLimitQLower(state).transpose() << std::endl;
  std::cout << std::endl;

  std::cout << "GetLimitQUpper()" << std::endl;
  std::cout << robot->GetLimitQUpper(state).transpose() << std::endl;
  std::cout << std::endl;

  //  robot->ComputeForwardKinematics(state);
  //  std::cout << "BodyJacobian" << std::endl;
  //  std::cout << robot.ComputeBodyJacobian(state, 0, 1) << std::endl;
  //  std::cout << robot.ComputeCenterOfMassBodyJacobian(state, 0, 1) << std::endl;

  //  std::cout << robot.ComputeTransformation(state, 0, 1) << std::endl;

  //  std::cout << robot->ComputeTotalInertial(state, 0) << std::endl;

  const auto& I = robot->ComputeTotalInertial(state, 1);
  std::cout << rb::dyn::Inertial::GetCOM(I).transpose() << std::endl;
  std::cout << robot->ComputeCenterOfMass(state, 1).transpose() << std::endl;

  std::cout << std::endl;

  //  std::cout << robot->ComputeCenterOfMass(state, 0, 1) << std::endl << std::endl;

  std::cout << "===== NUMERICAL DERIVATIVE =====" << std::endl;
  auto start = std::chrono::steady_clock::now();
  Eigen::Matrix<double, 3, 22> J;
  Eigen::Vector<double, 22> q{Eigen::Vector<double, 22>::Random()};
  double delta = 0.001;
  for (int i = 0; i < 22; i++) {
    Eigen::Vector<double, 22> h;
    h.setZero();
    h(i) = delta;

    state->SetQ(q + h);
    robot->ComputeForwardKinematics(state);

    Eigen::Vector<double, 3> com_plus = robot->ComputeCenterOfMass(state, 0);

    state->SetQ(q - h);
    robot->ComputeForwardKinematics(state);

    Eigen::Vector<double, 3> com_minus = robot->ComputeCenterOfMass(state, 0);

    J.col(i) = (com_plus - com_minus) / (2 * delta);
  }
  std::cout << "Duration: "
            << (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start)
                       .count() /
                   1.e6
            << " ms" << std::endl;

  std::cout << J << std::endl;

  std::cout << std::endl;

  std::cout << "===== ANALYTIC DERIVATIVE =====" << std::endl;
  start = std::chrono::steady_clock::now();
  state->SetQ(q);
  robot->ComputeForwardKinematics(state);
  const auto& J_com = robot->ComputeCenterOfMassJacobian(state, 0);
  std::cout << "Duration: "
            << (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start)
                       .count() /
                   1.e6
            << " ms" << std::endl;
  std::cout << J_com << std::endl;

  return 0;
}