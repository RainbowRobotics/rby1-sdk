#include <iostream>
#include "rby1-sdk/dynamics/robot.h"

#include "sample_robot.h"

Eigen::IOFormat fmt(3, 0, ", ", ";\n", "[", "]", "[", "]");

int main() {
  auto robot = std::make_shared<SampleRobot>();
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>(std::vector<std::string>{}, {"joint6", "joint2", "joint3", "joint4", "joint5", "joint1"});

  Eigen::Vector<double, 6 /* Degree of freedom */> q;
  q << 90, -45, 30, -30, 0, 90;
  q *= rb::math::kDeg2Rad;

  Eigen::Vector<double, 6> qdot = Eigen::Vector<double, 6>::Zero();
  Eigen::Vector<double, 6> qddot = Eigen::Vector<double, 6>::Constant(0.1);

  state->SetGravity({0, 0, 0, 0, 0, -9.8});
  state->SetQ(q);

  robot->ComputeForwardKinematics(state);

  const auto& mass_matrix = robot->ComputeMassMatrix(state);
  const auto& gravity_term = robot->ComputeGravityTerm(state);

  std::cout << "M(q)*qddot + g(q) = : " << std::endl;
  std::cout << (mass_matrix * qddot + gravity_term).transpose().format(fmt) << std::endl;

  return 0;
}