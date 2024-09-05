#include <iostream>
#include "rby1-sdk/dynamics/robot.h"

#include "sample_robot.h"

Eigen::IOFormat fmt(3, 0, ", ", ";\n", "[", "]", "[", "]");

int main() {
  auto robot = std::make_shared<SampleRobot>();
  auto state = robot->MakeState({"base", "tooltip"}, {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

  Eigen::Vector<double, 6 /* Degree of freedom */> q;
  q << 90, -45, 30, -30, 0, 90;
  q *= rb::math::kDeg2Rad;

  state->SetQ(q);

  robot->ComputeForwardKinematics(state);

  std::cout << R"(Reflective inertia of "tooltip" with respect to "base": )" << std::endl;
  std::cout << robot->ComputeReflectiveInertia(state, 0, 1).format(fmt) << std::endl;

  return 0;
}