// Dynamics Modeling Example
// This example builds a minimal two-link dynamics model, sets a joint angle,
// computes forward kinematics, and prints the transform from link_0 to link_1.
//
// Usage example:
//   ./example_18_dynamics_modeling 
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "rby1-sdk/dynamics/joint.h"
#include "rby1-sdk/dynamics/link.h"
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/math/liegroup.h"

using namespace rb;
using namespace rb::dyn;

int main() {
  auto link_0 = Link::Make("link_0");
  auto link_1 = Link::Make("link_1");

  auto joint_0 = Joint::MakeRevoluteJoint("joint_0", math::SE3::Identity(), Eigen::Vector3d(0, 0, 1));
  joint_0->ConnectLinks(link_0, link_1, math::SE3::Identity(), math::SE3::Identity());

  RobotConfiguration config;
  config.name = "sample_robot";
  config.base_link = link_0;
  config.mobile_base = nullptr;

  Robot<1> dyn_robot(config);
  auto dyn_state = dyn_robot.MakeState(std::vector<std::string>{"link_0", "link_1"},
                                       std::vector<std::string>{"joint_0"});
  dyn_state->SetQ(Eigen::Vector<double, 1>{3.14159265358979323846 / 2.0});

  std::cout << "link_0:" << std::endl;
  std::cout << link_0->GetName() << std::endl;

  dyn_robot.ComputeForwardKinematics(dyn_state);
  const auto transform = dyn_robot.ComputeTransformation(dyn_state, 0, 1);

  std::cout << "Transformation from link_0 to link_1:" << std::endl;
  std::cout << transform << std::endl;

  return 0;
}
