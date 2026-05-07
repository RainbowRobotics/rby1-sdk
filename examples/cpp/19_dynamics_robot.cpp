// Dynamics Robot Demo
// This example demonstrates how to use the robot's dynamics model for inverse dynamics
// calculation.  It connects to the robot, retrieves the dynamics model, and computes
// joint torques.
//
// Usage example:
//   ./example_19_dynamics_robot --address 127.0.0.1:50051 --model a
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  // 1. Connect to the robot and get its model information.
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Failed to connect to the robot." << std::endl;
    return 1;
  }

  // 2. Get the dynamics model (dyn_robot) from the robot.
  auto dyn_robot = robot->GetDynamics();

  // 3. Create a State object for dynamics calculations.
  //    Specify the names of the links and joints to be used in the calculations.
  constexpr size_t DOF = ModelT::kRobotDOF;
  std::vector<std::string> link_names = {"base", "ee_right"};
  std::vector<std::string> joint_names(ModelT::kRobotJointNames.begin(), ModelT::kRobotJointNames.end());
  auto dyn_state = dyn_robot->MakeState(link_names, joint_names);

  // 4. Define and set the robot's state (q, qdot, qddot).
  //    Here, we use random joint angles for demonstration.
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> dist(-0.5, 0.5);
  Eigen::Vector<double, DOF> q;
  for (size_t i = 0; i < DOF; ++i) {
    q(i) = dist(gen) * M_PI / 2.0;
  }

  dyn_state->SetGravity({0, 0, 0, 0, 0, -9.81});
  dyn_state->SetQ(q);
  dyn_state->SetQdot(Eigen::Vector<double, DOF>::Zero());
  dyn_state->SetQddot(Eigen::Vector<double, DOF>::Zero());

  std::cout << "State:" << std::endl;
  std::cout << "  q: " << q.transpose() << std::endl;

  // 5. Perform Inverse Dynamics calculation.
  //    To compute inverse dynamics, forward kinematics-related calculations must be performed first.
  //    These functions cache their results within the dyn_state object.

  // 5-1. Compute forward kinematics (FK) to get the position/orientation of each link.
  dyn_robot->ComputeForwardKinematics(dyn_state);

  // 5-2. Compute the first derivative of FK (Jacobian) to get the velocity of each link.
  //      Must be called after compute_forward_kinematics.
  dyn_robot->ComputeDiffForwardKinematics(dyn_state);

  // 5-3. Compute the second derivative of FK to get the acceleration of each link.
  //      Must be called after compute_diff_forward_kinematics.
  dyn_robot->Compute2ndDiffForwardKinematics(dyn_state);

  // 5-4. Based on the kinematic information calculated above,
  //      compute the joint torques (tau) required to maintain the current state (q, qdot, qddot).
  dyn_robot->ComputeInverseDynamics(dyn_state);

  // 6. Print the calculated torques.
  Eigen::IOFormat fmt(4, 0, ", ", "\n", "  [", "]");
  std::cout << "Inverse dynamics torque (Nm):" << std::endl;
  std::cout << dyn_state->GetTau().transpose().format(fmt) << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (address.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
