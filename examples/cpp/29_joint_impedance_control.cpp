// Joint Impedance Control Demo
// This example demonstrates how to control the robot's joints using joint impedance control.
// Scenario:
//   1. Move to the pre-control pose
//   2. Run joint impedance control for both arms
//   3. Command both arms toward the zero position with stiffness, damping, and torque limits
//
// Usage example:
//   ./example_28_joint_impedance_control --address 127.0.0.1:50051 --model a --power ".*" --servo ".*"
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
#include <string>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

template <typename ModelT>
std::shared_ptr<Robot<ModelT>> InitializeRobot(const std::string& address, const std::string& power,
                                               const std::string& servo) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return nullptr;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return nullptr;
  }
  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to turn power (" << power << ") on" << std::endl;
      return nullptr;
    }
  }
  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Failed to servo (" << servo << ") on" << std::endl;
      return nullptr;
    }
  }

  auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Failed to reset control manager" << std::endl;
      return nullptr;
    }
  }
  if (!robot->EnableControlManager()) {
    std::cerr << "Failed to enable control manager" << std::endl;
    return nullptr;
  }

  return robot;
}

template <typename ModelT>
bool MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso(6);
  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  Eigen::VectorXd right_arm(7);
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  Eigen::VectorXd left_arm(7);
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    return false;
  }
  return true;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  auto robot = InitializeRobot<ModelT>(address, power, servo);
  if (!robot) {
    return 1;
  }

  if (!MoveToPreControlPose(robot)) {
    return 1;
  }

  // Joint Impedance Control on BOTH arms (same as Python)
  constexpr size_t right_arm_dof = ModelT::kRightArmIdx.size();
  constexpr size_t left_arm_dof = ModelT::kLeftArmIdx.size();

  auto handler = robot->SendCommand(
      RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
          BodyComponentBasedCommandBuilder()
              .SetRightArmCommand(JointImpedanceControlCommandBuilder()
                                     .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(10.0))
                                     .SetPosition(Eigen::VectorXd::Zero(right_arm_dof))
                                     .SetMinimumTime(5.0)
                                     .SetStiffness(Eigen::VectorXd::Constant(right_arm_dof, 100.0))
                                     .SetDampingRatio(1.0)
                                     .SetTorqueLimit(Eigen::VectorXd::Constant(right_arm_dof, 10.0)))
              .SetLeftArmCommand(JointImpedanceControlCommandBuilder()
                                    .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(10.0))
                                    .SetPosition(Eigen::VectorXd::Zero(left_arm_dof))
                                    .SetMinimumTime(5.0)
                                    .SetStiffness(Eigen::VectorXd::Constant(left_arm_dof, 100.0))
                                    .SetDampingRatio(1.0)
                                    .SetTorqueLimit(Eigen::VectorXd::Constant(left_arm_dof, 10.0))))));

  auto rv = handler->Get();
  std::cout << "Finish Code: " << static_cast<int>(rv.finish_code()) << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";
  std::string servo = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg == "--servo" && i + 1 < argc) {
      servo = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
    } else if (servo == ".*") {
      servo = arg;
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
    return Run<y1_model::A>(address, power, servo);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power, servo);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power, servo);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
