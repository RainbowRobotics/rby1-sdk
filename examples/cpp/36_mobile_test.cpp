// ################### CAUTION ###################
// # CAUTION:
// # Mobile base commands move the robot in the surrounding workspace.
// # Ensure that the floor area is clear before running this example.
// ###############################################

// Mobile Command Demo
// This example brings up the robot and runs several mobility commands using SE(2) velocity control.
// See --help for arguments.
//
// Note: On the A model, the Y component of SE(2) commands is ignored.
//
// Usage example:
//     ./example_36_mobile_test --address 192.168.30.1:50051 --model m --power '.*' --servo '.*'
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#ifdef _WIN32
#  define _USE_MATH_DEFINES
#endif
#include <cmath>
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

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m] [--power <regex>] [--servo <regex>]" << std::endl;
}

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kMinimumTime = 2.5;

template <typename ModelT>
bool ReadyCommand(const std::shared_ptr<Robot<ModelT>>& robot) {
  std::cout << "example_ready_command" << std::endl;

  Eigen::VectorXd q_body(static_cast<Eigen::Index>(ModelT::kBodyIdx.size()));
  // torso: [0, 45, -90, 45, 0, 0] deg
  q_body(0) = 0;
  q_body(1) = 45 * kDeg2Rad;
  q_body(2) = -90 * kDeg2Rad;
  q_body(3) = 45 * kDeg2Rad;
  q_body(4) = 0;
  q_body(5) = 0;
  // right arm: [0, -5, 0, -120, 0, 70, 0] deg
  q_body(6) = 0;
  q_body(7) = -5 * kDeg2Rad;
  q_body(8) = 0;
  q_body(9) = -120 * kDeg2Rad;
  q_body(10) = 0;
  q_body(11) = 40 * kDeg2Rad;
  q_body(12) = 0;
  // left arm: [0, 5, 0, -120, 0, 70, 0] deg
  q_body(13) = 0;
  q_body(14) = 5 * kDeg2Rad;
  q_body(15) = 0;
  q_body(16) = -120 * kDeg2Rad;
  q_body(17) = 0;
  q_body(18) = 40 * kDeg2Rad;
  q_body(19) = 0;

  auto rv = robot
                ->SendCommand(
                    RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                        JointPositionCommandBuilder()
                            .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(2.0))
                            .SetMinimumTime(7.0)
                            .SetPosition(q_body))),
                    10)
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Error: Failed to conduct demo motion." << std::endl;
    return false;
  }
  return true;
}

template <typename ModelT>
bool SE2Command(const std::shared_ptr<Robot<ModelT>>& robot, const std::string& name, double lx, double ly,
                double angular) {
  std::cout << name << std::endl;

  auto rv = robot
                ->SendCommand(
                    RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetMobilityCommand(
                        SE2VelocityCommandBuilder()
                            .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.0))
                            .SetMinimumTime(kMinimumTime)
                            .SetVelocity(Eigen::Vector2d(lx, ly), angular))),
                    10)
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Error: Failed to conduct demo motion." << std::endl;
    return false;
  }
  std::cout << "finish motion" << std::endl;
  return true;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  std::cout << "Attempting to connect to the robot..." << std::endl;
  auto robot = Robot<ModelT>::Create(address);
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Failed to servo on" << std::endl;
      return 1;
    }
  }

  const auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
      return 1;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }

  std::cout << "Control Manager state is normal. No faults detected." << std::endl;

  std::cout << "Enabling the Control Manager..." << std::endl;
  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    return 1;
  }
  std::cout << "Control Manager enabled successfully." << std::endl;

  if (!ReadyCommand(robot)) {
    return 1;
  }

  SE2Command(robot, "example_SE2_x_forward_command", 0.1, 0.0, 0.0);
  SE2Command(robot, "example_SE2_x_backward_command", -0.1, 0.0, 0.0);
  // On the A model, the Y component of SE(2) commands is ignored.
  SE2Command(robot, "example_SE2_y_forward_command", 0.0, 0.1, 0.0);
  SE2Command(robot, "example_SE2_y_backward_command", 0.0, -0.1, 0.0);
  SE2Command(robot, "example_SE2_turn_right_command", 0.0, 0.0, 0.1);
  SE2Command(robot, "example_SE2_turn_left_command", 0.0, 0.0, -0.1);

  std::cout << "end of demo" << std::endl;
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

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
