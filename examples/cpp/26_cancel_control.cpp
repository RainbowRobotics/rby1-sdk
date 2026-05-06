// Cancel Command Demo
// This example demonstrates how to cancel robot control.
//
// Usage example:
//   ./example_25_cancel_control --address 192.168.30.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

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
void MoveToZeroPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso = Eigen::VectorXd::Zero(ModelT::kTorsoIdx.size());
  Eigen::VectorXd right_arm = Eigen::VectorXd::Zero(ModelT::kRightArmIdx.size());
  Eigen::VectorXd left_arm = Eigen::VectorXd::Zero(ModelT::kLeftArmIdx.size());

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(3.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(3.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(3.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "zero pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::exit(1);
  }
}

template <typename ModelT>
void MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
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
    std::exit(1);
  }
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

  MoveToZeroPose(robot);

  // Start move_to_pre_control_pose in a separate thread
  std::thread thread([&robot]() { MoveToPreControlPose(robot); });
  std::cout << "move start. minimum time 5 seconds..." << std::endl;
  std::this_thread::sleep_for(2s);
  std::cout << "cancel control" << std::endl;
  robot->CancelControl();
  thread.join();

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
