// ################### CAUTION ###################
// # CAUTION:
// # Ensure that the robot has enough surrounding clearance before running this example.
// ###############################################

// Command Stream Demo
// This example brings up the robot and streams body joint position commands while the last body
// joint follows a sinusoidal target. See --help for arguments.
//
// Usage example:
//     ./example_31_command_stream --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "00_helper.cpp"
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

std::atomic<bool> g_stop_requested{false};

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m] [--power <regex>] [--servo <regex>]" << std::endl;
}


void SignalHandler(int) { g_stop_requested.store(true); }

constexpr double kPi = 3.14159265358979323846;

template <typename ModelT>
bool MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso(6);
  Eigen::VectorXd right_arm(7);
  Eigen::VectorXd left_arm(7);

  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  return MoveJ<ModelT>(robot, &torso, &right_arm, &left_arm, 5.0);
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  auto robot = InitializeRobot<ModelT>(address, power, servo);
  if (!robot) {
    return 1;
  }

  std::cout << "===== Command Stream Example =====" << std::endl;

  std::cout << std::boolalpha
            << robot->SetParameter("joint_position_command.cutoff_frequency", "5") << std::endl;
  std::cout << std::boolalpha
            << robot->SetParameter("default.acceleration_limit_scaling", "0.8") << std::endl;

  if (!MoveToPreControlPose(robot)) {
    std::cerr << "Failed to move to pre-control pose" << std::endl;
    return 1;
  }

  auto stream = robot->CreateCommandStream(10);
  std::signal(SIGINT, SignalHandler);

  constexpr double dt = 0.001;
  for (int t = 0; t < 10000; ++t) {
    if (g_stop_requested.load()) {
      stream->Cancel();
      return 1;
    }
    
    Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(ModelT::kBodyIdx.size()));
    q(q.size() - 1) = kPi / 4.0 * std::sin(kPi * 2.0 * t * dt / 5.0);

    RobotCommandBuilder rc;
    rc.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
        JointPositionCommandBuilder()
            .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.0))
            .SetMinimumTime(dt)
            .SetPosition(q)));

    stream->SendCommand(rc);
    std::this_thread::sleep_for(std::chrono::duration<double>(dt * 0.5));
  }

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
