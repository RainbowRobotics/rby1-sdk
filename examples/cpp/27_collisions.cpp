// Collisions Demo
// This example connects to an RB-Y1 robot, powers on the specified devices,
// monitors collision distance during motion, and sends a stop command when the robot gets too close.
//
// Usage example:
//   ./example_26_collisions --address 192.168.30.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>

#include "00_helper.cpp"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

constexpr double kStopDistance = 0.03;
const Eigen::Vector<double, 7> kRightArmTarget{0.8, -0.8, 0.7, -2.0, 0.0, 0.0, 0.0};

std::atomic<bool> g_stop{false};
std::atomic<bool> g_stop_requested{false};

void SignalHandler(int) {
  g_stop = true;
  g_stop_requested = true;
}

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

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
int RunRobotCollisions(const std::string& address, const std::string& power_regex, const std::string& servo_regex) {
  auto robot = InitializeRobot<ModelT>(address, power_regex, servo_regex);
  if (!robot) {
    std::cerr << "Failed to initialize robot" << std::endl;
    return 1;
  }

  if (!MoveToPreControlPose(robot)) {
    std::cerr << "Failed to move to pre control pose" << std::endl;
    return 1;
  }

  g_stop_requested = false;
  robot->StartStateUpdate(
      [](const auto& state) {
        if (state.collisions.empty()) {
          return;
        }

        const auto nearest_it = std::min_element(
            state.collisions.begin(), state.collisions.end(),
            [](const auto& a, const auto& b) { return a.distance < b.distance; });
        const auto& nearest = *nearest_it;

        std::cout << std::fixed << std::setprecision(4) << "nearest collision distance: " << nearest.distance << " | "
                  << nearest.link1 << " <-> " << nearest.link2 << std::endl;

        if (nearest.distance < kStopDistance) {
          std::cout << "Collision Detected" << std::endl;
          g_stop_requested = true;
        }
      },
      50.0);

  auto motion_handle = robot->SendCommand(
      RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
          BodyComponentBasedCommandBuilder().SetRightArmCommand(
              JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(kRightArmTarget)))),
      10);

  while (!g_stop && !g_stop_requested) {
    std::this_thread::sleep_for(10ms);
  }

  if (g_stop_requested) {
    robot->CancelControl();
  }

  const auto motion_feedback = motion_handle->Get();
  std::cout << "motion finish_code: " << static_cast<int>(motion_feedback.finish_code()) << std::endl;

  robot->StopStateUpdate();

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);

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
    return RunRobotCollisions<y1_model::A>(address, power, servo);
  }
  if (model == "m") {
    return RunRobotCollisions<y1_model::M>(address, power, servo);
  }
  if (model == "ub") {
    return RunRobotCollisions<y1_model::UB>(address, power, servo);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
