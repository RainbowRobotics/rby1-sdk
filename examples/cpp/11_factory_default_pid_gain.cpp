// Note: This example does not run in simulation.
// Factory Default PID Gain Demo
// This example demonstrates how to set PID gains of the robot to factory default values.
// It is recommended not to use this example.
//
// Usage example:
//   ./example_11_factory_default_pid_gain --address 127.0.0.1:50051 --model a
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

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

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
void PrintPIDGains(const std::shared_ptr<Robot<ModelT>>& robot) {
  auto gain_list = robot->GetTorsoPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[torso_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetRightArmPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[right_arm_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetLeftArmPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[left_arm_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetHeadPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[head_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(".*")) {
    auto rv = robot->PowerOn(".*");
    std::this_thread::sleep_for(1s);
    if (!rv) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  // Get PID Gains
  std::cout << ">>> Before" << std::endl;
  PrintPIDGains(robot);

  // Set PID Gains (default value)
  // Torso joints
  robot->SetPositionPIDGain("torso_0", 100, 20, 900);
  robot->SetPositionPIDGain("torso_1", 1000, 38, 900);
  robot->SetPositionPIDGain("torso_2", 1000, 38, 900);
  robot->SetPositionPIDGain("torso_3", 220, 40, 400);
  robot->SetPositionPIDGain("torso_4", 50, 20, 400);
  robot->SetPositionPIDGain("torso_5", 220, 40, 400);

  // Right arm joints
  robot->SetPositionPIDGain("right_arm_0", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_1", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_2", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_3", 35, 5, 80);
  robot->SetPositionPIDGain("right_arm_4", 30, 5, 70);
  robot->SetPositionPIDGain("right_arm_5", 30, 5, 70);
  robot->SetPositionPIDGain("right_arm_6", 100, 5, 120);

  // Left arm joints
  robot->SetPositionPIDGain("left_arm_0", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_1", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_2", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_3", 35, 5, 80);
  robot->SetPositionPIDGain("left_arm_4", 30, 5, 70);
  robot->SetPositionPIDGain("left_arm_5", 30, 5, 70);
  robot->SetPositionPIDGain("left_arm_6", 100, 5, 150);

  // Head joints
  robot->SetPositionPIDGain("head_0", 800, 0, 4000);
  robot->SetPositionPIDGain("head_1", 800, 0, 4000);

  // Ensure PID Gain update completion
  std::this_thread::sleep_for(50ms);

  std::cout << "\n\n>>> After" << std::endl;
  PrintPIDGains(robot);

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
