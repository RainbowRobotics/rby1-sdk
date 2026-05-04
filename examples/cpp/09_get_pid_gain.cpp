// ################################ Note ################################
// # This example does not run in simulation.
// ######################################################################
// Get PID Gain Demo
// This example demonstrates how to get PID gains of the robot.
//
// Usage example:
//   ./example_09_get_pid_gain --address 127.0.0.1:50051 --model a
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
#include <vector>

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
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(".*")) {
    if (!robot->PowerOn(".*")) {
      std::cerr << "Failed to power on the robot" << std::endl;
      return 1;
    }
  }

  std::this_thread::sleep_for(500ms);  // Waits for all actuators to be fully powered on

  std::cout << ">>> Retrieving PID gains using component names" << std::endl;

  try {
    auto gain_list = robot->GetTorsoPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[torso_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get torso PID gains: " << e.what() << std::endl;
  }

  try {
    auto gain_list = robot->GetRightArmPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[right_arm_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get right arm PID gains: " << e.what() << std::endl;
  }

  try {
    auto gain_list = robot->GetLeftArmPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[left_arm_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get left arm PID gains: " << e.what() << std::endl;
  }

  try {
    auto gain_list = robot->GetHeadPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[head_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get head PID gains: " << e.what() << std::endl;
  }

  std::cout << ">>> Retrieving PID gains using joint names" << std::endl;

  for (const auto& joint_name : {"torso_0", "right_arm_0", "left_arm_0", "head_0"}) {
    try {
      auto gain = robot->GetPositionPIDGain(joint_name);
      std::cout << "[" << joint_name << "] P: " << gain.p_gain
                << ", I: " << gain.i_gain << ", D: " << gain.d_gain << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Failed to get PID gain for joint '" << joint_name << "': " << e.what() << std::endl;
    }
  }

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
