// ################### CAUTION ###################
// CAUTION:
// Start from a safe posture before running this example.
// Do not use torso joints or ".*" with this example.
// Releasing a brake can cause the target joint to move under gravity.
// ###############################################
//
// Brake Test Example
// This example connects to an RB-Y1 robot, powers on the specified devices if needed,
// releases the brake for a target joint, waits briefly, and then engages the brake again.
//
// Usage example:
//     ./example_35_brake_test --address 192.168.30.1:50051 --model a --power '.*' --joint right_arm_6
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

#include <chrono>
#include <algorithm>
#include <cctype>
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
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m] [--power <regex>] --joint <joint regex>" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] <joint_regex>" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& joint) {
  const std::string joint_lower = ToLower(joint);
  
  if (joint == ".*" || joint_lower.find("torso") != std::string::npos) {
    std::cerr << "Refusing to run brake_test with joint pattern " << joint
              << ". Use a single non-torso joint instead." << std::endl;
    return 1;
  }

  auto robot = Robot<ModelT>::Create(address);
  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to turn power (" << power << ") on" << std::endl;
      return 1;
    }
  }

  if (!robot->DisableControlManager()) {
    std::cerr << "Failed to disable control manager" << std::endl;
    return 1;
  }
  std::this_thread::sleep_for(500ms);

  std::cout << "Brake release requested for " << joint << std::endl;
  if (!robot->BreakRelease(joint)) {
    std::cerr << "Failed to release brake for " << joint << std::endl;
    return 1;
  }

  std::this_thread::sleep_for(500ms);

  std::cout << "Brake engage requested for " << joint << std::endl;
  if (!robot->BreakEngage(joint)) {
    std::cerr << "Failed to engage brake for " << joint << std::endl;
    return 1;
  }

  std::cout << "Brake test finished successfully." << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";
  std::string joint;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg == "--joint" && i + 1 < argc) {
      joint = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
    } else if (joint.empty()) {
      joint = arg;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (address.empty() || joint.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return Run<y1_model::A>(address, power, joint);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power, joint);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
