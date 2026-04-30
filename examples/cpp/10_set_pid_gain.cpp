// ############################## CAUTION ###############################
// # The motor PID values ​​currently set on the robot are internally optimized.
// # Please exercise caution to prevent accidents caused by changing the PID values.
// ######################################################################
// ################################ Note ################################
// # This example does not run in simulation.
// # This example does not apply when the control manager is enabled.
// ######################################################################
// Set PID Gain Demo
// This example connects to an RB-Y1 robot, reads the current PID gains for
// selected joints, updates those gains using different setter forms, and then
// restores the original values. See --help for arguments.
// Note: Disable the control manager before changing PID gains.
// Note: This example is not supported in simulation.
//
// Usage example:
//   ./example_10_set_pid_gain --address 192.168.30.1:50051 --model a
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
bool SetAndRestorePGain(const std::shared_ptr<Robot<ModelT>>& robot, const std::string& joint_name, uint16_t p_gain) {
  std::cout << ">>> [START] " << joint_name << std::endl;
  const auto original = robot->GetPositionPIDGain(joint_name);
  std::cout << "[Before] [" << joint_name << "] P: " << original.p_gain << ", I: " << original.i_gain
            << ", D: " << original.d_gain << std::endl;

  if (!robot->SetPositionPGain(joint_name, p_gain)) {
    std::cerr << "Failed to set P gain for " << joint_name << std::endl;
    return false;
  }
  std::this_thread::sleep_for(50ms);

  const auto updated = robot->GetPositionPIDGain(joint_name);
  std::cout << "[After]  [" << joint_name << "] P: " << updated.p_gain << ", I: " << updated.i_gain
            << ", D: " << updated.d_gain << std::endl;

  if (!robot->SetPositionPIDGain(joint_name, original)) {
    std::cerr << "Failed to restore PID gain for " << joint_name << std::endl;
    return false;
  }
  std::this_thread::sleep_for(50ms);

  const auto restored = robot->GetPositionPIDGain(joint_name);
  std::cout << "[Restored] [" << joint_name << "] P: " << restored.p_gain << ", I: " << restored.i_gain
            << ", D: " << restored.d_gain << std::endl;

  return true;
}

template <typename ModelT>
bool SetAndRestorePIDGain(const std::shared_ptr<Robot<ModelT>>& robot, const std::string& joint_name, uint16_t p_gain,
                          uint16_t i_gain, uint16_t d_gain) {
  std::cout << ">>> [START] " << joint_name << std::endl;
  const auto original = robot->GetPositionPIDGain(joint_name);
  std::cout << "[Before] [" << joint_name << "] P: " << original.p_gain << ", I: " << original.i_gain
            << ", D: " << original.d_gain << std::endl;

  if (!robot->SetPositionPIDGain(joint_name, p_gain, i_gain, d_gain)) {
    std::cerr << "Failed to set PID gain for " << joint_name << std::endl;
    return false;
  }
  std::this_thread::sleep_for(50ms);

  const auto updated = robot->GetPositionPIDGain(joint_name);
  std::cout << "[After]  [" << joint_name << "] P: " << updated.p_gain << ", I: " << updated.i_gain
            << ", D: " << updated.d_gain << std::endl;

  if (!robot->SetPositionPIDGain(joint_name, original)) {
    std::cerr << "Failed to restore PID gain for " << joint_name << std::endl;
    return false;
  }
  std::this_thread::sleep_for(50ms);

  const auto restored = robot->GetPositionPIDGain(joint_name);
  std::cout << "[Restored] [" << joint_name << "] P: " << restored.p_gain << ", I: " << restored.i_gain
            << ", D: " << restored.d_gain << std::endl;

  return true;
}


template <typename ModelT>
bool SetAndRestorePIDGain(const std::shared_ptr<Robot<ModelT>>& robot, const std::string& joint_name,
                          const PIDGain& new_gain) {
  std::cout << ">>> [START] " << joint_name << std::endl;
  const auto original = robot->GetPositionPIDGain(joint_name);
  std::cout << "[Before] [" << joint_name << "] P: " << original.p_gain << ", I: " << original.i_gain
            << ", D: " << original.d_gain << std::endl;

  if (!robot->SetPositionPIDGain(joint_name, new_gain)) {
    std::cerr << "Failed to set PID gain for " << joint_name << std::endl;
    return false;
  }
  std::this_thread::sleep_for(50ms);

  const auto updated = robot->GetPositionPIDGain(joint_name);
  std::cout << "[After]  [" << joint_name << "] P: " << updated.p_gain << ", I: " << updated.i_gain
            << ", D: " << updated.d_gain << std::endl;

  if (!robot->SetPositionPIDGain(joint_name, original)) {
    std::cerr << "Failed to restore PID gain for " << joint_name << std::endl;
    return false;
  }
  std::this_thread::sleep_for(50ms);

  const auto restored = robot->GetPositionPIDGain(joint_name);
  std::cout << "[Restored] [" << joint_name << "] P: " << restored.p_gain << ", I: " << restored.i_gain
            << ", D: " << restored.d_gain << std::endl;

  return true;
}

template <typename ModelT>
int Run(const std::string& address) {
  std::cout << "Creating robot with address='" << address << "'" << std::endl;
  auto robot = Robot<ModelT>::Create(address);
  robot->Connect();

  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(".*")) {
    std::cout << "Robot power is off. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(".*")) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  std::this_thread::sleep_for(500ms);

  if (!SetAndRestorePGain(robot, "right_arm_3", 100)) {
    std::cerr << "Failed on right_arm_3" << std::endl;
  }

  if (!SetAndRestorePIDGain(robot, "left_arm_3", 60, 10, 100)) {
    std::cerr << "Failed on left_arm_3" << std::endl;
  }

  if (!SetAndRestorePIDGain(robot, "head_0", PIDGain{700, 0, 3500})) {
    std::cerr << "Failed on head_0" << std::endl;
  }

  if (!SetAndRestorePGain(robot, "head_1", 300)) {
    std::cerr << "Failed on head_1" << std::endl;
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

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
