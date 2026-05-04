// ################################ Note ################################
// # This example does not run in simulation.
// ######################################################################
//
// Leader Arm Example
//
// This example powers on the UPC leader arm, initializes it with its URDF model, and runs a
// gravity-compensated current-control loop while printing the leader arm state.
// Note: This example is not supported in simulation.
//
// Usage example:
//   ./example_20_leader_arm_state_check --address 192.168.30.1:50051 --model a
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
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/leader_arm.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::function<void()> g_cleanup;

void SignalHandler(int) {
  if (g_cleanup) {
    g_cleanup();
  }
  std::_Exit(1);
}

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
    std::cerr << "Error: Robot connection failed." << std::endl;
    return 1;
  }

  if (!robot->PowerOn("12v")) {
    std::cerr << "Error: Failed to power on 12V." << std::endl;
    return 1;
  }

  auto leader_arm = std::make_shared<upc::LeaderArm>(upc::kLeaderArmDeviceName);

  g_cleanup = [robot, leader_arm]() {
    leader_arm->StopControl();
    robot->PowerOff("12v");
  };

  std::signal(SIGINT, SignalHandler);

  try {
    upc::InitializeDevice(upc::kLeaderArmDeviceName);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  leader_arm->SetModelPath(MODELS_PATH "/leader_arm/model.urdf");
  leader_arm->SetControlPeriod(0.01);
  const auto active_ids = leader_arm->Initialize(true);
  if (active_ids.size() != upc::LeaderArm::kDeivceCount) {
    std::cerr << "Error: Mismatch in the number of devices detected for RBY leader Arm." << std::endl;
    return 1;
  }

  leader_arm->StartControl([&](const upc::LeaderArm::State& state) {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local{};
#ifdef _WIN32
    localtime_s(&tm_local, &t);
#else
    localtime_r(&t, &tm_local);
#endif

    std::cout << "--- " << std::put_time(&tm_local, "%H:%M:%S") << " ---" << std::endl;
    std::cout << "q: " << state.q_joint.transpose() << std::endl;
    std::cout << "g: " << state.gravity_term.transpose() << std::endl;
    std::cout << "right: " << state.button_right.button << ", left: " << state.button_left.button << std::endl;

    upc::LeaderArm::ControlInput input;
    input.target_operating_mode.setConstant(DynamixelBus::kCurrentControlMode);
    input.target_torque = state.gravity_term;
    return input;
  });

  std::this_thread::sleep_for(100s);

  leader_arm->StopControl();
  robot->PowerOff("12v");

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
