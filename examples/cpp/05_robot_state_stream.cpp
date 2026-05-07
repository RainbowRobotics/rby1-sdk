// Robot State Stream Demo
// This example demonstrates how to get robot state via stream.
//
// Usage example:
//   ./example_05_robot_state_stream --address 192.168.30.1:50051 --model a --power ".*"
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

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};

void SignalHandler(int) { g_stop = true; }

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power) {
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }
  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  // Start state update stream at 10 Hz (same as Python: rate=10)
  robot->StartStateUpdate(
      [](const RobotState<ModelT>& state, const ControlManagerState& cm) {
        std::cout << "Timestamp: " << state.timestamp.tv_sec << "."
                  << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
        std::cout << "  Position: " << state.position.transpose() << std::endl;
        std::cout << "  Velocity: " << state.velocity.transpose() << std::endl;
        std::cout << "Control Manager State: " << rb::to_string(cm.state) << std::endl;
        for (std::size_t i = 0; i < state.joint_states.size(); ++i) {
          std::cout << "  Joint[" << i << "] Temperature: " << state.joint_states[i].temperature << std::endl;
        }
        std::cout << std::endl;
      },
      10);

  // Sleep until interrupted (Ctrl+C)
  auto deadline = std::chrono::steady_clock::now() + 100s;
  while (!g_stop && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(100ms);
  }

  if (g_stop) {
    std::cout << "Stopping state update..." << std::endl;
  }
  robot->StopStateUpdate();

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);

  std::string address;
  std::string model = "a";
  std::string power = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
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
    return Run<y1_model::A>(address, power);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
