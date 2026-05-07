// Tool Flange State Example
// This example connects to the robot, powers on the 48V tool flange supply if needed,
// subscribes to robot state updates, and prints the left and right tool flange states.
//
//
// Usage example:
//   ./example_06_state_tool_flange --address 192.168.30.1:50051 --model a
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
#include <sstream>
#include <string>
#include <thread>

#include <Eigen/Dense>

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

std::string FormatVec3(const Eigen::Vector3d& v) {
  Eigen::IOFormat fmt(3, 0, ", ", ", ", "[", "]", "[", "]");
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss << std::setprecision(3) << v.format(fmt);
  return ss.str();
}

std::string ToolFlangeStr(const ToolFlangeState& tf) {
  std::ostringstream ss;
  ss << "ToolFlangeState(gyro=" << FormatVec3(tf.gyro) << ", acc=" << FormatVec3(tf.acceleration)
     << ", switch_A=" << (tf.switch_A ? "True" : "False") << ", output_voltage=" << tf.output_voltage
     << ", dinA=" << (tf.digital_input_A ? "True" : "False")
     << ", dinB=" << (tf.digital_input_B ? "True" : "False")
     << ", doutA=" << (tf.digital_output_A ? "True" : "False")
     << ", doutB=" << (tf.digital_output_B ? "True" : "False") << ")";
  return ss.str();
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn("48v")) {
    if (!robot->PowerOn("48v")) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  robot->StartStateUpdate(
      [](const auto& state) {
        std::cout << "---" << std::endl;
        std::cout << "tool_flange_right = " << ToolFlangeStr(state.tool_flange_right) << std::endl;
        std::cout << "tool_flange_left  = " << ToolFlangeStr(state.tool_flange_left) << std::endl;
      },
      10.0);

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
