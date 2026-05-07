// Record Demo
// This example demonstrates how to record the robot's positions. See --help for arguments.
//
// Usage example:
//     ./example_21_record --address 192.168.30.1:50051 --model a
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
#include <fstream>
#include <iostream>
#include <mutex>
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

std::atomic<bool> g_stop_requested{false};
std::mutex g_mutex;
std::vector<std::vector<double>> g_recorded_traj;

void SignalHandler(int) { g_stop_requested.store(true); }

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
}

void SaveRecording(const std::string& filename) {
  std::lock_guard<std::mutex> lock(g_mutex);
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs) {
    std::cerr << "Failed to open " << filename << " for writing" << std::endl;
    return;
  }

  uint64_t num_frames = g_recorded_traj.size();
  ofs.write(reinterpret_cast<const char*>(&num_frames), sizeof(num_frames));
  if (num_frames > 0) {
    uint64_t dof = g_recorded_traj[0].size();
    ofs.write(reinterpret_cast<const char*>(&dof), sizeof(dof));
    for (const auto& frame : g_recorded_traj) {
      ofs.write(reinterpret_cast<const char*>(frame.data()), static_cast<std::streamsize>(sizeof(double) * dof));
    }
  }

  std::cout << "Recording stopped and data saved to '" << filename << "'. (" << num_frames << " frames)" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);
  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(".*")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(".*")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  std::signal(SIGINT, SignalHandler);

  std::cout << "Recording started..." << std::endl;

  robot->StartStateUpdate(
      [](const RobotState<ModelT>& state) {
        std::lock_guard<std::mutex> lock(g_mutex);
        std::vector<double> frame(state.position.data(), state.position.data() + state.position.size());
        g_recorded_traj.push_back(std::move(frame));
        std::cout << "---" << std::endl;
        std::cout << "target position: " << state.target_position.transpose() << std::endl;
      },
      10);

  while (!g_stop_requested.load()) {
    std::this_thread::sleep_for(100ms);
  }

  SaveRecording("recorded.bin");
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
