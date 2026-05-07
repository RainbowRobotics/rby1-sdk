// Log Example
// This example connects to the robot, retrieves the most recent log entries,
// and prints them to the console.
//
// Usage example:
//   ./example_15_log --address 192.168.30.1:50051 --model a --num-entries 10
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

#include "rby1-sdk/log.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [-n|--num-entries N]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [num_entries]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, unsigned int num_entries) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  const auto logs = robot->GetLastLog(num_entries);
  for (const auto& log : logs) {
    std::cout << log << std::endl;
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  unsigned int num_entries = 10;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if ((arg == "-n" || arg == "--num-entries") && i + 1 < argc) {
      num_entries = static_cast<unsigned int>(std::stoul(argv[++i]));
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (num_entries == 10) {
      num_entries = static_cast<unsigned int>(std::stoul(arg));
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
    return Run<y1_model::A>(address, num_entries);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, num_entries);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
