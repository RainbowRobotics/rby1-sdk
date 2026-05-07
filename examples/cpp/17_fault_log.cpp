// Fault Log Example
//
// This example connects to the robot, prints the available fault log files, and downloads the first fault
// log as ``fault.csv`` when one is available.
//
// Usage example:
//   ./example_17_fault_log --address 192.168.30.1:50051 --model a
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <string>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

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
    std::cerr << "Error: Robot connection failed." << std::endl;
    return 1;
  }

  const auto fault_log_list = robot->GetFaultLogList();
  std::cout << "Fault log list: [";
  for (size_t i = 0; i < fault_log_list.size(); ++i) {
    if (i) {
      std::cout << ", ";
    }
    std::cout << fault_log_list[i];
  }
  std::cout << "]" << std::endl;

  if (fault_log_list.empty()) {
    std::cout << "No fault logs found." << std::endl;
    return 0;
  }

  std::cout << "Downloading fault log: " << fault_log_list[0] << std::endl;
  std::ofstream out("fault.csv", std::ios::binary);
  if (!out) {
    std::cerr << "Failed to open output file fault.csv" << std::endl;
    return 1;
  }

  if (!robot->DownloadFile(fault_log_list[0], out)) {
    std::cerr << "Failed to download fault log." << std::endl;
    return 1;
  }

  std::cout << "Fault log downloaded successfully." << std::endl;
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
