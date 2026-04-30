// Parameter Demo
// This example demonstrates how to get & set the robot's parameter. See --help for arguments.
//
// Usage example:
//     ./example_04_parameter --address 192.168.30.1:50051 --model a
//
// Scenario
// 1. Get parameter list
// 2. Get parameter value
// 3. Reset parameter to default value
// 4. Get parameter value
// 5. Set parameter value
// 6. Get parameter value
// 7. Set invalid parameter value
// 8. Get parameter value
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

#include <iostream>
#include <string>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

constexpr const char* kParameterName = "joint_position_command.cutoff_frequency";

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

template <typename ModelT>
int Parameter(const std::string& address) {

  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  auto parameter_list = robot->GetParameterList();
  for (const auto& parameter : parameter_list) {
    std::cout << parameter.first << ", " << parameter.second << std::endl;
  }
  std::cout << "------------------" << std::endl;

  std::cout << "Current parameter value" << std::endl;
  std::cout << kParameterName << ": " << robot->GetParameter(kParameterName) << std::endl;
  std::cout << "------------------" << std::endl;

  // Reset
  robot->FactoryResetParameter(kParameterName);

  // Get
  std::cout << "Parameter value after reset" << std::endl;
  std::cout << kParameterName << ": " << robot->GetParameter(kParameterName) << std::endl;
  std::cout << "------------------" << std::endl;

  // Set
  bool rv = robot->SetParameter(kParameterName, "1");
  std::cout << "Parameter value after set valid value" << std::endl;
  std::cout << "Set parameter result: " << (rv ? "true" : "false") << std::endl;

  // Get
  std::cout << kParameterName << ": " << robot->GetParameter(kParameterName) << std::endl;
  std::cout << "------------------" << std::endl;

  // Set invalid value
  rv = robot->SetParameter(kParameterName, "1000");
  std::cout << "Parameter value after set invalid value" << std::endl;
  std::cout << "Set parameter result: " << (rv ? "true" : "false") << std::endl;

  // Get
  std::cout << kParameterName << ": " << robot->GetParameter(kParameterName) << std::endl;
  std::cout << "------------------" << std::endl;

  return 0;
}

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
    return Parameter<y1_model::A>(address);
  }
  if (model == "m") {
    return Parameter<y1_model::M>(address);
  }
 

}
