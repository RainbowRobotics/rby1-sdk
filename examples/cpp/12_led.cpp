// LED Demo
// This example demonstrates how to control the LED of the robot.
//
// Usage example:
//   ./example_12_led --address 127.0.0.1:50051 --model a
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

  std::cout << "#1 Red 0.5s" << std::endl;
  robot->SetLEDColor(Color(255, 0, 0), 0.5, 0.1, false);
  std::this_thread::sleep_for(500ms);

  std::cout << "#2 Green 0.5s" << std::endl;
  robot->SetLEDColor(Color(0, 255, 0), 0.5, 0.1, false);
  std::this_thread::sleep_for(500ms);

  std::cout << "#3 Blue 0.5s" << std::endl;
  robot->SetLEDColor(Color(0, 0, 255), 0.5, 0.1, false);
  std::this_thread::sleep_for(500ms);

  std::cout << "#4 White Blinking 1s" << std::endl;
  robot->SetLEDColor(Color(200, 200, 200), 1.0, 0.1, true, 4.0);
  std::this_thread::sleep_for(1s);

  // Rainbow colors
  std::vector<Color> rainbow_colors = {
      Color(255, 0, 0),    // Red
      Color(255, 127, 0),  // Orange
      Color(255, 255, 0),  // Yellow
      Color(0, 255, 0),    // Green
      Color(0, 0, 255),    // Blue
      Color(75, 0, 130),   // Indigo
      Color(148, 0, 211),  // Violet
  };

  std::cout << "#5 Rainbow" << std::endl;
  for (const auto& color : rainbow_colors) {
    robot->SetLEDColor(color, 0.5, 0.2, false);
    std::this_thread::sleep_for(500ms);
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
