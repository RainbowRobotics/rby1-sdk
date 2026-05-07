// Set System Time Example
//
// This example includes the following features:
// 1. Reading the robot's current system time and timezone.
// 2. Setting the robot system time with an explicit timezone string.
// 3. Setting the robot system time using a timezone-aware datetime object. See --help for arguments.
// Note: This example is not supported in simulation.
//
// Usage example:
//   ./example_13_set_system_time --address 192.168.30.1:50051 --model a
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
#include <tuple>

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

struct timespec NowTimespecUTC() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto sec = time_point_cast<seconds>(now);
  const auto nsec = duration_cast<nanoseconds>(now - sec);
  struct timespec ts {};
  ts.tv_sec = static_cast<time_t>(sec.time_since_epoch().count());
  ts.tv_nsec = static_cast<long>(nsec.count());
  return ts;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);
  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Failed to connect robot" << std::endl;
    return 1;
  }

  const auto [utc_time, tz, localtime_string] = robot->GetSystemTime();
  std::cout << "Robot System Time: " << utc_time.tv_sec << "." << utc_time.tv_nsec << ", " << localtime_string
            << " (" << tz << ")" << std::endl;

  std::cout << "# Change To TimeZone(EST)" << std::endl;
  const bool ok_est = robot->SetSystemTime(NowTimespecUTC(), "EST");
  std::cout << " -- " << (ok_est ? "SUCCESS" : "FAIL") << std::endl;
  std::this_thread::sleep_for(500ms);
  {
    const auto [t2, tz2, local2] = robot->GetSystemTime();
    std::cout << "Robot System Time: " << t2.tv_sec << "." << t2.tv_nsec << ", " << local2 << " (" << tz2 << ")"
              << std::endl;
  }

  std::cout << "# Change to TimeZone" << std::endl;
  const bool ok_seoul = robot->SetSystemTime(utc_time, "Asia/Seoul");
  std::cout << " -- " << (ok_seoul ? "SUCCESS" : "FAIL") << std::endl;
  std::this_thread::sleep_for(500ms);
  {
    const auto [t3, tz3, local3] = robot->GetSystemTime();
    std::cout << "Robot System Time: " << t3.tv_sec << "." << t3.tv_nsec << ", " << local3 << " (" << tz3 << ")"
              << std::endl;
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
