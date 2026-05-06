// ############################## CAUTION ###############################
// # Please be careful if you are using modules that return data immediately upon connection,
// # such as sensors, as this can overload the RPC and cause problems.
// ######################################################################
// ################################ Note ################################
// # This example does not run in simulation.
// # This example does not apply when the control manager is enabled.
// ######################################################################
//
// Serial Communication Demo
// This example demonstrates how to setup serial communication with the robot.
// See --help for arguments.
//
// Usage example:
//     ./example_38_serial_communication --address 192.168.30.1:50051 --model a --device /dev/ttyUSB0 --baudrate 19200
//
// Default device_path: /dev/ttyUSB1, default baudrate: 19200
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.


#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m] [--device <path> | --device_path <path>] [--baudrate <rate>]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [device_path] [baudrate]" << std::endl;
}

bool ParseHexToBytes(const std::string& hex, std::vector<char>& out) {
  if (hex.empty() || (hex.size() % 2 != 0)) {
    return false;
  }

  out.clear();
  out.reserve(hex.size() / 2);
  for (size_t i = 0; i < hex.size(); i += 2) {
    unsigned int byte_val;
    std::istringstream iss(hex.substr(i, 2));
    if (!(iss >> std::hex >> byte_val)) {
      return false;
    }
    out.push_back(static_cast<char>(byte_val & 0xFF));
  }
  return true;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& device_path, int baudrate) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Error: Robot connection failed." << std::endl;
    return 1;
  }

  auto dev = robot->OpenSerialStream(device_path, baudrate, 8, 'N', 1);
  if (!dev->Connect(true)) {
    std::cerr << "Failed to connect serial device" << std::endl;
    return 1;
  }

  std::cout << "Listening for incoming data..." << std::endl;

  dev->SetReadCallback([](const std::string& data) {
    std::cout << "<< ";
    for (unsigned char ch : data) {
      std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch);
    }
    std::cout << std::dec << std::endl;
  });

  std::thread input_thread([&]() {
    std::string line;
    while (true) {
      std::cout << "\n>> Enter hex data to send (e.g., 'B7B8010401C5C6'), or type 'exit': " << std::flush;
      if (!std::getline(std::cin, line)) {
        break;
      }

      // Trim whitespace
      size_t start = line.find_first_not_of(" \t\r\n");
      if (start == std::string::npos) {
        continue;
      }
      line = line.substr(start, line.find_last_not_of(" \t\r\n") - start + 1);

      if (ToLower(line) == "exit") {
        break;
      }

      std::vector<char> bytes;
      if (!ParseHexToBytes(line, bytes)) {
        if (line.size() % 2 != 0) {
          std::cerr << "Hex string must have even number of characters." << std::endl;
        } else {
          std::cerr << "Invalid hex input. Please enter valid hexadecimal characters (0-9, A-F)." << std::endl;
        }
        continue;
      }

      if (!dev->Write(bytes.data(), static_cast<int>(bytes.size()))) {
        std::cerr << "Failed to write data." << std::endl;
      }
    }
  });

  input_thread.join();

  std::cout << "Disconnecting serial stream..." << std::endl;
  dev->Disconnect();
  std::cout << "Disconnected. Exiting." << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string device_path = "/dev/ttyUSB1";
  int baudrate = 19200;
  bool device_provided = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if ((arg == "--device" || arg == "--device_path") && i + 1 < argc) {
      device_path = argv[++i];
      device_provided = true;
    } else if (arg == "--baudrate" && i + 1 < argc) {
      baudrate = std::stoi(argv[++i]);
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (!device_provided) {
      device_path = arg;
      device_provided = true;
    } else {
      baudrate = std::stoi(arg);
    }
  }

  if (address.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return Run<y1_model::A>(address, device_path, baudrate);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, device_path, baudrate);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
