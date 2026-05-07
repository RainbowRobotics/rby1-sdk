// Note: This example does not run in simulation.
// WiFi Setup Demo
// This example demonstrates how to setup the robot's WiFi connection.
// After changing the IP, please check the OLED on the robot's backpack to confirm.
// It may take 1-2 minutes for the change to take effect.
//
// Usage example:
//   ./example_14_wifi --address 127.0.0.1:50051 --model a
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
#include <sstream>
#include <string>
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

  // 1. Scan WiFi networks
  std::cout << "Scanning Wi-Fi..." << std::endl;
  auto wifi_networks = robot->ScanWifi();
  std::cout << "Scanning completed!" << std::endl;

  if (wifi_networks.empty()) {
    std::cout << "No WiFi networks found." << std::endl;
    return 1;
  }

  // 2. Display available networks (same as Python)
  std::cout << "\nAvailable Wi-Fi networks:" << std::endl;
  for (size_t i = 0; i < wifi_networks.size(); ++i) {
    std::cout << "  " << (i + 1) << ". " << wifi_networks[i].ssid
              << " (Signal: " << wifi_networks[i].signal_strength
              << ", Secured: " << (wifi_networks[i].secured ? "Yes" : "No") << ")" << std::endl;
  }

  // 3. Select a network
  std::cout << "\nSelect a WiFi network (number): ";
  int selection = 0;
  std::cin >> selection;
  std::cin.ignore();

  if (selection < 1 || selection > static_cast<int>(wifi_networks.size())) {
    std::cerr << "Invalid selection." << std::endl;
    return 1;
  }
  const auto& selected = wifi_networks[selection - 1];
  std::cout << "Selected: " << selected.ssid << std::endl;

  // 4. Enter password if secured
  std::string password;
  if (selected.secured) {
    std::cout << "Enter password: ";
    std::getline(std::cin, password);
  }

  // 5. DHCP setting
  std::cout << "Use DHCP? (y/n): ";
  std::string dhcp_input;
  std::getline(std::cin, dhcp_input);
  bool use_dhcp = (ToLower(dhcp_input) == "y");

  std::string ip_address, gateway;
  std::vector<std::string> dns;

  if (!use_dhcp) {
    std::cout << "Enter IP address: ";
    std::getline(std::cin, ip_address);

    std::cout << "Enter Gateway: ";
    std::getline(std::cin, gateway);

    std::cout << "Enter DNS (comma separated): ";
    std::string dns_input;
    std::getline(std::cin, dns_input);
    std::istringstream iss(dns_input);
    std::string item;
    while (std::getline(iss, item, ',')) {
      // Trim whitespace
      auto start = item.find_first_not_of(' ');
      auto end = item.find_last_not_of(' ');
      if (start != std::string::npos) {
        dns.push_back(item.substr(start, end - start + 1));
      }
    }
  }

  // 6. Connect
  std::cout << "Wait for connecting to " << selected.ssid << "..." << std::endl;
  robot->ConnectWifi(selected.ssid, password, use_dhcp, ip_address, gateway, dns);
  std::cout << "Connected to " << selected.ssid << "! Check OLED on the robot's backpack." << std::endl;

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
