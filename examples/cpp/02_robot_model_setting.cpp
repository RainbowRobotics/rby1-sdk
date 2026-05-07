// Robot Model Setting Example

// This example includes the following features:
// 1. Loading the current robot model (in URDF format) from the robot.
// 2. Saving a custom robot model to the robot with a specified name.
// 3. Assigning the robot model name for the robot to use (applied after reboot).

// Note:
// If the modified URDF causes any issue, users can recover by downloading the
// proper official URDF for their robot model from the rby1-sdk repository below,
// parsing it in the same way as this example, and importing it again with a new model name:
// https://github.com/RainbowRobotics/rby1-sdk/tree/main/models

// If an issue arises while modifying the urdf again after the above step,
// You can skip the previous step by running only the robot.set_parameter() command.
//
// Usage example:
//   ./example_02_robot_model --address 192.168.30.1:50051 --model a
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

#include "tinyxml2.h"

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
    std::cerr << "Failed to connect robot" << std::endl;
    return 1;
  }

  // 1. Load current robot model from robot
  const std::string urdf_str = robot->GetRobotModel();
  std::cout << "Current robot model loaded." << std::endl;

  // 2. Parse URDF
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_str.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse robot model XML" << std::endl;
    return 1;
  }

  auto* root = doc.RootElement();
  if (!root) {
    std::cerr << "Invalid robot model XML (no root element)" << std::endl;
    return 1;
  }

  // 3. Modify ee_right mass
  const std::string target_link_name = "ee_right";
  constexpr double mass_delta = 0.5;
  bool modified = false;

  for (auto* link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
    const char* name = link->Attribute("name");
    if (name && std::string(name) == target_link_name) {
      auto* inertial = link->FirstChildElement("inertial");
      if (!inertial) {
        std::cerr << "[ERROR] link '" << target_link_name << "' has no <inertial> tag." << std::endl;
        return 1;
      }

      auto* mass_tag = inertial->FirstChildElement("mass");
      if (!mass_tag) {
        std::cerr << "[ERROR] link '" << target_link_name << "' has no <mass> tag." << std::endl;
        return 1;
      }

      double old_mass = 0.0;
      if (mass_tag->QueryDoubleAttribute("value", &old_mass) != tinyxml2::XML_SUCCESS) {
        std::cerr << "[ERROR] link '" << target_link_name << "' has invalid <mass value>." << std::endl;
        return 1;
      }

      const double new_mass = old_mass + mass_delta;
      std::ostringstream mass_ss;
      mass_ss << std::fixed << std::setprecision(8) << new_mass;
      mass_tag->SetAttribute("value", mass_ss.str().c_str());

      std::cout << std::fixed << std::setprecision(8) << "Modified '" << target_link_name
                << "' mass: " << old_mass << " -> " << new_mass << std::endl;
      modified = true;
      break;
    }
  }

  if (!modified) {
    std::cerr << "[ERROR] link '" << target_link_name << "' not found in URDF." << std::endl;
    return 1;
  }

  // 4. Upload modified model with a new name
  const std::string new_model_name = "temp_ee_right_mass_up";
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  const bool ok = robot->ImportRobotModel(new_model_name, printer.CStr());
  std::cout << "Import robot model: " << std::boolalpha << ok << std::endl;
  if (!ok) {
    std::cerr << "[ERROR] Failed to import modified robot model." << std::endl;
    return 1;
  }

  // 5. Set robot model name (applied after reboot)
  if (!robot->SetParameter("model_name", "\"" + new_model_name + "\"")) {
    std::cerr << "[ERROR] Failed to set model_name parameter." << std::endl;
    return 1;
  }
  std::cout << "Set model_name to '" << new_model_name << "'" << std::endl;
  std::cout << "The modified robot model will be applied after reboot." << std::endl;

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
