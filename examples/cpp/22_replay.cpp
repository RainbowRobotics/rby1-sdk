// Replay Demo
// This example demonstrates how to replay the robot's positions. See --help for arguments.
//
// Usage example:
//     ./example_22_replay --address 192.168.30.1:50051 --model a
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
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
}

bool LoadRecording(const std::string& filename, std::vector<std::vector<double>>& traj) {
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    std::cerr << "Failed to open " << filename << std::endl;
    return false;
  }

  uint64_t num_frames = 0;
  ifs.read(reinterpret_cast<char*>(&num_frames), sizeof(num_frames));
  if (num_frames == 0) {
    std::cerr << "No frames in recording" << std::endl;
    return false;
  }

  uint64_t dof = 0;
  ifs.read(reinterpret_cast<char*>(&dof), sizeof(dof));

  traj.resize(num_frames);
  for (uint64_t i = 0; i < num_frames; ++i) {
    traj[i].resize(dof);
    ifs.read(reinterpret_cast<char*>(traj[i].data()), static_cast<std::streamsize>(sizeof(double) * dof));
  }

  std::cout << "Loaded " << num_frames << " frames (dof=" << dof << ") from " << filename << std::endl;
  return true;
}

template <typename ModelT>
bool InitializeRobot(const std::shared_ptr<Robot<ModelT>>& robot) {
  if (!robot->IsPowerOn(".*")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(".*")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return false;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  if (!robot->IsServoOn(".*")) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn(".*")) {
      std::cerr << "Error: Failed to activate servo." << std::endl;
      return false;
    }
    std::cout << "Servo activated successfully." << std::endl;
  } else {
    std::cout << "Servo is already ON." << std::endl;
  }

  const auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
      return false;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }

  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    return false;
  }

  return true;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);
  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return 1;
  }

  if (!InitializeRobot(robot)) {
    return 1;
  }

  std::vector<std::vector<double>> saved_traj;
  if (!LoadRecording("recorded.bin", saved_traj)) {
    return 1;
  }

  // Extract body joint indices
  std::vector<unsigned int> body_indices;
  body_indices.insert(body_indices.end(), ModelT::kTorsoIdx.begin(), ModelT::kTorsoIdx.end());
  body_indices.insert(body_indices.end(), ModelT::kRightArmIdx.begin(), ModelT::kRightArmIdx.end());
  body_indices.insert(body_indices.end(), ModelT::kLeftArmIdx.begin(), ModelT::kLeftArmIdx.end());

  auto stream = robot->CreateCommandStream(10);

  for (size_t i = 0; i < saved_traj.size(); ++i) {
    double dt = (i == 0) ? 5.0 : 0.1;

    // Extract body joints from full position vector
    Eigen::VectorXd q_body(static_cast<Eigen::Index>(body_indices.size()));
    for (size_t j = 0; j < body_indices.size(); ++j) {
      q_body(static_cast<Eigen::Index>(j)) = saved_traj[i][body_indices[j]];
    }

    RobotCommandBuilder rc;
    rc.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
        JointPositionCommandBuilder()
            .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.0))
            .SetMinimumTime(dt)
            .SetPosition(q_body)));

    stream->SendCommand(rc);
    std::this_thread::sleep_for(std::chrono::duration<double>(dt * 0.95));
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
