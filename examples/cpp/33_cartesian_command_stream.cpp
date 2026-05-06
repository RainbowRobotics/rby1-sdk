// ################### CAUTION ###################
// # CAUTION:
// # Ensure that the robot has enough surrounding clearance before running this example.
// ###############################################

// Cartesian Command Stream Demo
// This example brings the robot to a pre-control pose, moves to a Cartesian ready pose,
// and then streams right-arm Cartesian targets while monitoring feedback. See --help for arguments.
//
// Usage example:
//     ./example_32_cartesian_command_stream --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
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
#ifdef _WIN32
#  define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/math/liegroup.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"
#include "rby1-sdk/robot_command_feedback.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]" << std::endl;
}

constexpr double kDeg2Rad = M_PI / 180.0;

template <typename ModelT>
bool MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::Vector<double, 6> torso;
  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  Eigen::Vector<double, 7> right_arm;
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  Eigen::Vector<double, 7> left_arm;
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  auto rv = robot
                ->SendCommand(
                    RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                        BodyComponentBasedCommandBuilder()
                            .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                            .SetRightArmCommand(
                                JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                            .SetLeftArmCommand(
                                JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                    90)
                ->Get();

  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  return rv.finish_code() == RobotCommandFeedback::FinishCode::kOk;
}

template <typename ModelT>
bool MoveToReadyPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::Vector<double, 7> right_arm;
  right_arm << 0.0, -5.0 * kDeg2Rad, 0.0, -120.0 * kDeg2Rad, 0.0, 40.0 * kDeg2Rad, 0.0;
  Eigen::Vector<double, 7> left_arm;
  left_arm << 0.0, 5.0 * kDeg2Rad, 0.0, -120.0 * kDeg2Rad, 0.0, 40.0 * kDeg2Rad, 0.0;

  BodyComponentBasedCommandBuilder body_cmd;
  if constexpr (ModelT::kTorsoIdx.size() == 6) {
    Eigen::Vector<double, 6> torso;
    torso << 0.0, 45.0 * kDeg2Rad, -90.0 * kDeg2Rad, 45.0 * kDeg2Rad, 0.0, 0.0;
    body_cmd.SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(4.0).SetPosition(torso));
  }
  body_cmd.SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(4.0).SetPosition(right_arm));
  body_cmd.SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(4.0).SetPosition(left_arm));

  auto rv = robot
                ->SendCommand(
                    RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(body_cmd)),
                    10)
                ->Get();

  std::cout << "ready pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  return rv.finish_code() == RobotCommandFeedback::FinishCode::kOk;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  auto robot = Robot<ModelT>::Create(address);
  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to turn power (" << power << ") on" << std::endl;
      return 1;
    }
  }

  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Failed to servo (" << servo << ") on" << std::endl;
      return 1;
    }
  }

  const auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Failed to reset control manager" << std::endl;
      return 1;
    }
  }

  if (!robot->EnableControlManager()) {
    std::cerr << "Failed to enable control manager" << std::endl;
    return 1;
  }

  std::cout << "===== Cartesian Command Stream Example =====" << std::endl;

  std::cout << std::boolalpha << robot->SetParameter("cartesian_command.cutoff_frequency", "5") << std::endl;

  if (!MoveToPreControlPose(robot)) {
    std::cerr << "Failed to move to pre-control pose" << std::endl;
    return 1;
  }

  if (!MoveToReadyPose(robot)) {
    std::cerr << "Failed to move to ready pose" << std::endl;
    return 1;
  }

  // Compute forward kinematics for the reference transform
  auto dyn_robot = robot->GetDynamics();
  std::vector<std::string> link_names = {"base", "ee_right"};
  std::vector<std::string> joint_names(ModelT::kRobotJointNames.begin(), ModelT::kRobotJointNames.end());
  auto dyn_state = dyn_robot->MakeState(link_names, joint_names);
  constexpr unsigned int kBaseLinkIdx = 0;
  constexpr unsigned int kEeRightLinkIdx = 1;

  dyn_state->SetQ(robot->GetState().position);
  dyn_robot->ComputeForwardKinematics(dyn_state);
  Eigen::Matrix4d T_ref = dyn_robot->ComputeTransformation(dyn_state, kBaseLinkIdx, kEeRightLinkIdx);

  auto stream = robot->CreateCommandStream();

  std::atomic<bool> stop_requested{false};
  std::signal(SIGINT, [](int) { std::exit(1); });

  // Position offsets to visit
  const double pos_diffs[][3] = {
      {0, 0, -0.08},      {0, 0, 0.08},        {0.1, -0.16, 0.08}, {0.1, -0.16, -0.08},
      {0, 0, -0.08},      {0, 0, 0.08},         {0, -0.2, 0.1},     {0, -0.2, -0.1},
      {-0.05, 0.1, -0.08},{-0.05, 0.1, 0.08},   {0, -0.2, 0.1},     {0, -0.2, -0.1},
      {0, 0, -0.08},      {0, 0, 0.08},         {0, -0.16, 0.08},   {0, -0.16, -0.08},
      {0, 0, -0.08},      {0, 0, 0.08},
  };

  for (const auto& diff : pos_diffs) {
    Eigen::Matrix4d target = T_ref;
    target(0, 3) += diff[0];
    target(1, 3) += diff[1];
    target(2, 3) += diff[2];

    RobotCommandBuilder rc;
    rc.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
        BodyComponentBasedCommandBuilder().SetRightArmCommand(
            CartesianCommandBuilder()
                .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1e6))
                .AddJointPositionTarget("right_arm_2", 0.5, 1, 100)
                .AddTarget("base", "ee_right", target, 0.3, 100.0, 0.8)
                .SetMinimumTime(2))));

    stream->SendCommand(rc);

    int log_count = 0;
    while (true) {
      auto feedback = stream->RequestFeedback();
      const auto& cartesian_fb =
          feedback.component_based_command().body_command().body_component_based_command().right_arm_command().cartesian_command();

      if (log_count % 100 == 0) {
        if (!cartesian_fb.se3_pose_tracking_errors().empty()) {
          std::cout << "position error: " << cartesian_fb.se3_pose_tracking_errors()[0].position_error
                    << ", manipulability: " << cartesian_fb.manipulability() << std::endl;
        }
      }
      ++log_count;

      if (!cartesian_fb.se3_pose_tracking_errors().empty() &&
          cartesian_fb.se3_pose_tracking_errors()[0].position_error < 1e-2) {
        break;
      }
    }
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";
  std::string servo = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg == "--servo" && i + 1 < argc) {
      servo = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
    } else if (servo == ".*") {
      servo = arg;
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
    return Run<y1_model::A>(address, power, servo);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power, servo);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power, servo);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
