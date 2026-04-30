//  Helper Functions
//  This file contains helper functions for the examples.
//  1. initialize_robot: poweron, servo on, enable control manager.
//  2. movej: move robot to joint position.

//  Copyright (c) 2025 Rainbow Robotics. All rights reserved.

//  DISCLAIMER:
//  This is a sample code provided for educational and reference purposes only.
//  Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
//  the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;

template <typename ModelT>
std::shared_ptr<Robot<ModelT>> InitializeRobot(const std::string& address, const std::string& power = ".*",
                                               const std::string& servo = ".*") {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return nullptr;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return nullptr;
  }

  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to turn power (" << power << ") on" << std::endl;
      return nullptr;
    }
  }

  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Failed to servo (" << servo << ") on" << std::endl;
      return nullptr;
    }
  }

  const auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Failed to reset control manager" << std::endl;
      return nullptr;
    }
  }

  if (!robot->EnableControlManager()) {
    std::cerr << "Failed to enable control manager" << std::endl;
    return nullptr;
  }

  return robot;
}

template <typename ModelT>
bool MoveJ(const std::shared_ptr<Robot<ModelT>>& robot, const Eigen::VectorXd* torso = nullptr,
           const Eigen::VectorXd* right_arm = nullptr, const Eigen::VectorXd* left_arm = nullptr,
           double minimum_time = 0.0) {
  BodyComponentBasedCommandBuilder body_builder;

  if (torso != nullptr) {
    body_builder.SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(*torso));
  }
  if (right_arm != nullptr) {
    body_builder.SetRightArmCommand(
        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(*right_arm));
  }
  if (left_arm != nullptr) {
    body_builder.SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(*left_arm));
  }

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(body_builder)),
                              1)
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Failed to conduct movej." << std::endl;
    return false;
  }

  return true;
}
