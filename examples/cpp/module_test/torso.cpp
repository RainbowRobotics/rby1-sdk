#include <chrono>
#include <iostream>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

#define D2R 0.017453
#define R2D 57.296

const std::string kAll = "^((?!wheel|right|left|head).)*";
static constexpr int kDOF = 6;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};

  auto robot = Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn(kAll)) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(kAll)) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  std::cout << "Checking servo status..." << std::endl;
  if (!robot->IsServoOn(kAll)) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn(kAll)) {
      std::cerr << "Error: Failed to activate servo." << std::endl;
      return 1;
    }
    std::cout << "Servo activated successfully." << std::endl;
  } else {
    std::cout << "Servo is already ON." << std::endl;
  }

  const auto& control_manager_state = robot->GetControlManagerState();
  if (control_manager_state.state == ControlManagerState::State::kMajorFault ||
      control_manager_state.state == ControlManagerState::State::kMinorFault) {
    std::cerr << "Warning: Detected a "
              << (control_manager_state.state == ControlManagerState::State::kMajorFault ? "Major" : "Minor")
              << " Fault in the Control Manager." << std::endl;

    std::cout << "Attempting to reset the fault..." << std::endl;
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
      return 1;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }
  std::cout << "Control Manager state is normal. No faults detected." << std::endl;

  std::cout << "Enabling the Control Manager..." << std::endl;
  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    return 1;
  }
  std::cout << "Control Manager enabled successfully." << std::endl;

  robot->SetParameter("default.acceleration_limit_scaling", "0.8");
  robot->SetParameter("joint_position_command.cutoff_frequency", "5");
  robot->SetParameter("cartesian_command.cutoff_frequency", "5");
  robot->SetParameter("default.linear_acceleration_limit", "5");

  std::this_thread::sleep_for(1s);

  Eigen::Vector<double, kDOF> q_joint;
  Eigen::Vector<double, kDOF> q_joint_target;

  q_joint.setZero();

  double minimum_time = 3.;
  double moving_angle = 1.;

  q_joint_target = robot->GetState().position.block(2, 0, kDOF, 1) +
                   Eigen::Vector<double, kDOF>::Constant(moving_angle * 3.141592 / 180.);

  {
    // go to ready position
    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder().SetTorsoCommand(
                          JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_target)))))
                  ->Get();
    std::this_thread::sleep_for(1s);

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to move ." << std::endl;

      return 1;
    }
  }

  q_joint = robot->GetState().position.block(2, 0, kDOF, 1);

  if ((q_joint_target - q_joint).norm() < 1e-3) {
    std::cout << "OK\n";
  } else {
    for (int i = 0; i < kDOF; i++) {
      if (fabs(q_joint_target(i) - q_joint(i)) < 1e-3) {
        std::cout << "joint[" << i << "] fail to move\n";
      }
    }
  }

  q_joint_target = robot->GetState().position.block(2, 0, kDOF, 1) +
                   Eigen::Vector<double, kDOF>::Constant(-moving_angle * 3.141592 / 180.);

  {
    // go to ready position
    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder().SetTorsoCommand(
                          JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_target)))))
                  ->Get();
    std::this_thread::sleep_for(1s);

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to move ." << std::endl;

      return 1;
    }
  }

  q_joint = robot->GetState().position.block(2, 0, kDOF, 1);

  if ((q_joint_target - q_joint).norm() < 1e-3) {
    std::cout << "OK\n";
  } else {
    for (int i = 0; i < kDOF; i++) {
      if (fabs(q_joint_target(i) - q_joint(i)) < 1e-3) {
        std::cout << "joint[" << i << "] fail to move\n";
      }
    }
  }

  return 0;
}
