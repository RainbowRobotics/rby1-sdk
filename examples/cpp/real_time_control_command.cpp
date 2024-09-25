#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <thread>

#include "rby1-sdk/model.h"
#include "rby1-sdk/net/real_time_control_protocol.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

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
  if (!robot->IsPowerOn(".*")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(".*")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  std::cout << "Checking servo status..." << std::endl;
  if (!robot->IsServoOn("^(?!.*wheel).*")) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn("^(?!.*wheel).*")) {
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

  {
    Eigen::Vector<double, 6> q_joint_waist;
    Eigen::Vector<double, 7> q_joint_right_arm;
    Eigen::Vector<double, 7> q_joint_left_arm;

    q_joint_waist << 0, 30, -60, 30, 0, 0;
    q_joint_right_arm << -45, -30, 0, -90, 0, 45, 0;
    q_joint_left_arm << -45, 30, 0, -90, 0, 45, 0;

    Eigen::Vector<double, 20> q;
    q.block(0, 0, 6, 1) = q_joint_waist;
    q.block(6, 0, 7, 1) = q_joint_right_arm;
    q.block(6 + 7, 0, 7, 1) = q_joint_left_arm;

    q = q * 3.141592 / 180.;

    JointPositionCommandBuilder joint_position_command;
    joint_position_command.SetPosition(q).SetMinimumTime(5.0);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(joint_position_command)))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  auto dyn = robot->GetDynamics();
  auto dyn_state = dyn->MakeState({"base", "ee_right"}, y1_model::A::kRobotJointNames);

  int count = 0;

  auto rv = robot->Control(
      [&](const auto& state) {
        ControlInput<y1_model::A> input;

        {
          auto now = std::chrono::system_clock::now();
          auto now_time_t = std::chrono::system_clock::to_time_t(now);
          auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

          std::tm* now_tm = std::localtime(&now_time_t);

          std::cout << "Current time: ";
          std::cout << std::put_time(now_tm, "%H:%M:%S");
          std::cout << "." << std::setfill('0') << std::setw(6) << now_us.count();
          std::cout << std::endl;
        }

        std::cout << "time: " << state.t << std::endl;
        //        std::cout << "is_ready: " << state.is_ready.transpose() << std::endl;
        std::cout << "position: " << state.position.transpose() << std::endl;
        //        std::cout << "velocity: " << state.velocity.transpose() << std::endl;
        //        std::cout << "current: " << state.current.transpose() << std::endl;
        //        std::cout << "torque: " << state.torque.transpose() << std::endl;

        dyn_state->SetQ(state.position);

        dyn->ComputeForwardKinematics(dyn_state);

        // Calculate transformation from base to ee_right
        const auto& T = dyn->ComputeTransformation(dyn_state, 0, 1);
        std::cout << R"(T from "base" to "ee_right": )" << std::endl;
        std::cout << T << std::endl;

        // Calculate body jacobian from base to ee_right
        const auto& J = dyn->ComputeBodyJacobian(dyn_state, 0, 1);
        std::cout << R"(J_body from "base" to "ee_right": )" << std::endl;
        std::cout << J << std::endl;

        std::cout << std::endl;

        // Make control input
        input.mode.setConstant(kPositionControlMode);
        input.target = state.position * 0.9;
        input.feedback_gain.setConstant(2);
        input.feedforward_torque.setConstant(0);

        input.finish = (count++) > 2000;
        return input;
      },
      0, 10);

  std::cout << "Control Result: " << std::boolalpha << rv << std::endl;

  return 0;
}