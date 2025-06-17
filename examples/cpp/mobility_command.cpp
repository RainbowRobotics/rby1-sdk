#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

#define D2R 0.017453
#define R2D 57.296

const std::string kAll = ".*";

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

  std::cout << "Starting state update..." << std::endl;

  robot->StartStateUpdate(
      [](const auto& state) {
        std::cout << "State Update Received:" << std::endl;
        std::cout << "  Timestamp: " << state.timestamp.tv_sec << ".";
        std::cout << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
        std::cout << "  wasit [deg]     : " << state.position.block(2, 0, 6, 1).transpose() * R2D << std::endl;
        std::cout << "  right arm [deg] : " << state.position.block(2 + 6, 0, 7, 1).transpose() * R2D << std::endl;
        std::cout << "  left arm [deg]  : " << state.position.block(2 + 6 + 7, 0, 7, 1).transpose() * R2D << std::endl;
        std::cout << " odometry : \n" << state.odometry << std::endl;
      },
      1.0 /* Hz */);

  std::cout << "!!\n";

  std::this_thread::sleep_for(1s);

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

  robot->SetParameter("joint_position_command.cutoff_frequency", "5");
  std::cout << robot->GetParameter("joint_position_command.cutoff_frequency") << std::endl;

  robot->SetParameter("default.acceleration_limit_scaling", "0.8");

  std::this_thread::sleep_for(1s);

  if (1) {

    std::cout << "mobility command example 1\n";

    Eigen::Vector<double, 2> wheel_velocity;
    wheel_velocity << 1.5, 1.5;

    Eigen::Vector<double, 2> wheel_acceleration_limit;
    wheel_acceleration_limit << 100, 100;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetMobilityCommand(
                      MobilityCommandBuilder().SetCommand(JointVelocityCommandBuilder()
                                                              .SetVelocity(wheel_velocity)
                                                              .SetMinimumTime(10)
                                                              .SetAccelerationLimit(wheel_acceleration_limit)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (0) {

    std::cout << "mobility command example 2\n";

    Eigen::Vector<double, 2> wheel_velocity;
    wheel_velocity << 31.41592, -31.41592;

    Eigen::Vector<double, 2> wheel_acceleration_limit;
    wheel_acceleration_limit << 100, 100;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetMobilityCommand(
                      MobilityCommandBuilder().SetCommand(JointVelocityCommandBuilder()
                                                              .SetVelocity(wheel_velocity)
                                                              .SetMinimumTime(10)
                                                              .SetAccelerationLimit(wheel_acceleration_limit)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }
  if (0) {

    std::cout << "mobility command example 1\n";

    Eigen::Vector<double, 2> wheel_velocity;
    wheel_velocity << 31.41592, 31.41592;

    Eigen::Vector<double, 2> wheel_acceleration_limit;
    wheel_acceleration_limit << 100, 100;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetMobilityCommand(
                      MobilityCommandBuilder().SetCommand(JointVelocityCommandBuilder()
                                                              .SetVelocity(wheel_velocity)
                                                              .SetMinimumTime(10)
                                                              .SetAccelerationLimit(wheel_acceleration_limit)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (0) {

    std::cout << "mobility command example 3\n";

    double angular_velocity = 0.;
    Eigen::Vector<double, 2> linear_velocity;
    linear_velocity << 1, 0;

    double angular_acceleration_limit = 100.;
    Eigen::Vector<double, 2> linear_acceleration_limit;
    linear_acceleration_limit << 10, 100;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetMobilityCommand(MobilityCommandBuilder().SetCommand(
                          SE2VelocityCommandBuilder()
                              .SetVelocity(linear_velocity, angular_velocity)
                              .SetMinimumTime(10)
                              .SetAccelerationLimit(linear_acceleration_limit, angular_acceleration_limit)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  std::cout << "end of demo\n";

  return 0;
}
