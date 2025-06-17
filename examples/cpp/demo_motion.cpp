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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const std::string kAll = ".*";

// const std::string kAll = "^(?!.*wheel$).*";

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address> [servo]" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  std::string servo = ".*";  // 기본값

  if (argc >= 3) {
    servo = argv[2];
  }

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
        std::cout << "  torso [deg]     : " << state.position.block(2, 0, 6, 1).transpose() * R2D << std::endl;
        std::cout << "  right arm [deg] : " << state.position.block(2 + 6, 0, 7, 1).transpose() * R2D << std::endl;
        std::cout << "  left arm [deg]  : " << state.position.block(2 + 6 + 7, 0, 7, 1).transpose() * R2D << std::endl;
      },
      0.1 /* Hz */);

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
  if (!robot->IsServoOn(servo)) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn(servo)) {
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

  try {
    if (robot->IsPowerOn("48v")) {
      robot->SetToolFlangeOutputVoltage("right", 12);
    }
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  robot->SetParameter("default.acceleration_limit_scaling", "0.8");
  robot->SetParameter("joint_position_command.cutoff_frequency", "5");
  robot->SetParameter("cartesian_command.cutoff_frequency", "5");
  robot->SetParameter("default.linear_acceleration_limit", "5");

  std::this_thread::sleep_for(1s);

  Eigen::Vector<double, 6> q_joint_torso;
  Eigen::Vector<double, 7> q_joint_right_arm;
  Eigen::Vector<double, 7> q_joint_left_arm;
  q_joint_torso.setZero();
  q_joint_right_arm.setZero();
  q_joint_left_arm.setZero();

  double minimum_time = 2.5;

  if (1) {
    std::cout << "joint position command example 1\n";
    q_joint_torso.setZero();
    q_joint_right_arm.setZero();
    q_joint_left_arm.setZero();

    q_joint_right_arm(1) = -90 * D2R;
    q_joint_left_arm(1) = 90 * D2R;

    // go to ready position
    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder()
                    .SetTorsoCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_torso))
                    .SetRightArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_right_arm))
                    .SetLeftArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_left_arm)))))
            ->Get();
    std::this_thread::sleep_for(1s);

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;

      return 1;
    }
  }

  if (1) {
    std::cout << "joint position command example 2\n";

    q_joint_torso << 0, 30, -60, 30, 0, 0;
    q_joint_torso *= D2R;

    q_joint_right_arm << -45, -30, 0, -90, 0, 45, 0;
    q_joint_right_arm *= D2R;

    q_joint_left_arm << -45, 30, 0, -90, 0, 45, 0;
    q_joint_left_arm *= D2R;

    Eigen::Vector<double, 20> q;
    q.block(0, 0, 6, 1) = q_joint_torso;
    q.block(6, 0, 7, 1) = q_joint_right_arm;
    q.block(6 + 7, 0, 7, 1) = q_joint_left_arm;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      JointPositionCommandBuilder().SetPosition(q).SetMinimumTime(minimum_time))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  double angular_velocity_limit = 3.141592 * 1.5;
  double linear_velocity_limit = 1.5;
  double acceleration_limit = 1.0;
  double stop_orientation_tracking_error = 1e-5;
  double stop_position_tracking_error = 1e-5;

  Eigen::Matrix<double, 4, 4> T_torso, T_right, T_left;
  T_torso.setIdentity();
  T_right.setIdentity();
  T_left.setIdentity();

  if (1) {
    std::cout << "Cartesian command example 1\n";

    T_torso.block(0, 0, 3, 3).setIdentity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      BodyComponentBasedCommandBuilder()
                          .SetTorsoCommand(CartesianCommandBuilder()
                                               .AddTarget("base", "link_torso_5", T_torso, linear_velocity_limit,
                                                          angular_velocity_limit, acceleration_limit / 2)
                                               .SetMinimumTime(minimum_time * 1)
                                               .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                               .SetStopPositionTrackingError(stop_position_tracking_error))
                          .SetRightArmCommand(CartesianCommandBuilder()
                                                  .AddTarget("base", "ee_right", T_right, linear_velocity_limit,
                                                             angular_velocity_limit, acceleration_limit / 2)
                                                  /* Need to be verified */
                                                  .AddJointPositionTarget("right_arm_2", -10 * M_PI / 180, 3.14, 6.28)
                                                  .SetMinimumTime(minimum_time * 3)
                                                  .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                                  .SetStopPositionTrackingError(stop_position_tracking_error))
                          .SetLeftArmCommand(CartesianCommandBuilder()
                                                 .AddTarget("base", "ee_left", T_left, linear_velocity_limit,
                                                            angular_velocity_limit, acceleration_limit / 2)
                                                 /* Need to be verified */
                                                 .AddJointPositionTarget("left_arm_2", 10 * M_PI / 180, 3.14, 6.28)
                                                 .SetMinimumTime(minimum_time * 3)
                                                 .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                                 .SetStopPositionTrackingError(stop_position_tracking_error)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "Cartesian command example 2\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.4, 1.2;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.4, 1.2;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      CartesianCommandBuilder()
                          .AddTarget("base", "link_torso_5", T_torso, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("base", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("base", "ee_left", T_left, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddJointPositionTarget("right_arm_1", -30 * M_PI / 180, {}, {})
                          .AddJointPositionTarget("left_arm_1", 30 * M_PI / 180, {}, {})
                          .SetStopPositionTrackingError(stop_orientation_tracking_error)
                          .SetStopOrientationTrackingError(stop_position_tracking_error)
                          .SetMinimumTime(minimum_time))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "Cartesian command example 3\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.2;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.4, 0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.4, 0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      CartesianCommandBuilder()
                          .AddTarget("base", "link_torso_5", T_torso, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("link_torso_5", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("link_torso_5", "ee_left", T_left, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .SetStopPositionTrackingError(stop_orientation_tracking_error)
                          .SetStopOrientationTrackingError(stop_position_tracking_error)
                          .SetMinimumTime(minimum_time))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "impedance control command example 1\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.2;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.45, -0.4, -0.1;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.45, 0.4, -0.1;

    ImpedanceControlCommandBuilder torso_command;
    torso_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time))
        .SetReferenceLinkName("base")
        .SetLinkName("link_torso_5")
        .SetTranslationWeight({1000, 1000, 1000})
        .SetRotationWeight({100, 100, 100})
        .SetTransformation(T_torso);

    ImpedanceControlCommandBuilder right_arm_command;
    right_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time))
        .SetReferenceLinkName("link_torso_5")
        .SetLinkName("ee_right")
        .SetTranslationWeight({1000, 1000, 1000})
        .SetRotationWeight({50, 50, 50})
        .SetTransformation(T_right);

    ImpedanceControlCommandBuilder left_arm_command;
    left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time))
        .SetReferenceLinkName("link_torso_5")
        .SetLinkName("ee_left")
        .SetTranslationWeight({1000, 1000, 1000})
        .SetRotationWeight({50, 50, 50})
        .SetTransformation(T_left);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                                        .SetTorsoCommand(torso_command)
                                                                        .SetRightArmCommand(right_arm_command)
                                                                        .SetLeftArmCommand(left_arm_command))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }
  }

  if (1) {
    std::cout << "relative command example 1\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.4, 0.9;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.4, 0.9;

    CartesianCommandBuilder right_arm_command;
    right_arm_command.SetMinimumTime(minimum_time)
        .SetStopOrientationTrackingError(stop_orientation_tracking_error)
        .SetStopPositionTrackingError(stop_position_tracking_error)
        .AddTarget("base", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit, acceleration_limit);

    Eigen::Matrix<double, 4, 4> T_diff;
    T_diff.setIdentity();
    T_diff.block(0, 3, 3, 1) << 0, 0.8, 0;

    ImpedanceControlCommandBuilder left_arm_command;
    left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time * 2.0))
        .SetReferenceLinkName("ee_right")
        .SetLinkName("ee_left")
        .SetTranslationWeight({500, 500, 500})
        .SetRotationWeight({50, 50, 50})
        .SetTransformation(T_diff);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                                        .SetRightArmCommand(right_arm_command)
                                                                        .SetLeftArmCommand(left_arm_command))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }
  }

  if (1) {
    std::cout << "joint position command example 3\n";

    q_joint_torso << 0, 30, -60, 30, 0, 0;
    q_joint_torso *= D2R;

    q_joint_right_arm << -45, -30, 0, -90, 0, 45, 0;
    q_joint_right_arm *= D2R;

    q_joint_left_arm << -45, 30, 0, -90, 0, 45, 0;
    q_joint_left_arm *= D2R;

    Eigen::Vector<double, 20> q;
    q.block(0, 0, 6, 1) = q_joint_torso;
    q.block(6, 0, 7, 1) = q_joint_right_arm;
    q.block(6 + 7, 0, 7, 1) = q_joint_left_arm;

    JointPositionCommandBuilder joint_position_command;
    joint_position_command.SetPosition(q).SetMinimumTime(minimum_time);

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

  double velocity_tracking_gain = 0.01;
  double stop_cost = 1e-3;
  double weight = 1;
  double min_delta_cost = 1e-4;
  int patience = 10;

  if (1) {
    std::cout << "optimal control example 1\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::Identity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1.0;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.2, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.2, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      OptimalControlCommandBuilder()
                          .AddCartesianTarget("base", "link_torso_5", T_torso, weight, weight)
                          .AddCartesianTarget("base", "ee_right", T_right, weight, weight)
                          .AddCartesianTarget("base", "ee_left", T_left, weight, weight)
                          .AddJointPositionTarget("right_arm_2", 3.141592 / 2., weight / 5)
                          .AddJointPositionTarget("left_arm_2", -3.141592 / 2., weight / 5)
                          .SetVelocityLimitScaling(0.5)
                          .SetStopCost(stop_cost)
                          .SetMinDeltaCost(min_delta_cost)
                          .SetPatience(patience))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "optimal control example 2\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::Identity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1.0;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.2, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.2, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      OptimalControlCommandBuilder()
                          .AddCartesianTarget("base", "link_torso_5", T_torso, weight, weight)
                          .AddCartesianTarget("base", "ee_right", T_right, weight, weight)
                          .AddCartesianTarget("base", "ee_left", T_left, weight, weight)
                          .AddJointPositionTarget("right_arm_2", 0., weight / 2)
                          .AddJointPositionTarget("left_arm_2", -0., weight / 2)
                          .SetStopCost(stop_cost)
                          .SetMinDeltaCost(min_delta_cost)
                          .SetPatience(patience))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "optimal control example 3\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotZ(3.141592 / 4.);
    T_torso.block(0, 3, 3, 1) << 0, 0, 1.0;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.2;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.2;

    Eigen::Vector<double, 3> COM;
    COM << 0., 0., 0.5;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      OptimalControlCommandBuilder()
                          .SetCenterOfMassTarget("base", COM, weight * 5)
                          .AddCartesianTarget("base", "link_torso_5", Eigen::Matrix4d::Identity(), 0, weight)
                          .AddCartesianTarget("base", "ee_left", T_left, weight, weight)
                          .AddCartesianTarget("base", "ee_right", T_right, weight, weight)
                          // .AddJointPositionTarget("torso_4", 0., weight)
                          // .AddJointPositionTarget("torso_2", -3.141592 / 2., weight)
                          .AddJointPositionTarget("right_arm_2", 3.141592 / 4., weight / 20)
                          .AddJointPositionTarget("left_arm_2", -3.141592 / 4., weight / 20)
                          .SetStopCost(stop_cost)
                          .SetMinDeltaCost(min_delta_cost)
                          .SetPatience(patience))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "mixed command example 1\n";

    T_torso.block(0, 0, 3, 3).setIdentity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      BodyComponentBasedCommandBuilder()
                          .SetTorsoCommand(
                              OptimalControlCommandBuilder()
                                  .SetCenterOfMassTarget("base", Eigen::Vector<double, 3>{0, 0, 0.4}, weight * 1e-1)
                                  .AddCartesianTarget("base", "link_torso_5", T_torso, 0, weight)
                                  .AddJointPositionTarget("torso_2", -3.141592 / 2., weight)
                                  .AddJointPositionTarget("torso_0", 0., weight)
                                  .SetStopCost(stop_cost * 1e1)
                                  .SetMinDeltaCost(min_delta_cost)
                                  .SetPatience(patience))
                          .SetRightArmCommand(
                              JointPositionCommandBuilder()
                                  .SetPosition(Eigen::Vector<double, 7>{0, -3.141592 / 4., 0, -3.1415 / 2., 0, 0, 0})
                                  .SetVelocityLimit(Eigen::Vector<double, 7>::Constant(3.141592))
                                  .SetAccelerationLimit(Eigen::Vector<double, 7>::Constant(1.0))
                                  .SetMinimumTime(minimum_time))
                          .SetLeftArmCommand(CartesianCommandBuilder()
                                                 .AddTarget("base", "ee_left", T_left, linear_velocity_limit,
                                                            angular_velocity_limit, acceleration_limit)
                                                 .SetMinimumTime(minimum_time)
                                                 .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                                 .SetStopPositionTrackingError(stop_position_tracking_error)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "mixed command example 2\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotZ(3.141592 / 4.);
    T_torso.block(0, 3, 3, 1) << 0, 0, 1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.0;

    OptimalControlCommandBuilder torso_command;
    JointPositionCommandBuilder right_arm_command;
    GravityCompensationCommandBuilder left_arm_command;

    torso_command.SetCenterOfMassTarget("base", Eigen::Vector<double, 3>{0, 0, 0.4}, weight * 1e-1)
        .AddCartesianTarget("base", "link_torso_5", T_torso, 0, weight)
        .AddJointPositionTarget("torso_2", -3.141592 / 2., weight)
        .AddJointPositionTarget("torso_0", 0., weight)
        .SetStopCost(stop_cost)
        .SetMinDeltaCost(min_delta_cost / 10)
        .SetPatience(patience * 10);

    right_arm_command.SetPosition(Eigen::Vector<double, 7>{0, -3.141592 / 4., 0, -3.1415 / 2., 0, 0, 0})
        .SetVelocityLimit(Eigen::Vector<double, 7>::Constant(3.141592))
        .SetAccelerationLimit(Eigen::Vector<double, 7>::Constant(1.0))
        .SetMinimumTime(minimum_time);

    left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time)).SetOn(true);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                                        .SetTorsoCommand(torso_command)
                                                                        .SetRightArmCommand(right_arm_command)
                                                                        .SetLeftArmCommand(left_arm_command))))
                  ->Get();

    std::cout << "!!" << std::endl;

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return 1;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "go to home pose 1\n";
    q_joint_torso.setZero();
    q_joint_right_arm.setZero();
    q_joint_left_arm.setZero();

    q_joint_right_arm(1) = -135 * D2R;
    q_joint_left_arm(1) = 135 * D2R;

    // go to ready position
    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder()
                    .SetTorsoCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time * 2).SetPosition(q_joint_torso))
                    .SetRightArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time * 2).SetPosition(q_joint_right_arm))
                    .SetLeftArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time * 2).SetPosition(q_joint_left_arm)))))
            ->Get();
    std::this_thread::sleep_for(1s);

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;

      return 1;
    }
  }

  if (1) {
    std::cout << "go to home pose 2\n";

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      JointPositionCommandBuilder()
                          .SetPosition(Eigen::Vector<double, 20>::Constant(0.0))
                          .SetMinimumTime(minimum_time))))
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
