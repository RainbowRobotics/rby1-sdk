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

  Eigen::Vector<double, 24> q_joint_rby1_24x1;
  q_joint_rby1_24x1.setZero();

  robot->StartStateUpdate(
      [&](const auto& state) {
        std::cout << "State Update Received:" << std::endl;
        std::cout << "  Timestamp: " << state.timestamp.tv_sec << ".";
        std::cout << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
        std::cout << "  wasit [deg]     : " << state.position.block(2, 0, 6, 1).transpose() * R2D << std::endl;
        std::cout << "  right arm [deg] : " << state.position.block(2 + 6, 0, 7, 1).transpose() * R2D << std::endl;
        std::cout << "  left arm [deg]  : " << state.position.block(2 + 6 + 7, 0, 7, 1).transpose() * R2D << std::endl;
        q_joint_rby1_24x1.block(2, 0, 20, 1) = state.position.block(2, 0, 20, 1);
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

  // robot->SetParameter("default.acceleration_limit_scaling", "0.8");
  // robot->SetParameter("joint_position_command.cutoff_frequency", "5");
  // robot->SetParameter("cartesian_command.cutoff_frequency", "5");
  // robot->SetParameter("default.linear_acceleration_limit", "5");

  std::this_thread::sleep_for(1s);

  Eigen::Vector<double, 6> q_joint_waist;
  Eigen::Vector<double, 7> q_joint_right_arm;
  Eigen::Vector<double, 7> q_joint_left_arm;
  q_joint_waist.setZero();
  q_joint_right_arm.setZero();
  q_joint_left_arm.setZero();

  double minimum_time = 4.;

  std::unique_ptr<RobotCommandStreamHandler<y1_model::A>> stream;
  double right_arm_minimum_time = 1.;
  double left_arm_minimum_time = 1.;
  double lpf_update_ratio = 0.1;
  double torso_minimum_time = 1.0;
  double wheel_minimum_time = 5.0;

  Eigen::Vector<double, 2> wheel_velocity, wheel_acceleration;
  wheel_velocity << 1 * 3.14 * 2., 1 * 3.14 * 2.;
  wheel_acceleration.setConstant(100. / 10.);

  Eigen::Vector<double, 7> arm_acc_limit, arm_vel_limit;

  arm_acc_limit.setConstant(1200.0);
  arm_acc_limit *= D2R;

  arm_vel_limit << 160, 160, 160, 160, 330, 330, 330;
  arm_vel_limit *= D2R;

  Eigen::Matrix<double, 4, 4> T_torso;
  T_torso.setIdentity();
  T_torso.block(0, 3, 3, 1) << 0, 0, 1.1;
  double stop_orientation_tracking_error = 1e-5;
  double stop_position_tracking_error = 1e-5;

  Eigen::Vector<double, 7> q_joint_right_target, q_joint_left_target;
  q_joint_right_target.setZero();
  q_joint_left_target.setZero();

  auto dyn = robot->GetDynamics();
  auto dyn_state = dyn->MakeState({"base", "link_torso_5", "ee_right", "ee_left"}, y1_model::A::kRobotJointNames);
  Eigen::Vector<double, 7> q_joint_right_lb, q_joint_left_lb, q_joint_right_ub, q_joint_left_ub;
  q_joint_right_ub = dyn->GetLimitQUpper(dyn_state).block(2 + 6, 0, 7, 1);
  q_joint_right_lb = dyn->GetLimitQLower(dyn_state).block(2 + 6, 0, 7, 1);

  q_joint_left_ub = dyn->GetLimitQUpper(dyn_state).block(2 + 6 + 7, 0, 7, 1);
  q_joint_left_lb = dyn->GetLimitQLower(dyn_state).block(2 + 6 + 7, 0, 7, 1);

  Eigen::Vector<double, 2> linear_velocity = Eigen::Vector<double, 2>::Zero();
  double angular_velocity = 0.;

  double angular_acceleration_limit = 100.;
  Eigen::Vector<double, 2> linear_acceleration_limit;
  linear_acceleration_limit << 100, 100;

  stream = robot->CreateCommandStream();

  double control_hold_time = 1e6;

  {
    //go to init pos
    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      BodyComponentBasedCommandBuilder()
                          .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(3).SetPosition(
                              Eigen::Vector<double, 6>{0, 30, -60, 30, 0, 0} * D2R))
                          .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(3).SetPosition(
                              Eigen::Vector<double, 7>{45, 0, 0, -135, 0, 45, 0} * D2R))
                          .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(3).SetPosition(
                              Eigen::Vector<double, 7>{45, 0, 0, -135, 0, 45, 0} * D2R)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;

      return 1;
    }
  }

  while (1) {

    std::cout << "start !" << std::endl;
    MobilityCommandBuilder mobility_command;
    JointPositionCommandBuilder right_arm_command;
    JointPositionCommandBuilder left_arm_command;
    CartesianCommandBuilder torso_command;

    q_joint_right_target << 45, 0, 0, -135, 0, 45, 0;  //right arm command
    q_joint_left_target << 45, 0, 0, -135, 0, 45, 0;   //left arm command

    q_joint_right_target *= 3.141592 / 180.;
    q_joint_left_target *= 3.141592 / 180.;

    linear_velocity << 1, 0;  // mobility command
    angular_velocity = 1;     // mobility command

    dyn_state->SetQ(q_joint_rby1_24x1);
    dyn->ComputeForwardKinematics(dyn_state);

    T_torso = dyn->ComputeTransformation(dyn_state, 0, 1);

    Eigen::Vector<double, 3> torso_pos_command = Eigen::Vector<double, 3>::Zero();  // torso command
    Eigen::Vector<double, 3> torso_ori_command = Eigen::Vector<double, 3>::Zero();  // torso command

    Eigen::Vector<double, 3> torso_se3v_ori = math::SO3::Log(T_torso.block(0, 0, 3, 3)) + torso_ori_command;
    Eigen::Vector<double, 3> torso_se3v_pos = T_torso.block(0, 3, 3, 1) + torso_pos_command;

    T_torso.block(0, 0, 3, 3) = math::SO3::Exp(torso_se3v_ori);
    T_torso.block(0, 3, 3, 1) = torso_se3v_pos;

    {
      //mobility command

      mobility_command.SetCommand(SE2VelocityCommandBuilder()
                                      .SetVelocity(linear_velocity, angular_velocity)
                                      .SetAccelerationLimit(linear_acceleration_limit, angular_acceleration_limit)
                                      .SetMinimumTime(wheel_minimum_time)
                                      .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(control_hold_time)));
      std::cout << "set mobility command\n";
    }

    {
      //dual arm command
      for (int i = 0; i < 7; i++) {
        q_joint_right_target(i) = std::clamp(q_joint_right_target(i), q_joint_right_lb(i), q_joint_right_ub(i));
        q_joint_left_target(i) = std::clamp(q_joint_left_target(i), q_joint_left_lb(i), q_joint_left_ub(i));
      }

      right_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(control_hold_time))
          .SetMinimumTime(right_arm_minimum_time)
          .SetPosition(q_joint_right_target)
          .SetVelocityLimit(arm_vel_limit)
          .SetAccelerationLimit(arm_acc_limit);
      std::cout << "set right arm command\n";

      left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(control_hold_time))
          .SetMinimumTime(left_arm_minimum_time)
          .SetPosition(q_joint_left_target)
          .SetVelocityLimit(arm_vel_limit)
          .SetAccelerationLimit(arm_acc_limit);
      std::cout << "set left arm command\n";
    }

    {
      //torso command
      Eigen::Vector<double, 3> se3v_ori = math::SO3::Log(T_torso.block(0, 0, 3, 3));
      Eigen::Vector<double, 3> se3v_pos = T_torso.block(0, 3, 3, 1);

      for (int i = 0; i < 3; i++) {
        se3v_ori(i) = std::min(std::max(se3v_ori(i), -0.5236), 0.5236);
        if (i == 2) {
          se3v_pos(i) = std::min(std::max(se3v_pos(i), 0.9), 1.2);
        } else {
          se3v_pos(i) = std::min(std::max(se3v_pos(i), -0.2), 0.2);
        }
      }

      T_torso.block(0, 0, 3, 3) = math::SO3::Exp(se3v_ori);
      T_torso.block(0, 3, 3, 1) = se3v_pos;

      torso_command.AddTarget("base", "link_torso_5", T_torso, 1, 3.141592, 3)
          .SetMinimumTime(torso_minimum_time)
          .SetStopOrientationTrackingError(stop_orientation_tracking_error)
          .SetStopPositionTrackingError(stop_position_tracking_error)
          .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(control_hold_time));
      std::cout << "set torso command\n";
    }

    {
      //whole body command
      RobotCommandBuilder whole_body_command;
      whole_body_command.SetCommand(ComponentBasedCommandBuilder()
                                        .SetMobilityCommand(mobility_command)
                                        .SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                            .SetLeftArmCommand(left_arm_command)
                                                            .SetRightArmCommand(right_arm_command)
                                                            .SetTorsoCommand(torso_command)));

      std::cout << "set whole body command\n";
      stream->SendCommand(whole_body_command);
    }
    std::cout << "end !" << std::endl;
  }

  std::cout << "end of demo\n";

  return 0;
}
