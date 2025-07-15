#include <iomanip>
#include <iostream>
#include <chrono>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

const std::string kAll = ".*";

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  std::cout << "Initializing robot at address: " << address << std::endl;

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
        std::cout << "  Position: " << state.position.transpose() << std::endl;
        std::cout << "  Uptime: " << state.system_stat.uptime << " s" << std::endl;
        std::cout << "  Program Uptime: " << state.system_stat.program_uptime << " s" << std::endl;
        std::cout << "  Memory Usage: " << state.system_stat.memory_usage * 100 << " %" << std::endl;
        std::cout << "  CPU Usage: " << state.system_stat.cpu_usage * 100 << " %" << std::endl;
      },
      1 /* Hz */);

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

  //  RobotCommandBuilder command_builder;
  //  command_builder.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
  //      BodyComponentBasedCommandBuilder()
  //          .SetLeftArmCommand(JointPositionCommandBuilder().SetCommandHeader(1.).SetMinimumTime(1.).SetPosition(
  //              Eigen::Vector<double, 7>::Zero()))
  //          .SetRightArmCommand(GravityCompensationCommandBuilder().SetCommandHeader(10. /* sec */).SetOn(true))));
  //
  //  robot->SendCommand(command_builder)->Get();

  const double D2R = 0.017453;
  Eigen::Vector<double, 6> q_joint_wasit;
  Eigen::Vector<double, 7> q_joint_right_arm;
  Eigen::Vector<double, 7> q_joint_left_arm;
  {
    q_joint_wasit << 0, 30, -60, 30, 0, 0;
    q_joint_wasit *= D2R;

    q_joint_right_arm << -45, -10, 0, -90, 0, 45, 0;
    q_joint_right_arm *= D2R;

    q_joint_left_arm << -45, 10, 0, -90, 0, 45, 0;
    q_joint_left_arm *= D2R;

    Eigen::Vector<double, 20> q;
    q.block(0, 0, 6, 1) = q_joint_wasit;
    q.block(6, 0, 7, 1) = q_joint_right_arm;
    q.block(6 + 7, 0, 7, 1) = q_joint_left_arm;

    robot
        ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
            JointPositionCommandBuilder().SetPosition(q).SetMinimumTime(1.0))))
        ->Get();
    std::this_thread::sleep_for(1s);
  }

  Eigen::Vector<double, 7> zero = Eigen::Vector<double, 7>::Zero();
  Eigen::Vector<double, 7> position = {1., 0.5, 1, -1, 1, 1, 1};

  // TODO
  robot->SetParameter("joint_position_command.cutoff_frequency", "15.0");
  std::cout << robot->GetParameter("joint_position_command__cutoff_frequency") << std::endl;

  double sum = 0.;

  auto stream = robot->CreateCommandStream();
  for (int i = 0; i < 1000; i++) {
    Eigen::Vector<double, 7> target_position = (i % 2 == 0 ? zero : position);

    const auto& s = std::chrono::steady_clock::now();
    RobotCommandBuilder command_builder;
    command_builder.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
        BodyComponentBasedCommandBuilder()
            .SetLeftArmCommand(
                ImpedanceControlCommandBuilder()
                    .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                    .SetReferenceLinkName("base")
                    .SetLinkName("ee_left")
                    .SetTranslationWeight({3000, 3000, 3000})
                    .SetRotationWeight({2, 2, 2})
                    .SetTransformation(math::SE3::T(
                        math::SO3::RotZ(3.141592 / 2),
                        Eigen::Vector3d{0.4 + 0.05 * (1 - cos((double)i / 250. * 2. * math::kPi)), 0.2, 1.2})))
            .SetRightArmCommand(GravityCompensationCommandBuilder()
                                    .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                                    .SetOn(true))));
    stream->SendCommand(command_builder);
    const auto& e = std::chrono::steady_clock::now();

    sum += (double)std::chrono::duration_cast<std::chrono::nanoseconds>(e - s).count() / 1e9;

    std::this_thread::sleep_for(10ms);
  }

  stream->Wait();

  std::cout << sum / 1000. << std::endl;

  return 0;
}