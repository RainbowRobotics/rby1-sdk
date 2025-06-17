#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/master_arm.h"

using namespace rb;
using namespace std::chrono_literals;

auto robot = Robot<y1_model::A>::Create("192.168.30.1:50051");
auto master_arm = std::make_shared<upc::MasterArm>("/dev/rby1_master_arm");

void signalHandler(int signum) {

  master_arm->StopControl();
  robot->PowerOff("12v");
  exit(signum);
}

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  signal(SIGINT, signalHandler);
  // auto robot = Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn("12v")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn("12v")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  try {
    // Latency timer setting
    upc::InitializeDevice("/dev/rby1_master_arm");
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  master_arm->SetModelPath(MODELS_PATH "/master_arm/model.urdf");
  master_arm->SetControlPeriod(0.01);  // 100Hz

  auto active_ids = master_arm->Initialize();
  if (active_ids.size() != upc::MasterArm::kDeivceCount) {
    return 1;
  }

  bool init = false;
  Eigen::Vector<double, upc::MasterArm::kDOF / 2> q_right, q_left;
  q_right.setZero();
  q_left.setZero();
  master_arm->StartControl([&](const upc::MasterArm::State& state) {
    upc::MasterArm::ControlInput input;

    if (!init) {
      q_right = state.q_joint(Eigen::seq(0, 6));
      q_left = state.q_joint(Eigen::seq(7, 13));
      init = true;
    }

    if (state.button_right.button == 1) {
      input.target_operating_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(0, 6)) = state.gravity_term(Eigen::seq(0, 6));
      q_right = state.q_joint(Eigen::seq(0, 6));
    } else {
      input.target_operating_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = q_right;
    }

    if (state.button_left.button == 1) {
      input.target_operating_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(7, 13)) = state.gravity_term(Eigen::seq(7, 13));
      q_left = state.q_joint(Eigen::seq(7, 13));
    } else {
      input.target_operating_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = q_left;
    }
    std::cout << "button : [" << state.button_right.button << ", " << state.button_left.button << "]" << std::endl;
    std::cout << "trigger : [" << state.button_right.trigger << ", " << state.button_left.trigger << "]" << std::endl;
    return input;
  });

  std::this_thread::sleep_for(20s);

  master_arm->StopControl();
  robot->PowerOff("12v");

  return 0;
}