#include "rby1-sdk/upc/master_arm.h"

#include <iostream>

using namespace rb;
using namespace std::chrono_literals;

int main() {

  try {
    // Latency timer setting
    upc::InitializeDevice("/dev/rby1_master_arm");
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  auto master_arm = std::make_shared<upc::MasterArm>("/dev/rby1_master_arm");
  master_arm->SetModelPath(PATH "/master_arm.urdf");
  master_arm->SetControlPeriod(0.02);

  auto active_ids = master_arm->Initialize();
  if (active_ids.size() != upc::MasterArm::kDOF + 2) {
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
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(0, 6)) = state.gravity_term(Eigen::seq(0, 6));
      q_right = state.q_joint(Eigen::seq(0, 6));
    } else {
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = q_right;
    }

    if (state.button_left.button == 1) {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(7, 13)) = state.gravity_term(Eigen::seq(7, 13));
      q_left = state.q_joint(Eigen::seq(7, 13));
    } else {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = q_left;
    }
    std::cout<<"trigger : [" << state.button_right.button << ", " << state.button_left.button <<"]" <<std::endl;
    return input;
  });

  std::this_thread::sleep_for(100s);

  master_arm->StopControl();

  return 0;
}