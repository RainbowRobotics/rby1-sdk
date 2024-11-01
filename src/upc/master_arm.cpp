#include "rby1-sdk/upc/master_arm.h"

#include <Eigen/Core>
#include <iostream>

namespace rb::upc {

MasterArm::MasterArm(const std::string& dev_name)
    : handler_(std::make_shared<DynamixelBus>(dev_name)), control_period_(0.1) {}

MasterArm::~MasterArm() {
  this->StopControl();
}

void MasterArm::SetControlPeriod(double control_period) {
  control_period_ = control_period;
}

void MasterArm::SetModelPath(const std::string& model_path) {
  model_path_ = model_path;
}

std::vector<int> MasterArm::Initialize() {
  if (!handler_->OpenPort()) {
    std::cerr << "Failed to open the port!" << std::endl;
    return {};
  }
  if (!handler_->SetBaudRate(DynamixelBus::kDefaultBaudrate)) {
    std::cerr << "Failed to change the baudrate!" << std::endl;
    return {};
  }

  std::vector<int> active_ids;

  for (int id = 0; id < 14; ++id) {
    if (handler_->Ping(id)) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      active_ids.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }
  for (int id = 0x80; id < 0x80 + 2; id++) {
    if (handler_->Ping(id)) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      active_ids.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }
  if (active_ids.size() != 16) {
    std::cerr << "Unable to ping all devices for master arm" << std::endl;
    Eigen::Map<Eigen::VectorXi> ids(active_ids.data(), (long)active_ids.size());
    std::cerr << "active ids: " << ids.transpose() << std::endl;
    exit(1);
  }

  torque_constant_ = {1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
                      1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043};
  handler_->SetTorqueConstant(torque_constant_);

  active_ids_ = active_ids;
  return active_ids;
}

void MasterArm::StartControl(const std::function<ControlInput(const State& state)>& control) {
  if (is_running_.load()) {
    return;
  }
  is_running_ = true;
  control_ = control;

  auto rc = dyn::LoadRobotFromURDF(model_path_, "Base");
  dyn_robot_ = std::make_shared<dyn::Robot<kDOF>>(rc);
  dyn_state_ = dyn_robot_->MakeState(
      {"Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R", "Link_0L", "Link_1L",
       "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"},
      {"J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R", "J4_Wrist_Yaw1_R",
       "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R", "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L",
       "J10_Elbow_L", "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"});
  dyn_state_->SetGravity({0, 0, 0, 0, 0, -9.81});

  const int kBaseLinkId = 0;
  const int kRightLinkId = 7;
  const int kLeftLinkId = 14;

  for (int id : active_ids_) {
    if (id < 0x80) {
      if (!handler_->SendOperationMode(id, DynamixelBus::kCurrentControlMode)) {
        std::cerr << "Failed to write current control mode value: " << handler_->ReadOperationMode(id).value_or(-1)
                  << std::endl;
      }
      handler_->SendTorqueEnable(id, DynamixelBus::kTorqueEnable);
    }
  }

  state_.q_joint.setZero();
  state_.operation_mode.setConstant(-1);
  state_updated_ = false;
  ev_.PushCyclicTask(
      [=] {
        for (int id : active_ids_) {
          if (id >= kRightToolId) {
            //for hand board
            const auto& temp_button_status = handler_->ReadButtonStatus(id);
            if (temp_button_status.has_value()) {
              if (id == kRightToolId) {
                state_.button_right = temp_button_status.value().second;
              } else if (id == kLeftToolId) {
                state_.button_left = temp_button_status.value().second;
              }
            }
          }
        }

        auto temp_operation_mode_vector = handler_->BulkReadOperationMode(active_ids_);
        if (temp_operation_mode_vector.has_value()) {
          for (auto const& ret : temp_operation_mode_vector.value()) {
            if (ret.first < kDOF) {
              state_.operation_mode(ret.first) = ret.second;
            }
          }
        }

        // std::optional<std::vector<std::pair<int, double>>> temp_q_joint_vector = handler_->BulkReadEncoder(active_ids_);
        // if (temp_q_joint_vector.has_value()) {
        //   for (auto const& ret : temp_q_joint_vector.value()) {
        //     state_.q_joint(ret.first) = ret.second;
        //   }
        //   state_updated_ = true;
        // } else {
        //   return;
        // }

        std::optional<std::vector<std::pair<int, DynamixelBus::MotorState>>> temp_ms_vector =
            handler_->BulkReadMotorState(active_ids_);
        if (temp_ms_vector.has_value()) {
          for (auto const& ret : temp_ms_vector.value()) {
            if (ret.first < kDOF) {
              state_.q_joint(ret.first) = ret.second.position;
              state_.qvel_joint(ret.first) = ret.second.velocity;
              state_.torque_joint(ret.first) = ret.second.current * torque_constant_[ret.first] / kTorqueScaling;
            }
          }
          state_updated_ = true;
        } else {
          return;
        }

        dyn_state_->SetQ(state_.q_joint);
        dyn_robot_->ComputeForwardKinematics(dyn_state_);
        state_.gravity_term = dyn_robot_->ComputeGravityTerm(dyn_state_) * kTorqueScaling;

        state_.T_right = dyn_robot_->ComputeTransformation(dyn_state_, kBaseLinkId, kRightLinkId);
        state_.T_left = dyn_robot_->ComputeTransformation(dyn_state_, kBaseLinkId, kLeftLinkId);

        // Control
        if (control_ && state_updated_) {
          auto input = control_(state_);

          std::vector<std::pair<int, int>> changed_id_mode;
          std::vector<int> changed_id;

          std::vector<std::pair<int, double>> id_position;
          std::vector<std::pair<int, double>> id_torque;

          for (int i = 0; i < kDOF; i++) {
            if (state_.operation_mode(i) != input.target_operation_mode(i)) {
              changed_id.push_back(i);
              changed_id_mode.emplace_back(i, input.target_operation_mode(i));
            } else {
              if (state_.operation_mode(i) == DynamixelBus::kCurrentControlMode) {
                id_torque.emplace_back(i, input.target_torque(i));
              } else if (state_.operation_mode(i) == DynamixelBus::kCurrentBasedPositionControlMode) {
                id_torque.emplace_back(i, kMaximumTorque);
                id_position.emplace_back(i, input.target_position(i));
              }
            }
          }

          handler_->BulkWriteTorqueEnable(changed_id, 0);
          handler_->BulkWriteOperationMode(changed_id_mode);
          handler_->BulkWriteTorqueEnable(changed_id, 1);

          handler_->BulkWriteSendTorque(id_torque);
          handler_->BulkWriteSendPosition(id_position);
        }
      },
      std::chrono::nanoseconds((long)(control_period_ * 1e9)));
}

void MasterArm::StopControl() {
  {  // Remove all tasks in current event loop
    ev_.Pause();
    ev_.WaitForTasks();
    ev_.PurgeTasks();
  }

  dyn_state_ = nullptr;
  dyn_robot_ = nullptr;

  for (int id : active_ids_) {
    if (id < 0x80) {
      handler_->SendTorqueEnable(id, DynamixelBus::kTorqueDisable);
    }
  }

  is_running_ = false;
  control_ = nullptr;
}

}  // namespace rb::upc