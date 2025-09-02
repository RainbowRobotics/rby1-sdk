#include "rby1-sdk/upc/master_arm.h"

#include <Eigen/Core>
#include <iostream>

namespace rb::upc {

MasterArm::MasterArm(const std::string& dev_name)
    : handler_(std::make_shared<DynamixelBus>(dev_name)), control_period_(0.1) {
  torque_constant_ = {1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
                      1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043};  // Default torque constant
}

MasterArm::~MasterArm() {
  this->StopControl();

  // Set torque to zero
  if (initialized_) {
    std::vector<std::pair<int, double>> id_torque;
    for (const auto& id : active_ids_) {
      if (id < 0x80) {
        id_torque.emplace_back(id, 0);
      }
    }
    if (!id_torque.empty()) {
      handler_->GroupSyncWriteSendTorque(id_torque);
    }
  }

  this->DisableTorque();
}

void MasterArm::SetControlPeriod(double control_period) {
  control_period_ = control_period;
}

void MasterArm::SetModelPath(const std::string& model_path) {
  model_path_ = model_path;
}

void MasterArm::SetTorqueConstant(const std::array<double, MasterArm::kDOF>& torque_constant) {
  torque_constant_ = torque_constant;
}

std::vector<int> MasterArm::Initialize(bool verbose) {
  if (!handler_->OpenPort()) {
    if (verbose) {
      std::cerr << "Failed to open the port!" << std::endl;
    }
    return {};
  }
  if (!handler_->SetBaudRate(DynamixelBus::kDefaultBaudrate)) {
    if (verbose) {
      std::cerr << "Failed to change the baudrate!" << std::endl;
    }
    return {};
  }

  initialized_ = true;

  std::vector<int> active_ids;

  for (int id = 0; id < kDOF; ++id) {
    if (handler_->Ping(id)) {
      if (verbose) {
        std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      }
      active_ids.push_back(id);
    } else {
      if (verbose) {
        std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
      }
    }
  }
  for (int id = 0x80; id < 0x80 + 2; id++) {
    if (handler_->Ping(id)) {
      if (verbose) {
        std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      }
      active_ids.push_back(id);
    } else {
      if (verbose) {
        std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
      }
    }
  }
  if (active_ids.size() != 16) {
    if (verbose) {
      std::cerr << "Unable to ping all devices for master arm" << std::endl;
    }
    Eigen::Map<Eigen::VectorXi> ids(active_ids.data(), (long)active_ids.size());
    if (verbose) {
      std::cerr << "active ids: " << ids.transpose() << std::endl;
    }
  }

  handler_->SetTorqueConstant(std::vector<double>(torque_constant_.begin(), torque_constant_.end()));

  active_ids_ = active_ids;
  return active_ids;
}

bool MasterArm::StartControl(const std::function<ControlInput(const State& state)>& control) {
  if (!initialized_) {
    return false;
  }

  if (is_running_.load()) {
    return false;
  }
  is_running_ = true;

  ev_.Unpause();
  ctrl_ev_.Unpause();

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

  std::vector<int> motor_ids;  // Motor IDs for active motors
  for (int id : active_ids_) {
    if (id < 0x80) {
      motor_ids.push_back(id);
    }
  }

  if (!operating_mode_init_) {
    handler_->GroupSyncWriteTorqueEnable(motor_ids, DynamixelBus::kTorqueDisable);
    {
      std::vector<std::pair<int, int>> motor_id_modes;
      for (int id : motor_ids) {
        motor_id_modes.emplace_back(id, DynamixelBus::kCurrentControlMode);
      }
      handler_->GroupSyncWriteOperatingMode(motor_id_modes);
    }
    handler_->GroupSyncWriteTorqueEnable(motor_ids, DynamixelBus::kTorqueEnable);
  }

  state_.q_joint.setZero();
  state_.operating_mode.setConstant(-1);
  state_updated_ = false;
  ev_.PushCyclicTask(
      [=] {
        static double duration_sum = 0.;
        static int duration_count = 0;

        auto start_time = std::chrono::steady_clock::now();
        for (int id : active_ids_) {
          if (id >= kRightToolId) {
            // for hand board
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

        auto temp_operation_mode_vector = handler_->GroupFastSyncReadOperatingMode(active_ids_, true);
        if (temp_operation_mode_vector.has_value()) {
          for (auto const& ret : temp_operation_mode_vector.value()) {
            if (ret.first < kDOF) {
              state_.operating_mode(ret.first) = ret.second;
            }
          }
        } else {
          return;
        }

        std::optional<std::vector<std::pair<int, DynamixelBus::MotorState>>> temp_ms_vector =
            handler_->GetMotorStates(motor_ids);
        if (temp_ms_vector.has_value()) {
          for (auto const& ret : temp_ms_vector.value()) {
            if (ret.first < kDOF) {
              state_.q_joint(ret.first) = ret.second.position;
              state_.qvel_joint(ret.first) = ret.second.velocity;
              state_.torque_joint(ret.first) = ret.second.current * torque_constant_[ret.first];
            }
          }
          state_updated_ = true;
        } else {
          return;
        }

        const auto& temp_gp_vector = handler_->GroupFastSyncRead(motor_ids, DynamixelBus::kAddrGoalPosition, 4);
        if (temp_gp_vector.has_value()) {
          for (auto const& ret : temp_gp_vector.value()) {
            state_.target_position[ret.first] = (double)ret.second / 4096. * 2. * 3.141592;
          }
        } else {
          return;
        }

        dyn_state_->SetQ(state_.q_joint);
        dyn_robot_->ComputeForwardKinematics(dyn_state_);
        state_.gravity_term = dyn_robot_->ComputeGravityTerm(dyn_state_) * kTorqueScaling;

        state_.T_right = dyn_robot_->ComputeTransformation(dyn_state_, kBaseLinkId, kRightLinkId);
        state_.T_left = dyn_robot_->ComputeTransformation(dyn_state_, kBaseLinkId, kLeftLinkId);

        // Control
        if (control_ && state_updated_ && !ctrl_running_.load()) {
          ctrl_running_ = true;

          ctrl_ev_.PushTask([this, state = state_, operating_mode_init = operating_mode_init_] {
            auto input = control_(state);

            std::vector<std::pair<int, int>> changed_id_mode;
            std::vector<int> changed_id;

            std::vector<std::pair<int, double>> id_position;
            std::vector<std::pair<int, double>> id_torque;

            for (int i = 0; i < kDOF; i++) {
              if (state.operating_mode(i) != input.target_operating_mode(i) || !operating_mode_init) {
                changed_id.push_back(i);
                changed_id_mode.emplace_back(i, input.target_operating_mode(i));
              } else {
                if (state.operating_mode(i) == DynamixelBus::kCurrentControlMode) {
                  id_torque.emplace_back(i, input.target_torque(i));
                } else if (state.operating_mode(i) == DynamixelBus::kCurrentBasedPositionControlMode) {
                  id_torque.emplace_back(i, input.target_torque(i));
                  id_position.emplace_back(i, input.target_position(i));
                }
              }
            }

            ev_.PushTask([=] {
              handler_->GroupSyncWriteTorqueEnable(changed_id, 0);
              handler_->GroupSyncWriteOperatingMode(changed_id_mode);
              handler_->GroupSyncWriteTorqueEnable(changed_id, 1);

              operating_mode_init_ = true;

              handler_->GroupSyncWriteSendTorque(id_torque);
              handler_->GroupSyncWriteSendPosition(id_position);
            });

            ctrl_running_ = false;
          });

          auto end_time = std::chrono::steady_clock::now();

          duration_sum += std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1.0e9;
          duration_count++;
          if (duration_count % 100 == 0) {
            std::cout << "avg: " << duration_sum / duration_count * 1000 << " ms" << " (count: " << duration_count
                      << ")" << std::endl;
          }
        }
      },
      std::chrono::nanoseconds((long)(control_period_ * 1e9)));

  return true;
}

bool MasterArm::StopControl(bool torque_disable) {
  if (!initialized_) {
    return false;
  }

  if (!is_running_.load()) {
    return false;
  }

  {
    ev_.Pause();

    ctrl_ev_.Pause();
    ctrl_ev_.WaitForTasks();
    ctrl_ev_.PurgeTasks();

    ev_.WaitForTasks();
    ev_.PurgeTasks();
  }

  dyn_state_ = nullptr;
  dyn_robot_ = nullptr;

  if (torque_disable) {
    for (int id : active_ids_) {
      if (id < 0x80) {
        handler_->SendTorqueEnable(id, DynamixelBus::kTorqueDisable);
      }
    }
  }

  is_running_ = false;
  control_ = nullptr;

  return true;
}

bool MasterArm::EnableTorque() {
  if (!initialized_) {
    return false;
  }

  if (is_running_.load()) {
    return false;
  }

  for (int id : active_ids_) {
    if (id < 0x80) {
      handler_->SendTorqueEnable(id, DynamixelBus::kTorqueEnable);
    }
  }

  return true;
}

bool MasterArm::DisableTorque() {
  if (!initialized_) {
    return false;
  }

  if (is_running_.load()) {
    return false;
  }

  for (int id : active_ids_) {
    if (id < 0x80) {
      handler_->SendTorqueEnable(id, DynamixelBus::kTorqueDisable);
    }
  }

  return true;
}

}  // namespace rb::upc