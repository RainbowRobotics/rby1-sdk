#pragma once

#include <functional>

#include "rby1-sdk/base/dynamixel_bus.h"
#include "rby1-sdk/base/event_loop.h"
#include "rby1-sdk/base/thread.h"
#include "rby1-sdk/upc/device.h"

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/dynamics/state.h"

namespace rb::upc {

class MasterArm {
 public:
  static constexpr int kDOF = 14;
  static constexpr double kTorqueScaling = 0.5;
  static constexpr double kMaximumTorque = 4.;  // 3.0 Nm

  static constexpr int kRightToolId = 0x80;
  static constexpr int kLeftToolId = 0x81;

  struct State {
    Eigen::Vector<double, kDOF> q_joint;
    Eigen::Vector<double, kDOF> qvel_joint;
    Eigen::Vector<double, kDOF> torque_joint;
    Eigen::Vector<double, kDOF> gravity_term;

    Eigen::Vector<int, kDOF> operation_mode;

    DynamixelBus::ButtonState button_right;
    DynamixelBus::ButtonState button_left;

    math::SE3::MatrixType T_right;  // 4 x 4 matrix
    math::SE3::MatrixType T_left;   // 4 x 4 matrix
  };

  struct ControlInput {
    Eigen::Vector<int, kDOF> target_operation_mode;
    Eigen::Vector<double, kDOF> target_position;
    Eigen::Vector<double, kDOF> target_torque;
  };

  explicit MasterArm(const std::string& dev_name = kMasterArmDeviceName);

  ~MasterArm();

  void SetControlPeriod(double control_period);

  void SetModelPath(const std::string& model_path);

  std::vector<int> Initialize();

  void StartControl(const std::function<ControlInput(const State& state)>& control = nullptr);

  void StopControl();

 private:
  EventLoop ev_;
  double control_period_;  // (sec)
  std::vector<double> torque_constant_;

  std::shared_ptr<DynamixelBus> handler_;
  std::shared_ptr<dyn::Robot<kDOF>> dyn_robot_;
  std::shared_ptr<dyn::State<kDOF>> dyn_state_;

  std::string model_path_;

  std::atomic<bool> is_running_{false};
  std::vector<int> active_ids_;
  bool state_updated_;
  State state_;

  std::function<ControlInput(const State& state)> control_;
};

}  // namespace rb::upc