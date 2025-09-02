#pragma once

#include <array>
#include <functional>

#include "Eigen/Core"

#include "rby1-sdk/base/dynamixel_bus.h"
#include "rby1-sdk/base/event_loop.h"
#include "rby1-sdk/base/thread.h"
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/dynamics/state.h"
#include "rby1-sdk/export.h"
#include "rby1-sdk/upc/device.h"

namespace rb::upc {

class RBY1_SDK_API MasterArm {
 public:
  static constexpr int kDOF = 14;
  static constexpr int kDeivceCount = kDOF + 2;
  static constexpr double kTorqueScaling = 0.5;
  static constexpr double kMaximumTorque = 4.;  // 3.0 Nm

  static constexpr int kRightToolId = 0x80;
  static constexpr int kLeftToolId = 0x81;

  struct State {
    Eigen::Vector<double, kDOF> q_joint;
    Eigen::Vector<double, kDOF> qvel_joint;
    Eigen::Vector<double, kDOF> torque_joint;
    Eigen::Vector<double, kDOF> gravity_term;

    Eigen::Vector<int, kDOF> operating_mode;
    Eigen::Vector<double, kDOF> target_position; // Last target position

    DynamixelBus::ButtonState button_right;
    DynamixelBus::ButtonState button_left;

    math::SE3::MatrixType T_right;  // 4 x 4 matrix
    math::SE3::MatrixType T_left;   // 4 x 4 matrix
  };

  struct ControlInput {
    Eigen::Vector<int, kDOF> target_operating_mode;
    Eigen::Vector<double, kDOF> target_position;
    Eigen::Vector<double, kDOF> target_torque;
  };

  explicit MasterArm(const std::string& dev_name = kMasterArmDeviceName);

  ~MasterArm();

  void SetControlPeriod(double control_period);

  void SetModelPath(const std::string& model_path);

  void SetTorqueConstant(const std::array<double, kDOF>& torque_constant);

  std::vector<int> Initialize(bool verbose = false);

  bool StartControl(const std::function<ControlInput(const State& state)>& control = nullptr);

  bool StopControl(bool torque_disable = false);

  bool EnableTorque();

  bool DisableTorque();

 private:
  bool initialized_{false};

  EventLoop ev_;
  double control_period_;  // (sec)
  std::array<double, kDOF> torque_constant_;

  EventLoop ctrl_ev_;
  std::atomic<bool> ctrl_running_{false};

  std::shared_ptr<DynamixelBus> handler_;
  std::shared_ptr<dyn::Robot<kDOF>> dyn_robot_;
  std::shared_ptr<dyn::State<kDOF>> dyn_state_;

  std::string model_path_;

  std::atomic<bool> is_running_{false};
  std::vector<int> active_ids_;
  bool state_updated_;
  State state_;
  bool operating_mode_init_{false};

  std::function<ControlInput(const State& state)> control_;
};

}  // namespace rb::upc