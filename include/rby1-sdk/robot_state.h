#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "dynamics/link.h"
#include "export.h"
#include "math/liegroup.h"

namespace rb {

struct RBY1_SDK_API SystemStat {
  double cpu_usage{0.};       // (percent)
  double memory_usage{0.};    // (percent)
  double uptime{0.};          // (sec)
  double program_uptime{0.};  // (sec)
};

struct RBY1_SDK_API BatteryState {
  double voltage{0.};        // (V)
  double current{0.};        // (Amp)
  double level_percent{0.};  // (%)
};

struct RBY1_SDK_API PowerState {
  enum class State { kUnknown = 0, kPowerOn = 1, kPowerOff = 2 };

  State state{State::kUnknown};
  double voltage{0.};
};

struct RBY1_SDK_API EMOState {
  enum class State { kReleased = 0, kPressed = 1 };

  State state{State::kReleased};
};

struct RBY1_SDK_API JointState {
  enum class FETState { kUnknown = 0, kOn = 1, kOff = 2 };
  enum class RunState { kUnknown = 0, kControlOn = 1, kControlOff = 2 };
  enum class InitializationState { kUnknown = 0, kInitialized = 1, kUninitialized = 2 };

  struct timespec time_since_last_update{};

  bool is_ready{false};
  FETState fet_state{FETState::kUnknown};
  RunState run_state{RunState::kUnknown};
  InitializationState init_state{InitializationState::kUnknown};

  uint32_t motor_type{};  // 0: simulator, 1: rbmotor, 2: dynamixel
  uint64_t motor_state{};

  bool power_on{false};
  double position{0.};  // (rad)
  double velocity{0.};  // (rad/s)
  double current{0.};   // (amp)
  double torque{0.};    // (Nm)

  double target_position{0.};            // (rad)
  double target_velocity{0.};            // (rad/s)
  uint32_t target_feedback_gain{10};     // [0,10]
  double target_feedforward_torque{0.};  // (Nm)

  int temperature{};  // ºC
};

struct RBY1_SDK_API ToolFlangeState {
  struct timespec time_since_last_update{};

  Eigen::Vector<double, 3> gyro;          // (rad/s)
  Eigen::Vector<double, 3> acceleration;  // (m/s^2)

  bool switch_A{};

  int output_voltage;

  bool digital_input_A{};
  bool digital_input_B{};

  bool digital_output_A{};
  bool digital_output_B{};
};

struct RBY1_SDK_API FTSensorData {
  struct timespec time_since_last_update{};

  Eigen::Vector<double, 3> force;
  Eigen::Vector<double, 3> torque;
};

template <typename T>
struct RBY1_SDK_API RobotState {
  struct timespec timestamp{};  // Robot state update timestamp

  SystemStat system_stat{};  // System Statistic

  BatteryState battery_state{};  // Battery state

  std::vector<PowerState> power_states{};  //Power state

  std::vector<EMOState> emo_states{};  // EMO button state

  std::array<JointState, T::kRobotDOF> joint_states{};  // Joint state

  ToolFlangeState tool_flange_right;  // Tool flange state / right
  ToolFlangeState tool_flange_left;   // - / left

  FTSensorData ft_sensor_right;  // Force torque sensor data / right
  FTSensorData ft_sensor_left;   // - / left

  // Ready state, position, velocity, current, torque
  Eigen::Vector<bool, T::kRobotDOF> is_ready{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  Eigen::Vector<double, T::kRobotDOF> position{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> velocity{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> current{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};

  // Last reference
  Eigen::Vector<double, T::kRobotDOF> target_position{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> target_velocity{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<uint32_t, T::kRobotDOF> target_feedback_gain{Eigen::Vector<uint32_t, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> target_feedforward_torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};

  // Odometry, if there is mobility
  math::SE2::MatrixType odometry{math::SE2::Identity()};

  // Center of mass
  Eigen::Vector<double, 3> center_of_mass;  // Cent of mass position with respect to base link

  // Gravity term
  Eigen::Vector<double, T::kRobotDOF> gravity;

  // Collisions
  std::vector<dyn::CollisionResult> collisions;

  // Temperature
  Eigen::Vector<int, T::kRobotDOF> temperature{Eigen::Vector<int, T::kRobotDOF>::Zero()};
};

RBY1_SDK_API inline std::string to_string(PowerState::State s) {
  switch (s) {
    case PowerState::State::kUnknown:
      return "Unknown";
    case PowerState::State::kPowerOn:
      return "PowerOn";
    case PowerState::State::kPowerOff:
      return "PowerOff";
  }
  return "";
}

RBY1_SDK_API inline std::string to_string(EMOState::State s) {
  switch (s) {
    case EMOState::State::kReleased:
      return "Released";
    case EMOState::State::kPressed:
      return "Pressed";
  }
  return "";
}

RBY1_SDK_API inline std::string to_string(JointState::FETState s) {
  switch (s) {
    case JointState::FETState::kUnknown:
      return "Unknown";
    case JointState::FETState::kOn:
      return "On";
    case JointState::FETState::kOff:
      return "Off";
  }
  return "";
}

RBY1_SDK_API inline std::string to_string(JointState::RunState s) {
  switch (s) {
    case JointState::RunState::kUnknown:
      return "Unknown";
    case JointState::RunState::kControlOn:
      return "ControlOn";
    case JointState::RunState::kControlOff:
      return "ControlOff";
  }
  return "";
}

RBY1_SDK_API inline std::string to_string(JointState::InitializationState s) {
  switch (s) {
    case JointState::InitializationState::kUnknown:
      return "Unknown";
    case JointState::InitializationState::kInitialized:
      return "Initialized";
    case JointState::InitializationState::kUninitialized:
      return "Uninitialized";
  }
  return "";
}

}  // namespace rb