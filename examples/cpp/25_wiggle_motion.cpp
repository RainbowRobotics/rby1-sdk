// ################### CAUTION ###################
// # CAUTION:
// # Ensure that the robot has enough surrounding clearance before running this example.
// ###############################################
//
// Wiggle Motion Demo
// This example moves the robot to the ready pose and performs a continuous circular
// "wiggle" motion on the torso joints using joint impedance control.
// The arms hold the ready pose in impedance mode.
// Amplitude ramps up smoothly on start and tapers to zero on Ctrl+C.
//
// Circular motion parameters:
//   - Amplitude : 0.13 rad
//   - Period    : 2.0 s
//   - pitch joints (torso_1,2,3 / Y-axis): sin(ωt)
//   - roll  joints (torso_0,4   / X-axis): cos(ωt)  ← 90° phase shift → circular trajectory
//   - torso_5 (Z-axis yaw): held at start position
//
// Usage example:
//   ./example_25_wiggle_motion --address 192.168.30.1:50051 --model m --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "00_helper.cpp"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

std::atomic<bool> g_stop_requested{false};
void SignalHandler(int) { g_stop_requested.store(true); }

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

// Ready pose from AGENTS.md (values in degrees, converted to radians below)
constexpr double kDeg2Rad = M_PI / 180.0;

// Circular wiggle parameters
// pitch joints (torso_1,2,3 — Y-axis): driven with sin(ωt)
// roll  joints (torso_0,4   — X-axis): driven with cos(ωt)  ← 90° phase shift
// The 90° phase difference between pitch and roll produces a circular trajectory.
constexpr double kAmplitude = 0.13;         // rad
constexpr double kOmega     = M_PI;         // angular frequency: 2π / T = 2π / 2.0 s = π rad/s
constexpr double kDt        = 0.01;         // 100 Hz control loop period (s)
constexpr double kMinTime   = kDt * 1.02;   // minimum_time: slightly longer than dt to give the controller
                                            // enough headroom between successive stream commands
constexpr double kHoldTime  = 5.0;          // control_hold_time (s) — keeps control alive between commands
constexpr double kAccelTime = 2.0;          // amplitude ramp-up duration at start (s)
constexpr double kDecelTime = 2.0;          // amplitude taper duration on Ctrl+C (s)

// Impedance parameters
// Torso : stiffness=400, torque_limit=500, damping=0.7
//         source: lerobot-robot-rby1 config_rby1.py (impedance_stiffness / impedance_torque_limit / impedance_damping_ratio)
// Arms  : stiffness=100, torque_limit=10, damping=1.0
//         source: rby1-sdk example 28_joint_impedance_control
constexpr double kTorsoStiffness  = 400.0;
constexpr double kTorsoTorqueLim  = 500.0;
constexpr double kTorsoDamping    =   0.7;
constexpr double kArmStiffness    = 100.0;
constexpr double kArmTorqueLim    =  10.0;
constexpr double kArmDamping      =   1.0;

template <typename ModelT>
bool MoveToReadyPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  constexpr size_t kTorsoDof = ModelT::kTorsoIdx.size();

  // Ready pose from POSE_PRESETS (degrees → radians)
  // A / M v1.0-v1.2 : torso [0,45,-90,45,0,0],  arms [0,∓5,0,-120,0,70,0]
  // M v1.3           : arms [0,∓5,0,-120,0,30,0] — indistinguishable at runtime; use 70°
  // UB               : torso [0,0] (2-DOF, zero), arms same
  Eigen::VectorXd torso(kTorsoDof);
  if constexpr (kTorsoDof == 6) {
    torso << 0.0, 45.0 * kDeg2Rad, -90.0 * kDeg2Rad, 45.0 * kDeg2Rad, 0.0, 0.0;
  } else {
    torso.setZero();  // UB: 2-DOF torso stays at zero
  }

  Eigen::VectorXd right_arm(7);
  right_arm << 0.0, -5.0 * kDeg2Rad, 0.0, -120.0 * kDeg2Rad, 0.0, 70.0 * kDeg2Rad, 0.0;

  Eigen::VectorXd left_arm(7);
  left_arm  << 0.0,  5.0 * kDeg2Rad, 0.0, -120.0 * kDeg2Rad, 0.0, 70.0 * kDeg2Rad, 0.0;

  std::cout << "Moving to ready pose..." << std::endl;
  return MoveJ<ModelT>(robot, &torso, &right_arm, &left_arm, 5.0);
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  auto robot = InitializeRobot<ModelT>(address, power, servo);
  if (!robot) {
    return 1;
  }

  // Explicitly reset and re-enable control manager (mirrors pattern from example 28).
  // This clears any lingering fault from a previous run before starting motion.
  robot->ResetFaultControlManager();
  if (!robot->EnableControlManager()) {
    std::cerr << "Failed to enable control manager" << std::endl;
    return 1;
  }

  std::cout << "===== Wiggle Motion Example =====" << std::endl;

  if (!MoveToReadyPose(robot)) {
    std::cerr << "Failed to move to ready pose." << std::endl;
    return 1;
  }

  // Read actual joint positions after arriving at the ready pose.
  // These become the oscillation centres: wiggle = start_pos ± amplitude.
  constexpr size_t kTorsoDof = ModelT::kTorsoIdx.size();   // 6
  constexpr size_t kRightDof = ModelT::kRightArmIdx.size(); // 7
  constexpr size_t kLeftDof  = ModelT::kLeftArmIdx.size();  // 7

  const Eigen::VectorXd full_pos = robot->GetState().position;

  Eigen::VectorXd torso_start(kTorsoDof);
  for (size_t i = 0; i < kTorsoDof; ++i) {
    torso_start(i) = full_pos(ModelT::kTorsoIdx[i]);
  }

  Eigen::VectorXd right_start(kRightDof);
  for (size_t i = 0; i < kRightDof; ++i) {
    right_start(i) = full_pos(ModelT::kRightArmIdx[i]);
  }

  Eigen::VectorXd left_start(kLeftDof);
  for (size_t i = 0; i < kLeftDof; ++i) {
    left_start(i) = full_pos(ModelT::kLeftArmIdx[i]);
  }

  auto stream = robot->CreateCommandStream(10);
  std::signal(SIGINT, SignalHandler);

  std::cout << "Running wiggle motion. Press Ctrl+C to stop." << std::endl;

  const auto t_start = std::chrono::steady_clock::now();
  bool stop_triggered = false;
  std::chrono::steady_clock::time_point t_stop;

  while (true) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // Latch stop time on first detection of the signal
    if (g_stop_requested.load() && !stop_triggered) {
      stop_triggered = true;
      t_stop = std::chrono::steady_clock::now();
      std::cout << "\nCtrl+C received — decelerating over " << kDecelTime << " s..." << std::endl;
    }

    // Ramp-up at start, ramp-down on Ctrl+C
    double amplitude;
    if (stop_triggered) {
      const double t_since_stop = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_stop).count();
      amplitude = kAmplitude * std::max(0.0, 1.0 - t_since_stop / kDecelTime);
    } else {
      amplitude = kAmplitude * std::min(1.0, elapsed / kAccelTime);
    }

    const double sin_val = amplitude * std::sin(kOmega * elapsed);  // pitch component
    const double cos_val = amplitude * std::cos(kOmega * elapsed);  // roll  component (90° lead)

    // Circular motion: pitch joints (Y-axis, indices 1,2,3) follow sin,
    // roll joints (X-axis, indices 0,4) follow cos. torso_5 (yaw) held at start.
    Eigen::VectorXd torso_q = torso_start;
    if (kTorsoDof >= 6) {
      torso_q(0) += cos_val;  // torso_0: roll  (X-axis)
      torso_q(1) += sin_val;  // torso_1: pitch (Y-axis)
      torso_q(2) += sin_val;  // torso_2: pitch (Y-axis)
      torso_q(3) += sin_val;  // torso_3: pitch (Y-axis)
      torso_q(4) += cos_val;  // torso_4: roll  (X-axis)
      // torso_q(5) unchanged — torso_5 (Z-axis yaw) held at start position
    }

    RobotCommandBuilder rc;
    rc.SetCommand(
        ComponentBasedCommandBuilder().SetBodyCommand(
            BodyComponentBasedCommandBuilder()
                .SetTorsoCommand(
                    JointImpedanceControlCommandBuilder()
                        .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(kHoldTime))
                        .SetPosition(torso_q)
                        .SetMinimumTime(kMinTime)
                        .SetStiffness(Eigen::VectorXd::Constant(kTorsoDof, kTorsoStiffness))
                        .SetDampingRatio(kTorsoDamping)
                        .SetTorqueLimit(Eigen::VectorXd::Constant(kTorsoDof, kTorsoTorqueLim)))
                .SetRightArmCommand(
                    JointImpedanceControlCommandBuilder()
                        .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(kHoldTime))
                        .SetPosition(right_start)
                        .SetMinimumTime(kMinTime)
                        .SetStiffness(Eigen::VectorXd::Constant(kRightDof, kArmStiffness))
                        .SetDampingRatio(kArmDamping)
                        .SetTorqueLimit(Eigen::VectorXd::Constant(kRightDof, kArmTorqueLim)))
                .SetLeftArmCommand(
                    JointImpedanceControlCommandBuilder()
                        .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(kHoldTime))
                        .SetPosition(left_start)
                        .SetMinimumTime(kMinTime)
                        .SetStiffness(Eigen::VectorXd::Constant(kLeftDof, kArmStiffness))
                        .SetDampingRatio(kArmDamping)
                        .SetTorqueLimit(Eigen::VectorXd::Constant(kLeftDof, kArmTorqueLim)))));

    stream->SendCommand(rc);

    if (stop_triggered) {
      const double t_since_stop = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_stop).count();
      if (t_since_stop >= kDecelTime) {
        std::cout << "Deceleration complete. Cancelling stream." << std::endl;
        stream->Cancel();
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";
  std::string servo = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg == "--servo" && i + 1 < argc) {
      servo = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
    } else if (servo == ".*") {
      servo = arg;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (address.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return Run<y1_model::A>(address, power, servo);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power, servo);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power, servo);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
