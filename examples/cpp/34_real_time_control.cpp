// Real Time Control Demo
// This example demonstrates how to control the robot using real time control.
//
// Scenario:
//   - Real-time control cannot use the builder type commands provided by the existing SDK.
//   - In this example, a separate controller is implemented and used.
//   1. Move to zero position
//   2. Start real-time control
//   3. Move to target position using TrapezoidalMotionGenerator
//   4. Wait for done (press Ctrl+C to stop)
//
// Usage example:
//   ./example_33_real_time_control --address 127.0.0.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/math/trapezoidal_motion_generator.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

std::atomic<bool> g_stop{false};
void SignalHandler(int) { g_stop = true; }

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

template <typename ModelT>
std::shared_ptr<Robot<ModelT>> InitializeRobot(const std::string& address, const std::string& power,
                                               const std::string& servo) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return nullptr;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return nullptr;
  }
  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to turn power (" << power << ") on" << std::endl;
      return nullptr;
    }
  }
  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Failed to servo (" << servo << ") on" << std::endl;
      return nullptr;
    }
  }

  auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Failed to reset control manager" << std::endl;
      return nullptr;
    }
  }
  if (!robot->EnableControlManager()) {
    std::cerr << "Failed to enable control manager" << std::endl;
    return nullptr;
  }

  return robot;
}

template <typename ModelT>
void MoveToZeroPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso = Eigen::VectorXd::Zero(ModelT::kTorsoIdx.size());
  Eigen::VectorXd right_arm = Eigen::VectorXd::Zero(ModelT::kRightArmIdx.size());
  Eigen::VectorXd left_arm = Eigen::VectorXd::Zero(ModelT::kLeftArmIdx.size());

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::exit(1);
  }
}

// RealTimeControl class — mirrors the Python RealTimeControl class
template <typename ModelT>
class RealTimeControl {
 public:
  static constexpr size_t kDOF = ModelT::kRobotDOF;

  RealTimeControl(const std::string& address, const std::string& power, const std::string& servo) {
    robot_ = InitializeRobot<ModelT>(address, power, servo);
    if (!robot_) {
      std::exit(1);
    }

    vel_limit_.setConstant(M_PI);
    acc_limit_.setConstant(M_PI);

    // Go to zero position
    MoveToZeroPose(robot_);
  }

  void SetTarget(const Eigen::Vector<double, kDOF>& position, double minimum_time = 1.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_position_ = position;
    minimum_time_ = minimum_time;
    has_target_ = true;
  }

  void Start() {
    rt_thread_ = std::thread([this]() {
      robot_->Control([this](const ControlState<ModelT>& state) -> ControlInput<ModelT> {
        return ControlCallback(state);
      });
    });
  }

  void WaitForDone() {
    while (rt_thread_.joinable()) {
      if (g_stop) {
        std::cout << "\nInterrupted! Stopping control stream gracefully..." << std::endl;
        is_running_ = false;
        break;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (rt_thread_.joinable()) {
      rt_thread_.join();
    }
  }

 private:
  ControlInput<ModelT> ControlCallback(const ControlState<ModelT>& state) {
    ControlInput<ModelT> input;

    if (!initialized_) {
      last_target_position_ = state.position;
      last_target_velocity_ = state.velocity;
      initialized_ = true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_target_) {
        typename TrapezoidalMotionGenerator<kDOF>::Input gen_inp;
        gen_inp.current_position = last_target_position_;
        gen_inp.current_velocity = last_target_velocity_;
        gen_inp.target_position = target_position_;
        gen_inp.velocity_limit = vel_limit_;
        gen_inp.acceleration_limit = acc_limit_;
        gen_inp.minimum_time = minimum_time_;
        generator_.Update(gen_inp);
        local_t_ = 0.002;
        has_target_ = false;
      }
    }

    auto out = generator_(local_t_);
    last_target_position_ = out.position;
    last_target_velocity_ = out.velocity;

    input.target = last_target_position_;
    input.feedback_gain.setConstant(10);
    input.feedforward_torque.setConstant(0);
    input.finish = !is_running_;

    local_t_ += 0.002;
    return input;
  }

  std::shared_ptr<Robot<ModelT>> robot_;
  TrapezoidalMotionGenerator<kDOF> generator_;
  Eigen::Vector<double, kDOF> vel_limit_;
  Eigen::Vector<double, kDOF> acc_limit_;

  std::mutex mutex_;
  Eigen::Vector<double, kDOF> target_position_;
  double minimum_time_{1.0};
  bool has_target_{false};

  Eigen::Vector<double, kDOF> last_target_position_;
  Eigen::Vector<double, kDOF> last_target_velocity_;
  bool initialized_{false};
  double local_t_{0.0};

  std::atomic<bool> is_running_{true};
  std::thread rt_thread_;
};

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  RealTimeControl<ModelT> rt_control(address, power, servo);
  rt_control.Start();

  double robot_minimum_time = 5.0;

  // Target position in degrees, converted to radians (same as Python)
  Eigen::Vector<double, ModelT::kRobotDOF> target;
  target.setZero();

  if constexpr (std::string_view(ModelT::kModelName) == "A") {
    // wheel(2) + torso(6) + right_arm(7) + left_arm(7) + head(2) = 24
    target << 0.0, 0.0,                                  // wheel
        0.0, 45.0, -90.0, 45.0, 0.0, 0.0,                // torso
        0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0,          // right arm
        0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0,           // left arm
        0.0, 0.0;                                          // head
  } else if constexpr (std::string_view(ModelT::kModelName) == "M") {
    // wheel(4) + torso(6) + right_arm(7) + left_arm(7) + head(2) = 26
    target << 0.0, 0.0, 0.0, 0.0,                        // wheel
        0.0, 45.0, -90.0, 45.0, 0.0, 0.0,                // torso
        0.0, -5.0, 0.0, -120.0, 0.0, 50.0, 0.0,          // right arm
        0.0, 5.0, 0.0, -120.0, 0.0, 50.0, 0.0,           // left arm
        0.0, 0.0;                                          // head
  }

  target = target * M_PI / 180.0;

  rt_control.SetTarget(target, robot_minimum_time);
  rt_control.WaitForDone();

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);

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

  std::cerr << "Unknown model: " << model << " (only 'a' and 'm' supported for this example)" << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
