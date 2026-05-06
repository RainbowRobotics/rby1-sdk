// Impedance Control Demo
// This example demonstrates how to connect to an RB-Y1 robot, set motion parameters,
// perform an initial joint motion, send a Cartesian command, and then run an impedance
// control command for the right arm.
//
// Usage example:
//   ./example_28_cartesian_impedance_control --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.


#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "00_helper.cpp"
#include "rby1-sdk/math/liegroup.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;

namespace {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

Eigen::Matrix3d RotY(double angle_rad) {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  Eigen::Matrix3d r;
  r << c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c;
  return r;
}

math::SE3::MatrixType MakeTransform(const Eigen::Matrix3d& r, const Eigen::Vector3d& t) {
  math::SE3::MatrixType T = math::SE3::MatrixType::Identity();
  T.block<3, 3>(0, 0) = r;
  T.block<3, 1>(0, 3) = t;
  return T;
}

struct CartesianParams {
  double minimum_time{5.0};
  double linear_velocity_limit{1.5};
  double angular_velocity_limit{M_PI * 1.5};
  double acceleration_limit_scaling{1.0};
  double stop_position_tracking_error{1e-5};
  double stop_orientation_tracking_error{1e-5};
};

template <typename ModelT>
std::string BodyLinkName() {
  std::string model_name(ModelT::kModelName);
  if (model_name == "A" || model_name == "M") {
    return "link_torso_5";
  }
  return "link_torso_5";
}

template <typename ModelT>
bool MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso(6);
  Eigen::VectorXd right_arm(7);
  Eigen::VectorXd left_arm(7);
  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  return MoveJ<ModelT>(robot, &torso, &right_arm, &left_arm, 5.0);
}

template <typename ModelT>
bool ExampleCartesianCommand(const std::shared_ptr<Robot<ModelT>>& robot) {
  std::cout << "===== Cartesian Command Example =====" << std::endl;

  const double torso_angle = M_PI / 6.0;
  const double right_angle = -M_PI / 4.0;
  const double left_angle = -M_PI / 4.0;

  const auto T_torso = MakeTransform(RotY(torso_angle), Eigen::Vector3d(0.1, 0.0, 1.2));
  const auto T_right = MakeTransform(RotY(right_angle), Eigen::Vector3d(0.4, -0.4, 0.0));
  const auto T_left = MakeTransform(RotY(left_angle), Eigen::Vector3d(0.4, 0.4, 0.0));

  const CartesianParams params;
  const std::string body_link = BodyLinkName<ModelT>();

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    CartesianCommandBuilder()
                        .AddTarget("base", body_link, T_torso, params.linear_velocity_limit,
                                   params.angular_velocity_limit, params.acceleration_limit_scaling)
                        .AddTarget(body_link, "ee_right", T_right, params.linear_velocity_limit,
                                   params.angular_velocity_limit, params.acceleration_limit_scaling)
                        .AddTarget(body_link, "ee_left", T_left, params.linear_velocity_limit,
                                   params.angular_velocity_limit, params.acceleration_limit_scaling)
                        .SetStopPositionTrackingError(params.stop_position_tracking_error)
                        .SetStopOrientationTrackingError(params.stop_orientation_tracking_error)
                        .SetMinimumTime(params.minimum_time))))
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Failed to conduct 'Cartesian Command' example." << std::endl;
    return false;
  }

  return true;
}

template <typename ModelT>
bool ExampleImpedanceControlCommand(const std::shared_ptr<Robot<ModelT>>& robot) {
  std::cout << "===== Impedance Control Command Example =====" << std::endl;

  const double right_angle = -M_PI / 4.0;
  const auto T_right = MakeTransform(RotY(right_angle), Eigen::Vector3d(0.4, -0.4, 0.0));

  const std::string body_link = BodyLinkName<ModelT>();

  ImpedanceControlCommandBuilder right_arm_command;
  right_arm_command
      .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(10.0))
      .SetReferenceLinkName(body_link)
      .SetLinkName("ee_right")
      .SetTranslationWeight(Eigen::Vector3d(3000, 3000, 0))
      .SetRotationWeight(Eigen::Vector3d(50, 50, 50))
      .SetTransformation(T_right);

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(
                    ComponentBasedCommandBuilder().SetBodyCommand(
                        BodyComponentBasedCommandBuilder().SetRightArmCommand(right_arm_command))))
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Failed to conduct 'Impedance Control' example." << std::endl;
    return false;
  }

  return true;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  auto robot = InitializeRobot<ModelT>(address, power, servo);
  if (!robot) {
    return 1;
  }

  robot->SetParameter("default.acceleration_limit_scaling", "0.8");
  robot->SetParameter("joint_position_command.cutoff_frequency", "5");
  robot->SetParameter("cartesian_command.cutoff_frequency", "5");
  robot->SetParameter("default.linear_acceleration_limit", "5");

  if (!MoveToPreControlPose<ModelT>(robot)) {
    return 1;
  }

  if (!ExampleCartesianCommand<ModelT>(robot)) {
    return 1;
  }

  if (!ExampleImpedanceControlCommand<ModelT>(robot)) {
    return 1;
  }

  std::cout << "All examples finished successfully." << std::endl;
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

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
