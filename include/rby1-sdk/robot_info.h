#pragma once

#include "export.h"

namespace rb {

struct RBY1_SDK_API BatteryInfo {};

struct RBY1_SDK_API PowerInfo {
  std::string name{};
};

struct RBY1_SDK_API EMOInfo {
  std::string name{};
};

struct RBY1_SDK_API JointInfo {
  std::string name{};

  bool has_brake{};

  std::string product_name{};

  std::string firmware_version{};
};

struct RBY1_SDK_API RobotInfo {
  std::string version{};

  std::string sdk_version{};

  std::string robot_model_name{};

  std::string robot_model_version{};

  BatteryInfo battery_info;

  std::vector<PowerInfo> power_infos{};

  std::vector<EMOInfo> emo_infos{};

  int degree_of_freedom{};

  std::vector<JointInfo> joint_infos{};

  std::vector<unsigned int> mobility_joint_idx{};

  std::vector<unsigned int> body_joint_idx{};

  std::vector<unsigned int> head_joint_idx{};

  std::vector<unsigned int> torso_joint_idx{};

  std::vector<unsigned int> right_arm_joint_idx{};

  std::vector<unsigned int> left_arm_joint_idx{};

  /**
   * @deprecated Use `robot_model_name` instead.
   */
  std::string robot_version{};

  /**
   * @deprecated Use `sdk_version` instead.
   */
  std::string sdk_commit_id{};
};

}  // namespace rb