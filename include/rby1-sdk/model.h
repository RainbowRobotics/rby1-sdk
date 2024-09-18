#pragma once

#include <array>
#include <string>

#include "dynamics/robot.h"

namespace rb::y1_model {

class A {
 public:
  static constexpr std::string_view kModelName = "A";

  static constexpr size_t kRobotDOF = 24;

  static constexpr std::array<std::string_view, kRobotDOF> kRobotJointNames = {
      "right_wheel", "left_wheel",  "torso_0",     "torso_1",      "torso_2",     "torso_3",
      "torso_4",     "torso_5",     "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
      "right_arm_4", "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",  "left_arm_2",
      "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6",  "head_0",      "head_1"};
  static constexpr std::array<unsigned int, 2> kMobilityIdx = {0, 1};
  static constexpr std::array<unsigned int, 20> kBodyIdx = {2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                                                            12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
  static constexpr std::array<unsigned int, 2> kHeadIdx = {22, 23};
  static constexpr std::array<unsigned int, 7> kRightArmIdx = {8, 9, 10, 11, 12, 13, 14};
  static constexpr std::array<unsigned int, 7> kLeftArmIdx = {15, 16, 17, 18, 19, 20, 21};
  static constexpr std::array<unsigned int, 6> kTorsoIdx = {2, 3, 4, 5, 6, 7};
  static constexpr std::array<unsigned int, 20> kVelocityEstimationRequiredIdx = kBodyIdx;

  static constexpr double kControlPeriod = 0.002;  // (s)

  using DynRobotType = rb::dyn::Robot<kRobotDOF>;
  using DynRobotStateType = rb::dyn::State<kRobotDOF>;
};

}  // namespace rb::y1_model