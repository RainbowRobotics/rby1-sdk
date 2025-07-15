#pragma once

#include <array>
#include <string>

#include "dynamics/robot.h"
#include "export.h"

namespace rb::y1_model {

/**
 * @class A
 * @brief Represents the model description of Robot A.
 *
 * This class represents the model description of Robot A, which serves as the base model.
 */
class RBY1_SDK_API A {
 public:
  /** @brief The model name. */
  static constexpr std::string_view kModelName = "A";

  /** @brief Total degrees of freedom for the robot. */
  static constexpr size_t kRobotDOF = 24;

  /**
   * @brief Names of the robot joints.
   *
   * This array contains the names of all joints in the robot,
   * arranged in the order of their indices.
   */
  static constexpr std::array<std::string_view, kRobotDOF> kRobotJointNames = {
      "right_wheel", "left_wheel",  "torso_0",     "torso_1",     "torso_2",     "torso_3",
      "torso_4",     "torso_5",     "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
      "right_arm_4", "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",  "left_arm_2",
      "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6",  "head_0",      "head_1"};

  /**
   * @brief Indices for the mobility components (e.g., wheels).
   *
   * These indices represent the parts of the robot responsible for mobility.
   */
  static constexpr std::array<unsigned int, 2> kMobilityIdx = {0, 1};

  /**
   * @brief Indices for the main body components.
   *
   * These indices cover all joints excluding the mobility and head components.
   */
  static constexpr std::array<unsigned int, 20> kBodyIdx = {2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                                                            12, 13, 14, 15, 16, 17, 18, 19, 20, 21};

  /** @brief Indices for the head components. */
  static constexpr std::array<unsigned int, 2> kHeadIdx = {22, 23};

  /** @brief Indices for the right arm components. */
  static constexpr std::array<unsigned int, 7> kRightArmIdx = {8, 9, 10, 11, 12, 13, 14};

  /** @brief Indices for the left arm components. */
  static constexpr std::array<unsigned int, 7> kLeftArmIdx = {15, 16, 17, 18, 19, 20, 21};

  /** @brief Indices for the torso components. */
  static constexpr std::array<unsigned int, 6> kTorsoIdx = {2, 3, 4, 5, 6, 7};

  /**
   * @brief Indices for joints requiring velocity estimation.
   *
   * These indices include joints where velocity estimation is essential.
   */
  static constexpr std::array<unsigned int, 20> kVelocityEstimationRequiredIdx = kBodyIdx;

  /** @brief Control update period in seconds. */
  static constexpr double kControlPeriod = 0.002;

  /** @brief Type definition for the dynamic robot. */
  using DynRobotType = rb::dyn::Robot<kRobotDOF>;

  /** @brief Type definition for the dynamic robot state. */
  using DynRobotStateType = rb::dyn::State<kRobotDOF>;
};

/**
 * @class T5
 * @brief Represents the model description of Robot T5.
 *
 * This class represents the model description of Robot T5, which is derived from the base model A.
 * The T5 model includes torso with 5 degrees of freedom,
 * achieved by removing the torso_0 joint from the base model.
 */
class RBY1_SDK_API T5 {
 public:
  static constexpr std::string_view kModelName = "T5";

  static constexpr size_t kRobotDOF = 23;

  static constexpr std::array<std::string_view, kRobotDOF> kRobotJointNames = {
      "right_wheel", "left_wheel",  "torso_0",     "torso_1",     "torso_2",     "torso_3",
      "torso_4",     "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4",
      "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",  "left_arm_2",  "left_arm_3",
      "left_arm_4",  "left_arm_5",  "left_arm_6",  "head_0",      "head_1"};
  static constexpr std::array<unsigned int, 2> kMobilityIdx = {0, 1};
  static constexpr std::array<unsigned int, 19> kBodyIdx = {2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                                                            12, 13, 14, 15, 16, 17, 18, 19, 20};
  static constexpr std::array<unsigned int, 2> kHeadIdx = {21, 22};
  static constexpr std::array<unsigned int, 7> kRightArmIdx = {7, 8, 9, 10, 11, 12, 13};
  static constexpr std::array<unsigned int, 7> kLeftArmIdx = {14, 15, 16, 17, 18, 19, 20};
  static constexpr std::array<unsigned int, 5> kTorsoIdx = {2, 3, 4, 5, 6};
  static constexpr std::array<unsigned int, 19> kVelocityEstimationRequiredIdx = kBodyIdx;

  static constexpr double kControlPeriod = 0.002;

  using DynRobotType = rb::dyn::Robot<kRobotDOF>;
  using DynRobotStateType = rb::dyn::State<kRobotDOF>;
};

/**
 * @class M
 * @brief Represents the model description of Robot M.
 *
 * This class represents the model description of Robot M, which is derived from the base model A.
 * The M model includes mecanum wheel mobile base instead differential type mobile base.
 */
class RBY1_SDK_API M {
 public:
  static constexpr std::string_view kModelName = "M";

  static constexpr size_t kRobotDOF = 26;

  static constexpr std::array<std::string_view, kRobotDOF> kRobotJointNames = {
      "wheel_fr",    "wheel_fl",    "wheel_rr",    "wheel_rl",    "torso_0",     "torso_1",     "torso_2",
      "torso_3",     "torso_4",     "torso_5",     "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
      "right_arm_4", "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",  "left_arm_2",  "left_arm_3",
      "left_arm_4",  "left_arm_5",  "left_arm_6",  "head_0",      "head_1"};
  static constexpr std::array<unsigned int, 4> kMobilityIdx = {0, 1, 2, 3};
  static constexpr std::array<unsigned int, 20> kBodyIdx = {4,  5,  6,  7,  8,  9,  10, 11, 12, 13,
                                                            14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
  static constexpr std::array<unsigned int, 2> kHeadIdx = {24, 25};
  static constexpr std::array<unsigned int, 7> kRightArmIdx = {10, 11, 12, 13, 14, 15, 16};
  static constexpr std::array<unsigned int, 7> kLeftArmIdx = {17, 18, 19, 20, 21, 22, 23};
  static constexpr std::array<unsigned int, 6> kTorsoIdx = {4, 5, 6, 7, 8, 9};
  static constexpr std::array<unsigned int, 20> kVelocityEstimationRequiredIdx = kBodyIdx;

  static constexpr double kControlPeriod = 0.002;

  using DynRobotType = rb::dyn::Robot<kRobotDOF>;
  using DynRobotStateType = rb::dyn::State<kRobotDOF>;
};

/**
 * Standard Upper Body
 * @brief Represents the model description of Robot UB
 */
class RBY1_SDK_API UB {
 public:
  static constexpr std::string_view kModelName = "UB";

  static constexpr size_t kRobotDOF = 18;

  static constexpr std::array<std::string_view, kRobotDOF> kRobotJointNames = {
      "torso_hp",    "torso_5",     "right_arm_0", "right_arm_1", "right_arm_2", "right_arm_3",
      "right_arm_4", "right_arm_5", "right_arm_6", "left_arm_0",  "left_arm_1",  "left_arm_2",
      "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6",  "head_0",      "head_1"};
  static constexpr std::array<unsigned int, 0> kMobilityIdx = {};
  static constexpr std::array<unsigned int, 16> kBodyIdx = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  static constexpr std::array<unsigned int, 2> kHeadIdx = {16, 17};
  static constexpr std::array<unsigned int, 7> kRightArmIdx = {2, 3, 4, 5, 6, 7, 8};
  static constexpr std::array<unsigned int, 7> kLeftArmIdx = {9, 10, 11, 12, 13, 14, 15};
  static constexpr std::array<unsigned int, 2> kTorsoIdx = {0, 1};
  static constexpr std::array<unsigned int, 16> kVelocityEstimationRequiredIdx = kBodyIdx;

  static constexpr double kControlPeriod = 0.002;

  using DynRobotType = rb::dyn::Robot<kRobotDOF>;
  using DynRobotStateType = rb::dyn::State<kRobotDOF>;
};

}  // namespace rb::y1_model