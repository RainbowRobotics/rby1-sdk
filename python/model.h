#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/model.h"

namespace py = pybind11;
using namespace rb;

class PyModelBase {
 public:
  virtual ~PyModelBase() = default;

  virtual std::string_view get_model_name() = 0;
};

template <typename T>
class PyModel : public PyModelBase {
 public:
  ~PyModel() override = default;

  std::string_view get_model_name() override { return T::kModelName; }

  auto get_robot_dof() { return T::kRobotDOF; }

  auto get_robot_joint_names() { return T::kRobotJointNames; }

  auto get_mobility_idx() { return T::kMobilityIdx; }

  auto get_body_idx() { return T::kBodyIdx; }

  auto get_head_idx() { return T::kHeadIdx; }

  auto get_right_arm_idx() { return T::kRightArmIdx; }

  auto get_left_arm_idx() { return T::kLeftArmIdx; }

  auto get_torso_idx() { return T::kTorsoIdx; }

  auto get_velocity_estimation_required_idx() { return T::kVelocityEstimationRequiredIdx; }

  auto get_control_period() { return T::kControlPeriod; }
};