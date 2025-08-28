#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/model.h"

namespace py = pybind11;
using namespace rb;

class PyModelBase {
 public:
  virtual ~PyModelBase() = default;

  virtual std::string_view get_model_name() const = 0;
};

template <typename T>
class PyModel : public PyModelBase {
 public:
  ~PyModel() override = default;

  std::string_view get_model_name() const override { return T::kModelName; }

  const auto& get_robot_dof() const { return T::kRobotDOF; }

  const auto& get_robot_joint_names() const { return T::kRobotJointNames; }

  const auto& get_mobility_idx() const { return T::kMobilityIdx; }

  const auto& get_body_idx() const { return T::kBodyIdx; }

  const auto& get_head_idx() const { return T::kHeadIdx; }

  const auto& get_right_arm_idx() const { return T::kRightArmIdx; }

  const auto& get_left_arm_idx() const { return T::kLeftArmIdx; }

  const auto& get_torso_idx() const { return T::kTorsoIdx; }

  const auto& get_velocity_estimation_required_idx() const { return T::kVelocityEstimationRequiredIdx; }

  const auto& get_control_period() const { return T::kControlPeriod; }
};