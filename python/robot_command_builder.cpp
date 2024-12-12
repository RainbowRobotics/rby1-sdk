#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "rby1-sdk/robot_command_builder.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void pybind11_robot_command_builder(py::module_& m) {
  py::class_<CommandHeaderBuilder>(m, "CommandHeaderBuilder")
      .def(py::init<>())
      .def("set_control_hold_time", &CommandHeaderBuilder::SetControlHoldTime);

  py::class_<JointPositionCommandBuilder>(m, "JointPositionCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &JointPositionCommandBuilder::SetCommandHeader)
      .def("set_minimum_time", &JointPositionCommandBuilder::SetMinimumTime)
      .def("set_position", &JointPositionCommandBuilder::SetPosition)
      .def("set_velocity_limit", &JointPositionCommandBuilder::SetVelocityLimit)
      .def("set_acceleration_limit", &JointPositionCommandBuilder::SetAccelerationLimit);

  py::class_<OptimalControlCommandBuilder>(m, "OptimalControlCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &OptimalControlCommandBuilder::SetCommandHeader)
      .def("add_cartesian_target", &OptimalControlCommandBuilder::AddCartesianTarget)
      .def("set_center_of_mass_target", &OptimalControlCommandBuilder::SetCenterOfMassTarget)
      .def("add_joint_position_target", &OptimalControlCommandBuilder::AddJointPositionTarget)
      .def("set_velocity_limit_scaling", &OptimalControlCommandBuilder::SetVelocityLimitScaling)
      .def("set_velocity_tracking_gain", &OptimalControlCommandBuilder::SetVelocityTrackingGain)
      .def("set_stop_cost", &OptimalControlCommandBuilder::SetStopCost)
      .def("set_min_delta_cost", &OptimalControlCommandBuilder::SetMinDeltaCost)
      .def("set_patience", &OptimalControlCommandBuilder::SetPatience);

  py::class_<ImpedanceControlCommandBuilder>(m, "ImpedanceControlCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &ImpedanceControlCommandBuilder::SetCommandHeader)
      .def("set_reference_link_name", &ImpedanceControlCommandBuilder::SetReferenceLinkName)
      .def("set_link_name", &ImpedanceControlCommandBuilder::SetLinkName)
      .def("set_transformation", &ImpedanceControlCommandBuilder::SetTransformation)
      .def("set_translation_weight", &ImpedanceControlCommandBuilder::SetTranslationWeight)
      .def("set_rotation_weight", &ImpedanceControlCommandBuilder::SetRotationWeight);

  py::class_<JointVelocityCommandBuilder>(m, "JointVelocityCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &JointVelocityCommandBuilder::SetCommandHeader)
      .def("set_minimum_time", &JointVelocityCommandBuilder::SetMinimumTime)
      .def("set_velocity", &JointVelocityCommandBuilder::SetVelocity)
      .def("set_acceleration_limit", &JointVelocityCommandBuilder::SetAccelerationLimit);

  auto jcb = py::class_<JogCommandBuilder>(m, "JogCommandBuilder");

  py::class_<JogCommandBuilder::AbsolutePosition>(jcb, "AbsolutePosition")
      .def(py::init<double>())
      .def("value", &JogCommandBuilder::AbsolutePosition::value);

  py::class_<JogCommandBuilder::RelativePosition>(jcb, "RelativePosition")
      .def(py::init<double>())
      .def("value", &JogCommandBuilder::RelativePosition::value);

  py::class_<JogCommandBuilder::OneStep>(jcb, "OneStep")
      .def(py::init<double>())
      .def("value", &JogCommandBuilder::OneStep::value);

  jcb.def(py::init<>())
      .def("set_command_header", &JogCommandBuilder::SetCommandHeader)
      .def("set_joint_name", &JogCommandBuilder::SetJointName)
      .def("set_velocity_limit", &JogCommandBuilder::SetVelocityLimit)
      .def("set_acceleration_limit", &JogCommandBuilder::SetAccelerationLimit)
      .def("set_command", py::overload_cast<JogCommandBuilder::AbsolutePosition>(&JogCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<JogCommandBuilder::RelativePosition>(&JogCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<JogCommandBuilder::OneStep>(&JogCommandBuilder::SetCommand));

  py::class_<SE2VelocityCommandBuilder>(m, "SE2VelocityCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &SE2VelocityCommandBuilder::SetCommandHeader)
      .def("set_minimum_time", &SE2VelocityCommandBuilder::SetMinimumTime)
      .def("set_velocity", &SE2VelocityCommandBuilder::SetVelocity)
      .def("set_acceleration_limit", &SE2VelocityCommandBuilder::SetAccelerationLimit);

  py::class_<StopCommandBuilder>(m, "StopCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &StopCommandBuilder::SetCommandHeader);

  py::class_<CartesianCommandBuilder>(m, "CartesianCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &CartesianCommandBuilder::SetCommandHeader)
      .def("set_minimum_time", &CartesianCommandBuilder::SetMinimumTime)
      .def("add_target", &CartesianCommandBuilder::AddTarget)
      .def("add_joint_position_target", &CartesianCommandBuilder::AddJointPositionTarget, "joint_name"_a,
           "target_position"_a, "velocity_limit"_a = py::none(), "acceleration_limit"_a = py::none())
      .def("set_stop_position_tracking_error", &CartesianCommandBuilder::SetStopPositionTrackingError)
      .def("set_stop_orientation_tracking_error", &CartesianCommandBuilder::SetStopOrientationTrackingError);

  py::class_<GravityCompensationCommandBuilder>(m, "GravityCompensationCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &GravityCompensationCommandBuilder::SetCommandHeader)
      .def("set_on", &GravityCompensationCommandBuilder::SetOn);

  py::class_<ArmCommandBuilder>(m, "ArmCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def(py::init<const GravityCompensationCommandBuilder&>())
      .def(py::init<const CartesianCommandBuilder&>())
      .def(py::init<const ImpedanceControlCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&ArmCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&ArmCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&ArmCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const ImpedanceControlCommandBuilder&>(&ArmCommandBuilder::SetCommand));

  py::class_<TorsoCommandBuilder>(m, "TorsoCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def(py::init<const GravityCompensationCommandBuilder&>())
      .def(py::init<const CartesianCommandBuilder&>())
      .def(py::init<const ImpedanceControlCommandBuilder&>())
      .def(py::init<const OptimalControlCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&TorsoCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&TorsoCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&TorsoCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const ImpedanceControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const OptimalControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand));

  py::class_<BodyComponentBasedCommandBuilder>(m, "BodyComponentBasedCommandBuilder")
      .def(py::init<>())
      .def("set_right_arm_command", &BodyComponentBasedCommandBuilder::SetRightArmCommand)
      .def("set_left_arm_command", &BodyComponentBasedCommandBuilder::SetLeftArmCommand)
      .def("set_torso_command", &BodyComponentBasedCommandBuilder::SetTorsoCommand);

  py::class_<BodyCommandBuilder>(m, "BodyCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def(py::init<const OptimalControlCommandBuilder&>())
      .def(py::init<const GravityCompensationCommandBuilder&>())
      .def(py::init<const CartesianCommandBuilder&>())
      .def(py::init<const BodyComponentBasedCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&BodyCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const OptimalControlCommandBuilder&>(&BodyCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&BodyCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&BodyCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const BodyComponentBasedCommandBuilder&>(&BodyCommandBuilder::SetCommand));

  py::class_<MobilityCommandBuilder>(m, "MobilityCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointVelocityCommandBuilder&>())
      .def(py::init<const SE2VelocityCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointVelocityCommandBuilder&>(&MobilityCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const SE2VelocityCommandBuilder&>(&MobilityCommandBuilder::SetCommand));

  py::class_<HeadCommandBuilder>(m, "HeadCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&HeadCommandBuilder::SetCommand));

  py::class_<ComponentBasedCommandBuilder>(m, "ComponentBasedCommandBuilder")
      .def(py::init<>())
      .def("set_mobility_command", &ComponentBasedCommandBuilder::SetMobilityCommand)
      .def("set_body_command", &ComponentBasedCommandBuilder::SetBodyCommand)
      .def("set_head_command", &ComponentBasedCommandBuilder::SetHeadCommand);

  py::class_<WholeBodyCommandBuilder>(m, "WholeBodyCommandBuilder")
      .def(py::init<>())
      .def(py::init<const StopCommandBuilder&>())
      .def("set_command", py::overload_cast<const StopCommandBuilder&>(&WholeBodyCommandBuilder::SetCommand));

  py::class_<RobotCommandBuilder>(m, "RobotCommandBuilder")
      .def(py::init<>())
      .def(py::init<const WholeBodyCommandBuilder&>())
      .def(py::init<const ComponentBasedCommandBuilder&>())
      .def(py::init<const JogCommandBuilder&>())
      .def("set_command", py::overload_cast<const WholeBodyCommandBuilder&>(&RobotCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const ComponentBasedCommandBuilder&>(&RobotCommandBuilder::SetCommand))
      .def("set_command", py::overload_cast<const JogCommandBuilder&>(&RobotCommandBuilder::SetCommand));

  // Implicit conversion

  py::implicitly_convertible<double, JogCommandBuilder::AbsolutePosition>();

  py::implicitly_convertible<double, JogCommandBuilder::RelativePosition>();

  py::implicitly_convertible<double, JogCommandBuilder::OneStep>();

  py::implicitly_convertible<JointPositionCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<ImpedanceControlCommandBuilder, ArmCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<ImpedanceControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<OptimalControlCommandBuilder, TorsoCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<OptimalControlCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<BodyComponentBasedCommandBuilder, BodyCommandBuilder>();

  py::implicitly_convertible<JointVelocityCommandBuilder, MobilityCommandBuilder>();
  py::implicitly_convertible<SE2VelocityCommandBuilder, MobilityCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, HeadCommandBuilder>();

  py::implicitly_convertible<StopCommandBuilder, WholeBodyCommandBuilder>();

  py::implicitly_convertible<WholeBodyCommandBuilder, RobotCommandBuilder>();
  py::implicitly_convertible<ComponentBasedCommandBuilder, RobotCommandBuilder>();
  py::implicitly_convertible<JogCommandBuilder, RobotCommandBuilder>();
}