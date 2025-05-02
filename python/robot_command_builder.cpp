#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/robot_command_builder.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void pybind11_robot_command_builder(py::module_& m) {
  py::class_<CommandHeaderBuilder>(m, "CommandHeaderBuilder")
      .def(py::init<>())
      .def("set_control_hold_time", &CommandHeaderBuilder::SetControlHoldTime, "control_hold_time"_a);

  py::class_<JointPositionCommandBuilder>(m, "JointPositionCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &JointPositionCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_minimum_time", &JointPositionCommandBuilder::SetMinimumTime, "minimum_time"_a)
      .def("set_position", &JointPositionCommandBuilder::SetPosition, "position"_a)
      .def("set_velocity_limit", &JointPositionCommandBuilder::SetVelocityLimit, "velocity_limit"_a)
      .def("set_acceleration_limit", &JointPositionCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a);

  py::class_<JointImpedanceControlCommandBuilder>(m, "JointImpedanceControlCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &JointImpedanceControlCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_minimum_time", &JointImpedanceControlCommandBuilder::SetMinimumTime, "minimum_time"_a)
      .def("set_position", &JointImpedanceControlCommandBuilder::SetPosition, "position"_a)
      .def("set_velocity_limit", &JointImpedanceControlCommandBuilder::SetVelocityLimit, "velocity_limit"_a)
      .def("set_acceleration_limit", &JointImpedanceControlCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a)
      .def("set_stiffness", &JointImpedanceControlCommandBuilder::SetStiffness, "stiffness"_a)
      .def("set_torque_limit", &JointImpedanceControlCommandBuilder::SetTorqueLimit, "torque_limit"_a)
      .def("set_damping_ratio", &JointImpedanceControlCommandBuilder::SetDampingRatio, "damping_ratio"_a);

  py::class_<OptimalControlCommandBuilder>(m, "OptimalControlCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &OptimalControlCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("add_cartesian_target", &OptimalControlCommandBuilder::AddCartesianTarget, "ref_link_name"_a, "link_name"_a,
           "T"_a, "translation_weight"_a, "rotation_weight"_a)
      .def("set_center_of_mass_target", &OptimalControlCommandBuilder::SetCenterOfMassTarget, "ref_link_name"_a,
           "pose"_a, "weight"_a)
      .def("add_joint_position_target", &OptimalControlCommandBuilder::AddJointPositionTarget, "joint_name"_a,
           "target_position"_a, "weight"_a)
      .def("set_error_scaling", &OptimalControlCommandBuilder::SetErrorScaling, "error_scaling"_a)
      .def("set_velocity_limit_scaling", &OptimalControlCommandBuilder::SetVelocityLimitScaling,
           "velocity_limit_scaling"_a)
      .def("set_acceleration_limit_scaling", &OptimalControlCommandBuilder::SetAccelerationLimitScaling,
           "acceleration_limit_scaling"_a)
      .def("set_stop_cost", &OptimalControlCommandBuilder::SetStopCost, "stop_cost"_a)
      .def("set_min_delta_cost", &OptimalControlCommandBuilder::SetMinDeltaCost, "min_delta_cost"_a)
      .def("set_patience", &OptimalControlCommandBuilder::SetPatience, "patience"_a);

  py::class_<ImpedanceControlCommandBuilder>(m, "ImpedanceControlCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &ImpedanceControlCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_reference_link_name", &ImpedanceControlCommandBuilder::SetReferenceLinkName, "reference_link_name"_a)
      .def("set_link_name", &ImpedanceControlCommandBuilder::SetLinkName, "link_name"_a)
      .def("set_transformation", &ImpedanceControlCommandBuilder::SetTransformation, "T"_a)
      .def("set_translation_weight", &ImpedanceControlCommandBuilder::SetTranslationWeight, "weight"_a)
      .def("set_rotation_weight", &ImpedanceControlCommandBuilder::SetRotationWeight, "weight"_a)
      .def("set_damping_ratio", &ImpedanceControlCommandBuilder::SetDampingRatio, "damping_ratio"_a);

  py::class_<JointVelocityCommandBuilder>(m, "JointVelocityCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &JointVelocityCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_minimum_time", &JointVelocityCommandBuilder::SetMinimumTime, "minimum_time"_a)
      .def("set_velocity", &JointVelocityCommandBuilder::SetVelocity, "velocity"_a)
      .def("set_acceleration_limit", &JointVelocityCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a);

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
      .def("set_command_header", &JogCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_joint_name", &JogCommandBuilder::SetJointName, "joint_name"_a)
      .def("set_velocity_limit", &JogCommandBuilder::SetVelocityLimit, "velocity_limite"_a)
      .def("set_acceleration_limit", &JogCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a)
      .def("set_command", py::overload_cast<JogCommandBuilder::AbsolutePosition>(&JogCommandBuilder::SetCommand),
           "absolute_position"_a)
      .def("set_command", py::overload_cast<JogCommandBuilder::RelativePosition>(&JogCommandBuilder::SetCommand),
           "relative_position"_a)
      .def("set_command", py::overload_cast<JogCommandBuilder::OneStep>(&JogCommandBuilder::SetCommand), "one_step"_a);

  py::class_<SE2VelocityCommandBuilder>(m, "SE2VelocityCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &SE2VelocityCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_minimum_time", &SE2VelocityCommandBuilder::SetMinimumTime, "minimum_time"_a)
      .def("set_velocity", &SE2VelocityCommandBuilder::SetVelocity, "linear"_a, "angular"_a)
      .def("set_acceleration_limit", &SE2VelocityCommandBuilder::SetAccelerationLimit, "linear"_a, "angular"_a);

  py::class_<StopCommandBuilder>(m, "StopCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &StopCommandBuilder::SetCommandHeader, "stop_command_builder"_a);

  py::class_<CartesianCommandBuilder>(m, "CartesianCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &CartesianCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_minimum_time", &CartesianCommandBuilder::SetMinimumTime, "minimum_time"_a)
      .def("add_target", &CartesianCommandBuilder::AddTarget, "ref_link_name"_a, "link_name"_a, "T"_a,
           "linear_velocity_limit"_a, "angular_velocity_limit"_a, "acceleration_limit_scaling"_a)
      .def("add_joint_position_target", &CartesianCommandBuilder::AddJointPositionTarget, "joint_name"_a,
           "target_position"_a, "velocity_limit"_a = py::none(), "acceleration_limit"_a = py::none())
      .def("set_stop_position_tracking_error", &CartesianCommandBuilder::SetStopPositionTrackingError,
           "stop_position_tracking_error"_a)
      .def("set_stop_orientation_tracking_error", &CartesianCommandBuilder::SetStopOrientationTrackingError,
           "stop_orientation_tracking_error"_a)
      .def("set_stop_joint_position_tracking_error", &CartesianCommandBuilder::SetStopJointPositionTrackingError,
           "stop_joint_position_tracking_error"_a);

  py::class_<GravityCompensationCommandBuilder>(m, "GravityCompensationCommandBuilder")
      .def(py::init<>())
      .def("set_command_header", &GravityCompensationCommandBuilder::SetCommandHeader,
           "gravity_compensation_command_builder"_a)
      .def("set_on", &GravityCompensationCommandBuilder::SetOn, "on"_a);

  py::class_<ArmCommandBuilder>(m, "ArmCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def(py::init<const GravityCompensationCommandBuilder&>())
      .def(py::init<const CartesianCommandBuilder&>())
      .def(py::init<const ImpedanceControlCommandBuilder&>())
      .def(py::init<const JointImpedanceControlCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "joint_position_command_builder"_a)
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "gravity_compensation_command_builder"_a)
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "cartesian_command_builder"_a)
      .def("set_command", py::overload_cast<const ImpedanceControlCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "impedance_control_command_builder"_a)
      .def("set_command", py::overload_cast<const JointImpedanceControlCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "joint_impedance_control_command_builder"_a);

  py::class_<TorsoCommandBuilder>(m, "TorsoCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def(py::init<const GravityCompensationCommandBuilder&>())
      .def(py::init<const CartesianCommandBuilder&>())
      .def(py::init<const ImpedanceControlCommandBuilder&>())
      .def(py::init<const OptimalControlCommandBuilder&>())
      .def(py::init<const JointImpedanceControlCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "joint_position_command_builder"_a)
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "gravity_compensation_command_builder"_a)
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "cartesian_command_builder"_a)
      .def("set_command", py::overload_cast<const ImpedanceControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "impedance_control_command_builder"_a)
      .def("set_command", py::overload_cast<const OptimalControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "optimal_control_command_builder"_a)
      .def("set_command",
           py::overload_cast<const JointImpedanceControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "joint_impedance_control_command_builder"_a);

  py::class_<BodyComponentBasedCommandBuilder>(m, "BodyComponentBasedCommandBuilder")
      .def(py::init<>())
      .def("set_right_arm_command", &BodyComponentBasedCommandBuilder::SetRightArmCommand, "arm_command_builder"_a)
      .def("set_left_arm_command", &BodyComponentBasedCommandBuilder::SetLeftArmCommand, "arm_command_builder"_a)
      .def("set_torso_command", &BodyComponentBasedCommandBuilder::SetTorsoCommand, "torso_command_builder"_a);

  py::class_<BodyCommandBuilder>(m, "BodyCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def(py::init<const OptimalControlCommandBuilder&>())
      .def(py::init<const GravityCompensationCommandBuilder&>())
      .def(py::init<const CartesianCommandBuilder&>())
      .def(py::init<const BodyComponentBasedCommandBuilder&>())
      .def(py::init<const JointImpedanceControlCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "joint_position_command_builder"_a)
      .def("set_command", py::overload_cast<const OptimalControlCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "optimal_control_command_builder"_a)
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "gravity_compensation_command_builder"_a)
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "cartesian_command_builder"_a)
      .def("set_command", py::overload_cast<const BodyComponentBasedCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "body_component_based_command_builder"_a)
      .def("set_command", py::overload_cast<const JointImpedanceControlCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "joint_impedance_control_command_builder"_a);

  py::class_<MobilityCommandBuilder>(m, "MobilityCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointVelocityCommandBuilder&>())
      .def(py::init<const SE2VelocityCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointVelocityCommandBuilder&>(&MobilityCommandBuilder::SetCommand),
           "joint_velocity_command_builder"_a)
      .def("set_command", py::overload_cast<const SE2VelocityCommandBuilder&>(&MobilityCommandBuilder::SetCommand),
           "se2_velocity_command_builder"_a);

  py::class_<HeadCommandBuilder>(m, "HeadCommandBuilder")
      .def(py::init<>())
      .def(py::init<const JointPositionCommandBuilder&>())
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&HeadCommandBuilder::SetCommand),
           "joint_position_command_builder"_a);

  py::class_<ComponentBasedCommandBuilder>(m, "ComponentBasedCommandBuilder")
      .def(py::init<>())
      .def("set_mobility_command", &ComponentBasedCommandBuilder::SetMobilityCommand, "mobility_command_builder"_a)
      .def("set_body_command", &ComponentBasedCommandBuilder::SetBodyCommand, "body_command_builder"_a)
      .def("set_head_command", &ComponentBasedCommandBuilder::SetHeadCommand, "head_command_builder"_a);

  py::class_<WholeBodyCommandBuilder>(m, "WholeBodyCommandBuilder")
      .def(py::init<>())
      .def(py::init<const StopCommandBuilder&>())
      .def("set_command", py::overload_cast<const StopCommandBuilder&>(&WholeBodyCommandBuilder::SetCommand),
           "stop_command_builder"_a);

  py::class_<RobotCommandBuilder>(m, "RobotCommandBuilder")
      .def(py::init<>())
      .def(py::init<const WholeBodyCommandBuilder&>())
      .def(py::init<const ComponentBasedCommandBuilder&>())
      .def(py::init<const JogCommandBuilder&>())
      .def("set_command", py::overload_cast<const WholeBodyCommandBuilder&>(&RobotCommandBuilder::SetCommand),
           "whole_body_command_builder"_a)
      .def("set_command", py::overload_cast<const ComponentBasedCommandBuilder&>(&RobotCommandBuilder::SetCommand),
           "component_based_command_builder"_a)
      .def("set_command", py::overload_cast<const JogCommandBuilder&>(&RobotCommandBuilder::SetCommand),
           "jog_command_builder"_a);

  // Implicit conversion

  py::implicitly_convertible<double, JogCommandBuilder::AbsolutePosition>();

  py::implicitly_convertible<double, JogCommandBuilder::RelativePosition>();

  py::implicitly_convertible<double, JogCommandBuilder::OneStep>();

  py::implicitly_convertible<JointPositionCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<ImpedanceControlCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<JointImpedanceControlCommandBuilder, ArmCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<ImpedanceControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<OptimalControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<JointImpedanceControlCommandBuilder, TorsoCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<OptimalControlCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<BodyComponentBasedCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<JointImpedanceControlCommandBuilder, BodyCommandBuilder>();

  py::implicitly_convertible<JointVelocityCommandBuilder, MobilityCommandBuilder>();
  py::implicitly_convertible<SE2VelocityCommandBuilder, MobilityCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, HeadCommandBuilder>();

  py::implicitly_convertible<StopCommandBuilder, WholeBodyCommandBuilder>();

  py::implicitly_convertible<WholeBodyCommandBuilder, RobotCommandBuilder>();
  py::implicitly_convertible<ComponentBasedCommandBuilder, RobotCommandBuilder>();
  py::implicitly_convertible<JogCommandBuilder, RobotCommandBuilder>();
}