#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/robot_command_feedback.h"

namespace py = pybind11;
using namespace rb;

void pybind11_robot_command_feedback(py::module_& m) {
  py::class_<Feedback>(m, "Feedback")  //
      .def(py::init<>())               //
      .def_property_readonly("valid", &Feedback::valid);

  py::class_<CommandHeaderFeedback, Feedback>(m, "CommandHeaderFeedback")
      .def(py::init<>())
      .def_property_readonly("finished", &CommandHeaderFeedback::finished);

  py::class_<CommandFeedback, Feedback>(m, "CommandFeedback")
      .def(py::init<>())
      .def_property_readonly("command_header", &CommandFeedback::command_header);

  py::class_<StopCommandFeedback, CommandFeedback>(m, "StopCommandFeedback").def(py::init<>());

  py::class_<SE2VelocityCommandFeedback, CommandFeedback>(m, "SE2VelocityCommandFeedback").def(py::init<>());

  py::class_<JogCommandFeedback, CommandFeedback>(m, "JogCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("target_joint_name", &JogCommandFeedback::target_joint_name);

  py::class_<JointVelocityCommandFeedback, CommandFeedback>(m, "JointVelocityCommandFeedback").def(py::init<>());

  py::class_<JointPositionCommandFeedback, CommandFeedback>(m, "JointPositionCommandFeedback").def(py::init<>());

  py::class_<CartesianCommandFeedback, CommandFeedback> ccf(m, "CartesianCommandFeedback");

  py::class_<CartesianCommandFeedback::TrackingError>(ccf, "TrackingError")
      .def(py::init<>())
      .def_readonly("position_error", &CartesianCommandFeedback::TrackingError::position_error)
      .def_readonly("rotation_error", &CartesianCommandFeedback::TrackingError::rotation_error);

  ccf.def(py::init<>()).def_property_readonly("tracking_errors", &CartesianCommandFeedback::tracking_errors);

  py::class_<GravityCompensationCommandFeedback, CommandFeedback>(m, "GravityCompensationCommandFeedback")
      .def(py::init<>());

  py::class_<ImpedanceControlCommandFeedback, CommandFeedback> iccf(m, "ImpedanceControlCommandFeedback");

  py::class_<ImpedanceControlCommandFeedback::TrackingError>(iccf, "TrackingError")
      .def(py::init<>())
      .def_readonly("position_error", &ImpedanceControlCommandFeedback::TrackingError::position_error)
      .def_readonly("rotation_error", &ImpedanceControlCommandFeedback::TrackingError::rotation_error);

  iccf.def(py::init<>()).def_property_readonly("tracking_error", &ImpedanceControlCommandFeedback::tracking_error);

  py::class_<OptimalControlCommandFeedback, CommandFeedback>(m, "OptimalControlCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("total_cost", &OptimalControlCommandFeedback::total_cost)
      .def_property_readonly("cartesian_costs", &OptimalControlCommandFeedback::cartesian_costs)
      .def_property_readonly("center_of_mass_cost", &OptimalControlCommandFeedback::center_of_mass_cost)
      .def_property_readonly("joint_position_costs", &OptimalControlCommandFeedback::joint_position_costs);

  py::class_<WholeBodyCommandFeedback, CommandFeedback>(m, "WholeBodyCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("stop_command", &WholeBodyCommandFeedback::stop_command);

  py::class_<ArmCommandFeedback, CommandFeedback>(m, "ArmCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("joint_position_command", &ArmCommandFeedback::joint_position_command)
      .def_property_readonly("gravity_compensation_command", &ArmCommandFeedback::gravity_compensation_command)
      .def_property_readonly("cartesian_command", &ArmCommandFeedback::cartesian_command)
      .def_property_readonly("impedance_control_command", &ArmCommandFeedback::impedance_control_command);

  py::class_<TorsoCommandFeedback, CommandFeedback>(m, "TorsoCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("joint_position_command", &TorsoCommandFeedback::joint_position_command)
      .def_property_readonly("gravity_compensation_command", &TorsoCommandFeedback::gravity_compensation_command)
      .def_property_readonly("cartesian_command", &TorsoCommandFeedback::cartesian_command)
      .def_property_readonly("impedance_control_command", &TorsoCommandFeedback::impedance_control_command)
      .def_property_readonly("optimal_control_command", &TorsoCommandFeedback::optimal_control_command);

  py::class_<BodyComponentBasedCommandFeedback, CommandFeedback>(m, "BodyComponentBasedCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("right_arm_command", &BodyComponentBasedCommandFeedback::right_arm_command)
      .def_property_readonly("left_arm_command", &BodyComponentBasedCommandFeedback::left_arm_command)
      .def_property_readonly("torso_command", &BodyComponentBasedCommandFeedback::torso_command);

  py::class_<HeadCommandFeedback, CommandFeedback>(m, "HeadCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("joint_position_command", &HeadCommandFeedback::joint_position_command);

  py::class_<BodyCommandFeedback, CommandFeedback>(m, "BodyCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("joint_position_command", &BodyCommandFeedback::joint_position_command)
      .def_property_readonly("optimal_control_command", &BodyCommandFeedback::optimal_control_command)
      .def_property_readonly("gravity_compensation_command", &BodyCommandFeedback::gravity_compensation_command)
      .def_property_readonly("cartesian_command", &BodyCommandFeedback::cartesian_command)
      .def_property_readonly("body_component_based_command", &BodyCommandFeedback::body_component_based_command);

  py::class_<MobilityCommandFeedback, CommandFeedback>(m, "MobilityCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("joint_velocity_command", &MobilityCommandFeedback::joint_velocity_command)
      .def_property_readonly("se2_velocity_command", &MobilityCommandFeedback::se2_velocity_command);

  py::class_<ComponentBasedCommandFeedback, CommandFeedback>(m, "ComponentBasedCommandFeedback")
      .def(py::init<>())
      .def_property_readonly("head_command", &ComponentBasedCommandFeedback::head_command)
      .def_property_readonly("body_command", &ComponentBasedCommandFeedback::body_command)
      .def_property_readonly("mobility_command", &ComponentBasedCommandFeedback::mobility_command);

  py::class_<RobotCommandFeedback, CommandFeedback> rcf(m, "RobotCommandFeedback");

  py::enum_<RobotCommandFeedback::Status>(rcf, "Status")
      .value("Idle", RobotCommandFeedback::Status::kIdle)
      .value("Initializing", RobotCommandFeedback::Status::kInitializing)
      .value("Running", RobotCommandFeedback::Status::kRunning)
      .value("Finished", RobotCommandFeedback::Status::kFinished);

  py::enum_<RobotCommandFeedback::FinishCode>(rcf, "FinishCode")
      .value("Unknown", RobotCommandFeedback::FinishCode::kUnknown)
      .value("Ok", RobotCommandFeedback::FinishCode::kOk)
      .value("Canceled", RobotCommandFeedback::FinishCode::kCanceled)
      .value("Preempted", RobotCommandFeedback::FinishCode::kPreempted)
      .value("InitializedFailed", RobotCommandFeedback::FinishCode::kInitializedFailed)
      .value("ControlManagerIdle", RobotCommandFeedback::FinishCode::kControlManagerIdle)
      .value("ControlManagerFault", RobotCommandFeedback::FinishCode::kControlManagerFault)
      .value("UnexpectedState", RobotCommandFeedback::FinishCode::kUnexpectedState);

  rcf.def(py::init<>())
      .def_property_readonly("whole_body_command", &RobotCommandFeedback::whole_body_command)
      .def_property_readonly("component_based_command", &RobotCommandFeedback::component_based_command)
      .def_property_readonly("jog_command", &RobotCommandFeedback::jog_command)
      .def_property_readonly("status", &RobotCommandFeedback::status)
      .def_property_readonly("finish_code", &RobotCommandFeedback::finish_code);
}
