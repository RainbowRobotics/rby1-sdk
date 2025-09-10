#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/robot_command_feedback.h"

namespace py = pybind11;
using namespace rb;

void pybind11_robot_command_feedback(py::module_& m) {
  py::class_<Feedback>(m, "Feedback", R"doc(
Base type for all feedback messages.

Use :py:meth:`valid` to check whether the parser filled this feedback instance.

Attributes
----------
valid : bool
    Whether this feedback object contains valid data.
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``Feedback`` instance.
)doc")
      .def_property_readonly("valid", &Feedback::valid, R"doc(
Whether this feedback object contains valid data.

Returns
-------
bool
    ``True`` if the parser filled this feedback; otherwise ``False``.
)doc");

  py::class_<CommandHeaderFeedback, Feedback>(m, "CommandHeaderFeedback", R"doc(
Header-level feedback common to all commands.

Attributes
----------
finished : bool
    Whether the command has finished (as assessed by the controller).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``CommandHeaderFeedback`` instance.
)doc")
      .def_property_readonly("finished", &CommandHeaderFeedback::finished, R"doc(
Whether the command has finished.

Returns
-------
bool
    ``True`` if the command is finished; otherwise ``False``.
)doc");

  py::class_<CommandFeedback, Feedback>(m, "CommandFeedback", R"doc(
Base type of feedback for specific command categories.

Attributes
----------
command_header : CommandHeaderFeedback
    Header-level feedback shared by all command types.
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``CommandFeedback`` instance.
)doc")
      .def_property_readonly("command_header", &CommandFeedback::command_header, R"doc(
Header-level feedback.

Returns
-------
CommandHeaderFeedback
    Header feedback (e.g., finished flag).
)doc");

  py::class_<StopCommandFeedback, CommandFeedback>(m, "StopCommandFeedback", R"doc(
Feedback for a whole-body stop command.

(Contains only header-level fields via :pyattr:`CommandFeedback.command_header`.)
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``StopCommandFeedback`` instance.
)doc");

  py::class_<SE2VelocityCommandFeedback, CommandFeedback>(m, "SE2VelocityCommandFeedback", R"doc(
Feedback for an SE(2) base velocity command.

(Contains only header-level fields via :pyattr:`CommandFeedback.command_header`.)
)doc")
      .def(py::init<>(), R"doc(
      Construct an ``SE2VelocityCommandFeedback`` instance.
)doc");

  py::class_<JogCommandFeedback, CommandFeedback>(m, "JogCommandFeedback", R"doc(
Feedback for a single-joint jog command.

Attributes
----------
target_joint_name : str
    The name of the jogged joint.
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``JogCommandFeedback`` instance.
)doc")
      .def_property_readonly("target_joint_name", &JogCommandFeedback::target_joint_name, R"doc(
Name of the jogged joint.

Returns
-------
str
    Target joint name.
)doc");

  py::class_<JointVelocityCommandFeedback, CommandFeedback>(m, "JointVelocityCommandFeedback", R"doc(
Feedback for a joint-space velocity command.

(Contains only header-level fields via :pyattr:`CommandFeedback.command_header`.)
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``JointVelocityCommandFeedback`` instance.
)doc");

  py::class_<JointPositionCommandFeedback, CommandFeedback>(m, "JointPositionCommandFeedback", R"doc(
Feedback for a joint-space position command.

Attributes
----------
time_based_progress : float
    Progress estimate [0.0, 1.0] based on time.
position_based_progress : float
    Progress estimate [0.0, 1.0] based on position error reduction.
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``JointPositionCommandFeedback`` instance.
)doc")
      .def_property_readonly("time_based_progress", &JointPositionCommandFeedback::time_based_progress, R"doc(
Time-based progress estimate.

Returns
-------
float
    Value in [0.0, 1.0].
)doc")
      .def_property_readonly("position_based_progress", &JointPositionCommandFeedback::position_based_progress, R"doc(
Position-based progress estimate.

Returns
-------
float
    Value in [0.0, 1.0].
)doc");

  py::class_<JointGroupPositionCommandFeedback, CommandFeedback>(m, "JointGroupPositionCommandFeedback", R"doc(
Feedback for a joint group position command.

Attributes
----------
joint_indices : list[int]
    Indices of the joints targeted by this group command (model order).
time_based_progress : float
    Progress estimate [0.0, 1.0] based on time.
position_based_progress : float
    Progress estimate [0.0, 1.0] based on position error reduction.
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``JointGroupPositionCommandFeedback`` instance.
)doc")
      .def_property_readonly("joint_indices", &JointGroupPositionCommandFeedback::joint_indices, R"doc(
Indices of the joints in the group command.

Returns
-------
list[int]
    Model-order joint indices.
)doc")
      .def_property_readonly("time_based_progress", &JointGroupPositionCommandFeedback::time_based_progress, R"doc(
Time-based progress estimate.

Returns
-------
float
    Value in [0.0, 1.0].
)doc")
      .def_property_readonly("position_based_progress", &JointGroupPositionCommandFeedback::position_based_progress,
                             R"doc(
Position-based progress estimate.

Returns
-------
float
    Value in [0.0, 1.0].
)doc");

  py::class_<CartesianCommandFeedback, CommandFeedback> ccf(m, "CartesianCommandFeedback", R"doc(
Feedback for a Cartesian command with one or more SE(3) pose targets.

Attributes
----------
se3_pose_tracking_errors : list[CartesianCommandFeedback.TrackingError]
    Per-target pose tracking errors (position/orientation).
joint_position_tracking_errors : list[float]
    Per-joint position tracking errors [rad].
remain_time : float
    Estimated remaining time to reach targets [s].
manipulability : float
    Manipulability index (dimensionless).
)doc");

  py::class_<CartesianCommandFeedback::TrackingError>(ccf, "TrackingError", R"doc(
Per-target SE(3) tracking error.

Attributes
----------
position_error : float
    Translational error magnitude [m].
orientation_error : float
    Rotational error magnitude [rad] (e.g., angle-axis norm).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``CartesianCommandFeedback.TrackingError`` instance.
)doc")
      .def_readonly("position_error", &CartesianCommandFeedback::TrackingError::position_error)
      .def_readonly("orientation_error", &CartesianCommandFeedback::TrackingError::orientation_error);

  ccf.def(py::init<>(), R"doc(
      Construct a ``CartesianCommandFeedback`` instance.
)doc")
      .def_property_readonly("se3_pose_tracking_errors", &CartesianCommandFeedback::se3_pose_tracking_errors, R"doc(
Per-target SE(3) pose tracking errors.

Returns
-------
list[CartesianCommandFeedback.TrackingError]
    Tracking error objects for each Cartesian target.
)doc")
      .def_property_readonly("joint_position_tracking_errors",
                             &CartesianCommandFeedback::joint_position_tracking_errors, R"doc(
Per-joint position tracking errors.

Returns
-------
list[float]
    Joint-wise position error magnitudes [rad].
)doc")
      .def_property_readonly("remain_time", &CartesianCommandFeedback::remain_time, R"doc(
Estimated remaining time to reach targets.

Returns
-------
float
    Time remaining [s].
)doc")
      .def_property_readonly("manipulability", &CartesianCommandFeedback::manipulability, R"doc(
Manipulability index.

Returns
-------
float
    Dimensionless manipulability measure (higher is generally better).
)doc");

  py::class_<CartesianImpedanceControlCommandFeedback, CommandFeedback>(m, "CartesianImpedanceControlCommandFeedback",
                                                                        R"doc(
Feedback for a Cartesian-impedance control command.

Attributes
----------
set_position : numpy.ndarray, shape (N,), dtype=float64
    Current joint setpoint used by the internal joint-space controller [rad].
remain_time : float
    Estimated remaining time to settle [s].
manipulability : float
    Manipulability index (dimensionless).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``CartesianImpedanceControlCommandFeedback`` instance.
)doc")
      .def_property_readonly("set_position", &CartesianImpedanceControlCommandFeedback::set_position, R"doc(
Current joint setpoint used by the internal joint-space controller.

Returns
-------
numpy.ndarray
    Joint setpoint vector [rad], shape (N,).
)doc")
      .def_property_readonly("remain_time", &CartesianImpedanceControlCommandFeedback::remain_time, R"doc(
Estimated remaining time to settle.

Returns
-------
float
    Time remaining [s].
)doc")
      .def_property_readonly("manipulability", &CartesianImpedanceControlCommandFeedback::manipulability, R"doc(
Manipulability index.

Returns
-------
float
    Dimensionless manipulability measure.
)doc");

  py::class_<GravityCompensationCommandFeedback, CommandFeedback>(m, "GravityCompensationCommandFeedback", R"doc(
Feedback for a gravity-compensation command.

(Contains only header-level fields via :pyattr:`CommandFeedback.command_header`.)
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``GravityCompensationCommandFeedback`` instance.
)doc");

  py::class_<ImpedanceControlCommandFeedback, CommandFeedback> iccf(m, "ImpedanceControlCommandFeedback", R"doc(
Feedback for a Cartesian impedance control (single-link) command.

Attributes
----------
tracking_error : ImpedanceControlCommandFeedback.TrackingError
    Current tracking error (translation/rotation).
)doc");

  py::class_<ImpedanceControlCommandFeedback::TrackingError>(iccf, "TrackingError", R"doc(
Tracking error for Cartesian impedance.

Attributes
----------
position_error : float
    Translational error magnitude [m].
rotation_error : float
    Rotational error magnitude [rad] (e.g., angle-axis norm).
)doc")
      .def(py::init<>(), R"doc(
      Construct an ``ImpedanceControlCommandFeedback.TrackingError`` instance.
)doc")
      .def_readonly("position_error", &ImpedanceControlCommandFeedback::TrackingError::position_error)
      .def_readonly("rotation_error", &ImpedanceControlCommandFeedback::TrackingError::rotation_error);

  iccf.def(py::init<>(), R"doc(
      Construct an ``ImpedanceControlCommandFeedback`` instance.
)doc")
      .def_property_readonly("tracking_error", &ImpedanceControlCommandFeedback::tracking_error, R"doc(
Current tracking error.

Returns
-------
ImpedanceControlCommandFeedback.TrackingError
    Translational and rotational error magnitudes.
)doc");

  py::class_<JointImpedanceControlCommandFeedback, CommandFeedback>(m, "JointImpedanceControlCommandFeedback", R"doc(
Feedback for a joint-space impedance control command.

Attributes
----------
set_position : list[float]
    Current joint setpoints [rad].
error : list[float]
    Joint-space errors (q_d - q) [rad].
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``JointImpedanceControlCommandFeedback`` instance.
)doc")
      .def_property_readonly("set_position", &JointImpedanceControlCommandFeedback::set_position, R"doc(
Current joint setpoints.

Returns
-------
list[float]
    Joint setpoints [rad].
)doc")
      .def_property_readonly("error", &JointImpedanceControlCommandFeedback::error, R"doc(
Joint-space errors.

Returns
-------
list[float]
    Per-joint position errors [rad].
)doc");

  py::class_<OptimalControlCommandFeedback, CommandFeedback>(m, "OptimalControlCommandFeedback", R"doc(
Feedback for an optimal-control command.

Attributes
----------
total_cost : float
    Total objective value (dimensionless).
cartesian_costs : list[float]
    Per-Cartesian-target costs (dimensionless).
center_of_mass_cost : float
    Center-of-mass cost (dimensionless).
joint_position_costs : list[float]
    Per-joint position costs (dimensionless).
)doc")
      .def(py::init<>(), R"doc(
      Construct an ``OptimalControlCommandFeedback`` instance.
)doc")
      .def_property_readonly("total_cost", &OptimalControlCommandFeedback::total_cost, R"doc(
Total objective value.

Returns
-------
float
    Dimensionless cost value.
)doc")
      .def_property_readonly("cartesian_costs", &OptimalControlCommandFeedback::cartesian_costs, R"doc(
Per-target Cartesian costs.

Returns
-------
list[float]
    Dimensionless cost values for each Cartesian target.
)doc")
      .def_property_readonly("center_of_mass_cost", &OptimalControlCommandFeedback::center_of_mass_cost, R"doc(
Center-of-mass cost.

Returns
-------
float
    Dimensionless COM cost.
)doc")
      .def_property_readonly("joint_position_costs", &OptimalControlCommandFeedback::joint_position_costs, R"doc(
Per-joint position costs.

Returns
-------
list[float]
    Dimensionless cost values for joint position terms.
)doc");

  py::class_<WholeBodyCommandFeedback, CommandFeedback>(m, "WholeBodyCommandFeedback", R"doc(
Feedback for a whole-body command container.

Attributes
----------
stop_command : StopCommandFeedback
    Feedback of the contained stop command (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``WholeBodyCommandFeedback`` instance.
)doc")
      .def_property_readonly("stop_command", &WholeBodyCommandFeedback::stop_command, R"doc(
Contained stop command feedback.

Returns
-------
StopCommandFeedback
    Stop command feedback object.
)doc");

  py::class_<ArmCommandFeedback, CommandFeedback>(m, "ArmCommandFeedback", R"doc(
Feedback for an arm command container.

Attributes
----------
joint_position_command : JointPositionCommandFeedback
    Feedback for a joint position command (if present).
gravity_compensation_command : GravityCompensationCommandFeedback
    Feedback for a gravity compensation command (if present).
cartesian_command : CartesianCommandFeedback
    Feedback for a Cartesian command (if present).
impedance_control_command : ImpedanceControlCommandFeedback
    Feedback for a Cartesian impedance command (if present).
cartesian_impedance_control_command : CartesianImpedanceControlCommandFeedback
    Feedback for a Cartesian impedance (OC + joint-space impedance) command (if present).
joint_impedance_control_command : JointImpedanceControlCommandFeedback
    Feedback for a joint-space impedance command (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct an ``ArmCommandFeedback`` instance.
)doc")
      .def_property_readonly("joint_position_command", &ArmCommandFeedback::joint_position_command, R"doc(
Joint position command feedback.

Returns
-------
JointPositionCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("gravity_compensation_command", &ArmCommandFeedback::gravity_compensation_command, R"doc(
Gravity compensation command feedback.

Returns
-------
GravityCompensationCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("cartesian_command", &ArmCommandFeedback::cartesian_command, R"doc(
Cartesian command feedback.

Returns
-------
CartesianCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("impedance_control_command", &ArmCommandFeedback::impedance_control_command, R"doc(
Cartesian impedance command feedback.

Returns
-------
ImpedanceControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("cartesian_impedance_control_command",
                             &ArmCommandFeedback::cartesian_impedance_control_command, R"doc(
Cartesian impedance (OC + joint-space impedance) command feedback.

Returns
-------
CartesianImpedanceControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("joint_impedance_control_command", &ArmCommandFeedback::joint_impedance_control_command,
                             R"doc(
Joint-space impedance command feedback.

Returns
-------
JointImpedanceControlCommandFeedback
    Feedback object.
)doc");

  py::class_<TorsoCommandFeedback, CommandFeedback>(m, "TorsoCommandFeedback", R"doc(
Feedback for a torso command container.

Attributes
----------
joint_position_command : JointPositionCommandFeedback
    Feedback for a joint position command (if present).
gravity_compensation_command : GravityCompensationCommandFeedback
    Feedback for a gravity compensation command (if present).
cartesian_command : CartesianCommandFeedback
    Feedback for a Cartesian command (if present).
impedance_control_command : ImpedanceControlCommandFeedback
    Feedback for a Cartesian impedance command (if present).
optimal_control_command : OptimalControlCommandFeedback
    Feedback for an optimal-control command (if present).
cartesian_impedance_control_command : CartesianImpedanceControlCommandFeedback
    Feedback for a Cartesian impedance (OC + joint-space impedance) command (if present).
joint_impedance_control_command : JointImpedanceControlCommandFeedback
    Feedback for a joint-space impedance command (if present).
joint_group_position_command : JointGroupPositionCommandFeedback
    Feedback for a joint-group position command (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``TorsoCommandFeedback`` instance.
)doc")
      .def_property_readonly("joint_position_command", &TorsoCommandFeedback::joint_position_command, R"doc(
Joint position command feedback.

Returns
-------
JointPositionCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("gravity_compensation_command", &TorsoCommandFeedback::gravity_compensation_command, R"doc(
Gravity compensation command feedback.

Returns
-------
GravityCompensationCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("cartesian_command", &TorsoCommandFeedback::cartesian_command, R"doc(
Cartesian command feedback.

Returns
-------
CartesianCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("impedance_control_command", &TorsoCommandFeedback::impedance_control_command, R"doc(
Cartesian impedance command feedback.

Returns
-------
ImpedanceControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("optimal_control_command", &TorsoCommandFeedback::optimal_control_command, R"doc(
Optimal-control command feedback.

Returns
-------
OptimalControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("cartesian_impedance_control_command",
                             &TorsoCommandFeedback::cartesian_impedance_control_command, R"doc(
Cartesian impedance (OC + joint-space impedance) command feedback.

Returns
-------
CartesianImpedanceControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("joint_impedance_control_command", &TorsoCommandFeedback::joint_impedance_control_command,
                             R"doc(
Joint-space impedance command feedback.

Returns
-------
JointImpedanceControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("joint_group_position_command", &TorsoCommandFeedback::joint_group_position_command, R"doc(
Joint-group position command feedback.

Returns
-------
JointGroupPositionCommandFeedback
    Feedback object.
)doc");

  py::class_<BodyComponentBasedCommandFeedback, CommandFeedback>(m, "BodyComponentBasedCommandFeedback", R"doc(
Feedback for a body command composed of named components.

Attributes
----------
right_arm_command : ArmCommandFeedback
    Feedback for the right arm component.
left_arm_command : ArmCommandFeedback
    Feedback for the left arm component.
torso_command : TorsoCommandFeedback
    Feedback for the torso component.
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``BodyComponentBasedCommandFeedback`` instance.
)doc")
      .def_property_readonly("right_arm_command", &BodyComponentBasedCommandFeedback::right_arm_command, R"doc(
Right-arm component feedback.

Returns
-------
ArmCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("left_arm_command", &BodyComponentBasedCommandFeedback::left_arm_command, R"doc(
Left-arm component feedback.

Returns
-------
ArmCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("torso_command", &BodyComponentBasedCommandFeedback::torso_command, R"doc(
Torso component feedback.

Returns
-------
TorsoCommandFeedback
    Feedback object.
)doc");

  py::class_<HeadCommandFeedback, CommandFeedback>(m, "HeadCommandFeedback", R"doc(
Feedback for a head command container.

Attributes
----------
joint_position_command : JointPositionCommandFeedback
    Feedback for a joint position command (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``HeadCommandFeedback`` instance.
)doc")
      .def_property_readonly("joint_position_command", &HeadCommandFeedback::joint_position_command, R"doc(
Head joint position command feedback.

Returns
-------
JointPositionCommandFeedback
    Feedback object.
)doc");

  py::class_<BodyCommandFeedback, CommandFeedback>(m, "BodyCommandFeedback", R"doc(
Feedback for a body command container.

Attributes
----------
joint_position_command : JointPositionCommandFeedback
    Feedback for a joint position command (if present).
optimal_control_command : OptimalControlCommandFeedback
    Feedback for an optimal-control command (if present).
gravity_compensation_command : GravityCompensationCommandFeedback
    Feedback for a gravity compensation command (if present).
cartesian_command : CartesianCommandFeedback
    Feedback for a Cartesian command (if present).
body_component_based_command : BodyComponentBasedCommandFeedback
    Feedback for a component-based body command (if present).
cartesian_impedance_control_command : CartesianImpedanceControlCommandFeedback
    Feedback for a Cartesian impedance (OC + joint-space impedance) command (if present).
joint_impedance_control_command : JointImpedanceControlCommandFeedback
    Feedback for a joint-space impedance command (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``BodyCommandFeedback`` instance.
)doc")
      .def_property_readonly("joint_position_command", &BodyCommandFeedback::joint_position_command, R"doc(
Joint position command feedback.

Returns
-------
JointPositionCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("optimal_control_command", &BodyCommandFeedback::optimal_control_command, R"doc(
Optimal-control command feedback.

Returns
-------
OptimalControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("gravity_compensation_command", &BodyCommandFeedback::gravity_compensation_command, R"doc(
Gravity compensation command feedback.

Returns
-------
GravityCompensationCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("cartesian_command", &BodyCommandFeedback::cartesian_command, R"doc(
Cartesian command feedback.

Returns
-------
CartesianCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("body_component_based_command", &BodyCommandFeedback::body_component_based_command, R"doc(
Component-based body command feedback.

Returns
-------
BodyComponentBasedCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("cartesian_impedance_control_command",
                             &BodyCommandFeedback::cartesian_impedance_control_command, R"doc(
Cartesian impedance (OC + joint-space impedance) command feedback.

Returns
-------
CartesianImpedanceControlCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("joint_impedance_control_command", &BodyCommandFeedback::joint_impedance_control_command,
                             R"doc(
Joint-space impedance command feedback.

Returns
-------
JointImpedanceControlCommandFeedback
    Feedback object.
)doc");

  py::class_<MobilityCommandFeedback, CommandFeedback>(m, "MobilityCommandFeedback", R"doc(
Feedback for a mobility command container.

Attributes
----------
joint_velocity_command : JointVelocityCommandFeedback
    Feedback for a joint velocity command (if present).
se2_velocity_command : SE2VelocityCommandFeedback
    Feedback for a base SE(2) velocity command (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``MobilityCommandFeedback`` instance.
)doc")
      .def_property_readonly("joint_velocity_command", &MobilityCommandFeedback::joint_velocity_command, R"doc(
Joint velocity command feedback.

Returns
-------
JointVelocityCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("se2_velocity_command", &MobilityCommandFeedback::se2_velocity_command, R"doc(
SE(2) velocity command feedback.

Returns
-------
SE2VelocityCommandFeedback
    Feedback object.
)doc");

  py::class_<ComponentBasedCommandFeedback, CommandFeedback>(m, "ComponentBasedCommandFeedback", R"doc(
Feedback for a component-based command container.

Attributes
----------
head_command : HeadCommandFeedback
    Feedback for the head component (if present).
body_command : BodyCommandFeedback
    Feedback for the body component (if present).
mobility_command : MobilityCommandFeedback
    Feedback for the mobility component (if present).
)doc")
      .def(py::init<>(), R"doc(
      Construct a ``ComponentBasedCommandFeedback`` instance.
)doc")
      .def_property_readonly("head_command", &ComponentBasedCommandFeedback::head_command, R"doc(
Head component feedback.

Returns
-------
HeadCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("body_command", &ComponentBasedCommandFeedback::body_command, R"doc(
Body component feedback.

Returns
-------
BodyCommandFeedback
    Feedback object.
)doc")
      .def_property_readonly("mobility_command", &ComponentBasedCommandFeedback::mobility_command, R"doc(
Mobility component feedback.

Returns
-------
MobilityCommandFeedback
    Feedback object.
)doc");

  py::class_<RobotCommandFeedback, CommandFeedback> rcf(m, "RobotCommandFeedback", R"doc(
Top-level feedback for a robot command.

Attributes
----------
whole_body_command : WholeBodyCommandFeedback
    Feedback for a whole-body command (if present).
component_based_command : ComponentBasedCommandFeedback
    Feedback for a component-based command (if present).
jog_command : JogCommandFeedback
    Feedback for a jog command (if present).
status : RobotCommandFeedback.Status
    High-level status of the command execution.
finish_code : RobotCommandFeedback.FinishCode
    Finish code indicating why execution finished (if finished).
)doc");

  py::enum_<RobotCommandFeedback::Status>(rcf, "Status", R"doc(
High-level execution status.

Members
-------
Idle : int
    No command is active.
Initializing : int
    Command is initializing.
Running : int
    Command is running.
Finished : int
    Command has finished.
)doc")
      .value("Idle", RobotCommandFeedback::Status::kIdle)
      .value("Initializing", RobotCommandFeedback::Status::kInitializing)
      .value("Running", RobotCommandFeedback::Status::kRunning)
      .value("Finished", RobotCommandFeedback::Status::kFinished);

  py::enum_<RobotCommandFeedback::FinishCode>(rcf, "FinishCode", R"doc(
Reason for finishing (if any).

Members
-------
Unknown : int
    Unknown reason.
Ok : int
    Finished successfully.
Canceled : int
    Canceled by user/system.
Preempted : int
    Preempted by another command.
InitializationFailed : int
    Failed during initialization.
ControlManagerIdle : int
    Control manager entered idle unexpectedly.
ControlManagerFault : int
    Control manager reported a fault.
UnexpectedState : int
    Reached an unexpected state.
)doc")
      .value("Unknown", RobotCommandFeedback::FinishCode::kUnknown)
      .value("Ok", RobotCommandFeedback::FinishCode::kOk)
      .value("Canceled", RobotCommandFeedback::FinishCode::kCanceled)
      .value("Preempted", RobotCommandFeedback::FinishCode::kPreempted)
      .value("InitializationFailed", RobotCommandFeedback::FinishCode::kInitializationFailed)
      .value("ControlManagerIdle", RobotCommandFeedback::FinishCode::kControlManagerIdle)
      .value("ControlManagerFault", RobotCommandFeedback::FinishCode::kControlManagerFault)
      .value("UnexpectedState", RobotCommandFeedback::FinishCode::kUnexpectedState);

  rcf.def(py::init<>(), R"doc(
      Construct a ``RobotCommandFeedback`` instance.
)doc")
      .def_property_readonly("whole_body_command", &RobotCommandFeedback::whole_body_command, R"doc(
Whole-body command feedback.

Returns
-------
WholeBodyCommandFeedback
    Feedback object (if present).
)doc")
      .def_property_readonly("component_based_command", &RobotCommandFeedback::component_based_command, R"doc(
Component-based command feedback.

Returns
-------
ComponentBasedCommandFeedback
    Feedback object (if present).
)doc")
      .def_property_readonly("jog_command", &RobotCommandFeedback::jog_command, R"doc(
Jog command feedback.

Returns
-------
JogCommandFeedback
    Feedback object (if present).
)doc")
      .def_property_readonly("status", &RobotCommandFeedback::status, R"doc(
High-level execution status.

Returns
-------
RobotCommandFeedback.Status
    One of :pydata:`RobotCommandFeedback.Status`.
)doc")
      .def_property_readonly("finish_code", &RobotCommandFeedback::finish_code, R"doc(
Finish reason (if finished).

Returns
-------
RobotCommandFeedback.FinishCode
    One of :pydata:`RobotCommandFeedback.FinishCode`.
)doc");
}
