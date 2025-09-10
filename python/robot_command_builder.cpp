#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "rby1-sdk/robot_command_builder.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

void pybind11_robot_command_builder(py::module_& m) {
  py::class_<CommandHeaderBuilder>(m, "CommandHeaderBuilder", R"doc(
Command header builder.

Builds command headers with control hold time settings.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> header = rby.CommandHeaderBuilder().set_control_hold_time(5.0)
>>> # Use this header in any command builder:
>>> jp = rby.JointPositionCommandBuilder().set_command_header(header)
>>> # Control hold time keeps the command active for 5 seconds
)doc")
      .def(py::init<>(), R"doc(
      Construct a CommandHeaderBuilder instance.
)doc")
      .def("set_control_hold_time", &CommandHeaderBuilder::SetControlHoldTime, "control_hold_time"_a, R"doc(
Set the control hold time.

Parameters
----------
control_hold_time : float
    Control hold time in seconds.

Returns
-------
CommandHeaderBuilder
    Self reference for method chaining.
)doc");

  py::class_<JointPositionCommandBuilder>(m, "JointPositionCommandBuilder", R"doc(
Joint position command builder.

Builds joint position commands with optional velocity/acceleration limits and a minimum execution time.

Notes
-----
- **Vector length rule**: For any vector argument (e.g., ``position``, ``velocity_limit``, ``acceleration_limit``),
  the length **N must equal the DOF of the component this builder commands**.
  Examples:
  - ``set_right_arm_command(...)`` on model ``A`` → right arm N = 7
  - ``set_body_command(...)`` on model ``A`` → body N = 20

Examples
--------
Move the right arm to zero with limits and a 3-second minimum time:

>>> import numpy as np, rby1_sdk as rby
>>> jp = (
...     rby.JointPositionCommandBuilder()
...     .set_position(np.zeros(7))                      # N=7 (right arm)
...     .set_velocity_limit(np.full(7, np.pi))          # rad/s
...     .set_acceleration_limit(np.full(7, 1.0))        # rad/s^2
...     .set_minimum_time(3.0)
... )
>>> body = rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
...     rby.ArmCommandBuilder().set_command(jp)
... )
>>> rc = rby.RobotCommandBuilder().set_command(
...     rby.ComponentBasedCommandBuilder().set_body_command(body)
... )
>>> # Send with priority 10
>>> # robot.send_command(rc, 10).get()
)doc")
      .def(py::init<>(), R"doc(
      Construct a JointPositionCommandBuilder instance.
)doc")
      .def("set_command_header", &JointPositionCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Set the command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header builder.

Returns
-------
JointPositionCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_minimum_time", &JointPositionCommandBuilder::SetMinimumTime, "minimum_time"_a, R"doc(
Set the minimum execution time in seconds.

Parameters
----------
minimum_time : float
    Minimum time in seconds. This parameter provides an additional degree
    of freedom to control the arrival time to a target. Instead of relying
    solely on velocity/acceleration limits, you can set high limits and
    control the arrival time using minimum_time. For streaming commands,
    this helps ensure continuous motion by preventing the robot from
    stopping if it arrives too early before the next command.

Returns
-------
JointPositionCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_position", &JointPositionCommandBuilder::SetPosition, "position"_a, R"doc(
Set the target joint positions in radians.

Parameters
----------
position : numpy.ndarray, shape (N,), dtype=float64
    Target joint positions [rad]

Returns
-------
JointPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
**Vector length rule**: ``N`` must equal the DOF of the commanded component.
Examples on model ``A``: right arm → ``N=7``; full body → ``N=20``.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> jp = rby.JointPositionCommandBuilder().set_position(np.zeros(7))  # right arm
)doc")
      .def("set_velocity_limit", &JointPositionCommandBuilder::SetVelocityLimit, "velocity_limit"_a, R"doc(
Set joint velocity limits.

Parameters
----------
velocity_limit : numpy.ndarray, shape (N,), dtype=float64
    Per-joint velocity limits [rad/s].

Returns
-------
JointPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
- **Vector length rule**: ``N`` must equal the DOF of the commanded component
  (e.g., model ``A`` right arm ``N=7``, full body ``N=20``).
- If not set, URDF limits for the current robot model are used (see ``get_robot_model``).
- Values beyond hardware limits may be accepted for profiling; actual tracking is not guaranteed.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> jp = rby.JointPositionCommandBuilder().set_velocity_limit(np.full(7, np.pi))
)doc")
      .def("set_acceleration_limit", &JointPositionCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a, R"doc(
Set joint acceleration limits.

Parameters
----------
acceleration_limit : numpy.ndarray, shape (N,), dtype=float64
    Per-joint acceleration limits [rad/s²].

Returns
-------
JointPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
- **Vector length rule**: ``N`` must equal the DOF of the commanded component
  (e.g., model ``A`` right arm ``N=7``, full body ``N=20``).
- If not set, URDF limits for the current robot model are used (see ``get_robot_model``).
- Values beyond hardware limits may be accepted for profiling; actual tracking is not guaranteed.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> jp = rby.JointPositionCommandBuilder().set_acceleration_limit(np.full(7, 1.0))
)doc");

  py::class_<JointGroupPositionCommandBuilder>(m, "JointGroupPositionCommandBuilder", R"doc(
Joint group position command builder.

Builds joint position commands for a named subset (group) of joints.

Notes
-----
- **Vector length rule**: For vector arguments (e.g., ``position``, ``velocity_limit``, ``acceleration_limit``),
  ``N`` equals the number of joint names set by ``set_joint_names(...)``.

Examples
--------
Command 6 torso joints by name:

>>> import numpy as np, rby1_sdk as rby
>>> torso = (
...   rby.JointGroupPositionCommandBuilder()
...   .set_joint_names(["torso_3","torso_4","torso_5"])
...   .set_position(np.zeros(3))        # N=3
...   .set_minimum_time(2.0)
... )
)doc")
      .def(py::init<>())
      .def("set_command_header", &JointGroupPositionCommandBuilder::SetCommandHeader, "command_header_builder"_a)
      .def("set_joint_names", &JointGroupPositionCommandBuilder::SetJointNames, "joint_names"_a, R"doc(
Set the joint names that this group command applies to.

Parameters
----------
joint_names : list of str
    List of joint name identifiers. The order defines the mapping for
    subsequent vector arguments such as ``position``, ``velocity_limit``,
    and ``acceleration_limit``.

Returns
-------
JointGroupPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
- **Vector length rule**: After calling ``set_joint_names([...])``,
  the length ``N`` for all vector arguments must match the number of names provided.
  For example, if 6 joint names are given, ``position``, ``velocity_limit``,
  and ``acceleration_limit`` must all be length 6.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> torso = (
...   rby.JointGroupPositionCommandBuilder()
...   .set_joint_names(["torso_0","torso_1","torso_2","torso_3","torso_4","torso_5"])
...   .set_position(np.zeros(6))                 # N=6
...   .set_velocity_limit(np.full(6, 1.0))
...   .set_acceleration_limit(np.full(6, 2.0))
...   .set_minimum_time(2.0)
... )
)doc")
      .def("set_minimum_time", &JointGroupPositionCommandBuilder::SetMinimumTime, "minimum_time"_a)
      .def("set_position", &JointGroupPositionCommandBuilder::SetPosition, "position"_a, R"doc(
Set group joint target positions in radians.

Parameters
----------
position : numpy.ndarray, shape (N,), dtype=float64
    Target joint positions (radians) for the group.

Returns
-------
JointGroupPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
**Vector length rule**: ``N`` equals the number of joint names set by ``set_joint_names(...)``.
)doc")
      .def("set_velocity_limit", &JointGroupPositionCommandBuilder::SetVelocityLimit, "velocity_limit"_a, R"doc(
Set per-joint velocity limits for the group.

Parameters
----------
velocity_limit : numpy.ndarray, shape (N,), dtype=float64
    Velocity limits [rad/s].

Returns
-------
JointGroupPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
**Vector length rule**: ``N`` equals the number of joint names set by ``set_joint_names(...)``.
)doc")
      .def("set_acceleration_limit", &JointGroupPositionCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a,
           R"doc(
Set per-joint acceleration limits for the group.

Parameters
----------
acceleration_limit : numpy.ndarray, shape (N,), dtype=float64
    Acceleration limits [rad/s²].

Returns
-------
JointGroupPositionCommandBuilder
    Self reference for method chaining.

Notes
-----
**Vector length rule**: ``N`` equals the number of joint names set by ``set_joint_names(...)``.
)doc");

  py::class_<JointImpedanceControlCommandBuilder>(m, "JointImpedanceControlCommandBuilder", R"doc(
Joint impedance control command builder.

Creates joint-space impedance commands with position targets, stiffness, damping ratio,
optional velocity/acceleration limits, and per-joint torque limits.

Notes
-----
- **Vector length rule**: All vector arguments (``position``, ``stiffness``, ``torque_limit``,
  ``velocity_limit``, ``acceleration_limit``) must have length :math:`N` equal to the DOF of
  the component where this builder is applied.

  Examples (Model ``A``):
  
  - ``set_right_arm_command(...)`` → right arm has :math:`N=7`
  - ``set_body_command(...)``      → whole body has :math:`N=20`

- Impedance control model:

  .. math::

      \tau = K (q_d - q) + D (\dot{q}_d - \dot{q}) + \tau_{ff}

  with :math:`K` (stiffness), :math:`D` (damping), and :math:`\tau_{ff}` the
  feed-forward torque.

- Damping from damping ratio :math:`\zeta`:

  .. math::

      D = \zeta \left( \sqrt{M}\sqrt{K} + \sqrt{K}\sqrt{M} \right)

  where :math:`M` is the (joint) inertia matrix.

- Feed-forward torque :math:`\tau_{ff}` is the gravity compensation torque.
  The final applied torque is saturated by the per-joint limits:

  .. math::

      \tau_{applied} = \mathrm{clip}(\tau,\; -\tau_{max},\; \tau_{max})

Examples
--------
Compliant positioning on one arm (Model A):

>>> import numpy as np, rby1_sdk as rby
>>> imp = (
...   rby.JointImpedanceControlCommandBuilder()
...   .set_position(np.zeros(7))                # N=7
...   .set_stiffness(np.full(7, 50.0))          # Nm/rad
...   .set_damping_ratio(0.7)                   # ζ
...   .set_torque_limit(np.full(7, 60.0))       # Nm
...   .set_velocity_limit(np.full(7, np.pi))    # rad/s
...   .set_acceleration_limit(np.full(7, 2.0))  # rad/s²
...   .set_minimum_time(1.0)                    # s
... )
)doc")
      .def(py::init<>())
      .def("set_command_header", &JointImpedanceControlCommandBuilder::SetCommandHeader, "command_header_builder"_a,
           R"doc(
Attach a command header to this builder.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Examples
--------
>>> header = rby.CommandHeaderBuilder().set_control_hold_time(3.0)
>>> imp = rby.JointImpedanceControlCommandBuilder().set_command_header(header)
)doc")
      .def("set_minimum_time", &JointImpedanceControlCommandBuilder::SetMinimumTime, "minimum_time"_a, R"doc(
Set the minimum execution time in seconds.

Parameters
----------
minimum_time : float
    Minimum time in seconds. Helpful in streaming to avoid finishing too early
    before the next command arrives.

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_position", &JointImpedanceControlCommandBuilder::SetPosition, "position"_a, R"doc(
Set target joint positions.

Parameters
----------
position : numpy.ndarray, shape (N,), dtype=float64
    Desired joint positions :math:`q_d` [rad]

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
Vector length :math:`N` must match the DOF of the target component
(e.g., Model A arm :math:`N=7`, whole body :math:`N=20`).
)doc")
      .def("set_velocity_limit", &JointImpedanceControlCommandBuilder::SetVelocityLimit, "velocity_limit"_a, R"doc(
Set per-joint velocity limits.

Parameters
----------
velocity_limit : numpy.ndarray, shape (N,), dtype=float64
    Velocity limits [rad/s].

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- Vector length :math:`N` must match the component DOF.
- If not set, limits from the current robot URDF are used (see ``get_robot_model``).
- Values beyond hardware capability may be accepted for profiling; actual tracking is not guaranteed.
)doc")
      .def("set_acceleration_limit", &JointImpedanceControlCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a,
           R"doc(
Set per-joint acceleration limits.

Parameters
----------
acceleration_limit : numpy.ndarray, shape (N,), dtype=float64
    Acceleration limits [rad/s²].

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- Vector length :math:`N` must match the component DOF.
- If not set, limits from the current robot URDF are used (see ``get_robot_model``).
- Values beyond hardware capability may be accepted for profiling; actual tracking is not guaranteed.
)doc")
      .def("set_stiffness", &JointImpedanceControlCommandBuilder::SetStiffness, "stiffness"_a, R"doc(
Set per-joint stiffness values.

Parameters
----------
stiffness : numpy.ndarray, shape (N,), dtype=float64
    Per-joint stiffness values, interpreted as the diagonal elements of
    the stiffness matrix :math:`K \in \mathbb{R}^{N \times N}` (Nm/rad).

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- Vector length :math:`N` must match the component DOF.
- The full stiffness matrix is assumed to be diagonal:

  .. math::

      K = \mathrm{diag}(k_1, k_2, \ldots, k_N)

- Stiffness contributes torque as:

  .. math::

      \tau_K = K (q_d - q)
)doc")
      .def("set_torque_limit", &JointImpedanceControlCommandBuilder::SetTorqueLimit, "torque_limit"_a, R"doc(
Set per-joint torque limits.

Parameters
----------
torque_limit : numpy.ndarray, shape (N,), dtype=float64
    Maximum allowable torque :math:`\tau_{max}` [Nm].

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The controller torque is:

  .. math::

      \tau = K (q_d - q) + D (\dot{q}_d - \dot{q}) + \tau_{ff}

  where :math:`\tau_{ff}` is the feed-forward gravity compensation torque.

- The applied torque is saturated per joint:

  .. math::

      \tau_{applied} = \mathrm{clip}(\tau,\; -\tau_{max},\; \tau_{max})
)doc")
      .def("set_damping_ratio", &JointImpedanceControlCommandBuilder::SetDampingRatio, "damping_ratio"_a, R"doc(
Set the damping ratio.

Parameters
----------
damping_ratio : float
    Damping ratio :math:`\zeta` (dimensionless) in [0, 1].

Returns
-------
JointImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
The damping matrix is computed from :math:`\zeta`, stiffness :math:`K`, and inertia :math:`M`:

.. math::

    D = \zeta \left( \sqrt{M}\sqrt{K} + \sqrt{K}\sqrt{M} \right)
)doc");

  py::class_<OptimalControlCommandBuilder>(m, "OptimalControlCommandBuilder", R"doc(
Optimal control command builder.

Constructs an optimal control objective composed of Cartesian pose costs,
center-of-mass (COM) costs, and joint position costs, with global error and
limit scalings.

Notes
-----
- The overall objective can be viewed as a weighted least-squares form:

  .. math::

      J \;=\; \sum_{i} w_t^{(i)} \,\|\, p^{(i)}_d - p^{(i)} \,\|^2
      \;+\; \sum_{i} w_r^{(i)} \,\|\, r^{(i)}_d \ominus r^{(i)} \,\|^2
      \;+\; w_{\mathrm{com}} \,\|\, c_d - c \,\|^2
      \;+\; \sum_j w_q^{(j)} \,(q^{(j)}_d - q^{(j)})^2

  where :math:`p, r` denote translation and rotation of a link in a reference
  frame, :math:`c` is the COM position, and :math:`q` are joint positions.

- Velocity/acceleration limit **scalings** multiply internal limits used by the
  optimizer; they do **not** redefine hardware limits.

Examples
--------
Set a right-wrist Cartesian target and a joint hint, with moderate limit scalings:

>>> import numpy as np, rby1_sdk as rby
>>> T = np.eye(4); T[0,3] = 0.35; T[2,3] = 0.55
>>> oc = (
...   rby.OptimalControlCommandBuilder()
...   .add_cartesian_target("base_link", "ee_right", T, translation_weight=1.0, rotation_weight=0.5)
...   .add_joint_position_target("right_arm_4", 0.4, weight=0.2)
...   .set_velocity_limit_scaling(0.8)
...   .set_acceleration_limit_scaling(0.6)
...   .set_error_scaling(1.0)
... )
)doc")
      .def(py::init<>())
      .def("set_command_header", &OptimalControlCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Attach a command header to this builder.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("add_cartesian_target", &OptimalControlCommandBuilder::AddCartesianTarget, "ref_link_name"_a, "link_name"_a,
           "T"_a, "translation_weight"_a, "rotation_weight"_a, R"doc(
Add a Cartesian pose target for a link.

Parameters
----------
ref_link_name : str
    Reference frame (link) name in which the target pose is expressed.
link_name : str
    Controlled link name.
T : numpy.ndarray, shape (4,4), dtype=float64
    Target homogeneous transform :math:`{}^{ref}\!T^{link}_d`.
translation_weight : float
    Weight :math:`w_t` for translational error.
rotation_weight : float
    Weight :math:`w_r` for rotational error.

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The cost terms are typically of the form

  .. math::

      J_t = w_t \, \|\, p_d - p \,\|^2, \qquad
      J_r = w_r \, \|\, r_d \ominus r \,\|^2,

  where :math:`p` is position and :math:`r` is orientation (e.g., axis-angle error)
  of ``link_name`` measured in ``ref_link_name``.
)doc")
      .def("set_center_of_mass_target", &OptimalControlCommandBuilder::SetCenterOfMassTarget, "ref_link_name"_a,
           "pose"_a, "weight"_a, R"doc(
Set a center-of-mass (COM) position target.

Parameters
----------
ref_link_name : str
    Reference frame (link) name for the COM target.
pose : numpy.ndarray, shape (3,), dtype=float64
    Desired COM position :math:`c_d = [x,y,z]^\\top` in meters, expressed in ``ref_link_name``.
weight : float
    Weight :math:`w_{\mathrm{com}}` for COM position error.

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The COM cost is:

  .. math::

      J_{\mathrm{com}} \;=\; w_{\mathrm{com}} \,\|\, c_d - c \,\|^2 .
)doc")
      .def("add_joint_position_target", &OptimalControlCommandBuilder::AddJointPositionTarget, "joint_name"_a,
           "target_position"_a, "weight"_a, R"doc(
Add a joint position target.

Parameters
----------
joint_name : str
    Joint name.
target_position : float
    Desired joint position :math:`q_d` [rad].
weight : float
    Weight :math:`w_q` for this joint's position error.

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The joint cost term is:

  .. math::

      J_q \;=\; w_q \,(q_d - q)^2 .
)doc")
      .def("set_error_scaling", &OptimalControlCommandBuilder::SetErrorScaling, "error_scaling"_a, R"doc(
Set a global error scaling applied to cost terms.

Parameters
----------
error_scaling : float
    A positive scalar multiplying error magnitudes internally (e.g., to balance
    different unit scales).

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- Conceptually, this scales error vectors before squaring, e.g., :math:`e \leftarrow \alpha e`,
  which yields :math:`\alpha^2` scaling on quadratic costs.
)doc")
      .def("set_velocity_limit_scaling", &OptimalControlCommandBuilder::SetVelocityLimitScaling,
           "velocity_limit_scaling"_a, R"doc(
Scale internal joint velocity limits used by the optimizer.

Parameters
----------
velocity_limit_scaling : float
    Scaling factor :math:`\alpha_v > 0`. The internal velocity limits become
    :math:`\alpha_v \cdot v_{\max}`.

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- This does **not** change hardware limits; it affects only the optimization's
  internal constraints and trajectory shaping.
)doc")
      .def("set_acceleration_limit_scaling", &OptimalControlCommandBuilder::SetAccelerationLimitScaling,
           "acceleration_limit_scaling"_a, R"doc(
Scale internal joint acceleration limits used by the optimizer.

Parameters
----------
acceleration_limit_scaling : float
    Scaling factor :math:`\alpha_a > 0`. The internal acceleration limits become
    :math:`\alpha_a \cdot a_{\max}`.

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- Like velocity scaling, this only affects internal optimization constraints.
)doc")
      .def("set_stop_cost", &OptimalControlCommandBuilder::SetStopCost, "stop_cost"_a, R"doc(
Set the absolute objective threshold to consider the problem "solved".

Parameters
----------
stop_cost : float
    If the total cost :math:`J` falls below this value, the optimizer may terminate early.

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_min_delta_cost", &OptimalControlCommandBuilder::SetMinDeltaCost, "min_delta_cost"_a, R"doc(
Set the minimum required improvement in cost between iterations.

Parameters
----------
min_delta_cost : float
    If the reduction :math:`\Delta J` between iterations drops below this value,
    the optimizer may stop (convergence).

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_patience", &OptimalControlCommandBuilder::SetPatience, "patience"_a, R"doc(
Set the iteration patience before stopping on small improvements.

Parameters
----------
patience : int
    Number of iterations to tolerate without sufficient cost improvement
    (in combination with ``set_min_delta_cost``).

Returns
-------
OptimalControlCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<ImpedanceControlCommandBuilder>(m, "ImpedanceControlCommandBuilder", R"doc(
Cartesian impedance control command builder.

Builds a Cartesian impedance command for a given link in a reference frame.
The control is formulated in the **se(3)** space with a 6D error
:math:`\xi = [\,\Delta x;\,\Delta\phi\,] \in \mathbb{R}^6` (translation and
rotation in the Lie algebra), yielding the wrench:

.. math::

    w = K\,\xi \;+\; D\,\dot{\xi}

where :math:`K \in \mathbb{R}^{6\times 6}` is the stiffness matrix and
:math:`D \in \mathbb{R}^{6\times 6}` is the damping matrix.

Notes
-----
- The stiffness matrix :math:`K` is assumed **block-diagonal**:

  .. math::

      K \;=\; \mathrm{diag}(K_t,\; K_r), \qquad
      K_t, K_r \in \mathbb{R}^{3\times 3}\ \text{(diagonal)}

  ``set_translation_weight([k_x,k_y,k_z])`` fills :math:`K_t=\mathrm{diag}(k_x,k_y,k_z)`,
  and ``set_rotation_weight([k_{rx},k_{ry},k_{rz}])`` fills
  :math:`K_r=\mathrm{diag}(k_{rx},k_{ry},k_{rz})`.

- Damping is derived from a dimensionless ratio :math:`\zeta`:

  .. math::

      D \;=\; \zeta \left(\sqrt{M}\sqrt{K} \;+\; \sqrt{K}\sqrt{M}\right),

  with :math:`M \in \mathbb{R}^{6\times 6}` the Cartesian inertia matrix.

- ``set_reference_link_name`` and ``set_link_name`` must be set before
  applying the target pose (``set_transformation``).

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> T = np.eye(4); T[0,3] = 0.30  # desired pose, 30 cm forward
>>> cmd = (
...   rby.ImpedanceControlCommandBuilder()
...   .set_reference_link_name("base_link")
...   .set_link_name("right_wrist")
...   .set_transformation(T)
...   .set_translation_weight(np.array([1200.0, 1200.0, 1600.0]))  # N/m
...   .set_rotation_weight(np.array([60.0, 60.0, 60.0]))           # Nm/rad
...   .set_damping_ratio(0.7)
... )
)doc")
      .def(py::init<>())
      .def("set_command_header", &ImpedanceControlCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Attach a command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_reference_link_name", &ImpedanceControlCommandBuilder::SetReferenceLinkName, "reference_link_name"_a,
           R"doc(
Set the reference frame (link) name in which the pose is expressed.

Parameters
----------
reference_link_name : str
    Reference link/frame name.

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_link_name", &ImpedanceControlCommandBuilder::SetLinkName, "link_name"_a, R"doc(
Set the controlled link name.

Parameters
----------
link_name : str
    Name of the robot link to be regulated in Cartesian impedance.

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_transformation", &ImpedanceControlCommandBuilder::SetTransformation, "T"_a, R"doc(
Set the desired Cartesian pose (homogeneous transform).

Parameters
----------
T : numpy.ndarray, shape (4,4), dtype=float64
    Desired pose :math:`{}^{ref}\!T^{link}_d`.

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The se(3) error :math:`\xi = [\Delta x;\Delta\phi]` is computed from
  :math:`T_d` and the current pose.
)doc")
      .def("set_translation_weight", &ImpedanceControlCommandBuilder::SetTranslationWeight, "weight"_a, R"doc(
Set translational stiffness (fills the translational block of K).

Parameters
----------
weight : numpy.ndarray, shape (3,), dtype=float64
    Per-axis translational stiffness (N/m), used to form
    :math:`K_t=\mathrm{diag}(k_x,k_y,k_z)`.

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The 6×6 stiffness matrix is :math:`K=\mathrm{diag}(K_t,K_r)`.
- The translational wrench component is :math:`F = K_t\,\Delta x`.
)doc")
      .def("set_rotation_weight", &ImpedanceControlCommandBuilder::SetRotationWeight, "weight"_a, R"doc(
Set rotational stiffness (fills the rotational block of K).

Parameters
----------
weight : numpy.ndarray, shape (3,), dtype=float64
    Per-axis rotational stiffness (Nm/rad), used to form
    :math:`K_r=\mathrm{diag}(k_{rx},k_{ry},k_{rz})`.

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- The rotational wrench component is :math:`\tau = K_r\,\Delta\phi`.
)doc")
      .def("set_damping_ratio", &ImpedanceControlCommandBuilder::SetDampingRatio, "damping_ratio"_a, R"doc(
Set the (dimensionless) damping ratio used to build the 6×6 damping matrix.

Parameters
----------
damping_ratio : float
    Damping ratio :math:`\zeta` (typically :math:`0 < \zeta \le 1.5`).

Returns
-------
ImpedanceControlCommandBuilder
    Self reference for method chaining.

Notes
-----
- Damping in se(3) space is computed from :math:`K` and the Cartesian inertia 
  (reflective inertia) :math:`M` as:

  .. math::

      D \;=\; \zeta \left(\sqrt{M}\sqrt{K} \;+\; \sqrt{K}\sqrt{M}\right).

- :math:`\zeta=1` approximates critical damping; lower/higher values yield
  under/over-damped responses.
)doc");

  py::class_<JointVelocityCommandBuilder>(m, "JointVelocityCommandBuilder", R"doc(
Joint velocity command builder.

Builds a joint-space velocity command with optional acceleration limits.
When a target velocity vector :math:`v_d` is set, the command generates a
linear ramp from the current velocity :math:`v_{curr}` to :math:`v_d`,
considering both acceleration limits and the specified minimum time.

The controller’s objective is to **reach the target velocity**.
If you need the robot to **hold the target velocity after reaching it, you
must specify a control hold time in the command header.**

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> # Ramp torso joints to 0.5 rad/s with acceleration limits
>>> cmd = (
...     rby.JointVelocityCommandBuilder()
...     .set_velocity(np.full(6, 0.5))
...     .set_acceleration_limit(np.full(6, 2.0))
...     .set_minimum_time(1.0)
... )
>>> # To maintain the velocity after reaching 0.5 rad/s,
>>> # add a CommandHeader with control_hold_time.
>>> header = rby.CommandHeaderBuilder().set_control_hold_time(5.0)
>>> cmd.set_command_header(header)
)doc")
      .def(py::init<>())
      .def("set_command_header", &JointVelocityCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Attach a command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
JointVelocityCommandBuilder
    Self reference for method chaining.

Notes
-----
- Use ``control_hold_time`` to maintain the commanded velocity once reached.
)doc")
      .def("set_minimum_time", &JointVelocityCommandBuilder::SetMinimumTime, "minimum_time"_a, R"doc(
Set the minimum execution time.

Parameters
----------
minimum_time : float
    Minimum execution time in seconds. Ensures the ramping process to the
    target velocity is stretched at least over this duration.

Returns
-------
JointVelocityCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_velocity", &JointVelocityCommandBuilder::SetVelocity, "velocity"_a, R"doc(
Set desired joint velocities.

Parameters
----------
velocity : numpy.ndarray, shape (N,), dtype=float64
    Per-joint target velocities [rad/s].
    Length :math:`N` must equal the number of joints in the targeted component.

Returns
-------
JointVelocityCommandBuilder
    Self reference for method chaining.

Notes
-----
- The command ramps linearly from the current velocity to the target velocity.
- To **hold** the target velocity after reaching it, add a command header with
  a control hold time.
)doc")
      .def("set_acceleration_limit", &JointVelocityCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a, R"doc(
Set per-joint acceleration limits.

Parameters
----------
acceleration_limit : numpy.ndarray, shape (N,), dtype=float64
    Maximum allowable accelerations [rad/s²].
    Length :math:`N` must equal the number of joints in the targeted component.

Returns
-------
JointVelocityCommandBuilder
    Self reference for method chaining.

Notes
-----
- These limits shape the linear ramp profile toward the target velocity.
- If not set, URDF defaults are used (see ``get_robot_model``).
)doc");

  auto jcb = py::class_<JogCommandBuilder>(m, "JogCommandBuilder", R"doc(
Jog command builder.

Builds a jog command for a single joint. A jog command applies an incremental
position change using one of three types: AbsolutePosition, RelativePosition,
or OneStep.

Notes
-----
- Jog is an incremental **position** command, not a velocity command.
- To repeat or accumulate jogs on the same joint, send additional jog commands
  or use a stream command.

Examples
--------
>>> import rby1_sdk as rby
>>> jog = (
...     rby.JogCommandBuilder()
...     .set_joint_name("right_arm_0")
...     .set_command(rby.JogCommandBuilder.RelativePosition(0.1))
... )
>>> cmd = rby.RobotCommandBuilder().set_command(jog)
>>> robot.send_command(cmd, priority=1)
)doc");

  py::class_<JogCommandBuilder::AbsolutePosition>(jcb, "AbsolutePosition", R"doc(
Absolute jog target.

Sets the joint to an absolute angle in radians.

Parameters
----------
value : float
    Target absolute joint angle [rad].

Examples
--------
>>> rby.JogCommandBuilder.AbsolutePosition(1.57)  # ~90 degrees
)doc")
      .def(py::init<double>())
      .def("value", &JogCommandBuilder::AbsolutePosition::value, R"doc(
Get the target absolute angle in radians.
)doc");

  py::class_<JogCommandBuilder::RelativePosition>(jcb, "RelativePosition", R"doc(
Relative jog target.

Offsets the current joint angle by a relative amount in radians.

Parameters
----------
value : float
    Relative offset [rad]. Positive values increase the angle; negative values decrease it.

Examples
--------
>>> rby.JogCommandBuilder.RelativePosition(0.1)  # +0.1 rad
)doc")
      .def(py::init<double>())
      .def("value", &JogCommandBuilder::RelativePosition::value, R"doc(
Get the relative offset in radians.
)doc");

  py::class_<JogCommandBuilder::OneStep>(jcb, "OneStep", R"doc(
One-step jog target.

Applies a single incremental step to the joint angle in radians.
Useful for discrete button/step interfaces.

Parameters
----------
value : float
    Step size [rad].

Examples
--------
>>> rby.JogCommandBuilder.OneStep(-0.05)  # one step backward
)doc")
      .def(py::init<double>())
      .def("value", &JogCommandBuilder::OneStep::value, R"doc(
Get the step size in radians.
)doc");

  jcb.def(py::init<>())
      .def("set_command_header", &JogCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Attach a command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_joint_name", &JogCommandBuilder::SetJointName, "joint_name"_a, R"doc(
Specify the joint to jog.

Parameters
----------
joint_name : str
    Name of the single joint to control.

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_velocity_limit", &JogCommandBuilder::SetVelocityLimit, "velocity_limit"_a, R"doc(
Set the maximum velocity used during the jog.

Parameters
----------
velocity_limit : float
    Maximum allowed joint velocity [rad/s].

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_acceleration_limit", &JogCommandBuilder::SetAccelerationLimit, "acceleration_limit"_a, R"doc(
Set the maximum acceleration used during the jog.

Parameters
----------
acceleration_limit : float
    Maximum allowed joint acceleration [rad/s²].

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<JogCommandBuilder::AbsolutePosition>(&JogCommandBuilder::SetCommand),
           "absolute_position"_a, R"doc(
Set the jog as an absolute position target.

Parameters
----------
absolute_position : JogCommandBuilder.AbsolutePosition
    Absolute target angle [rad].

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<JogCommandBuilder::RelativePosition>(&JogCommandBuilder::SetCommand),
           "relative_position"_a, R"doc(
Set the jog as a relative position target.

Parameters
----------
relative_position : JogCommandBuilder.RelativePosition
    Relative offset [rad] from the current joint angle.

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<JogCommandBuilder::OneStep>(&JogCommandBuilder::SetCommand), "one_step"_a,
           R"doc(
Set the jog as a single incremental step.

Parameters
----------
one_step : JogCommandBuilder.OneStep
    Step size [rad] to apply once.

Returns
-------
JogCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<SE2VelocityCommandBuilder>(m, "SE2VelocityCommandBuilder", R"doc(
SE(2) velocity command builder.

Defines planar velocity commands in SE(2) space (x, y translation and yaw rotation).
The target velocity is reached by linear ramp-up/ramp-down subject to acceleration
limits and the specified minimum execution time.

Examples
--------
Move forward at 0.5 m/s while yawing at 0.2 rad/s, with limits and minimum time:

>>> import numpy as np, rby1_sdk as rby
>>> se2 = (
...     rby.SE2VelocityCommandBuilder()
...     .set_velocity(np.array([0.5, 0.0]), 0.2)        # [vx, vy], wz
...     .set_acceleration_limit(np.array([1.0, 1.0]), 0.5)
...     .set_minimum_time(1.0)
... )
>>> # (Optional) hold the commanded velocity after ramping:
>>> header = rby.CommandHeaderBuilder().set_control_hold_time(3.0)
>>> se2.set_command_header(header)
>>> cmd = rby.RobotCommandBuilder().set_command(
...     rby.ComponentBasedCommandBuilder().set_mobility_command(
...         rby.MobilityCommandBuilder().set_command(se2)
...     )
... )
>>> # robot.send_command(cmd, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a SE2VelocityCommandBuilder instance.
)doc")
      .def("set_command_header", &SE2VelocityCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Attach a command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
SE2VelocityCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_minimum_time", &SE2VelocityCommandBuilder::SetMinimumTime, "minimum_time"_a, R"doc(
Set the minimum execution time.

Parameters
----------
minimum_time : float
    Minimum execution time [s]. Ensures the ramp toward the target velocity
    spans at least this duration.

Returns
-------
SE2VelocityCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_velocity", &SE2VelocityCommandBuilder::SetVelocity, "linear"_a, "angular"_a, R"doc(
Set the target SE(2) velocity.

Parameters
----------
linear : numpy.ndarray, shape (2,), dtype=float64
    Linear velocity along x and y axes [m/s].
angular : float
    Angular velocity around the z-axis [rad/s].

Returns
-------
SE2VelocityCommandBuilder
    Self reference for method chaining.

Notes
-----
- To maintain the commanded velocity after reaching it, set a control hold time
  via ``CommandHeaderBuilder`` and attach it with ``set_command_header``.
)doc")
      .def("set_acceleration_limit", &SE2VelocityCommandBuilder::SetAccelerationLimit, "linear"_a, "angular"_a, R"doc(
Set maximum acceleration limits for the SE(2) command.

Parameters
----------
linear : numpy.ndarray, shape (2,), dtype=float64
    Maximum linear accelerations along x and y axes [m/s²].
angular : float
    Maximum angular acceleration around the z-axis [rad/s²].

Returns
-------
SE2VelocityCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<StopCommandBuilder>(m, "StopCommandBuilder", R"doc(
Stop command builder.

Generates a stop command that halts all robot motion immediately.
Typically used in whole-body command contexts to enforce a safe and
deterministic stop.

Examples
--------
>>> import rby1_sdk as rby
>>> stop = rby.StopCommandBuilder()
>>> cmd = rby.RobotCommandBuilder().set_command(
...     rby.WholeBodyCommandBuilder().set_command(stop)
... )
>>> # robot.send_command(cmd, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a StopCommandBuilder instance.
)doc")
      .def("set_command_header", &StopCommandBuilder::SetCommandHeader, "stop_command_builder"_a, R"doc(
Attach a command header.

Parameters
----------
stop_command_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
StopCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<CartesianCommandBuilder>(m, "CartesianCommandBuilder", R"doc(
Cartesian command builder.

Builds end-effector Cartesian pose targets and optional per-joint auxiliary
targets. Each Cartesian target specifies a desired homogeneous transform
:math:`{}^{ref}\!T^{link}_d \in SE(3)` together with velocity/acceleration
constraints used to time the motion.

Notes
-----
- Cartesian error is computed in the reference frame and tracked subject to
  velocity/acceleration limits provided per target.
- Joint targets added via ``add_joint_position_target`` are auxiliary hints
  that can help disambiguate IK or bias the posture during Cartesian tracking.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> T = np.eye(4); T[0,3] = 0.40; T[2,3] = 0.60           # 40 cm forward, 60 cm up
>>> cart = (
...   rby.CartesianCommandBuilder()
...   .add_target("base", "link_toso_5", T,
...               linear_velocity_limit=0.4,     # [m/s]
...               angular_velocity_limit=1.0,    # [rad/s]
...               acceleration_limit_scaling=0.6)
...   .add_joint_position_target("right_arm_3", 0.3, velocity_limit=1.2)
...   .set_minimum_time(2.0)
... )
)doc")
      .def(py::init<>(), R"doc(
Construct a CartesianCommandBuilder instance.
)doc")
      .def("set_command_header", &CartesianCommandBuilder::SetCommandHeader, "command_header_builder"_a, R"doc(
Attach a command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_minimum_time", &CartesianCommandBuilder::SetMinimumTime, "minimum_time"_a, R"doc(
Set the minimum execution time in seconds.

Parameters
----------
minimum_time : float
    Minimum time [s]. Ensures the motion is timed to take at least this long,
    independent of the velocity limits (useful for streaming and smoothness).

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.
)doc")
      .def("add_target", &CartesianCommandBuilder::AddTarget, "ref_link_name"_a, "link_name"_a, "T"_a,
           "linear_velocity_limit"_a, "angular_velocity_limit"_a, "acceleration_limit_scaling"_a, R"doc(
Add a Cartesian pose target for a link.

Parameters
----------
ref_link_name : str
    Reference frame (link) in which the target pose is expressed.
link_name : str
    Controlled link name (end-effector).
T : numpy.ndarray, shape (4,4), dtype=float64
    Desired homogeneous transform :math:`{}^{ref}\!T^{link}_d \in SE(3)`.
linear_velocity_limit : float
    Maximum allowed translational speed [m/s].
angular_velocity_limit : float
    Maximum allowed rotational speed [rad/s].
acceleration_limit_scaling : float
    Dimensionless scaling applied to internal Cartesian acceleration limits
    (e.g., ``0.5`` halves the default internal limits).

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.

Notes
-----
- Limits here shape trajectory timing; they do **not** redefine hardware limits.
)doc")
      .def("add_joint_position_target", &CartesianCommandBuilder::AddJointPositionTarget, "joint_name"_a,
           "target_position"_a, "velocity_limit"_a = py::none(), "acceleration_limit"_a = py::none(), R"doc(
Add an auxiliary joint position target.

Parameters
----------
joint_name : str
    Joint name to bias.
target_position : float
    Desired joint position [rad].
velocity_limit : Optional[float], default=None
    Per-joint velocity limit [rad/s] for this target (if provided).
acceleration_limit : Optional[float], default=None
    Per-joint acceleration limit [rad/s²] for this target (if provided).

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.

Notes
-----
- Useful to resolve redundancy or bias posture while tracking Cartesian goals.
- If limits are omitted, internal defaults/URDF-derived limits are used.
)doc")
      .def("set_stop_position_tracking_error", &CartesianCommandBuilder::SetStopPositionTrackingError,
           "stop_position_tracking_error"_a, R"doc(
Set the stop threshold on translational tracking error.

Parameters
----------
stop_position_tracking_error : float
    Threshold on :math:`\|\Delta x\|` [m]. When the position error falls below
    this value (and other conditions are met), the controller may stop.

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_stop_orientation_tracking_error", &CartesianCommandBuilder::SetStopOrientationTrackingError,
           "stop_orientation_tracking_error"_a, R"doc(
Set the stop threshold on rotational tracking error.

Parameters
----------
stop_orientation_tracking_error : float
    Threshold on orientation error [rad]. When the
    orientation error falls below this value, the controller may stop.

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_stop_joint_position_tracking_error", &CartesianCommandBuilder::SetStopJointPositionTrackingError,
           "stop_joint_position_tracking_error"_a, R"doc(
Set the stop threshold on joint-space position tracking error.

Parameters
----------
stop_joint_position_tracking_error : float
    Threshold on per-joint position error [rad]. When joint errors fall below
    this value (and other conditions are met), the controller may stop.

Returns
-------
CartesianCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<CartesianImpedanceControlCommandBuilder>(m, "CartesianImpedanceControlCommandBuilder", R"doc(
Cartesian impedance control command builder.

Takes **Cartesian pose targets** (in SE(3)) but executes control in **joint space**:
first, an **optimal control / inverse-kinematics** problem finds joint targets
:math:`q^\*` that best realize the Cartesian objectives; then a **joint-space
impedance controller** tracks :math:`q^\*` using the stiffness/damping/torque
limits you set.

Pipeline
--------
1) **Cartesian objective (se(3) error):** for each target transform
   :math:`{}^{ref}\!T^{link}_d`, an error
   :math:`\xi = [\,\Delta x;\,\Delta\phi\,] \in \mathbb{R}^6` is formed
   (translation/orientation in se(3)) and contributes to an optimal-control cost.
2) **Optimal control solve:** solves for joint configuration :math:`q^\*`
   (and, if needed, velocities/accelerations) that minimize a weighted sum of
   Cartesian errors and optional joint terms while respecting internal
   (scaled) velocity/acceleration limits and joint bounds.
3) **Joint-space impedance tracking:** applies

   .. math::

      \tau \;=\; K \,(q^\* - q) \;+\; D \,(\dot{q}^\* - \dot{q}) \;+\; \tau_{ff},

   where :math:`K=\mathrm{diag}(k_1,\ldots,k_N)` is set via
   ``set_joint_stiffness(...)`` (diagonal terms only), :math:`D` is built from a
   damping ratio :math:`\zeta` as

   .. math::

      D \;=\; \zeta \left(\sqrt{M}\sqrt{K} \;+\; \sqrt{K}\sqrt{M}\right),

   with :math:`M` the joint-space inertia, and :math:`\tau_{ff}` is the
   **gravity feed-forward** torque. The final torque is saturated per-joint by
   ``set_joint_torque_limit(...)``:

   .. math::

      \tau_{\text{applied}} \;=\; \mathrm{clip}(\tau,\,-\tau_{\max},\,\tau_{\max}).

What each setting affects
------------------------
- ``add_target(...)``: defines Cartesian pose costs used **only in the optimal-control
  solve** that produces :math:`q^\*`.
- ``set_joint_stiffness(...)``, ``set_joint_damping_ratio(...)``,
  ``set_joint_torque_limit(...)``: shape the **joint-space impedance** that tracks
  :math:`q^\*` (they do not change the Cartesian cost; they change how :math:`q^\*`
  is followed).
- ``add_joint_position_target(...)`` and ``add_joint_limit(...)``: add joint-space
  costs/constraints to the **optimal-control** stage.
- Velocity/acceleration limits passed to ``add_target(...)`` are used as **internal
  bounds/scalings** for the solver, not hardware limits.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> T = np.eye(4); T[0,3] = 0.45; T[2,3] = 0.60   # desired wrist pose in base_link
>>> cmd = (
...   rby.CartesianImpedanceControlCommandBuilder()
...   .add_target("base_link", "right_wrist", T,
...               linear_velocity_limit=0.3,    # solver internal bounds
...               angular_velocity_limit=0.8)
...   .add_joint_position_target("right_arm_3", 0.5)  # joint hint in the OC stage
...   .set_joint_stiffness(np.full(7, 40.0))          # K diag for joint impedance
...   .set_joint_damping_ratio(0.7)                   # ζ → D
...   .set_joint_torque_limit(np.full(7, 35.0))       # τ saturation
...   .set_minimum_time(2.0)
... )
)doc")
      .def(py::init<>(), R"doc(
Construct a CartesianImpedanceControlCommandBuilder instance.
)doc")
      .def("set_command_header", &CartesianImpedanceControlCommandBuilder::SetCommandHeader, "command_header_builder"_a,
           R"doc(
Attach a command header.

Parameters
----------
command_header_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_minimum_time", &CartesianImpedanceControlCommandBuilder::SetMinimumTime, "minimum_time"_a, R"doc(
Set minimum execution time.

Parameters
----------
minimum_time : float
    Minimum time [s] to execute trajectory.

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("add_target", &CartesianImpedanceControlCommandBuilder::AddTarget, "ref_link_name"_a, "link_name"_a, "T"_a,
           "linear_velocity_limit"_a = py::none(), "angular_velocity_limit"_a = py::none(),
           "linear_acceleration_limit"_a = py::none(), "angular_acceleration_limit"_a = py::none(), R"doc(
Add a Cartesian impedance target.

Parameters
----------
ref_link_name : str
    Reference frame (link) name.
link_name : str
    Controlled link name (end-effector).
T : numpy.ndarray, shape (4,4), dtype=float64
    Desired homogeneous transform :math:`{}^{ref}\!T^{link}_d`.
linear_velocity_limit : Optional[float]
    Max translational velocity [m/s].
angular_velocity_limit : Optional[float]
    Max angular velocity [rad/s].
linear_acceleration_limit : Optional[float]
    Max translational acceleration [m/s²].
angular_acceleration_limit : Optional[float]
    Max angular acceleration [rad/s²].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("add_joint_position_target", &CartesianImpedanceControlCommandBuilder::AddJointPositionTarget,
           "joint_name"_a, "target_position"_a, "velocity_limit"_a = py::none(), "acceleration_limit"_a = py::none(),
           R"doc(
Add an auxiliary joint position target.

Parameters
----------
joint_name : str
    Joint name.
target_position : float
    Desired position [rad].
velocity_limit : Optional[float]
    Max velocity [rad/s].
acceleration_limit : Optional[float]
    Max acceleration [rad/s²].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_stop_position_tracking_error", &CartesianImpedanceControlCommandBuilder::SetStopPositionTrackingError,
           "stop_position_tracking_error"_a, R"doc(
Set stop threshold on translational error.

Parameters
----------
stop_position_tracking_error : float
    Threshold [m].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("set_stop_orientation_tracking_error",
           &CartesianImpedanceControlCommandBuilder::SetStopOrientationTrackingError,
           "stop_orientation_tracking_error"_a, R"doc(
Set stop threshold on orientation error.

Parameters
----------
stop_orientation_tracking_error : float
    Threshold [rad].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("set_stop_joint_position_tracking_error",
           &CartesianImpedanceControlCommandBuilder::SetStopJointPositionTrackingError,
           "stop_joint_position_tracking_error"_a, R"doc(
Set stop threshold on joint position error.

Parameters
----------
stop_joint_position_tracking_error : float
    Threshold [rad].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("set_joint_stiffness", &CartesianImpedanceControlCommandBuilder::SetJointStiffness, "stiffness"_a, R"doc(
Set per-joint stiffness.

Parameters
----------
stiffness : numpy.ndarray, shape (N,), dtype=float64
    Joint stiffness values (Nm/rad).

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("set_joint_torque_limit", &CartesianImpedanceControlCommandBuilder::SetJointTorqueLimit, "torque_limit"_a,
           R"doc(
Set per-joint torque limits.

Parameters
----------
torque_limit : numpy.ndarray, shape (N,), dtype=float64
    Max torques [Nm].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("set_joint_damping_ratio", &CartesianImpedanceControlCommandBuilder::SetJointDampingRatio, "damping_ratio"_a,
           R"doc(
Set joint damping ratio.

Parameters
----------
damping_ratio : float
    Dimensionless damping ratio ζ.

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("add_joint_limit", &CartesianImpedanceControlCommandBuilder::AddJointLimit, "joint_name"_a, "lower"_a,
           "upper"_a, R"doc(
Add a joint position limit.

Parameters
----------
joint_name : str
    Joint name.
lower : float
    Lower limit [rad].
upper : float
    Upper limit [rad].

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc")

      .def("set_reset_reference", &CartesianImpedanceControlCommandBuilder::SetResetReference, "reset_reference"_a,
           R"doc(
Set whether to reset the reference pose.

Parameters
----------
reset_reference : bool
    If True, resets the reference configuration before applying new targets.

Returns
-------
CartesianImpedanceControlCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<GravityCompensationCommandBuilder>(m, "GravityCompensationCommandBuilder", R"doc(
Gravity compensation command builder.

Creates a command that enables or disables gravity compensation on a
component (e.g., an arm or the whole body). When enabled, the controller
generates feed-forward torques to cancel the effect of gravity so the
robot feels weightless.

Examples
--------
>>> import rby1_sdk as rby
>>> # Enable gravity compensation for the right arm
>>> gc = rby.GravityCompensationCommandBuilder().set_on(True)
>>> cmd = rby.RobotCommandBuilder().set_command(
...     rby.ArmCommandBuilder().set_command(gc)
... )
>>> # robot.send_command(cmd, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a GravityCompensationCommandBuilder instance.
)doc")
      .def("set_command_header", &GravityCompensationCommandBuilder::SetCommandHeader,
           "gravity_compensation_command_builder"_a, R"doc(
Attach a command header.

Parameters
----------
gravity_compensation_command_builder : CommandHeaderBuilder
    Command header configuration (e.g., control hold time).

Returns
-------
GravityCompensationCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_on", &GravityCompensationCommandBuilder::SetOn, "on"_a, R"doc(
Enable or disable gravity compensation.

Parameters
----------
on : bool
    ``True`` to enable gravity compensation, ``False`` to disable.

Returns
-------
GravityCompensationCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<ArmCommandBuilder>(m, "ArmCommandBuilder", R"doc(
Arm command builder.

Wraps a command for an arm component.  
This builder accepts multiple command types, such as joint position,
gravity compensation, Cartesian impedance, or joint impedance commands,
and provides a unified interface to send them to an arm.

Notes
-----
- In practice, you often do **not** need to explicitly construct an
  ArmCommandBuilder.  
  Functions like ``set_right_arm_command(...)`` or ``set_left_arm_command(...)``
  automatically cast and wrap a command (e.g., JointPositionCommandBuilder)
  into an ArmCommandBuilder.  
  For example:

  >>> body.set_right_arm_command(rby.JointPositionCommandBuilder().set_position(...))

  is equivalent to:

  >>> body.set_right_arm_command(rby.ArmCommandBuilder(
  ...     rby.JointPositionCommandBuilder().set_position(...)))

- Therefore, explicitly constructing ArmCommandBuilder is usually unnecessary
  unless you want to be explicit; repeatedly wrapping it is inefficient.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> # Send a joint position command to the right arm
>>> jp = (
...     rby.JointPositionCommandBuilder()
...     .set_position(np.zeros(7))
...     .set_minimum_time(2.0)
... )
>>> body = rby.BodyComponentBasedCommandBuilder()
>>> # Implicit wrapping into ArmCommandBuilder
>>> body.set_right_arm_command(jp)
>>> cmd = rby.RobotCommandBuilder().set_command(body)
>>> # robot.send_command(cmd, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct an empty ArmCommandBuilder instance.
)doc")
      .def(py::init<const JointPositionCommandBuilder&>(), "joint_position_command_builder"_a, R"doc(
Construct an ArmCommandBuilder with a joint position command.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Joint position command to wrap.
)doc")
      .def(py::init<const GravityCompensationCommandBuilder&>(), "gravity_compensation_command_builder"_a, R"doc(
Construct an ArmCommandBuilder with a gravity compensation command.

Parameters
----------
gravity_compensation_command_builder : GravityCompensationCommandBuilder
    Gravity compensation command to wrap.
)doc")
      .def(py::init<const CartesianCommandBuilder&>(), "cartesian_command_builder"_a, R"doc(
Construct an ArmCommandBuilder with a Cartesian command.

Parameters
----------
cartesian_command_builder : CartesianCommandBuilder
    Cartesian command to wrap.
)doc")
      .def(py::init<const ImpedanceControlCommandBuilder&>(), "impedance_control_command_builder"_a, R"doc(
Construct an ArmCommandBuilder with a Impedance control command.

Parameters
----------
impedance_control_command_builder : ImpedanceControlCommandBuilder
    Cartesian impedance command to wrap.
)doc")
      .def(py::init<const JointImpedanceControlCommandBuilder&>(), "joint_impedance_control_command_builder"_a, R"doc(
Construct an ArmCommandBuilder with a joint impedance control command.

Parameters
----------
joint_impedance_control_command_builder : JointImpedanceControlCommandBuilder
    Joint impedance control command to wrap.
)doc")
      .def(py::init<const CartesianImpedanceControlCommandBuilder&>(), "cartesian_impedance_control_command_builder"_a,
           R"doc(
Construct an ArmCommandBuilder with a Cartesian impedance control command.

Parameters
----------
cartesian_impedance_control_command_builder : CartesianImpedanceControlCommandBuilder
    Cartesian impedance control command to wrap.
)doc")
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "joint_position_command_builder"_a, R"doc(
Set a joint position command for the arm.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Joint position command to wrap.

Returns
-------
ArmCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "gravity_compensation_command_builder"_a, R"doc(
Set a gravity compensation command for the arm.

Parameters
----------
gravity_compensation_command_builder : GravityCompensationCommandBuilder
    Gravity compensation command to wrap.

Returns
-------
ArmCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "cartesian_command_builder"_a, R"doc(
Set a Cartesian command for the arm.

Parameters
----------
cartesian_command_builder : CartesianCommandBuilder
    Cartesian command to wrap.

Returns
-------
ArmCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const ImpedanceControlCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "impedance_control_command_builder"_a, R"doc(
Set a Impedance control command for the arm.

Parameters
----------
impedance_control_command_builder : ImpedanceControlCommandBuilder
    Cartesian impedance control command to wrap.

Returns
-------
ArmCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const JointImpedanceControlCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "joint_impedance_control_command_builder"_a, R"doc(
Set a joint impedance control command for the arm.

Parameters
----------
joint_impedance_control_command_builder : JointImpedanceControlCommandBuilder
    Joint impedance control command to wrap.

Returns
-------
ArmCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command",
           py::overload_cast<const CartesianImpedanceControlCommandBuilder&>(&ArmCommandBuilder::SetCommand),
           "cartesian_impedance_control_command_builder"_a, R"doc(
Set a Cartesian impedance control command for the arm.

Parameters
----------
cartesian_impedance_control_command_builder : CartesianImpedanceControlCommandBuilder
    Cartesian impedance control command to wrap.

Returns
-------
ArmCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<TorsoCommandBuilder>(m, "TorsoCommandBuilder", R"doc(
Torso command builder.

Builds commands specifically for the torso component.  
Accepts different command types (joint position, impedance, Cartesian, etc.) and wraps them into a torso command.

Notes
-----
- You can pass a command builder (e.g., `JointPositionCommandBuilder`) directly into
  `.set_torso_command(...)` without explicitly creating a `TorsoCommandBuilder`.  
  Implicit casting automatically wraps it:  
  `TorsoCommandBuilder(JointPositionCommandBuilder)` is created internally.  
  Therefore, directly using `TorsoCommandBuilder` is often unnecessary unless you want
  to explicitly wrap or extend torso-specific logic.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> torso_cmd = (
...     rby.TorsoCommandBuilder(
...         rby.JointPositionCommandBuilder().set_position(np.zeros(6))
...     )
... )
)doc")
      .def(py::init<>(), R"doc(
Construct an empty TorsoCommandBuilder.
)doc")
      .def(py::init<const JointPositionCommandBuilder&>(), "joint_position_command_builder"_a, R"doc(
Construct from a joint position command.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Command specifying torso joint positions.
)doc")
      .def(py::init<const GravityCompensationCommandBuilder&>(), "gravity_compensation_command_builder"_a, R"doc(
Construct from a gravity compensation command.

Parameters
----------
gravity_compensation_command_builder : GravityCompensationCommandBuilder
    Command enabling/disabling gravity compensation for the torso.
)doc")
      .def(py::init<const CartesianCommandBuilder&>(), "cartesian_command_builder"_a, R"doc(
Construct from a Cartesian command.

Parameters
----------
cartesian_command_builder : CartesianCommandBuilder
    Command specifying Cartesian targets for the torso.
)doc")
      .def(py::init<const ImpedanceControlCommandBuilder&>(), "impedance_control_command_builder"_a, R"doc(
Construct from a Cartesian impedance control command.

Parameters
----------
impedance_control_command_builder : ImpedanceControlCommandBuilder
    Command specifying Cartesian impedance for the torso.
)doc")
      .def(py::init<const OptimalControlCommandBuilder&>(), "optimal_control_command_builder"_a, R"doc(
Construct from an optimal control command.

Parameters
----------
optimal_control_command_builder : OptimalControlCommandBuilder
    Command specifying optimal control targets/costs for the torso.
)doc")
      .def(py::init<const JointImpedanceControlCommandBuilder&>(), "joint_impedance_control_command_builder"_a, R"doc(
Construct from a joint impedance control command.

Parameters
----------
joint_impedance_control_command_builder : JointImpedanceControlCommandBuilder
    Command specifying joint-space impedance for the torso.
)doc")
      .def(py::init<const CartesianImpedanceControlCommandBuilder&>(), "cartesian_impedance_control_command_builder"_a,
           R"doc(
Construct from a Cartesian impedance control command.

Parameters
----------
cartesian_impedance_control_command_builder : CartesianImpedanceControlCommandBuilder
    Command specifying Cartesian impedance control for the torso.
)doc")
      .def(py::init<const JointGroupPositionCommandBuilder&>(), "joint_group_position_command_builder"_a, R"doc(
Construct from a joint group position command.

Parameters
----------
joint_group_position_command_builder : JointGroupPositionCommandBuilder
    Command specifying joint positions for a named torso joint group.
)doc")
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "joint_position_command_builder"_a, R"doc(
Set the torso command from a joint position builder.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Command specifying torso joint positions.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "gravity_compensation_command_builder"_a, R"doc(
Set the torso command from a gravity compensation builder.

Parameters
----------
gravity_compensation_command_builder : GravityCompensationCommandBuilder
    Command enabling/disabling torso gravity compensation.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "cartesian_command_builder"_a, R"doc(
Set the torso command from a Cartesian builder.

Parameters
----------
cartesian_command_builder : CartesianCommandBuilder
    Command specifying Cartesian pose targets for the torso.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const ImpedanceControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "impedance_control_command_builder"_a, R"doc(
Set the torso command from a Cartesian impedance builder.

Parameters
----------
impedance_control_command_builder : ImpedanceControlCommandBuilder
    Command specifying Cartesian impedance for the torso.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const OptimalControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "optimal_control_command_builder"_a, R"doc(
Set the torso command from an optimal control builder.

Parameters
----------
optimal_control_command_builder : OptimalControlCommandBuilder
    Command specifying optimal control targets/costs for the torso.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command",
           py::overload_cast<const JointImpedanceControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "joint_impedance_control_command_builder"_a, R"doc(
Set the torso command from a joint impedance builder.

Parameters
----------
joint_impedance_control_command_builder : JointImpedanceControlCommandBuilder
    Command specifying joint-space impedance for the torso.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command",
           py::overload_cast<const CartesianImpedanceControlCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "cartesian_impedance_control_command_builder"_a, R"doc(
Set the torso command from a Cartesian impedance builder.

Parameters
----------
cartesian_impedance_control_command_builder : CartesianImpedanceControlCommandBuilder
    Command specifying Cartesian impedance control for the torso.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const JointGroupPositionCommandBuilder&>(&TorsoCommandBuilder::SetCommand),
           "joint_group_position_command_builder"_a, R"doc(
Set the torso command from a joint group position builder.

Parameters
----------
joint_group_position_command_builder : JointGroupPositionCommandBuilder
    Command specifying joint positions for a named torso joint group.

Returns
-------
TorsoCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<BodyComponentBasedCommandBuilder>(m, "BodyComponentBasedCommandBuilder", R"doc(
Body component-based command builder.

Provides methods to assign commands to specific body parts: right arm,
left arm, and torso.  
Each setter accepts either the corresponding *CommandBuilder wrapper*
(e.g., ArmCommandBuilder, TorsoCommandBuilder) **or** a lower-level builder
like JointPositionCommandBuilder, which will be automatically cast.

Notes
-----
- For convenience, you can directly pass a JointPositionCommandBuilder
  (or other valid builders) without explicitly wrapping it in an
  ArmCommandBuilder or TorsoCommandBuilder. The system will automatically
  perform the conversion.
- This avoids redundant wrapping and makes usage more concise.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> body = rby.BodyComponentBasedCommandBuilder()
>>> # Implicitly wraps into ArmCommandBuilder
>>> body.set_right_arm_command(
...     rby.JointPositionCommandBuilder().set_position(np.zeros(7))
... )
>>> # Implicitly wraps into TorsoCommandBuilder
>>> body.set_torso_command(
...     rby.JointPositionCommandBuilder().set_position(np.zeros(6))
... )
>>> cmd = rby.RobotCommandBuilder().set_command(body)
>>> # robot.send_command(cmd, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a BodyComponentBasedCommandBuilder instance.
)doc")
      .def("set_right_arm_command", &BodyComponentBasedCommandBuilder::SetRightArmCommand, "arm_command_builder"_a,
           R"doc(
Assign a command to the right arm.

Parameters
----------
arm_command_builder : ArmCommandBuilder or compatible builder
    Command for the right arm. If a lower-level builder is provided
    (e.g., JointPositionCommandBuilder), it will be automatically
    wrapped into an ArmCommandBuilder.

Returns
-------
BodyComponentBasedCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_left_arm_command", &BodyComponentBasedCommandBuilder::SetLeftArmCommand, "arm_command_builder"_a, R"doc(
Assign a command to the left arm.

Parameters
----------
arm_command_builder : ArmCommandBuilder or compatible builder
    Command for the left arm. If a lower-level builder is provided
    (e.g., JointPositionCommandBuilder), it will be automatically
    wrapped into an ArmCommandBuilder.

Returns
-------
BodyComponentBasedCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_torso_command", &BodyComponentBasedCommandBuilder::SetTorsoCommand, "torso_command_builder"_a, R"doc(
Assign a command to the torso.

Parameters
----------
torso_command_builder : TorsoCommandBuilder or compatible builder
    Command for the torso. If a lower-level builder is provided
    (e.g., JointPositionCommandBuilder, OptimalControlCommandBuilder),
    it will be automatically wrapped into a TorsoCommandBuilder.

Returns
-------
BodyComponentBasedCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<BodyCommandBuilder>(m, "BodyCommandBuilder", R"doc(
Body command builder.

Wraps commands for the robot’s body subsystem (torso + both arms).  
It can hold a variety of command types including joint position,
optimal control, gravity compensation, Cartesian motion, and impedance control.

Notes
-----
- Typically used with ``set_body_command(...)`` of a ComponentBasedCommandBuilder.
- You do not need to explicitly wrap commands in BodyCommandBuilder;
  passing a supported builder (e.g., JointPositionCommandBuilder) into
  ``set_body_command`` will automatically cast it to BodyCommandBuilder.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> # Example 1: Joint position body command
>>> jp = rby.JointPositionCommandBuilder().set_position(np.zeros(20))  # full body, N=20
>>> body = rby.BodyCommandBuilder().set_command(jp)
>>> comp = rby.ComponentBasedCommandBuilder().set_body_command(body)
>>> rc = rby.RobotCommandBuilder().set_command(comp)
>>> # robot.send_command(rc, priority=1).get()
>>>
>>> # Example 2: Gravity compensation
>>> gc = rby.GravityCompensationCommandBuilder().set_on(True)
>>> comp = rby.ComponentBasedCommandBuilder().set_body_command(gc)  # implicit wrap
>>> rc = rby.RobotCommandBuilder().set_command(comp)
)doc")
      .def(py::init<>(), R"doc(
Construct a BodyCommandBuilder instance.
)doc")
      .def(py::init<const JointPositionCommandBuilder&>(), "joint_position_command_builder"_a, R"doc(
Construct from a joint position command.
)doc")
      .def(py::init<const OptimalControlCommandBuilder&>(), "optimal_control_command_builder"_a, R"doc(
Construct from an optimal control command.
)doc")
      .def(py::init<const GravityCompensationCommandBuilder&>(), "gravity_compensation_command_builder"_a, R"doc(
Construct from a gravity compensation command.
)doc")
      .def(py::init<const CartesianCommandBuilder&>(), "cartesian_command_builder"_a, R"doc(
Construct from a Cartesian command.
)doc")
      .def(py::init<const BodyComponentBasedCommandBuilder&>(), "body_component_based_command_builder"_a, R"doc(
Construct from a body component–based command (right/left arms, torso).
)doc")
      .def(py::init<const JointImpedanceControlCommandBuilder&>(), "joint_impedance_control_command_builder"_a, R"doc(
Construct from a joint impedance control command.
)doc")
      .def(py::init<const CartesianImpedanceControlCommandBuilder&>(), "cartesian_impedance_control_command_builder"_a,
           R"doc(
Construct from a Cartesian impedance control command.
)doc")
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "joint_position_command_builder"_a, R"doc(
Assign a joint position command.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Joint-space position command (vector length N = body DOF).

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const OptimalControlCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "optimal_control_command_builder"_a, R"doc(
Assign an optimal control command.

Parameters
----------
optimal_control_command_builder : OptimalControlCommandBuilder
    Optimal control objective.

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const GravityCompensationCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "gravity_compensation_command_builder"_a, R"doc(
Assign a gravity compensation command.

Parameters
----------
gravity_compensation_command_builder : GravityCompensationCommandBuilder
    Command enabling or disabling gravity compensation.

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const CartesianCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "cartesian_command_builder"_a, R"doc(
Assign a Cartesian command.

Parameters
----------
cartesian_command_builder : CartesianCommandBuilder
    Cartesian trajectory command.

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const BodyComponentBasedCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "body_component_based_command_builder"_a, R"doc(
Assign a body component–based command (arms, torso).

Parameters
----------
body_component_based_command_builder : BodyComponentBasedCommandBuilder
    Command composed of right arm, left arm, and torso subcommands.

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command",
           py::overload_cast<const JointImpedanceControlCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "joint_impedance_control_command_builder"_a, R"doc(
Assign a joint impedance control command.

Parameters
----------
joint_impedance_control_command_builder : JointImpedanceControlCommandBuilder
    Joint-space impedance control command.

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command",
           py::overload_cast<const CartesianImpedanceControlCommandBuilder&>(&BodyCommandBuilder::SetCommand),
           "cartesian_impedance_control_command_builder"_a, R"doc(
Assign a Cartesian impedance control command.

Parameters
----------
cartesian_impedance_control_command_builder : CartesianImpedanceControlCommandBuilder
    Cartesian impedance control command.

Returns
-------
BodyCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<MobilityCommandBuilder>(m, "MobilityCommandBuilder", R"doc(
Mobility command builder.

Wraps commands for the robot’s mobility subsystem.  
It supports both joint-space velocity commands (e.g., differential/mecanum wheels)
and planar SE(2) velocity commands.

Notes
-----
- Typically used inside ``set_mobility_command(...)`` of a ComponentBasedCommandBuilder.
- You do not need to explicitly construct MobilityCommandBuilder if passing
  a JointVelocityCommandBuilder or SE2VelocityCommandBuilder, as implicit casting
  will wrap them automatically.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> # Example 1: Joint velocity mobility (e.g., wheel joints)
>>> vel = rby.JointVelocityCommandBuilder().set_velocity(np.full(4, 1.0))
>>> mobility = rby.MobilityCommandBuilder().set_command(vel)
>>> comp = rby.ComponentBasedCommandBuilder().set_mobility_command(mobility)
>>> rc = rby.RobotCommandBuilder().set_command(comp)
>>> # robot.send_command(rc, priority=1).get()
>>>
>>> # Example 2: SE(2) velocity mobility
>>> se2 = rby.SE2VelocityCommandBuilder().set_velocity(np.array([0.2, 0.0]), 0.1)  # vx=0.2 m/s, yaw=0.1 rad/s
>>> comp = rby.ComponentBasedCommandBuilder().set_mobility_command(se2)  # implicit wrap
>>> rc = rby.RobotCommandBuilder().set_command(comp)
)doc")
      .def(py::init<>(), R"doc(
Construct a MobilityCommandBuilder instance.
)doc")
      .def(py::init<const JointVelocityCommandBuilder&>(), "joint_velocity_command_builder"_a, R"doc(
Construct a MobilityCommandBuilder from a joint velocity command.

Parameters
----------
joint_velocity_command_builder : JointVelocityCommandBuilder
    Joint-space velocity command (e.g., wheel joint velocities).
)doc")
      .def(py::init<const SE2VelocityCommandBuilder&>(), "se2_velocity_command_builder"_a, R"doc(
Construct a MobilityCommandBuilder from an SE(2) velocity command.

Parameters
----------
se2_velocity_command_builder : SE2VelocityCommandBuilder
    Planar velocity command in SE(2).
)doc")
      .def("set_command", py::overload_cast<const JointVelocityCommandBuilder&>(&MobilityCommandBuilder::SetCommand),
           "joint_velocity_command_builder"_a, R"doc(
Assign a joint velocity command.

Parameters
----------
joint_velocity_command_builder : JointVelocityCommandBuilder
    Joint-space velocity command (rad/s for each mobility joint).

Returns
-------
MobilityCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const SE2VelocityCommandBuilder&>(&MobilityCommandBuilder::SetCommand),
           "se2_velocity_command_builder"_a, R"doc(
Assign an SE(2) velocity command.

Parameters
----------
se2_velocity_command_builder : SE2VelocityCommandBuilder
    Planar velocity command with linear [m/s] and angular [rad/s] components.

Returns
-------
MobilityCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<HeadCommandBuilder>(m, "HeadCommandBuilder", R"doc(
Head command builder.

Wraps commands for the robot head.  
Currently, it supports **JointPositionCommandBuilder** to control the head joints.

Notes
-----
- You can pass a JointPositionCommandBuilder directly into
  ``set_head_command(...)`` without explicitly wrapping it in a HeadCommandBuilder,
  since implicit casting is provided.
- Explicit construction is only needed if you want to be verbose; otherwise,
  ``set_head_command(JointPositionCommandBuilder(...))`` is sufficient.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> # Example 1: Explicit HeadCommandBuilder
>>> jp = rby.JointPositionCommandBuilder().set_position(np.zeros(2))  # assume 2 DOF head
>>> head = rby.HeadCommandBuilder(jp)
>>> comp = rby.ComponentBasedCommandBuilder().set_head_command(head)
>>> rc = rby.RobotCommandBuilder().set_command(comp)
>>> # robot.send_command(rc, priority=1).get()
>>>
>>> # Example 2: Implicit wrapping (preferred)
>>> comp = rby.ComponentBasedCommandBuilder().set_head_command(
...     rby.JointPositionCommandBuilder().set_position(np.zeros(2))
... )
>>> rc = rby.RobotCommandBuilder().set_command(comp)
)doc")
      .def(py::init<>(), R"doc(
Construct a HeadCommandBuilder instance.
)doc")
      .def(py::init<const JointPositionCommandBuilder&>(), "joint_position_command_builder"_a, R"doc(
Construct a HeadCommandBuilder from a joint position command.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Command specifying target joint positions for the head.
)doc")
      .def("set_command", py::overload_cast<const JointPositionCommandBuilder&>(&HeadCommandBuilder::SetCommand),
           "joint_position_command_builder"_a, R"doc(
Assign a joint position command to the head.

Parameters
----------
joint_position_command_builder : JointPositionCommandBuilder
    Joint position command for the head.

Returns
-------
HeadCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<ComponentBasedCommandBuilder>(m, "ComponentBasedCommandBuilder", R"doc(
Component-based command builder.

Provides a unified builder interface to compose commands for the robot’s
main subsystems: mobility, body, and head.  
Each setter accepts either the corresponding *CommandBuilder wrapper*
(e.g., MobilityCommandBuilder, BodyCommandBuilder, HeadCommandBuilder)  
or lower-level builders (e.g., JointVelocityCommandBuilder, JointPositionCommandBuilder),
which are automatically cast into the appropriate wrapper.

Notes
-----
- This is typically used inside a RobotCommandBuilder to assemble
  subsystem-level commands into a single robot command.
- Automatic casting removes the need to explicitly wrap builders,
  simplifying code.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> comp = rby.ComponentBasedCommandBuilder()
>>> # Mobility: send SE2 velocity
>>> comp.set_mobility_command(
...     rby.SE2VelocityCommandBuilder().set_velocity(np.array([0.1, 0.0]), 0.2)
... )
>>> # Body: send a joint position command to the right arm
>>> comp.set_body_command(
...     rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
...         rby.JointPositionCommandBuilder().set_position(np.zeros(7))
...     )
... )
>>> # Head: send a joint position command to head joints
>>> comp.set_head_command(
...     rby.JointPositionCommandBuilder().set_position(np.zeros(2))
... )
>>> cmd = rby.RobotCommandBuilder().set_command(comp)
>>> # robot.send_command(cmd, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a ComponentBasedCommandBuilder instance.
)doc")
      .def("set_mobility_command", &ComponentBasedCommandBuilder::SetMobilityCommand, "mobility_command_builder"_a,
           R"doc(
Assign a mobility command.

Parameters
----------
mobility_command_builder : MobilityCommandBuilder or compatible builder
    Command for the mobility subsystem. If a lower-level builder is provided
    (e.g., SE2VelocityCommandBuilder, JointVelocityCommandBuilder),
    it will be automatically wrapped into a MobilityCommandBuilder.

Returns
-------
ComponentBasedCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_body_command", &ComponentBasedCommandBuilder::SetBodyCommand, "body_command_builder"_a, R"doc(
Assign a body command.

Parameters
----------
body_command_builder : BodyCommandBuilder or compatible builder
    Command for the body subsystem. If a lower-level builder is provided
    (e.g., BodyComponentBasedCommandBuilder, JointPositionCommandBuilder),
    it will be automatically wrapped into a BodyCommandBuilder.

Returns
-------
ComponentBasedCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_head_command", &ComponentBasedCommandBuilder::SetHeadCommand, "head_command_builder"_a, R"doc(
Assign a head command.

Parameters
----------
head_command_builder : HeadCommandBuilder or compatible builder
    Command for the head subsystem. If a lower-level builder is provided
    (e.g., JointPositionCommandBuilder),
    it will be automatically wrapped into a HeadCommandBuilder.

Returns
-------
ComponentBasedCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<WholeBodyCommandBuilder>(m, "WholeBodyCommandBuilder", R"doc(
Whole-body command builder.

Provides a builder interface to construct commands that apply to the **entire robot**.  
Currently, its primary role is to wrap and send a StopCommand to halt all motion.

Notes
-----
- While ComponentBasedCommandBuilder targets subsystems (mobility, body, head),
  WholeBodyCommandBuilder applies at the global level.

Examples
--------
>>> import rby1_sdk as rby
>>> # Construct a stop command for the whole robot
>>> stop = rby.StopCommandBuilder()
>>> whole = rby.WholeBodyCommandBuilder().set_command(stop)
>>> cmd = rby.RobotCommandBuilder().set_command(whole)
>>> # robot.send_command(cmd, priority=0).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a WholeBodyCommandBuilder instance.
)doc")
      .def(py::init<const StopCommandBuilder&>(), "stop_command_builder"_a, R"doc(
Construct a WholeBodyCommandBuilder with an initial stop command.

Parameters
----------
stop_command_builder : StopCommandBuilder
    Stop command to initialize this builder with.
)doc")
      .def("set_command", py::overload_cast<const StopCommandBuilder&>(&WholeBodyCommandBuilder::SetCommand),
           "stop_command_builder"_a, R"doc(
Assign a stop command to the whole body.

Parameters
----------
stop_command_builder : StopCommandBuilder
    Stop command to apply globally.

Returns
-------
WholeBodyCommandBuilder
    Self reference for method chaining.
)doc");

  py::class_<RobotCommandBuilder>(m, "RobotCommandBuilder", R"doc(
Robot command builder.

Top-level builder that aggregates all types of commands into a single
robot command.  
It can wrap a **WholeBodyCommandBuilder**, **ComponentBasedCommandBuilder**, or
**JogCommandBuilder**, and is the final object passed to
``robot.send_command(...)``.

Notes
-----
- Typically, you do not build this manually for each subsystem.  
  Instead, construct subsystem builders (e.g., ComponentBasedCommandBuilder
  with body/head/mobility commands) and wrap them in RobotCommandBuilder.
- Jog commands can also be wrapped directly for lightweight incremental motions.
- Implicit casting is supported, so passing compatible builders automatically
  creates the appropriate wrapper.

Examples
--------
>>> import numpy as np, rby1_sdk as rby
>>> # Example 1: Full stop
>>> stop = rby.StopCommandBuilder()
>>> whole = rby.WholeBodyCommandBuilder().set_command(stop)
>>> rc = rby.RobotCommandBuilder().set_command(whole)
>>> # robot.send_command(rc, priority=0).get()
>>>
>>> # Example 2: Move right arm with a joint position command
>>> jp = rby.JointPositionCommandBuilder().set_position(np.zeros(7))
>>> body = rby.BodyComponentBasedCommandBuilder().set_right_arm_command(jp)
>>> comp = rby.ComponentBasedCommandBuilder().set_body_command(body)
>>> rc = rby.RobotCommandBuilder().set_command(comp)
>>> # robot.send_command(rc, priority=1).get()
>>>
>>> # Example 3: Send a jog command
>>> jog = rby.JogCommandBuilder().set_joint_name("right_arm_0") \
...     .set_command(rby.JogCommandBuilder.RelativePosition(0.1))
>>> rc = rby.RobotCommandBuilder().set_command(jog)
>>> # robot.send_command(rc, priority=1).get()
)doc")
      .def(py::init<>(), R"doc(
Construct a RobotCommandBuilder instance.
)doc")
      .def(py::init<const WholeBodyCommandBuilder&>(), "whole_body_command_builder"_a, R"doc(
Construct a RobotCommandBuilder from a WholeBodyCommandBuilder.

Parameters
----------
whole_body_command_builder : WholeBodyCommandBuilder
    Whole-body command wrapper.
)doc")
      .def(py::init<const ComponentBasedCommandBuilder&>(), "component_based_command_builder"_a, R"doc(
Construct a RobotCommandBuilder from a ComponentBasedCommandBuilder.

Parameters
----------
component_based_command_builder : ComponentBasedCommandBuilder
    Component-based command wrapper (mobility, body, head).
)doc")
      .def(py::init<const JogCommandBuilder&>(), "jog_command_builder"_a, R"doc(
Construct a RobotCommandBuilder from a JogCommandBuilder.

Parameters
----------
jog_command_builder : JogCommandBuilder
    Jog command wrapper for single-joint incremental motion.
)doc")
      .def("set_command", py::overload_cast<const WholeBodyCommandBuilder&>(&RobotCommandBuilder::SetCommand),
           "whole_body_command_builder"_a, R"doc(
Assign a whole-body command.

Parameters
----------
whole_body_command_builder : WholeBodyCommandBuilder
    Command to apply to the entire robot.

Returns
-------
RobotCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const ComponentBasedCommandBuilder&>(&RobotCommandBuilder::SetCommand),
           "component_based_command_builder"_a, R"doc(
Assign a component-based command.

Parameters
----------
component_based_command_builder : ComponentBasedCommandBuilder
    Command to apply to subsystems (mobility, body, head).

Returns
-------
RobotCommandBuilder
    Self reference for method chaining.
)doc")
      .def("set_command", py::overload_cast<const JogCommandBuilder&>(&RobotCommandBuilder::SetCommand),
           "jog_command_builder"_a, R"doc(
Assign a jog command.

Parameters
----------
jog_command_builder : JogCommandBuilder
    Jog command for single-joint incremental motion.

Returns
-------
RobotCommandBuilder
    Self reference for method chaining.
)doc");

  // Implicit conversion

  py::implicitly_convertible<double, JogCommandBuilder::AbsolutePosition>();

  py::implicitly_convertible<double, JogCommandBuilder::RelativePosition>();

  py::implicitly_convertible<double, JogCommandBuilder::OneStep>();

  py::implicitly_convertible<JointPositionCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<ImpedanceControlCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<JointImpedanceControlCommandBuilder, ArmCommandBuilder>();
  py::implicitly_convertible<CartesianImpedanceControlCommandBuilder, ArmCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<ImpedanceControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<OptimalControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<JointImpedanceControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<CartesianImpedanceControlCommandBuilder, TorsoCommandBuilder>();
  py::implicitly_convertible<JointGroupPositionCommandBuilder, TorsoCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<OptimalControlCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<GravityCompensationCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<CartesianCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<BodyComponentBasedCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<JointImpedanceControlCommandBuilder, BodyCommandBuilder>();
  py::implicitly_convertible<CartesianImpedanceControlCommandBuilder, BodyCommandBuilder>();

  py::implicitly_convertible<JointVelocityCommandBuilder, MobilityCommandBuilder>();
  py::implicitly_convertible<SE2VelocityCommandBuilder, MobilityCommandBuilder>();

  py::implicitly_convertible<JointPositionCommandBuilder, HeadCommandBuilder>();

  py::implicitly_convertible<StopCommandBuilder, WholeBodyCommandBuilder>();

  py::implicitly_convertible<WholeBodyCommandBuilder, RobotCommandBuilder>();
  py::implicitly_convertible<ComponentBasedCommandBuilder, RobotCommandBuilder>();
  py::implicitly_convertible<JogCommandBuilder, RobotCommandBuilder>();
}