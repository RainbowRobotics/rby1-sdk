import rby1_sdk as rby
import rby1_sdk.dynamics as rby_dyn

import numpy as np

np.set_printoptions(precision=2, suppress=True)

# 1. Connect to the robot and get its model information.
robot = rby.create_robot("localhost:50051", "a")
robot.connect()
model = robot.model()

# 2. Get the dynamics model (dyn_robot) from the robot.
dyn_robot = robot.get_dynamics()

# 3. Create a State object for dynamics calculations.
#    Specify the names of the links and joints to be used in the calculations.
dyn_state = dyn_robot.make_state(["base", "ee_right"], model.robot_joint_names)

# 4. Define and set the robot's state (q, qdot, qddot).
#    Here, we use random joint angles for demonstration.
q = (np.random.rand(model.robot_dof) - 0.5) * np.pi / 2
dyn_state.set_gravity(np.array([0, 0, 0, 0, 0, -9.81]))
dyn_state.set_q(q)
dyn_state.set_qdot(np.zeros(model.robot_dof))
dyn_state.set_qddot(np.zeros(model.robot_dof))

print(f"{dyn_state = }")

# 5. Perform Inverse Dynamics calculation.
#    To compute inverse dynamics, forward kinematics-related calculations must be performed first.
#    These functions cache their results within the dyn_state object.

# 5-1. Compute forward kinematics (FK) to get the position/orientation of each link.
dyn_robot.compute_forward_kinematics(dyn_state)

# 5-2. Compute the first derivative of FK (Jacobian) to get the velocity of each link.
#      Must be called after `compute_forward_kinematics`.
dyn_robot.compute_diff_forward_kinematics(dyn_state)

# 5-3. Compute the second derivative of FK to get the acceleration of each link.
#      Must be called after `compute_diff_forward_kinematics`.
dyn_robot.compute_2nd_diff_forward_kinematics(dyn_state)

# 5-4. Based on the kinematic information calculated above,
#      compute the joint torques (tau) required to maintain the current state (q, qdot, qddot).
dyn_robot.compute_inverse_dynamics(dyn_state)

# 6. Print the calculated torques.
with np.printoptions(precision=4, suppress=True):
    print(f"Inverse dynamics torque (Nm): {dyn_state.get_tau()}")