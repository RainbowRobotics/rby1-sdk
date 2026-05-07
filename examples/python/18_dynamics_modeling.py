# Dynamics Modeling Example
# This example builds a minimal two-link dynamics model, sets a joint angle,
# computes forward kinematics, and prints the transform from link_0 to link_1. See --help for arguments.
#
# Usage example:
#   python 18_dynamics_modeling.py
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import numpy as np

link_0 = rby.dynamics.Link("link_0")
link_1 = rby.dynamics.Link("link_1")

joint_0 = rby.dynamics.Joint.make_revolute(
    "joint_0", np.identity(4), np.array([0, 0, 1])
)
joint_0.connect_links(link_0, link_1, np.identity(4), np.identity(4))

dyn_robot = rby.dynamics.Robot(
    rby.dynamics.RobotConfiguration(name="sample_robot", base_link=link_0)
)

dyn_state = dyn_robot.make_state(["link_0", "link_1"], ["joint_0"])
dyn_state.set_q(np.array([np.pi / 2]))  # Angle of joint_0 is 90 degrees

print("link_0:")
print(f"{link_0}")

dyn_robot.compute_forward_kinematics(dyn_state)
# Calculate transformation from link_0 to link_1
transform = dyn_robot.compute_transformation(dyn_state, 0, 1)  # 0: link_0, 1: link_1
print("Transformation from link_0 to link_1:")
print(f"{transform}")
