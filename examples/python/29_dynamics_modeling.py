import rby1_sdk.dynamics as rby_dyn
import numpy as np

link_0 = rby_dyn.Link("link_0")
link_1 = rby_dyn.Link("link_1")

joint_0 = rby_dyn.Joint.make_revolute("joint_0", np.identity(4), np.array([0, 0, 1]))
joint_0.connect_links(link_0, link_1, np.identity(4), np.identity(4))

dyn_robot = rby_dyn.Robot(
    rby_dyn.RobotConfiguration(name="sample_robot", base_link=link_0)
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