import rby1_sdk
import numpy as np

robot = rby1_sdk.create_robot_a("0.0.0.0:50051")
robot.connect()

robot.power_on(".*")

robot_state = robot.get_state()

print(robot_state.collisions)