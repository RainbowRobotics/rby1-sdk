import rby1_sdk
import numpy as np

ROBOT_ADDRESS = "192.168.1.229:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

state = robot.get_state()

print(f"time [{state.timestamp}]")
print(f"current position: {state.position}")
print(f"center of mass: {state.center_of_mass}")
