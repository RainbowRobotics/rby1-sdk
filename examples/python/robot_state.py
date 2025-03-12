import rby1_sdk
import numpy as np
import time

ROBOT_ADDRESS = "localhost:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

np.set_printoptions(precision=3, suppress=True)


def cb(state: rby1_sdk.RobotState_A):
    print(state)


robot.start_state_update(cb, 10)  # (Hz)

robot.power_on(".*")
robot.servo_on(".*")
robot.disable_control_manager()
time.sleep(10000)
