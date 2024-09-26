import rby1_sdk
import numpy as np
import time

ROBOT_ADDRESS = "192.168.30.1:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()


def cb(state):
    print("---")
    print(f"time [{state.timestamp}]")
    print(f"current position: {state.position}")
    print(f"center of mass: {state.center_of_mass}")


robot.start_state_update(cb,
                         10  # (Hz)
                         )

robot.power_on(".*")
time.sleep(10)
