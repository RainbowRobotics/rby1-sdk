import rby1_sdk
import numpy as np
import time

ROBOT_ADDRESS = "localhost:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()


def cb(state):
    print("---")
    print(f"time [{state.timestamp}]")
    print(f"current position: {state.position}")
    print(f"center of mass: {state.center_of_mass}")
    print(f"# of collisions: {len(state.collisions)}")
    print(f" - [0] distance: {state.collisions[0].distance}")


robot.start_state_update(cb,
                         10  # (Hz)
                         )

robot.power_on(".*")
robot.servo_on(".*")
robot.disable_control_manager()
time.sleep(10000)
