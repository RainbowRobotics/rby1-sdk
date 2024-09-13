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
    print(f"joint 0 - time_since_last_update: {state.joint_states[0].time_since_last_update}")


robot.start_state_update(cb,
                         10  # (Hz)
                         )

time.sleep(1)

robot.start_time_sync(1)

time.sleep(2)

robot.stop_time_sync()

robot.stop_state_update()