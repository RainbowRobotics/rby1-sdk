import time

import rby1_sdk
import numpy as np

robot = rby1_sdk.create_robot_a("0.0.0.0:50051")
robot.connect()

recorded_position = []
recorded_velocity = []


def cb(rs):
    global recorded_position, recorded_velocity
    recorded_position = recorded_position + [rs.target_position[3]]
    recorded_velocity = recorded_velocity + [rs.target_position[11]]


robot.start_state_update(cb, 0.1)  # (Hz)

robot.power_on(".*")
robot.servo_on(".*")
robot.reset_fault_control_manager()
robot.enable_control_manager()

count = 0


def control(state: rby1_sdk.Robot_A_ControlState):
    global count

    print(f"current time: {time.time_ns() / 1_000_000_000}")

    print(f"time: {state.t}")
    print(f"position: {state.position}")

    i = rby1_sdk.Robot_A_ControlInput()

    i.target = state.position * 0.9
    i.feedback_gain.fill(10)
    i.feedforward_torque.fill(0)
    i.finish = count > 1000

    count = count + 1

    return i


print(robot.control(control))
