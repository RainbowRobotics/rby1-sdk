import time

import rby1_sdk
import numpy as np

robot = rby1_sdk.create_robot_a("localhost:50051")
robot.connect()


def cb(state: rby1_sdk.RobotState_A):
    print(state)


robot.start_state_update(cb, 10)  # (Hz)

robot.power_on(".*")
robot.servo_on(".*")
robot.reset_fault_control_manager()
robot.enable_control_manager()

target_position = (
    [0, 0.5, 0, 0, 0, 0] + [0, 0, 0, -1.2, 0, 0, 0] + [0, 0, 0, -1.2, 0, 0, 0]
)
rc = rby1_sdk.RobotCommandBuilder().set_command(
    rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
        rby1_sdk.JointPositionCommandBuilder()
        .set_command_header(rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1))
        .set_minimum_time(5)
        .set_position(target_position)
    )
)
rv = robot.send_command(rc, 10).get()

time.sleep(0.5)

zero_position = np.array([0] * 20)
rc = rby1_sdk.RobotCommandBuilder().set_command(
    rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
        rby1_sdk.BodyCommandBuilder(
            rby1_sdk.JointPositionCommandBuilder()
            .set_minimum_time(5)
            .set_position(zero_position)
        )
    )
)
robot.send_command(rc, 10).get()

robot.stop_state_update()
