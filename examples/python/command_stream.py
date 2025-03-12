import time

import rby1_sdk
import numpy as np
import math

robot = rby1_sdk.create_robot_a("0.0.0.0:50051")
robot.connect()


def cb(state):
    print("---")
    print(f"time [{state.timestamp}]")
    print(f"current position: {state.position}")
    print(f"center of mass: {state.center_of_mass}")


# robot.start_state_update(cb,
#                          10  # (Hz)
#                          )

print(robot.set_parameter("joint_position_command.cutoff_frequency", "5"))
print(robot.set_parameter("default.acceleration_limit_scaling", "0.8"))

robot.power_on(".*")
robot.servo_on(".*")
robot.reset_fault_control_manager()
robot.enable_control_manager()

stream = robot.create_command_stream(10)

dt = 0.001
for t in range(0, 10000):
    rc = rby1_sdk.RobotCommandBuilder().set_command(
        rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
            rby1_sdk.JointPositionCommandBuilder()
            .set_command_header(
                rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1)
            )
            .set_minimum_time(dt)
            .set_position(
                [0] * 6
                + [0] * 7
                + [0] * 6
                + [math.pi / 2.0 * math.sin(math.pi * 2 * t * dt / 5)]
            )
        )
    )
    rv = stream.send_command(rc)

    time.sleep(dt * 0.5)
