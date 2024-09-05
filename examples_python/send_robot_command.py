import time

import rby1_sdk
import numpy as np

robot = rby1_sdk.create_robot_a("0.0.0.0:50051")
robot.connect()

recorded_position = []
recorded_velocity = []


def cb(rs):
    global recorded_position, recorded_velocity
    recorded_position = recorded_position + [rs.position[3]]
    recorded_velocity = recorded_velocity + [rs.velocity[3]]


robot.start_state_update(cb,
                         100  # (Hz)
                         )

robot.set_parameter("joint_position_command.cutoff_frequency", "5")
robot.set_parameter("default.acceleration_limit_scaling", "0.8")

robot.power_on(".*")
robot.servo_on(".*")
robot.reset_fault_control_manager()
robot.enable_control_manager()

target_position = [0, 0.5, 0, 0, 0, 0] + [0, 0, 0, -1.2, 0, 0, 0] + [0, 0, 0, -1.2, 0, 0, 0]
rc = rby1_sdk.RobotCommandBuilder().set_command(
    rby1_sdk.ComponentBasedCommandBuilder()
    .set_body_command(rby1_sdk.JointPositionCommandBuilder()
                      .set_minimum_time(5)
                      .set_position(target_position)
    )
)
rv = robot.send_command(rc, 10).get()

time.sleep(0.5)

zero_position = np.array([0] * 20)
rc = rby1_sdk.RobotCommandBuilder().set_command(
    rby1_sdk.ComponentBasedCommandBuilder()
    .set_body_command(
        rby1_sdk.BodyCommandBuilder(rby1_sdk.JointPositionCommandBuilder()
                                    .set_minimum_time(5)
                                    .set_position(zero_position))
    )
)
robot.send_command(rc, 10).get()

robot.stop_state_update()

print(recorded_position)
print(recorded_velocity)
