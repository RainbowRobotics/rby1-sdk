import rby1_sdk
import numpy as np
import sys
from time import sleep
import time

from rby1_sdk import *

ROBOT_ADDRESS = "192.168.30.1:50051"

print("Attempting to connect to the robot...")

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)

if not robot.connect():
    print("Error: Unable to establish connection to the robot at")
    sys.exit(1)

print("Successfully connected to the robot")

print("Starting state update...")


def cb(rs):
    print(f"Timestamp: {rs.timestamp - rs.ft_sensor_right.time_since_last_update}")
    position = rs.position * 180 / 3.141592
    print(f"waist [deg]: {position[2:2 + 6]}")
    print(f"right arm [deg]: {position[8:8 + 7]}")
    print(f"left arm [deg]: {position[15:15 + 8]}")


robot.start_state_update(cb, 0.1)

print("Checking power status...")
if not robot.is_power_on(".*"):
    print("Power is currently OFF. Attempting to power on...")
    if not robot.power_on(".*"):
        print("Error: Failed to power on the robot.")
        sys.exit(1)
    print("Robot powered on successfully.")
else:
    print("Power is already ON.")

print("Checking servo status...")
if not robot.is_servo_on(".*"):
    print("Servo is currently OFF. Attempting to activate servo...")
    if not robot.servo_on(".*"):
        print("Error: Failed to activate servo.")
        sys.exit(1)
    print("Servo activated successfully.")
else:
    print("Servo is already ON.")

control_manager_state = robot.get_control_manager_state()

if (
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault or control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault):

    if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
        print("Warning: Detected a Major Fault in the Control Manager.")
    else:
        print("Warning: Detected a Minor Fault in the Control Manager.")

    print("Attempting to reset the fault...")
    if not robot.reset_fault_control_manager():
        print("Error: Unable to reset the fault in the Control Manager.")
        sys.exit(1)
    print("Fault reset successfully.")

print("Control Manager state is normal. No faults detected.")

print("Enabling the Control Manager...")
if not robot.enable_control_manager():
    print("Error: Failed to enable the Control Manager.")
    sys.exit(1)
print("Control Manager enabled successfully.")

robot.set_parameter("default.acceleration_limit_scaling", "0.8")
robot.set_parameter("joint_position_command.cutoff_frequency", "5")
robot.set_parameter("cartesian_command.cutoff_frequency", "5")
robot.set_parameter("default.linear_acceleration_limit", "5")

print("parameters setting is done")

minimum_time = 5.0
D2R = np.pi / 180  # Degrees to radians conversion factor


def example_joint_position_command_1(robot):
    print("joint position command example 1")

    # Initialize joint positions
    q_joint_waist = np.deg2rad(np.array([0, 60, -120, 60, 0, 0]))
    q_joint_right_arm = np.deg2rad(np.array([-45, -30, 0, -45, 0, -90, 90]))
    q_joint_left_arm = np.deg2rad(np.array([-45, 30, 0, -45, 0, -90, -90]))

    print(f"{q_joint_waist=}")
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(
                JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(q_joint_waist)
            )
            .set_right_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(q_joint_right_arm)
            )
            .set_left_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(q_joint_left_arm)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

def example_joint_position_command_2(robot):
    print("joint position command example 1")

    # Initialize joint positions
    q_joint_waist = np.deg2rad(np.array([0, 25, -50, 25, 0, 0]))
    q_joint_right_arm = np.deg2rad(np.array([-45, -30, 0, -45, 0, -90, 90]))
    q_joint_left_arm = np.deg2rad(np.array([-45, 30, 0, -45, 0, -90, -90]))

    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(
                JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(q_joint_waist)
            )
            .set_right_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(q_joint_right_arm)
            )
            .set_left_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(q_joint_left_arm)
            )
        )
    )
    rv = robot.send_command(rc, 10).get()

    if rv.fsleepinish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0





if not example_joint_position_command_1(robot):
    print("finish motion")

sleep(7.0)
if not example_joint_position_command_2(robot):
    print("finish motion")

print("end of demo")
