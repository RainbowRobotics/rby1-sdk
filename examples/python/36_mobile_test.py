################### CAUTION ###################
# CAUTION:
# Mobile base commands move the robot in the surrounding workspace.
# Ensure that the floor area is clear before running this example.
###############################################

# Mobile Command Demo
# This example brings up the robot and runs several mobility commands using joint velocity and SE(2) velocity control. See --help for arguments.
#
# Usage example:
#     python 36_mobile_test.py --address 192.168.30.1:50051 --model m --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import importlib
import rby1_sdk as rby
import numpy as np
import sys
import argparse

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

# Note: On the A model, the Y component of SE(2) commands is ignored.


D2R = np.pi / 180  # Degree to Radian conversion factor
MINIMUM_TIME = 2.5



def example_ready_command(robot):
    print("example_ready_command")

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 45 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 45 * D2R, 0]

    if not movej(robot, torso=q_joint_waist, right_arm=q_joint_right_arm, left_arm=q_joint_left_arm, minimum_time=7.0):
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_x_backward_command(robot):
    print("example_SE2_x_backward_command")
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([-0.1, 0], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_x_forward_command(robot):
    print("example_SE2_x_forward_command")

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([0.1, 0], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_y_backward_command(robot):
    print("example_SE2_y_backward_command")
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([0, -0.1], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1
    return 0

def example_SE2_y_forward_command(robot):
    print("example_SE2_y_forward_command")

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([0, 0.1], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1
    return 0

def example_SE2_turn_left_command(robot):
    print("example_SE2_turn_left_command")

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([0, 0.0], -0.1)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1
    return 0

def example_SE2_turn_right_command(robot):
    print("example_SE2_turn_right_command")

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([0, 0.0], 0.1)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )
    rv = robot.send_command(rc, 10).get()
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1
    return 0

def main(address, model, power, servo):
    print("Attempting to connect to the robot...")
    robot = initialize_robot(address, model, power, servo)
    print("Control Manager enabled successfully.")

    
    example_ready_command(robot)

    if not example_SE2_x_forward_command(robot):
        print("finish motion")
    if not example_SE2_x_backward_command(robot):
        print("finish motion")
    # On the A model, the Y component of SE(2) commands is ignored.
    if not example_SE2_y_forward_command(robot):
        print("finish motion")
    if not example_SE2_y_backward_command(robot):
        print("finish motion")
    if not example_SE2_turn_right_command(robot):
        print("finish motion")
    if not example_SE2_turn_left_command(robot):
        print("finish motion")
    
    print("end of demo")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="36_mobile_test")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    parser.add_argument(
        "--servo",
        type=str,
        default=".*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        model=args.model,
        power=args.power,
        servo=args.servo,
    )
