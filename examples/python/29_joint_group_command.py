################################ Note ################################
# This example does not run in simulation.
# Joint Group Command is available only on torso.
######################################################################
# Joint Group Command Demo
# This example demonstrates how to control the robot using joint group command. See --help for arguments.
#
# Usage example:
#     python 29_joint_group_command.py --address 127.0.0.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import numpy as np
import logging
import argparse
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

def move_to_pre_control_pose(robot):
    """ Move to Pre Control Position Before Starting the Motion """
    torso = np.array([0.0, 0.1, -0.2, 0.1, 0.0, 0.0])
    right_arm = np.array([0.2, -0.2, 0.0, -1.0, 0, 0.7, 0.0])
    left_arm = np.array([0.2, 0.2, 0.0, -1.0, 0, 0.7, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)

    move_to_pre_control_pose(robot)
    
    minimum_time = 2

    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.zeros(7))
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.zeros(7))
                )
                .set_torso_command(
                    rby.JointGroupPositionCommandBuilder()
                    .set_joint_names(["torso_1", "torso_2", "torso_3"])
                    .set_position(np.array([0.3, -0.6, 0.3]))
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        1,
    ).get()
    print(f"joint group command finish_code: {rv.finish_code}")
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        exit(1)

    print(f"joint group command finish_code: {rv.finish_code}")
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="29_joint_group_command")
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
