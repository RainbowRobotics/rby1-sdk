# Multi-Controls Example
# This example initializes the robot, sends overlapping body and arm joint position
# commands with different priorities, and prints each command handle's finish code. See --help for arguments.
#
# Usage example:
#   python 30_multi_controls.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

import rby1_sdk as rby
import numpy as np
import time
import argparse
import logging
from dataclasses import dataclass

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@dataclass
class Pose:
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


TARGET_POSE = Pose(
    right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
)

def move_to_pre_control_pose(robot):
    """Move to the pre-control pose before starting the motion."""
    torso = np.array([0.0, 0.1, -0.2, 0.1, 0.0, 0.0])
    right_arm = np.array([0.2, -0.2, 0.0, -1.0, 0, 0.7, 0.0])
    left_arm = np.array([0.2, 0.2, 0.0, -1.0, 0, 0.7, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)
    
    robot_model = robot.model()
    move_to_pre_control_pose(robot)

    minimum_time = 2
    robot_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_position(np.zeros(len(robot_model.body_idx)))
                .set_minimum_time(minimum_time)
            )
        ),
        priority=1,
    )
    time.sleep(1)

    right_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(TARGET_POSE.right_arm)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        priority=10,
    )
    time.sleep(1)

    left_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(TARGET_POSE.left_arm)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        priority=1,
    )

    print(f"{robot_handle.get().finish_code = }")
    print(f"{right_handle.get().finish_code = }")
    print(f"{left_handle.get().finish_code = }")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="30_multi_controls")
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
