################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Command Stream Demo
# This example brings up the robot and streams body joint position commands while the last body joint follows a sinusoidal target. See --help for arguments.
#
# Usage example:
#     python 31_command_stream.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import logging
import math
import signal
import rby1_sdk as rby
import numpy as np
import time
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

def move_to_zero_pose(robot):
    """ Move to Zero Position Before Starting the Motion """
    torso = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    right_arm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    left_arm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)

    logging.info("===== Command Stream Example =====")

    logging.info(robot.set_parameter("joint_position_command.cutoff_frequency", "5"))
    logging.info(robot.set_parameter("default.acceleration_limit_scaling", "0.8"))

    
    move_to_zero_pose(robot)
    stream = robot.create_command_stream(10)

    def handler(signum, frame):
        stream.cancel()
        exit(1)

    signal.signal(signal.SIGINT, handler)

    dt = 0.001
    for t in range(0, 10000):
        q = [0.0] * 6 + [0.0] * 7 + [0.0] * 4 + [math.pi / 4.0 * math.sin(math.pi * 2 * t * dt / 5)] + [0.0] * 2
        rc = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1)
                )
                .set_minimum_time(dt)
                .set_position(q)
            )
        )
        stream.send_command(rc)
        time.sleep(dt * 0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="31_command_stream")
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
