################################ Note ################################
# This example does not run in simulation.
######################################################################
# Get PID Gain Demo
# This example demonstrates how to get PID gains of the robot. See --help for arguments.
# 
# Usage example:
#     python 09_get_pid_gain.py --address 127.0.0.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import time
import argparse
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        logging.error("Robot connection failed")
        exit(1)

    if not robot.is_power_on(".*"):
        rv = robot.power_on(".*")
        if not rv:
            logging.error("Failed to power on the robot")
            exit(1)

    time.sleep(0.5)  # Waits for all actuators to be fully powered on

    logging.info(">>> Retrieving PID gains using component names")

    try:
        gain_list = robot.get_torso_position_pid_gains()
        for i, gain in enumerate(gain_list):
            logging.info(
                f"[torso_{i}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
            )
    except RuntimeError:
        logging.error("Failed to get torso PID gains")

    try:
        gain_list = robot.get_right_arm_position_pid_gains()
        for i, gain in enumerate(gain_list):
            logging.info(
                f"[right_arm_{i}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
            )
    except RuntimeError:
        logging.error("Failed to get right arm PID gains")

    try:
        gain_list = robot.get_left_arm_position_pid_gains()
        for i, gain in enumerate(gain_list):
            logging.info(
                f"[left_arm_{i}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
            )
    except RuntimeError:
        logging.error("Failed to get left arm PID gains")

    try:
        gain_list = robot.get_head_position_pid_gains()
        for i, gain in enumerate(gain_list):
            logging.info(
                f"[head_{i}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
            )
    except RuntimeError:
        logging.error("Failed to get head PID gains")

    logging.info(">>> Retrieving PID gains using joint names")

    for joint_name in ["torso_0", "right_arm_0", "left_arm_0", "head_0"]:
        try:
            gain = robot.get_position_pid_gain(joint_name)
            logging.info(
                f"[{joint_name}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
            )
        except RuntimeError:
            logging.error(f"Failed to get PID gain for joint '{joint_name}'")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="09_get_pid_gain")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
