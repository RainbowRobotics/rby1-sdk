# Collisions Demo
# This example connects to an RB-Y1 robot, powers on the specified devices,
# monitors collision distance during motion, and sends a stop command when the robot gets too close. See --help for arguments.
#
# Usage example:
#     python 26_collisions.py --address 192.168.30.1:50051 --model a --power ".*"
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import logging
import threading
import time

import numpy as np
import rby1_sdk as rby
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

STOP_DISTANCE = 0.03
RIGHT_ARM_TARGET = np.array(
    [0.8, -0.8, 0.7, -2.0, 0, 0.0, 0]
    
)
stop_requested = False



def callback(robot_state):
    global stop_requested
    if not robot_state.collisions:
        return

    nearest = min(robot_state.collisions, key=lambda collision: collision.distance)
    print(
        f"nearest collision distance: {nearest.distance:.4f} | "
        f"{nearest.link1} <-> {nearest.link2}"
    )

    if nearest.distance < STOP_DISTANCE:
        print(f"Collision Detected")
        stop_requested = True

def move_to_pre_control_pose(robot):
    """Move to the pre-control pose before starting the motion."""
    torso = np.array([0.0, 0.1, -0.2, 0.1, 0.0, 0.0])
    right_arm = np.array([0.2, -0.2, 0.0, -1.0, 0, 0.7, 0.0])
    left_arm = np.array([0.2, 0.2, 0.0, -1.0, 0, 0.7, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)

def main(address, model, power, servo):
    global stop_requested
    robot = initialize_robot(address, model, power, servo)

    move_to_pre_control_pose(robot)

    stop_requested = False
    robot.start_state_update(callback, rate=50)

    rv = movej(robot, right_arm=RIGHT_ARM_TARGET, minimum_time=5.0)

    while not stop_requested:
        time.sleep(0.01)

    
    robot.cancel_control()
    robot.stop_state_update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="26_collisions")
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

    main(address=args.address, model=args.model, power=args.power, servo=args.servo)
