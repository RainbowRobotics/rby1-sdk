import time
import os, sys
import rby1_sdk
from rby1_sdk import *
import argparse
import numpy as np
import sys


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()

    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)

    if not robot.is_power_on(".*"):
        rv = robot.power_on(".*")
        if not rv:
            print("Failed to power on")
            exit(1)

    """
    Get PID Gains
    """
    print(">>> Before")
    gain_list = robot.get_torso_position_pid_gains()
    [
        print(
            f"[torso_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    gain_list = robot.get_right_arm_position_pid_gains()
    [
        print(
            f"[right_arm_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    gain_list = robot.get_left_arm_position_pid_gains()
    [
        print(
            f"[left_arm_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    gain_list = robot.get_head_position_pid_gains()
    [
        print(
            f"[head_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    """
    Set PID Gains(default value)
    """
    # Torso joints
    robot.set_position_pid_gain("torso_0", 100, 20, 900)
    robot.set_position_pid_gain("torso_1", 1000, 38, 900)
    robot.set_position_pid_gain("torso_2", 1000, 38, 900)
    robot.set_position_pid_gain("torso_3", 220, 40, 400)
    robot.set_position_pid_gain("torso_4", 50, 20, 400)
    robot.set_position_pid_gain("torso_5", 220, 40, 400)

    # Right arm joints
    robot.set_position_pid_gain("right_arm_0", 80, 15, 200)
    robot.set_position_pid_gain("right_arm_1", 80, 15, 200)
    robot.set_position_pid_gain("right_arm_2", 80, 15, 200)
    robot.set_position_pid_gain("right_arm_3", 35, 5, 80)
    robot.set_position_pid_gain("right_arm_4", 30, 5, 70)
    robot.set_position_pid_gain("right_arm_5", 30, 5, 70)
    robot.set_position_pid_gain("right_arm_6", 100, 5, 120)

    # Left arm joints
    robot.set_position_pid_gain("left_arm_0", 80, 15, 200)
    robot.set_position_pid_gain("left_arm_1", 80, 15, 200)
    robot.set_position_pid_gain("left_arm_2", 80, 15, 200)
    robot.set_position_pid_gain("left_arm_3", 35, 5, 80)
    robot.set_position_pid_gain("left_arm_4", 30, 5, 70)
    robot.set_position_pid_gain("left_arm_5", 30, 5, 70)
    robot.set_position_pid_gain("left_arm_6", 100, 5, 150)

    # Head joints
    robot.set_position_pid_gain("head_0", 800, 0, 4000)
    robot.set_position_pid_gain("head_1", 800, 0, 4000)

    # Ensure PID Gain update compleation
    time.sleep(0.05)

    print("\n\n>>> After")
    gain_list = robot.get_torso_position_pid_gains()
    [
        print(
            f"[torso_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    gain_list = robot.get_right_arm_position_pid_gains()
    [
        print(
            f"[right_arm_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    gain_list = robot.get_left_arm_position_pid_gains()
    [
        print(
            f"[left_arm_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]

    gain_list = robot.get_head_position_pid_gains()
    [
        print(
            f"[head_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
        )
        for i, gain in enumerate(gain_list)
    ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="12_fatory_default_pid_gain")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
