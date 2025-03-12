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
    Right Arm 3, Set Only P Gain
    """
    target_joint_name = "right_arm_3"
    print(">>> Before")
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    # Set P Gain
    robot.set_position_p_gain(target_joint_name, 100)

    print(">>> After")
    # Ensure PID Gain update compleation
    time.sleep(0.05)
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    """
    Left Arm 3,  Set all PID gains
    """
    target_joint_name = "left_arm_3"
    print(">>> Before")
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    # Set all PID gains (P, I, D) for the target joint using individual gain values
    robot.set_position_pid_gain(target_joint_name, 60, 10, 100)

    print(">>> After")
    # Ensure PID Gain update compleation
    time.sleep(0.05)
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    """
    Head 0, PIDGain
    """
    target_joint_name = "head_0"
    print(">>> Before")
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    # Set all PID gains (P, I, D) for the target joint using a predefined PIDGain object
    target_gain = PIDGain(700, 0, 3500)
    robot.set_position_pid_gain(target_joint_name, target_gain)

    print(">>> After")
    # Ensure PID Gain update compleation
    time.sleep(0.05)
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    """
    Head 1, PIDGain
    """
    target_joint_name = "head_1"
    print(">>> Before")
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )

    # Set all PID gains (P, I, D) for the target joint using a predefined PIDGain object
    robot.set_position_p_gain(target_joint_name, 300)

    print(">>> After")
    # Ensure PID Gain update compleation
    time.sleep(0.05)
    gain = robot.get_position_pid_gain(target_joint_name)
    print(
        f"[{target_joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="11_set_pid_gain")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
