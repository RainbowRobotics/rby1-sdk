import time
import os, sys
import rby1_sdk
from rby1_sdk import *
import argparse
import numpy as np
import sys

def main(address):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()
    
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
        
    if not robot.is_power_on(".*"):
        rv = robot.power_on(".*")
        if not rv:
            print("Failed to power on")
            exit(1)
    
    print(">>> Using Component Name")
    gain_list = robot.get_torso_position_pid_gains()
    [print(f"[torso_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}") for i, gain in enumerate(gain_list) ]
    
    gain_list = robot.get_right_arm_position_pid_gains()
    [print(f"[right_arm_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}") for i, gain in enumerate(gain_list) ]
    
    gain_list = robot.get_left_arm_position_pid_gains()
    [print(f"[left_arm_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}") for i, gain in enumerate(gain_list) ]
    
    gain_list = robot.get_head_position_pid_gains()
    [print(f"[head_{i}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}") for i, gain in enumerate(gain_list) ]
    
    print("\n\n>>> Using Joint Name")
    
    joint_name = "torso_0"
    gain = robot.get_position_pid_gain(joint_name)
    print(f"[{joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}")
    
    joint_name = "right_arm_0"
    gain = robot.get_position_pid_gain(joint_name)
    print(f"[{joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}")
    
    joint_name = "left_arm_0"
    gain = robot.get_position_pid_gain(joint_name)
    print(f"[{joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}")
    
    joint_name = "head_0"
    gain = robot.get_position_pid_gain(joint_name)
    print(f"[{joint_name}] p gain: {gain.p_gain}, i gain: {gain.i_gain}, d gain: {gain.d_gain}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="10_get_pid_gain")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
