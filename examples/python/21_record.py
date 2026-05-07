# Record Demo
# This example demonstrates how to record the robot's positions. See --help for arguments.
#
# Usage example:
#     python 21_record.py --address 127.0.0.1:50051
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import numpy as np
import time
import sys
import argparse
import signal

recorded_traj = []  # list for recording
recording = False  # recording state


def cb(state):
    global recorded_traj, recording
    if recording:
        print("---")
        print(f"target position: {state.target_position}")

        current_state = state.position.copy()
        recorded_traj.append(current_state)


def pre_processing(address, model):
    robot = rby.create_robot(address, model)
    robot.connect()

    if not robot.is_power_on(".*"):
        print("Power is currently OFF. Attempting to power on...")
        if not robot.power_on(".*"):
            print("Error: Failed to power on the robot.")
            sys.exit(1)
        print("Robot powered on successfully.")
    else:
        print("Power is already ON.")

    return robot


def start_recording():
    global recording, recorded_traj
    recorded_traj = []  # initialize list when recording starts
    recording = True
    print("Recording started...")


def stop_recording():
    global recording
    recording = False
    np.savez_compressed("recorded.npz", data=recorded_traj)
    print("Recording stopped and data saved to 'recorded.npz'.")


def handle_exit(sig, frame):
    """handler called when the program exits"""
    stop_recording()
    sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="21_record")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--model", type=str, required=True, help="Robot model")
    args = parser.parse_args()

    # register handler called when the program exits
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    robot = pre_processing(args.address, args.model)

    robot.start_state_update(cb, 10)

    start_recording()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        handle_exit(None, None)  # save file when Ctrl+C is pressed
