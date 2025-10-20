# Stop Command Demo
# This example is part of the RB-Y1 SDK examples. See --help for arguments.
#
# Usage example:
#     python 06_stop_command.py --help
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk
import time
import argparse


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    robot.cancel_control()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="06_stop_command")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
