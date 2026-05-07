# Hello RB-Y1 Demo
# This example demonstrates basic connection to RB-Y1 robot and retrieves robot information
# in different string representation formats. See --help for arguments.
#
# Usage example:
#     python 01_hello_rby1.py --address 192.168.30.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import rby1_sdk as rby

import rby1_sdk as rby


def main(address, model):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        print("Failed to connect to the robot.")
        exit(1)

    model_info = robot.model()
    if len(model_info.mobility_idx) == 2 and model == "m":
        print("wrong model argument. this robot model is a")
        exit(1)
    elif len(model_info.mobility_idx) == 4 and model == "a":
        print("wrong model argument. this robot model is m")
        exit(1)

    robot_info = robot.get_robot_info()
    print(f"Hello, RB-Y1! (Robot model name: {model_info.model_name})")

    print("\n== Robot info (__str__ format):")
    print(robot_info)

    print("\n== Robot info (repr, single-line format):")
    with rby.printoptions(multiline_repr=False):
        print(repr(robot_info))

    print("\n=== Robot info (repr, multi-line format):")
    print(repr(robot_info))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="01_hello_rby1")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
