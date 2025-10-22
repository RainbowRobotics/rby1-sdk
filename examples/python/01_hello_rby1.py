# Hello RB-Y1 Demo
# This example demonstrates basic connection to RB-Y1 robot and retrieves robot information
# in different string representation formats.
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk
import argparse


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)

    # Model
    m = robot.model()

    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    robot_info = robot.get_robot_info()
    print(f"Hello, RB-Y1! (Robot model name: {m.model_name})")

    print("\n== Robot info (__str__ format):")
    print(str(robot_info))

    print("\n== Robot info (repr, single-line format):")
    with rby1_sdk.printoptions(multiline_repr=False):
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
