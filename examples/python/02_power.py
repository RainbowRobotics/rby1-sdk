# Power Control Demo
# This example demonstrates how to connect to an RB-Y1 robot and power on devices
# matching a regex pattern. It shows a minimal flow: connect, check power state,
# and power on the device if it's off.
#
# Usage example:
#     python 02_power.py --address 192.168.30.1:50051 --model a --device '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk
import argparse


def main(address, model, power_device):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)

    # Power off
    # robot.power_off(power_device)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="02_power")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--device",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model, power_device=args.device)
