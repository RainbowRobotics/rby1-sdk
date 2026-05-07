# Power Control Demo
# This example demonstrates how to power on devices. See --help for arguments.
#
# Usage example: 
#     python 07_power.py --address 192.168.30.1:50051 --model a --device '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse


def main(address, model, power_device):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        print("Robot is not connected")
        exit(1)
    # Power on the device if it's off.
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)
    # Power off
    # robot.power_off(power_device)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="07_power")
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
