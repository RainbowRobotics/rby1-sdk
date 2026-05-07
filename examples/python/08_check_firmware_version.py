# Check Firmware Version Demo
# This example demonstrates how to check the motor info(name, brake, product_name, firmware version) of the robot. See --help for arguments.
#
# Usage example:
#     python 08_check_firmware_version.py --address 127.0.0.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse
import time


def main(address, power, model):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        print("Robot is not connected")
        exit(1)

    if not robot.is_power_on(power):
        rv = robot.power_on(power)
        if not rv:
            print("Failed to power on")
            exit(1)
    time.sleep(0.5)
    robot_info = robot.get_robot_info()
    for ji in robot_info.joint_infos:
        print(f"{ji.name}, {ji.has_brake}, {ji.product_name}, {ji.firmware_version}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="08_check_firmware_version")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, power=args.power, model=args.model)
