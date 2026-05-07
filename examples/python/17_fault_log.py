# Fault Log Example
#
# This example connects to the robot, prints the available fault log files, and downloads the first fault
# log as ``fault.csv`` when one is available. See --help for arguments.
#
# Usage example:
#     python 17_fault_log.py --address 192.168.30.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse


def main(address, model):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        print("Error: Robot connection failed.")
        exit(1)

    fault_log_list = robot.get_fault_log_list()
    print(f"Fault log list: {fault_log_list}")

    if len(fault_log_list) == 0:
        print("No fault logs found.")
        return

    print(f"Downloading fault log: {fault_log_list[0]}")
    with open("fault.csv", "wb") as f:
        robot.download_file(fault_log_list[0], f)
    print("Fault log downloaded successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="17_fault_log")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
