# Log Example
# This example connects to the robot, retrieves the most recent log entries,
# and prints them to the console. See --help for arguments.
#
# Usage example:
# python 15_log.py --address 192.168.30.1:50051 --model a --num-entries 10
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse

def main(address, model, num_entries):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        print("Robot is not connected")
        exit(1)

    logs = robot.get_last_log(num_entries)
    for log in logs:
        print(f"{log}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="15_log")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "-n",
        "--num-entries",
        type=int,
        default=10,
        help="Number of log entries to retrieve (default: 10)",
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model, num_entries=args.num_entries)
