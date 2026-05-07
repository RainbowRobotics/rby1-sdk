# Set System Time Example
#
# This example includes the following features:
# 1. Reading the robot's current system time and timezone.
# 2. Setting the robot system time with an explicit timezone string.
# 3. Setting the robot system time using a timezone-aware datetime object. See --help for arguments.
# Note: This example is not supported in simulation.
#
# Usage example:
#     python 13_set_system_time.py --address 192.168.30.1:50051 --model a
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
from datetime import datetime
from zoneinfo import ZoneInfo

def main(address, model):
    robot = rby.create_robot(address, model)
    
    if not robot.connect():
        print("Failed to connect robot")
        exit(1)
    
    dt, local_time_string = robot.get_system_time()
    print(f"Robot System Time: {dt}, {local_time_string}")

    print("# Change To TimeZone(EST)")
    print(
        f" -- {'SUCCESS' if robot.set_system_time(datetime.now(), 'EST') else 'FAIL'}"
    )
    time.sleep(0.5)
    print(f"Robot System Time: {robot.get_system_time()}")

    print("# Change to TimeZone")
    dt = dt.astimezone(ZoneInfo("Asia/Seoul"))
    print(f" -- {'SUCCESS' if robot.set_system_time(dt) else 'FAIL'}")
    time.sleep(0.5)  # Need for changing timezone
    print(f"Robot System Time: {robot.get_system_time()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="13_set_system_time")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
