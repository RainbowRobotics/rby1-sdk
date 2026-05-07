# Log Stream Demo
# This example demonstrates how to get logs from the robot. See --help for arguments.
#
# Usage example:
#     python 16_log_stream.py --address 127.0.0.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import time
import argparse


def main(address, model):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        print("Robot is not connected")
        exit(1)
        
    def cb(logs):
        for log in logs:
            if int(log.level) >= int(rby.Log.Level.Info):
                print(f"{log}")
        
    robot.sync_time()
    robot.start_log_stream(cb, 1)
    time.sleep(60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="16_log_stream")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
