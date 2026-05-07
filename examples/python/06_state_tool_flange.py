# Tool Flange State Example
# This example connects to the robot, powers on the 48V tool flange supply if needed,
# subscribes to robot state updates, and prints the left and right tool flange states. See --help for arguments.
#
# Usage example:
#   python 06_state_tool_flange.py --address 192.168.30.1:50051 --model a
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


def callback(rs):
    print("---")
    print(f"tool_flange_right: {rs.tool_flange_right}")
    print(f"tool_flange_left: {rs.tool_flange_left}")


def main(address, model):
    robot = rby.create_robot(address, model)
    
    if not robot.connect():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on(".*"):
        if not robot.power_on(".*"):
            print("Failed to power on")
            exit(1)

    robot.start_state_update(callback, rate=10)  # Hz
    try:
        time.sleep(100)
    except KeyboardInterrupt:
        print("Stopping state update...")
    finally:
        robot.stop_state_update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="06_state_tool_flange")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
