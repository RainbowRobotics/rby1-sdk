# LED Demo
# This example demonstrates how to control the LED of the robot. See --help for arguments.
#
# Usage example:
#     python 12_led.py --address 127.0.0.1:50051 --model a
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


def main(address, model):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        print("Failed to connect robot")
        exit(1)

    print("#1 Red 0.5s")
    robot.set_led_color(
        rby.Color(255, 0, 0), duration=0.5, transition_time=0.1, blinking=False
    )
    time.sleep(0.5)

    print("#2 Green 0.5s")
    robot.set_led_color(
        rby.Color(0, 255, 0), duration=0.5, transition_time=0.1, blinking=False
    )
    time.sleep(0.5)

    print("#3 Blue 0.5s")
    robot.set_led_color(
        rby.Color(0, 0, 255), duration=0.5, transition_time=0.1, blinking=False
    )
    time.sleep(0.5)

    print("#4 White Blinking 1s")
    robot.set_led_color(
        rby.Color(200, 200, 200),
        duration=1,
        transition_time=0.1,
        blinking=True,
        blinking_freq=4,
    )
    time.sleep(1)

    # Rainbow colors
    rainbow_colors = [
        rby.Color(255, 0, 0),  # Red
        rby.Color(255, 127, 0),  # Orange
        rby.Color(255, 255, 0),  # Yellow
        rby.Color(0, 255, 0),  # Green
        rby.Color(0, 0, 255),  # Blue
        rby.Color(75, 0, 130),  # Indigo
        rby.Color(148, 0, 211),  # Violet
    ]

    print("#5 Rainbow")
    for color in rainbow_colors:
        robot.set_led_color(color, duration=0.5, transition_time=0.2, blinking=False)
        time.sleep(0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="12_led")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)