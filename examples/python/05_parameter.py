# Parameter Demo
# This example demonstrates how to get & set the robot's parameter. See --help for arguments.
#
# Usage example:
#     python 05_parameter.py --address 127.0.0.1:50051
#
# Scenario
# 1. Get parameter list
# 2. Get parameter value
# 3. Reset parameter to default value
# 4. Get parameter value
# 5. Set parameter value
# 6. Get parameter value
# 7. Set invalid parameter value
# 8. Get parameter value
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse

PARAMETER_NAME = "joint_position_command.cutoff_frequency"
def main(address):
    robot = rby.create_robot_a(address)

    robot.connect()

    parameter_list = robot.get_parameter_list()
    for parameter in parameter_list:
        print(parameter)
    print("------------------")

    print(f"Current parameter value")
    print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
    print("------------------")

    # Reset
    robot.reset_parameter_to_default(PARAMETER_NAME)

    # Get
    print(f"Parameter value after reset")
    print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
    print("------------------")

    ###

    # Set
    rv = robot.set_parameter(PARAMETER_NAME, "1")
    print(f"Parameter value after set valid value")
    print(f"Set parameter result: {rv}")

    # Get
    print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
    print("------------------")

    ###

    # Set invalid value
    rv = robot.set_parameter(PARAMETER_NAME, "1000")
    print(f"Parameter value after set invalid value")
    print(f"Set parameter result: {rv}")

    # Get
    print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
    print("------------------")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="04_parameter")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)