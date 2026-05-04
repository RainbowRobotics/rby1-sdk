################################ Note ################################
# This example does not run in simulation.
# This example does not apply when the control manager is enabled.
######################################################################
# Battery Configuration Demo
# This example demonstrates how to get the serial_device in RPC.
# 
# Usage example:
#     python 37_rpc_serial_device.py --address 192.168.30.1:50051 
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse


def main(address):
    robot = rby.create_robot_a(address)
    robot.connect()

    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)

    serial_devices = robot.get_serial_device_list()
    for [i, device] in enumerate(serial_devices):
        print(f"[{i}] {device}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="37_rpc_serial_device")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
