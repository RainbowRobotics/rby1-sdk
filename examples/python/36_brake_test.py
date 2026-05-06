################### CAUTION ###################
# CAUTION:
# Start from a safe posture before running this example.
# Do not use torso joints or ".*" with this example.
# Releasing a brake can cause the target joint to move under gravity.
###############################################

# Brake Test Example
# This example connects to an RB-Y1 robot, powers on the specified devices if needed,
# releases the brake for a target joint, waits briefly, and then engages the brake again. See --help for arguments.
#
# Usage example:
#     python 35_brake_test.py --address 192.168.30.1:50051 --model a --power '.*' --joint right_arm_0
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import logging
import time

import rby1_sdk as rby

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

def main(address, model, power, joint):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)

    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)

    robot.disable_control_manager()
    time.sleep(0.5)


    logging.info(f"Brake release requested for {joint}")
    if not robot.brake_release(joint):
        logging.error(f"Failed to release brake for {joint}")
        exit(1)

    time.sleep(0.5)

    logging.info(f"Brake engage requested for {joint}")
    if not robot.brake_engage(joint):
        logging.error(f"Failed to engage brake for {joint}")
        exit(1)

    logging.info("Brake test finished successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="35_brake_test")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    parser.add_argument(
        "--joint", type=str, required=True, help="Joint name regex pattern"
    )

    args = parser.parse_args()

    if "torso" in args.joint or args.joint == ".*":
        logging.error(
            f"Refusing to run brake_test with joint pattern {args.joint}. "
            "Use a single non-torso joint instead."
        )
        exit(1)

    main(
        address=args.address,
        model=args.model,
        power=args.power,
        joint=args.joint,
    )
