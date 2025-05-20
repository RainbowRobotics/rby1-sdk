import time

import rby1_sdk as rby
from helper import *
import argparse
import numpy as np
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)

    logging.info(
        "Robot was successfully brought up. The control manager will be disabled and the servo will be turned off in "
        "5 second.")
    time.sleep(5)

    robot.disable_control_manager()
    robot.servo_off(servo)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="21_servo_off")
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
        "--servo",
        type=str,
        default="^(head).*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        model=args.model,
        power=args.power,
        servo=args.servo,
    )
