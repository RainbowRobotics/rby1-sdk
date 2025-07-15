import rby1_sdk as rby
from helper import *
import numpy as np
import argparse
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)

    minimum_time = 2

    robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(np.zeros(7))
                    .set_minimum_time(minimum_time)
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(np.zeros(7))
                    .set_minimum_time(minimum_time)
                )
                .set_torso_command(
                    rby.JointGroupPositionCommandBuilder()
                    .set_joint_names(['torso_0', 'torso_1'])
                    .set_position(np.array([0, 0]))
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        1,
    ).get()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="26_joint_group_command")
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
        default=".*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        model=args.model,
        power=args.power,
        servo=args.servo,
    )
