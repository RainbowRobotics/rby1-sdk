import rby1_sdk as rby
from helper import *
import argparse
import numpy as np
import math
import sys
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

BODY_LINK_NAME = {"A": "link_torso_5", "T5": "link_torso_4", "M": "link_torso_5"}

CARTESIAN_COMMAND_PARAMETER = {
    "minimum_time": 5,
    "linear_velocity_limit": 1.5,
    "angular_velocity_limit": np.pi * 1.5,
    "acceleration_limit_scaling": 1.0,
    "stop_position_tracking_error": 1e-5,
    "stop_orientation_tracking_error": 1e-5,
}


def example_cartesian_command(robot):
    logging.info("===== Cartesian Command Example =====")

    # Initialize transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    # Define transformation matrices
    angle = np.pi / 6
    T_torso[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_torso[:3, 3] = [0.1, 0, 1.2]

    angle = -np.pi / 4
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.4, -0.4, 0]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.4, 0.4, 0]

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.CartesianCommandBuilder()
            .add_target(
                "base",
                BODY_LINK_NAME[robot.model().model_name],
                T_torso,
                CARTESIAN_COMMAND_PARAMETER["linear_velocity_limit"],
                CARTESIAN_COMMAND_PARAMETER["angular_velocity_limit"],
                CARTESIAN_COMMAND_PARAMETER["acceleration_limit_scaling"],
            )
            .add_target(
                BODY_LINK_NAME[robot.model().model_name],
                "ee_right",
                T_right,
                CARTESIAN_COMMAND_PARAMETER["linear_velocity_limit"],
                CARTESIAN_COMMAND_PARAMETER["angular_velocity_limit"],
                CARTESIAN_COMMAND_PARAMETER["acceleration_limit_scaling"],
            )
            .add_target(
                BODY_LINK_NAME[robot.model().model_name],
                "ee_left",
                T_left,
                CARTESIAN_COMMAND_PARAMETER["linear_velocity_limit"],
                CARTESIAN_COMMAND_PARAMETER["angular_velocity_limit"],
                CARTESIAN_COMMAND_PARAMETER["acceleration_limit_scaling"],
            )
            .set_stop_position_tracking_error(
                CARTESIAN_COMMAND_PARAMETER["stop_position_tracking_error"]
            )
            .set_stop_orientation_tracking_error(
                CARTESIAN_COMMAND_PARAMETER["stop_orientation_tracking_error"]
            )
            .set_minimum_time(CARTESIAN_COMMAND_PARAMETER["minimum_time"])
        )
    )
    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        logging.error("Failed to conduct 'Cartesian Command' example.")
        return False

    return True


def example_impedance_control_command(robot):
    logging.info("===== Impedance Control Command Example =====")

    # Initialize transformation matrices
    T_right = np.eye(4)

    # Define transformation matrices

    angle = -np.pi / 4
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.4, -0.4, 0]

    # Build commands
    right_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time=10)
        )
        .set_reference_link_name(BODY_LINK_NAME[robot.model().model_name])
        .set_link_name("ee_right")
        .set_translation_weight([3000, 3000, 0])
        .set_rotation_weight([50, 50, 50])
        .set_transformation(T_right)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                right_arm_command
            )
        )
    )
    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct impedance motion.")
        return 1

    return 0


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)

    robot.set_parameter("default.acceleration_limit_scaling", "0.8")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "5")

    if not movej(
        robot,
        (
            np.deg2rad([0, 30, -60, 30, 0, 0])
            if robot.model().model_name != "T5"
            else np.deg2rad([30, -60, 30, 0, 0])
        ),
        np.deg2rad([30, -10, 0, -100, 0, 20, 0]),
        np.deg2rad([30, 10, 0, -100, 0, 20, 0]),
        5,
    ):
        exit(1)

    if not example_cartesian_command(robot):
        exit(1)
    if not example_impedance_control_command(robot):
        print("Finish Impedance")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="07_impedance_control")
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
