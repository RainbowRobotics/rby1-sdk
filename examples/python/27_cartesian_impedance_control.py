# Impedance Control Demo
# This example demonstrates how to connect to an RB-Y1 robot, set motion parameters,
# perform an initial joint motion, send a Cartesian command, and then run an impedance
# control command for the right arm. See --help for arguments.
#
# Usage example:
#     python 27_impedance_control.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import logging

import numpy as np
import rby1_sdk as rby
from typing import Iterable
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)



class CartesianParams:
    """Container for Cartesian command parameters."""

    minimum_time: float = 5.0
    linear_velocity_limit: float = 1.5
    angular_velocity_limit: float = float(np.pi * 1.5)
    acceleration_limit_scaling: float = 1.0
    stop_position_tracking_error: float = 1e-5
    stop_orientation_tracking_error: float = 1e-5

def move_to_pre_control_pose(robot):
    """Move to the pre-control pose before starting the motion."""
    torso = np.array([0.0, 0.1, -0.2, 0.1, 0.0, 0.0])
    right_arm = np.array([0.2, -0.2, 0.0, -1.0, 0, 0.7, 0.0])
    left_arm = np.array([0.2, 0.2, 0.0, -1.0, 0, 0.7, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)

def rot_y(angle_rad: float) -> np.ndarray:
    """Rotation matrix about Y-axis.

    Args:
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation numpy array.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def make_transform(r: np.ndarray, t: Iterable[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from rotation and translation.

    Args:
        r: 3x3 rotation matrix.
        t: Iterable of 3 floats [x, y, z].

    Returns:
        4x4 homogeneous transform.
    """
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = np.asarray(t, dtype=float)
    return T

def example_cartesian_command(robot):
    """Send a multi-target Cartesian command for torso and both arms.

    Returns:
        True if the command finished with Ok, otherwise False.
    """
    logging.info("===== Cartesian Command Example =====")

    # Build transforms
    torso_angle = np.pi / 6
    right_angle = -np.pi / 4
    left_angle = -np.pi / 4

    T_torso = make_transform(rot_y(torso_angle), [0.1, 0.0, 1.2])
    T_right = make_transform(rot_y(right_angle), [0.4, -0.4, 0.0])
    T_left = make_transform(rot_y(left_angle), [0.4, 0.4, 0.0])

    params = CartesianParams()

    # Resolve base/body link names safely
    body_link = "link_torso_5"

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.CartesianCommandBuilder()
            .add_target(
                "base",
                body_link,
                T_torso,
                params.linear_velocity_limit,
                params.angular_velocity_limit,
                params.acceleration_limit_scaling,
            )
            .add_target(
                body_link,
                "ee_right",
                T_right,
                params.linear_velocity_limit,
                params.angular_velocity_limit,
                params.acceleration_limit_scaling,
            )
            .add_target(
                body_link,
                "ee_left",
                T_left,
                params.linear_velocity_limit,
                params.angular_velocity_limit,
                params.acceleration_limit_scaling,
            )
            .set_stop_position_tracking_error(params.stop_position_tracking_error)
            .set_stop_orientation_tracking_error(params.stop_orientation_tracking_error)
            .set_minimum_time(params.minimum_time)
        )
    )
    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        logging.error("Failed to conduct 'Cartesian Command' example.")
        return False

    return True


def example_impedance_control_command(robot):
    """Send an impedance control command to the right arm.

    Returns:
        True if the command finished with Ok, otherwise False.
    """
    logging.info("===== Impedance Control Command Example =====")

    # Desired right arm pose relative to body
    right_angle = -np.pi / 4
    T_right = make_transform(rot_y(right_angle), [0.4, -0.4, 0.0])

    body_link = "link_torso_5"

    # Build commands
    right_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time=10)
        )
        .set_reference_link_name(body_link)
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
        logging.error("Failed to conduct 'Impedance Control' example.")
        return False

    return True


def main(address, model, power, servo):
    """
    Main example flow.
    - Initializes the robot
    - Sets common parameters
    - Executes a joint motion, a Cartesian command, and an impedance command
    """
    robot = initialize_robot(address, model, power, servo)

    robot.set_parameter("default.acceleration_limit_scaling", "0.8")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "5")

    
    move_to_pre_control_pose(robot)
    if not example_cartesian_command(robot):
        exit(1)

    if not example_impedance_control_command(robot):
        exit(1)

    logging.info("All examples finished successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="27_impedance_control")
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
