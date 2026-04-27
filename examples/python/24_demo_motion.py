################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Motion Demo
# This example connects to an RB-Y1 robot, configures the control manager,
# and runs joint position, Cartesian, impedance, optimal control,
# and mixed command demos in sequence. See --help for arguments.
#
# Usage example:
#     python 24_demo_motion.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk as rby
import numpy as np
import argparse
from typing import Iterable
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

D2R = np.pi / 180  # Degree to Radian conversion factor
MINIMUM_TIME = 2
LINEAR_VELOCITY_LIMIT = 1.5
ANGULAR_VELOCITY_LIMIT = np.pi * 1.5
ACCELERATION_LIMIT = 1.0
STOP_ORIENTATION_TRACKING_ERROR = 1e-4
STOP_POSITION_TRACKING_ERROR = 1e-3
WEIGHT = 1
STOP_COST = WEIGHT * WEIGHT * 2e-3
MIN_DELTA_COST = WEIGHT * WEIGHT * 2e-3
PATIENCE = 10

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


def rot_z(angle_rad: float) -> np.ndarray:
    """Rotation matrix about Z-axis.

    Args:
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation numpy array.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])



def example_joint_position_command_1(robot):
    print("joint position command example 1")

    # Initialize joint positions
    q_joint_torso = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_right_arm[3] = -90 * D2R
    q_joint_left_arm[3] = -90 * D2R

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_torso)
            )
            .set_right_arm_command(
                rby.JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_right_arm)
            )
            .set_left_arm_command(
                rby.JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_left_arm)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_joint_position_command_2(robot):
    print("joint position command example 2")

    # Define joint positions
    q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R

    q_joint_right_arm = np.array([-45, -30, 0, -90, 0, 45, 0]) * D2R
    q_joint_left_arm = np.array([-45, 30, 0, -90, 0, 45, 0]) * D2R

    # Combine joint positions
    q = np.concatenate([q_joint_torso, q_joint_right_arm, q_joint_left_arm])

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyCommandBuilder().set_command(
                rby.JointPositionCommandBuilder()
                .set_position(q)
                .set_minimum_time(MINIMUM_TIME)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

def example_joint_position_command_3(robot):
    print("Joint position command example 3")

    # Define joint angles in degrees and convert to radians
    q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R

    q_joint_right_arm = np.array([-45, -30, 0, -90, 0, 45, 0]) * D2R
    q_joint_left_arm = np.array([-45, 30, 0, -90, 0, 45, 0]) * D2R

    # Concatenate joint positions
    q = np.concatenate((q_joint_torso, q_joint_right_arm, q_joint_left_arm))

    # Build joint position command
    joint_position_command = (
        rby.JointPositionCommandBuilder().set_position(q).set_minimum_time(MINIMUM_TIME)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(joint_position_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

def example_cartesian_command_1(robot):
    print("Cartesian command example 1")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1])

    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.5, -0.3, 1.0])
    T_left = make_transform(rot_y(angle), [0.5, 0.3, 1.0])

    target_link = "link_torso_5"


    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.CartesianCommandBuilder()
                .add_target(
                    "base",
                    target_link,
                    T_torso,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME * 2)
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
            .set_right_arm_command(
                rby.CartesianCommandBuilder()
                .add_target(
                    "base",
                    "ee_right",
                    T_right,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME)
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(3))
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
            .set_left_arm_command(
                rby.CartesianCommandBuilder()
                .add_target(
                    "base",
                    "ee_left",
                    T_left,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME)
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_2(robot):
    print("Cartesian command example 2")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_y(angle), [0.1, 0, 1.1])
    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.5, -0.4, 1.2])
    T_left = make_transform(rot_y(angle), [0.5, 0.4, 1.2])

    target_link = "link_torso_5"

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.CartesianCommandBuilder()
            .add_target(
                "base",
                target_link,
                T_torso,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "base",
                "ee_right",
                T_right,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "base",
                "ee_left",
                T_left,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_joint_position_target("right_arm_1", -np.pi / 3)
            .add_joint_position_target("left_arm_1", np.pi / 3)
            .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
            .set_minimum_time(MINIMUM_TIME)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_3(robot):
    print("Cartesian command example 3")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_y(angle), [0.1, 0, 1.2])

    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.35, -0.4, -0.2])
    T_left = make_transform(rot_y(angle), [0.35, 0.4, -0.2])

    target_link = "link_torso_5"

    # Build command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.CartesianCommandBuilder()
            .add_target(
                "base",
                target_link,
                T_torso,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "link_torso_5",
                "ee_right",
                T_right,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_target(
                "link_torso_5",
                "ee_left",
                T_left,
                LINEAR_VELOCITY_LIMIT,
                ANGULAR_VELOCITY_LIMIT,
                ACCELERATION_LIMIT,
            )
            .add_joint_position_target("right_arm_1", -np.pi / 3)
            .add_joint_position_target("left_arm_1", np.pi / 3)
            .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
            .set_minimum_time(MINIMUM_TIME)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_impedance_control_command_1(robot):
    print("Impedance control command example 1")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_y(angle), [0.1, 0, 1.2])

    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.35, -0.4, -0.2])
    T_left = make_transform(rot_y(angle), [0.35, 0.4, -0.2])

    target_link = "link_torso_5"

    # Build commands
    torso_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name("base")
        .set_link_name(target_link)
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([100, 100, 100])
        .set_transformation(T_torso)
    )

    right_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name(target_link)
        .set_link_name("ee_right")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_right)
    )

    left_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name(target_link)
        .set_link_name("ee_left")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_left)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(torso_command)
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_relative_command_1(robot):
    print("Relative command example 1")


    # Define transformation matrices
    angle = -np.pi / 4
    T_right = make_transform(rot_y(angle), [0.5, -0.4, 0.9])

    # Build Cartesian command
    right_arm_command = (
        rby.CartesianCommandBuilder()
        .set_minimum_time(MINIMUM_TIME)
        .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
        .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
        .add_target(
            "base",
            "ee_right",
            T_right,
            LINEAR_VELOCITY_LIMIT,
            ANGULAR_VELOCITY_LIMIT,
            ACCELERATION_LIMIT,
        )
        # .add_joint_position_target("right_arm_1", -np.pi/3)
    )

    # Define transformation difference
    T_diff = make_transform(np.eye(3), [0, 0.8, 0])

    # Build Impedance control command
    left_arm_command = (
        rby.ImpedanceControlCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name("ee_right")
        .set_link_name("ee_left")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_diff)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0




def example_optimal_control_1(robot):
    print("Optimal control example 1")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1.0])

    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.5, -0.2, 1.0])
    T_left = make_transform(rot_y(angle), [0.5, 0.2, 1.0])

    target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        rby.OptimalControlCommandBuilder()
        .add_cartesian_target("base", target_link, T_torso, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_joint_position_target("right_arm_2", np.pi / 2, WEIGHT / 5)
        .add_joint_position_target("left_arm_2", -np.pi / 2, WEIGHT / 5)
        .set_velocity_limit_scaling(0.5)
        .set_error_scaling(1.5)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_2(robot):
    print("Optimal control example 2")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1.0])

    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.4, -0.2, 1.0])
    T_left = make_transform(rot_y(angle), [0.4, 0.2, 1.0])

    target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        rby.OptimalControlCommandBuilder()
        .add_cartesian_target("base", target_link, T_torso, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_joint_position_target("right_arm_2", 0.05, WEIGHT / 2)
        .add_joint_position_target("left_arm_2", -0.05, WEIGHT / 2)
        .set_velocity_limit_scaling(1)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_3(robot):
    print("Optimal control example 3")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 0])

    angle = -np.pi / 2
    T_right = make_transform(rot_y(angle), [0.5, -0.3, 1.2])
    T_left = make_transform(rot_y(angle), [0.5, 0.3, 1.2])

    COM = np.array([-0.0, 0.0, 0.47])

    target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        rby.OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", COM, WEIGHT * 5)
        .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        # .add_joint_position_target("torso_2", -np.pi / 2, WEIGHT / 4)
        # .add_joint_position_target("torso_1", np.pi/4, WEIGHT)
        # .add_joint_position_target("torso_5", 0, WEIGHT)
        .add_joint_position_target("right_arm_2", np.pi / 4, WEIGHT / 20)
        .add_joint_position_target("left_arm_2", -np.pi / 4, WEIGHT / 20)
        .set_velocity_limit_scaling(0.5)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_mixed_command_1(robot):
    print("Mixed command example 1")

    # Define transformation matrices
    T_torso = make_transform(np.eye(3), [0, 0, 1])

    target_link = "link_torso_5"
    target_joint = "torso_2"
    torso_command = (
        rby.OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
        .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
        .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
        .add_joint_position_target("torso_0", 0, WEIGHT)
        .set_stop_cost(STOP_COST * 1e1)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )


    right_arm_command = (
        rby.JointPositionCommandBuilder()
        .set_position(np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, 0]))
        .set_velocity_limit(np.array([np.pi] * 7))
        .set_acceleration_limit(np.array([1.0] * 7))
        .set_minimum_time(MINIMUM_TIME)
    )

    # Send command
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(torso_command)
                .set_right_arm_command(right_arm_command)
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_mixed_command_2(robot):
    print("Mixed command example 2")

    # Define transformation matrices
    angle = np.pi / 6
    T_torso = make_transform(rot_z(angle), [0, 0, 1])

    target_link = "link_torso_5"
    target_joint = "torso_2"
    torso_command = (
        rby.OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
        .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
        .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
        .add_joint_position_target("torso_0", 0, WEIGHT)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST / 10)
        .set_patience(PATIENCE * 10)
    )

    right_arm_command = (
        rby.JointPositionCommandBuilder()
        .set_position(np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, 0]))
        .set_velocity_limit(np.array([np.pi] * 7))
        .set_acceleration_limit(np.array([1.0] * 7))
        .set_minimum_time(MINIMUM_TIME)
    )

    left_arm_command = (
        rby.GravityCompensationCommandBuilder()
        .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_on(True)
    )

    # Send command
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(torso_command)
                .set_right_arm_command(right_arm_command)
                .set_left_arm_command(left_arm_command)
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def go_to_home_pose_1(robot):
    print("Go to home pose 1")

    q_joint_torso = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    q_joint_right_arm[1] = -135 * D2R
    q_joint_left_arm[1] = 135 * D2R

    # Send command to go to ready position
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_torso)
                )
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_right_arm)
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_left_arm)
                )
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def go_to_home_pose_2(robot):
    print("Go to home pose 2")

    target_joint = np.zeros(20)

    # Send command to go to home pose
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_position(target_joint)
                .set_minimum_time(MINIMUM_TIME)
            )
        ),
        10,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def main(address, model_name, power, servo):
    robot = initialize_robot(address, model_name, power, servo)

    # robot.factory_reset_all_parameters()
    robot.set_parameter("default.acceleration_limit_scaling", "1.0")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "20")
    robot.set_parameter("default.angular_acceleration_limit", "10")
    robot.set_parameter("manipulability_threshold", "1e4")
    # robot.set_time_scale(1.0)

    print("parameters setting is done")

    move_to_pre_control_pose(robot)

    if not example_joint_position_command_1(robot):
        print("finish motion")
    if not example_joint_position_command_2(robot):
        print("finish motion")
    if not example_cartesian_command_1(robot):
        print("finish motion")
    if not example_cartesian_command_2(robot):
        print("finish motion")
    if not example_cartesian_command_3(robot):
        print("finish motion")
    if not example_impedance_control_command_1(robot):
        print("finish motion")
    if not example_relative_command_1(robot):
        print("finish motion")
    if not example_joint_position_command_3(robot):
        print("finish motion")
    if not example_optimal_control_1(robot):
        print("finish motion")
    if not example_optimal_control_2(robot):
        print("finish motion")
    if not example_optimal_control_3(robot):
        print("finish motion")
    if not example_mixed_command_1(robot):
        print("finish motion")
    if not example_mixed_command_2(robot):
        print("finish motion")
    # if not go_to_home_pose_1(robot):
    #     print("finish motion")
    if not go_to_home_pose_2(robot):
        print("finish motion")

    print("end of demo")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="24_demo_motion")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--model", type=str, default='a', help="Robot Model Name (default: 'a')")
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

    main(address=args.address, model_name = args.model, power=args.power, servo=args.servo)
