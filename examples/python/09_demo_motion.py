import rby1_sdk
import numpy as np
import sys
import time
import argparse
import re
from rby1_sdk import *

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


def cb(rs):
    print(f"Timestamp: {rs.timestamp - rs.ft_sensor_right.time_since_last_update}")
    position = rs.position * 180 / 3.141592
    print(f"torso [deg]: {position[2:2 + 6]}")
    print(f"right arm [deg]: {position[8:8 + 7]}")
    print(f"left arm [deg]: {position[15:15 + 7]}")


def example_joint_position_command_1(robot, model_name):
    print("joint position command example 1")

    # Initialize joint positions
    if model_name == "a":
        q_joint_torso = np.zeros(6)
    elif model_name == "t5":
        q_joint_torso = np.zeros(5)
    elif model_name == "m":
        q_joint_torso = np.zeros(6)
        
    q_joint_torso = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_right_arm[1] = -90 * D2R
    q_joint_left_arm[1] = 90 * D2R

    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(
                JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_torso)
            )
            .set_right_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_right_arm)
            )
            .set_left_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_left_arm)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_joint_position_command_2(robot, model_name):
    print("joint position command example 2")

    # Define joint positions
    if model_name == "a":
        q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R
    elif model_name == "t5":
        q_joint_torso = np.array([30, -60, 30, 0, 0]) * D2R
    elif model_name == "m":
        q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R
        
    q_joint_right_arm = np.array([-45, -30, 0, -90, 0, 45, 0]) * D2R
    q_joint_left_arm = np.array([-45, 30, 0, -90, 0, 45, 0]) * D2R

    # Combine joint positions
    q = np.concatenate([q_joint_torso, q_joint_right_arm, q_joint_left_arm])

    # Build command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyCommandBuilder().set_command(
                JointPositionCommandBuilder()
                .set_position(q)
                .set_minimum_time(MINIMUM_TIME)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_1(robot, model_name):
    print("Cartesian command example 1")

    # Initialize transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    # Define transformation matrices
    T_torso[:3, :3] = np.eye(3)
    T_torso[:3, 3] = [0, 0, 1]

    angle = -np.pi / 4
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.5, -0.3, 1.0]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.5, 0.3, 1.0]
    
    if model_name == "a":
        target_link = "link_torso_5"
    elif model_name == "t5":
        target_link = "link_torso_4"
    elif model_name == "m":
        target_link = "link_torso_5"

    # Build command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(
                CartesianCommandBuilder()
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
                CartesianCommandBuilder()
                .add_target(
                    "base",
                    "ee_right",
                    T_right,
                    LINEAR_VELOCITY_LIMIT,
                    ANGULAR_VELOCITY_LIMIT,
                    ACCELERATION_LIMIT,
                )
                .set_minimum_time(MINIMUM_TIME)
                .set_command_header(CommandHeaderBuilder().set_control_hold_time(3))
                .set_stop_orientation_tracking_error(STOP_ORIENTATION_TRACKING_ERROR)
                .set_stop_position_tracking_error(STOP_POSITION_TRACKING_ERROR)
            )
            .set_left_arm_command(
                CartesianCommandBuilder()
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

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_2(robot, model_name):
    print("Cartesian command example 2")

    q = robot.get_state().position
    m = robot.model()

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
    T_torso[:3, 3] = [0.1, 0, 1.1]

    angle = -np.pi / 2
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.5, -0.4, 1.2]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.5, 0.4, 1.2]
    
    if model_name=="a":
        target_link = "link_torso_5"
    elif model_name=="t5":
        target_link = "link_torso_4"
    elif model_name=="m":
        target_link = "link_torso_5"

    # Build command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            CartesianCommandBuilder()
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

    with np.printoptions(precision=3, suppress=True):
        print(np.rad2deg(robot.get_state().position))

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_cartesian_command_3(robot, model_name):
    print("Cartesian command example 3")

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

    if model_name=="a":
        target_link = "link_torso_5"
    elif model_name=="t5":
        target_link = "link_torso_4"
    elif model_name=="m":
        target_link = "link_torso_5"
        
    # Build command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            CartesianCommandBuilder()
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

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_impedance_control_command_1(robot, model_name):
    print("Impedance control command example 1")

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
    T_right[:3, 3] = [0.45, -0.4, -0.1]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.45, 0.4, -0.1]

    if model_name=="a":
        target_link = "link_torso_5"
    elif model_name=="t5":
        target_link = "link_torso_4"
    elif model_name=="m":
        target_link = "link_torso_5"

    # Build commands
    torso_command = (
        ImpedanceControlCommandBuilder()
        .set_command_header(CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name("base")
        .set_link_name(target_link)
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([100, 100, 100])
        .set_transformation(T_torso)
    )

    right_arm_command = (
        ImpedanceControlCommandBuilder()
        .set_command_header(CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name(target_link)
        .set_link_name("ee_right")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_right)
    )

    left_arm_command = (
        ImpedanceControlCommandBuilder()
        .set_command_header(CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name(target_link)
        .set_link_name("ee_left")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_left)
    )

    # Send command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(torso_command)
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_relative_command_1(robot, model_name):
    print("Relative command example 1")

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
    T_torso[:3, 3] = [0.1, 0, 1.1]

    angle = -np.pi / 4
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.5, -0.4, 0.9]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.5, 0.4, 0.9]

    # Build Cartesian command
    right_arm_command = (
        CartesianCommandBuilder()
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
    T_diff = np.eye(4)
    T_diff[:3, 3] = [0, 0.8, 0]

    # Build Impedance control command
    left_arm_command = (
        ImpedanceControlCommandBuilder()
        .set_command_header(CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_reference_link_name("ee_right")
        .set_link_name("ee_left")
        .set_translation_weight([1000, 1000, 1000])
        .set_rotation_weight([50, 50, 50])
        .set_damping_ratio(0.85)
        .set_transformation(T_diff)
    )

    # Send command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_right_arm_command(right_arm_command)
            .set_left_arm_command(left_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_joint_position_command_3(robot, model_name):
    print("Joint position command example 3")

    # Define joint angles in degrees and convert to radians
    if model_name=="a":
        q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R
    elif model_name =="t5":
        q_joint_torso = np.array([30, -60, 30, 0, 0]) * D2R
    elif model_name =="m":
        q_joint_torso = np.array([0, 30, -60, 30, 0, 0]) * D2R
        
    q_joint_right_arm = np.array([-45, -30, 0, -90, 0, 45, 0]) * D2R
    q_joint_left_arm = np.array([-45, 30, 0, -90, 0, 45, 0]) * D2R

    # Concatenate joint positions
    q = np.concatenate((q_joint_torso, q_joint_right_arm, q_joint_left_arm))

    # Build joint position command
    joint_position_command = (
        JointPositionCommandBuilder().set_position(q).set_minimum_time(MINIMUM_TIME)
    )

    # Send command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(joint_position_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_1(robot, model_name):
    print("Optimal control example 1")

    # Define transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    angle = 0  # Identity rotation
    T_torso[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_torso[:3, 3] = [0, 0, 1.0]

    angle = -np.pi / 2
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.4, -0.2, 1.0]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.4, 0.2, 1.0]
    
    if model_name=="a":
        target_link = "link_torso_5"
    elif model_name=="t5":
        target_link = "link_torso_4"
    elif model_name=="m":
        target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        OptimalControlCommandBuilder()
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
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_2(robot, model_name):
    print("Optimal control example 2")

    # Define transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    angle = 0  # Identity rotation
    T_torso[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_torso[:3, 3] = [0, 0, 1.0]

    angle = -np.pi / 2
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.4, -0.2, 1.0]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.4, 0.2, 1.0]
    
    if model_name=="a":
        target_link = "link_torso_5"
    elif model_name=="t5":
        target_link = "link_torso_4"
    elif model_name=="m":
        target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        OptimalControlCommandBuilder()
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
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_optimal_control_3(robot, model_name):
    print("Optimal control example 3")

    # Define transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    angle = np.pi / 4
    T_torso[:3, :3] = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    T_torso[:3, 3] = [0, 0, 1.0]

    angle = -np.pi / 2
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.5, -0.3, 1.2]

    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.5, 0.3, 1.2]

    COM = np.array([-0.0, 0.0, 0.47])
    
    if model_name=="a":
        target_link = "link_torso_5"
    elif model_name=="t5":
        target_link = "link_torso_4"
    elif model_name=="m":
        target_link = "link_torso_5"

    # Build optimal control command
    optimal_control_command = (
        OptimalControlCommandBuilder()
        .set_center_of_mass_target("base", COM, WEIGHT * 5)
        .add_cartesian_target("base", target_link, np.eye(4), 0, WEIGHT)
        .add_cartesian_target("base", "ee_left", T_left, WEIGHT, WEIGHT)
        .add_cartesian_target("base", "ee_right", T_right, WEIGHT, WEIGHT)
        # .add_joint_position_target("torso_2", -np.pi / 2, WEIGHT / 4)
        #    .add_joint_position_target("torso_1", np.pi/4, WEIGHT)
        #    .add_joint_position_target("torso_5", 0, WEIGHT)
        .add_joint_position_target("right_arm_2", np.pi / 4, WEIGHT / 20)
        .add_joint_position_target("left_arm_2", -np.pi / 4, WEIGHT / 20)
        .set_velocity_limit_scaling(0.5)
        .set_stop_cost(STOP_COST)
        .set_min_delta_cost(MIN_DELTA_COST)
        .set_patience(PATIENCE)
    )

    # Send command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(optimal_control_command)
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_mixed_command_1(robot, model_name):
    print("Mixed command example 1")

    # Define transformation matrices
    T_torso = np.eye(4)
    T_torso[:3, 3] = [0, 0, 1]

    angle = -np.pi / 4
    T_right = np.eye(4)
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.5, -0.3, 1.0]

    T_left = np.eye(4)
    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.5, 0.3, 1.0]
    
    if model_name =="a":
        target_link = "link_torso_5"
        target_joint = "torso_2"
        torso_command = (
            OptimalControlCommandBuilder()
            .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
            .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
            .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
            .add_joint_position_target("torso_0", 0, WEIGHT)
            .set_stop_cost(STOP_COST * 1e1)
            .set_min_delta_cost(MIN_DELTA_COST)
            .set_patience(PATIENCE)
        )
    elif model_name == "t5":
        target_link = "link_torso_4"
        target_joint = "torso_1"
        torso_command = (
            OptimalControlCommandBuilder()
            .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
            .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
            .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
            .set_stop_cost(STOP_COST * 1e1)
            .set_min_delta_cost(MIN_DELTA_COST)
            .set_patience(PATIENCE)
        )
    elif model_name == "m":
        target_link = "link_torso_5"
        target_joint = "torso_2"
        torso_command = (
            OptimalControlCommandBuilder()
            .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
            .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
            .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
            .add_joint_position_target("torso_0", 0, WEIGHT)
            .set_stop_cost(STOP_COST * 1e1)
            .set_min_delta_cost(MIN_DELTA_COST)
            .set_patience(PATIENCE)
        )


    right_arm_command = (
        JointPositionCommandBuilder()
        .set_position(np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, 0]))
        .set_velocity_limit(np.array([np.pi] * 7))
        .set_acceleration_limit(np.array([1.0] * 7))
        .set_minimum_time(MINIMUM_TIME)
    )

    # Send command
    rv = robot.send_command(
        RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(
                BodyComponentBasedCommandBuilder()
                .set_torso_command(torso_command)
                .set_right_arm_command(right_arm_command)
            )
        ),
        10,
    ).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_mixed_command_2(robot, model_name):
    print("Mixed command example 2")

    # Define transformation matrices
    T_torso = np.eye(4)
    angle = np.pi / 4
    T_torso[:3, :3] = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    T_torso[:3, 3] = [0, 0, 1]

    angle = -np.pi / 2
    T_right = np.eye(4)
    T_right[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_right[:3, 3] = [0.5, -0.3, 1.0]

    T_left = np.eye(4)
    T_left[:3, :3] = np.array(
        [
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)],
        ]
    )
    T_left[:3, 3] = [0.5, 0.3, 1.0]
    
    if model_name =="a":
        target_link = "link_torso_5"
        target_joint = "torso_2"
        torso_command = (
            OptimalControlCommandBuilder()
            .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
            .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
            .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
            .add_joint_position_target("torso_0", 0, WEIGHT)
            .set_stop_cost(STOP_COST)
            .set_min_delta_cost(MIN_DELTA_COST / 10)
            .set_patience(PATIENCE * 10)
        )
    elif model_name == "t5":
        target_link = "link_torso_4"
        target_joint = "torso_1"
        torso_command = (
            OptimalControlCommandBuilder()
            .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
            .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
            .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
            .set_stop_cost(STOP_COST)
            .set_min_delta_cost(MIN_DELTA_COST / 10)
            .set_patience(PATIENCE * 10)
        )
    elif model_name == "m":
        target_link = "link_torso_5"
        target_joint = "torso_2"
        torso_command = (
            OptimalControlCommandBuilder()
            .set_center_of_mass_target("base", np.array([0, 0, 0.4]), WEIGHT * 1e-1)
            .add_cartesian_target("base", target_link, T_torso, 0, WEIGHT)
            .add_joint_position_target(target_joint, -np.pi / 2, WEIGHT)
            .add_joint_position_target("torso_0", 0, WEIGHT)
            .set_stop_cost(STOP_COST)
            .set_min_delta_cost(MIN_DELTA_COST / 10)
            .set_patience(PATIENCE * 10)
        )

    right_arm_command = (
        JointPositionCommandBuilder()
        .set_position(np.array([0, -np.pi / 4, 0, -np.pi / 2, 0, 0, 0]))
        .set_velocity_limit(np.array([np.pi] * 7))
        .set_acceleration_limit(np.array([1.0] * 7))
        .set_minimum_time(MINIMUM_TIME)
    )

    left_arm_command = (
        GravityCompensationCommandBuilder()
        .set_command_header(CommandHeaderBuilder().set_control_hold_time(MINIMUM_TIME))
        .set_on(True)
    )

    # Send command
    rv = robot.send_command(
        RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(
                BodyComponentBasedCommandBuilder()
                .set_torso_command(torso_command)
                .set_right_arm_command(right_arm_command)
                .set_left_arm_command(left_arm_command)
            )
        ),
        10,
    ).get()

    print("!!")

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def go_to_home_pose_1(robot, model_name):
    print("Go to home pose 1")

    if model_name == "a":
        q_joint_torso = np.zeros(6)
    elif model_name == "t5":
        q_joint_torso = np.zeros(5)
    elif model_name == "m":
        q_joint_torso = np.zeros(6)
        
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    q_joint_right_arm[1] = -135 * D2R
    q_joint_left_arm[1] = 135 * D2R

    # Send command to go to ready position
    rv = robot.send_command(
        RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(
                BodyComponentBasedCommandBuilder()
                .set_torso_command(
                    JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_torso)
                )
                .set_right_arm_command(
                    JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_right_arm)
                )
                .set_left_arm_command(
                    JointPositionCommandBuilder()
                    .set_minimum_time(MINIMUM_TIME * 2)
                    .set_position(q_joint_left_arm)
                )
            )
        ),
        10,
    ).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def go_to_home_pose_2(robot, model_name):
    print("Go to home pose 2")

    if model_name =="a":
        target_joint = np.zeros(20)
    elif model_name == "t5":
        target_joint = np.zeros(19)
    elif model_name == "m":
        target_joint = np.zeros(20)
        
    # Send command to go to home pose
    rv = robot.send_command(
        RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(
                JointPositionCommandBuilder()
                .set_position(target_joint)
                .set_minimum_time(MINIMUM_TIME)
            )
        ),
        10,
    ).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def main(address, model_name, power, servo):
    print("Attempting to connect to the robot...")

    robot = rby1_sdk.create_robot(address, model_name)

    if not robot.connect():
        print("Error: Unable to establish connection to the robot at")
        sys.exit(1)

    print("Successfully connected to the robot")

    print("Starting state update...")
    robot.start_state_update(cb, 0.1)

    # robot.factory_reset_all_parameters()
    robot.set_parameter("default.acceleration_limit_scaling", "1.0")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "20")
    robot.set_parameter("default.angular_acceleration_limit", "10")
    robot.set_parameter("manipulability_threshold", "1e4")
    # robot.set_time_scale(1.0)

    print("parameters setting is done")

    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)

    if not robot.is_power_on(power):
        rv = robot.power_on(power)
        if not rv:
            print("Failed to power on")
            exit(1)

    print(servo)
    if not robot.is_servo_on(servo):
        rv = robot.servo_on(servo)
        if not rv:
            print("Fail to servo on")
            exit(1)

    control_manager_state = robot.get_control_manager_state()

    if (
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault
        or control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault
    ):

        if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
            print(
                "Warning: Detected a Major Fault in the Control Manager!!!!!!!!!!!!!!!."
            )
        else:
            print(
                "Warning: Detected a Minor Fault in the Control Manager@@@@@@@@@@@@@@@@."
            )

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    print("Control Manager state is normal. No faults detected.")

    print("Enabling the Control Manager...")
    if not robot.enable_control_manager(unlimited_mode_enabled=True):
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled successfully.")

    # if not example_joint_position_command_1(robot):
    #     print("finish motion")
    if not example_joint_position_command_2(robot, model_name):
        print("finish motion")
    if not example_cartesian_command_1(robot, model_name):
        print("finish motion")
    if not example_cartesian_command_2(robot, model_name):
        print("finish motion")
    if not example_cartesian_command_3(robot, model_name):
        print("finish motion")
    if not example_impedance_control_command_1(robot, model_name):
        print("finish motion")
    if not example_relative_command_1(robot, model_name):
        print("finish motion")
    if not example_joint_position_command_3(robot, model_name):
        print("finish motion")
    if not example_optimal_control_1(robot, model_name):
        print("finish motion")
    if not example_optimal_control_2(robot, model_name):
        print("finish motion")
    if not example_optimal_control_3(robot, model_name):
        print("finish motion")
    if not example_mixed_command_1(robot, model_name):
        print("finish motion")
    if not example_mixed_command_2(robot, model_name):
        print("finish motion")
    # if not go_to_home_pose_1(robot):
    #     print("finish motion")
    if not go_to_home_pose_2(robot, model_name):
        print("finish motion")

    print("end of demo")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="09_demo_motion")
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
