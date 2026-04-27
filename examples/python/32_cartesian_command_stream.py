################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Cartesian Command Stream Demo
# This example brings the robot to a pre-control pose, moves to a Cartesian ready pose,
# and then streams right-arm Cartesian targets while monitoring feedback. See --help for arguments.
#
# Usage example:
#     python 32_cartesian_command_stream.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import numpy as np
import time
import argparse
import logging
import signal
from typing import Iterable
import importlib

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

def move_to_pre_control_pose(robot):
    """Move to the pre-control pose before starting the motion."""
    torso = np.array([0.0, 0.1, -0.2, 0.1, 0.0, 0.0])
    right_arm = np.array([0.2, -0.2, 0.0, -1.0, 0, 0.7, 0.0])
    left_arm = np.array([0.2, 0.2, 0.0, -1.0, 0, 0.7, 0.0])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        exit(1)


def main(address, model, power, servo):
    logging.info("===== Cartesian Command Stream Example =====")

    robot = initialize_robot(address, model, power, servo)

    robot.set_parameter("cartesian_command.cutoff_frequency", "5")

    move_to_pre_control_pose(robot)

    robot_model = robot.model()

    if not movej(
        robot,
        torso=None if robot_model.model_name == "UB" else np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 40.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 40.0, 0.0]),
        minimum_time=4.0,
    ):
        exit(1)

    dyn_robot = robot.get_dynamics()
    dyn_state = dyn_robot.make_state(["base", "ee_right"], robot_model.robot_joint_names)
    BASE_LINK_IDX = 0
    EE_RIGHT_LINK_IDX = 1

    dyn_state.set_q(robot.get_state().position)
    dyn_robot.compute_forward_kinematics(dyn_state)
    T_ref = dyn_robot.compute_transformation(
        dyn_state, BASE_LINK_IDX, EE_RIGHT_LINK_IDX
    )

    def build_cartesian_command(T: np.typing.NDArray):
        rc = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                    rby.CartesianCommandBuilder()
                    .set_command_header(
                        rby.CommandHeaderBuilder().set_control_hold_time(1e6)
                    )
                    .add_joint_position_target("right_arm_2", 0.5, 1, 100)
                    .add_target("base", "ee_right", T, 0.3, 100.0, 0.8)
                    .set_minimum_time(2)
                )
            )
        )
        return rc

    stream = robot.create_command_stream()

    def handler(signum, frame):
        stream.cancel()
        exit(1)

    signal.signal(signal.SIGINT, handler)

    for pos_diff in [
        [0, 0, -0.08],
        [0, 0, 0.08],
        [0.1, -0.16, 0.08],
        [0.1, -0.16, -0.08],
        [0, 0, -0.08],
        [0, 0, 0.08],
        [0, -0.2, 0.1],
        [0, -0.2, -0.1],
    ]:
        target = T_ref.copy()
        target[0, 3] += pos_diff[0]
        target[1, 3] += pos_diff[1]
        target[2, 3] += pos_diff[2]
        rc = build_cartesian_command(target)
        stream.send_command(rc)

        log_count = 0
        while True:
            feedback = stream.request_feedback()

            def extract_cartesian_command_feedback(f):
                return (
                    f.component_based_command.body_command.body_component_based_command.right_arm_command.cartesian_command
                )

            feedback = extract_cartesian_command_feedback(feedback)
            if log_count % 100 == 0:
                logging.info(
                    f"position error: {feedback.se3_pose_tracking_errors[0].position_error}, manipulability: {feedback.manipulability}"
                )
            log_count += 1
            if feedback.se3_pose_tracking_errors[0].position_error < 1e-2:
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="32_cartesian_command_stream")
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
