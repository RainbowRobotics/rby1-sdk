# Joint Impedance Control Demo
# This example demonstrates how to control the robot's joints using joint impedance control. See --help for arguments.
# Scenario
# 1. Move to the pre-control pose
# 2. Run joint impedance control for both arms
# 3. Command both arms toward the zero position with stiffness, damping, and torque limits
# Usage example:
#     python 28_joint_impedance_control.py --address 127.0.0.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import importlib
import argparse
import numpy as np
import logging
import rby1_sdk as rby

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

def example_joint_command(robot):
    """Move to the joint pose before appling impedance control."""
    logging.info("===== joint Control Command Example =====")

    torso = np.array([0.0,0.3386,-0.8486,1.0331,0.0,0.0])
    right_arm = np.array([0.1112, -1.4714,  -0.1801, -2.0418, -1.9436,  0.8052,  -0.6198])
    left_arm = np.array([0.1112,  1.4715,  0.1801, -2.0418,  1.9436,  0.8052,    0.6198])
    if not movej(robot, torso=torso, right_arm=right_arm, left_arm=left_arm, minimum_time=5.0):
        logging.error("Failed to conduct 'joint Control Command' example.")
        return False

    return True

def example_impedance_control_command(robot):
    """Send an impedance control command to the right arm.

    Returns:
        True if the command finished with Ok, otherwise False.
    """
    logging.info("===== Impedance Control Command Example =====")
    model = robot.model()
    # Joint Impedance Control
    rc_builder = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_right_arm_command(
                rby.JointImpedanceControlCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(10)
                )
                .set_position([0.1112, -1.4714,  -0.1801, -2.0418, -1.9436,  0.8052,  -0.6198])
                .set_minimum_time(5)
                .set_stiffness([100.0] * len(model.right_arm_idx))
                .set_damping_ratio(1.0)
                .set_torque_limit([10] * len(model.right_arm_idx))
            )
        )
    )
    handler = robot.send_command(rc_builder,10)
    rv = handler.get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        logging.error("Failed to conduct 'Impedance Control' example.")
        return False
    return True

def main(address, model, power, servo):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)
    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            logging.error(f"Failed to servo ({servo}) on")
            exit(1)
    if robot.get_control_manager_state().state in [
        rby.ControlManagerState.State.MajorFault,
        rby.ControlManagerState.State.MinorFault,
    ]:
        if not robot.reset_fault_control_manager():
            logging.error(f"Failed to reset control manager")
            exit(1)
    if not robot.enable_control_manager():
        logging.error(f"Failed to enable control manager")
        exit(1)


    move_to_pre_control_pose(robot)

    if not example_joint_command(robot):
        exit(1)

    if not example_impedance_control_command(robot):
        exit(1)

    logging.info("All examples finished successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="28_joint_impedance_control")
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
