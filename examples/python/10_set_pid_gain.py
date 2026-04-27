############################## CAUTION ###############################
# The motor PID values ​​currently set on the robot are internally optimized.
# Please exercise caution to prevent accidents caused by changing the PID values.
######################################################################
################################ Note ################################
# This example does not run in simulation.
# This example does not apply when the control manager is enabled.
######################################################################
# Set PID Gain Demo
# This example connects to an RB-Y1 robot, reads the current PID gains for
# selected joints, updates those gains using different setter forms, and then
# restores the original values. See --help for arguments.
# Note: Disable the control manager before changing PID gains.
# Note: This example is not supported in simulation.
#
# Usage example:
#     python 10_set_pid_gain.py --address 192.168.30.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk as rby
import time
import argparse
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model):
    logging.info(f"Creating robot with address='{address}', model='{model}'")
    robot = rby.create_robot(address, model)

    if not robot.connect():
        logging.error("Robot is not connected")
        exit(1)

    if not robot.is_power_on(".*"):
        logging.info("Robot power is off. Attempting to power on...")
        if not robot.power_on(".*"):
            logging.error("Failed to power on")
            exit(1)

    time.sleep(0.5)  # Waits for all actuators to be fully powered on

    # === Joint PID Gain Setting Sequences ===

    # Joint: right_arm_3, set only P gain
    target_joint_name = "right_arm_3"
    try:
        logging.info(f">>> [START] {target_joint_name}")
        original_gain = robot.get_position_pid_gain(target_joint_name)
        print(original_gain)
        logging.info(
            f"[Before] [{target_joint_name}] P: {original_gain.p_gain}, I: {original_gain.i_gain}, D: {original_gain.d_gain}"
        )

        robot.set_position_p_gain(target_joint_name, 100)
        time.sleep(0.05)
        gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[After]  [{target_joint_name}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
        )

        robot.set_position_pid_gain(target_joint_name, original_gain)
        time.sleep(0.05)
        restored = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Restored] [{target_joint_name}] P: {restored.p_gain}, I: {restored.i_gain}, D: {restored.d_gain}"
        )
    except RuntimeError as e:
        logging.error(f"Failed on {target_joint_name}: {e}")

    # Joint: left_arm_3, set all PID gains
    target_joint_name = "left_arm_3"
    try:
        logging.info(f">>> [START] {target_joint_name}")
        original_gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Before] [{target_joint_name}] P: {original_gain.p_gain}, I: {original_gain.i_gain}, D: {original_gain.d_gain}"
        )

        robot.set_position_pid_gain(target_joint_name, 60, 10, 100)
        time.sleep(0.05)
        gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[After]  [{target_joint_name}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
        )

        robot.set_position_pid_gain(target_joint_name, original_gain)
        time.sleep(0.05)
        restored = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Restored] [{target_joint_name}] P: {restored.p_gain}, I: {restored.i_gain}, D: {restored.d_gain}"
        )
    except RuntimeError as e:
        logging.error(f"Failed on {target_joint_name}: {e}")

    # Joint: head_0, set all gains via PIDGain
    target_joint_name = "head_0"
    try:
        logging.info(f">>> [START] {target_joint_name}")
        original_gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Before] [{target_joint_name}] P: {original_gain.p_gain}, I: {original_gain.i_gain}, D: {original_gain.d_gain}"
        )

        robot.set_position_pid_gain(target_joint_name, rby.PIDGain(700, 0, 3500))
        time.sleep(0.05)
        gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[After]  [{target_joint_name}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
        )

        robot.set_position_pid_gain(target_joint_name, original_gain)
        time.sleep(0.05)
        restored = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Restored] [{target_joint_name}] P: {restored.p_gain}, I: {restored.i_gain}, D: {restored.d_gain}"
        )
    except RuntimeError as e:
        logging.error(f"Failed on {target_joint_name}: {e}")

    # Joint: head_1, set only P gain
    target_joint_name = "head_1"
    try:
        logging.info(f">>> [START] {target_joint_name}")
        original_gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Before] [{target_joint_name}] P: {original_gain.p_gain}, I: {original_gain.i_gain}, D: {original_gain.d_gain}"
        )

        robot.set_position_p_gain(target_joint_name, 300)
        time.sleep(0.05)
        gain = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[After]  [{target_joint_name}] P: {gain.p_gain}, I: {gain.i_gain}, D: {gain.d_gain}"
        )

        robot.set_position_pid_gain(target_joint_name, original_gain)
        time.sleep(0.05)
        restored = robot.get_position_pid_gain(target_joint_name)
        logging.info(
            f"[Restored] [{target_joint_name}] P: {restored.p_gain}, I: {restored.i_gain}, D: {restored.d_gain}"
        )
    except RuntimeError as e:
        logging.error(f"Failed on {target_joint_name}: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="10_set_pid_gain")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
