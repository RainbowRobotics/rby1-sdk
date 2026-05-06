################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Wiggle Motion Demo
# This example moves the robot to the ready pose and performs a continuous sinusoidal
# "wiggle" motion on all torso joints using joint impedance control.
# The arms hold the ready pose in impedance mode.
# Press Ctrl+C to stop.
#
# Circular motion parameters:
#   - Amplitude : 0.1 rad
#   - Period    : 2.0 s
#   - pitch joints (torso_1,2,3 / Y-axis): sin(ωt)
#   - roll  joints (torso_0,4   / X-axis): cos(ωt)  ← 90° phase shift → circular trajectory
#   - torso_5 (Z-axis yaw): held at start position
#
# Usage example:
#     python 25_wiggle_motion.py --address 192.168.30.1:50051 --model m --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import importlib
import logging
import math
import signal
import time

import numpy as np
import rby1_sdk as rby

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot
movej = helper.movej

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

# Ready pose from POSE_PRESETS (degrees → radians)
# A / M v1.0-v1.2 : torso [0,45,-90,45,0,0],  arms [0,∓5,0,-120,0,70,0]
# M v1.3           : arms [0,∓5,0,-120,0,30,0] — indistinguishable at runtime; use 70°
# UB               : torso [0,0] (2-DOF, zero), arms same
_RIGHT_READY = np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0])
_LEFT_READY  = np.deg2rad([0.0,  5.0, 0.0, -120.0, 0.0, 70.0, 0.0])

# Circular wiggle parameters
# pitch joints (torso_1,2,3 — Y-axis): driven with sin(ωt)
# roll  joints (torso_0,4   — X-axis): driven with cos(ωt)  ← 90° phase shift
# The 90° phase difference between pitch and roll produces a circular trajectory.
_AMPLITUDE = 0.13       # rad
_OMEGA     = math.pi    # 2π / T = 2π / 2.0 s = π rad/s  → period = 2 s
_DT        = 0.01       # 100 Hz control loop period (s)
_MIN_TIME  = _DT * 1.02 # minimum_time: slightly longer than dt to give the controller
                        # enough headroom between successive stream commands
_HOLD_TIME  = 5.0        # control_hold_time (s) — keeps control alive between commands
_ACCEL_TIME = 2.0        # amplitude ramp-up duration at start (s)
_DECEL_TIME = 2.0        # amplitude taper duration on Ctrl+C (s)

# Impedance parameters
# Torso : stiffness=400, torque_limit=500, damping=0.7
#         source: lerobot-robot-rby1 config_rby1.py (impedance_stiffness / impedance_torque_limit / impedance_damping_ratio)
# Arms  : stiffness=100, torque_limit=10, damping=1.0
#         source: rby1-sdk example 28_joint_impedance_control
_TORSO_STIFFNESS  = [400.0] * 6
_TORSO_TORQUE_LIM = [500.0] * 6
_TORSO_DAMPING    = 0.7
_ARM_STIFFNESS    = [100.0] * 7
_ARM_TORQUE_LIM   = [ 10.0] * 7
_ARM_DAMPING      = 1.0


def move_to_ready_pose(robot):
    logging.info("Moving to ready pose...")
    robot_model = robot.model()
    # UB has 2-DOF torso → zero; A/M have 6-DOF torso → [0,45,-90,45,0,0]°
    torso = (
        np.zeros(len(robot_model.torso_idx))
        if robot_model.model_name == "UB"
        else np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0])
    )
    if not movej(
        robot,
        torso=torso,
        right_arm=_RIGHT_READY,
        left_arm=_LEFT_READY,
        minimum_time=5.0,
    ):
        logging.error("Failed to move to ready pose.")
        exit(1)


def main(address, model, power, servo):
    robot = initialize_robot(address, model, power, servo)

    # Explicitly reset and re-enable control manager (mirrors pattern from example 28).
    # This clears any lingering fault from a previous run before starting motion.
    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        logging.error("Failed to enable control manager")
        exit(1)

    logging.info("===== Wiggle Motion Example =====")

    move_to_ready_pose(robot)

    # Read actual joint positions after arriving at the ready pose.
    # These become the oscillation centres: wiggle = start_pos ± amplitude.
    robot_model = robot.model()
    state = robot.get_state()
    torso_start = np.array([state.position[i] for i in robot_model.torso_idx])
    right_start  = np.array([state.position[i] for i in robot_model.right_arm_idx])
    left_start   = np.array([state.position[i] for i in robot_model.left_arm_idx])

    stream = robot.create_command_stream(10)

    # stop_requested[0]: bool flag; stop_requested[1]: timestamp when stop was requested
    stop_requested = [False, 0.0]

    def _stop(signum, frame):
        if not stop_requested[0]:
            logging.info("Ctrl+C received — decelerating over %.1f s...", _DECEL_TIME)
            stop_requested[0] = True
            stop_requested[1] = time.monotonic()

    signal.signal(signal.SIGINT, _stop)

    logging.info("Running wiggle motion. Press Ctrl+C to stop.")

    t_start = time.monotonic()

    while True:
        elapsed = time.monotonic() - t_start

        # Ramp-up at start, ramp-down on Ctrl+C
        t_since_stop = time.monotonic() - stop_requested[1] if stop_requested[0] else 0.0
        if stop_requested[0]:
            amplitude = _AMPLITUDE * max(0.0, 1.0 - t_since_stop / _DECEL_TIME)
        else:
            amplitude = _AMPLITUDE * min(1.0, elapsed / _ACCEL_TIME)

        sinval = amplitude * math.sin(_OMEGA * elapsed)  # pitch component
        cosval = amplitude * math.cos(_OMEGA * elapsed)  # roll  component (90° lead)

        # Circular motion: pitch joints (Y-axis) follow sin, roll joints (X-axis) follow cos.
        # 90° phase difference between the two axes traces a circle in joint space.
        torso_q = torso_start.copy()
        torso_q[0] += cosval   # torso_0: roll  (X-axis)
        torso_q[1] += sinval   # torso_1: pitch (Y-axis)
        torso_q[2] += sinval   # torso_2: pitch (Y-axis)
        torso_q[3] += sinval   # torso_3: pitch (Y-axis)
        torso_q[4] += cosval   # torso_4: roll  (X-axis)
        # torso_5 (Z-axis yaw) held at start position
        torso_q = torso_q.tolist()

        rc = (
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(
                    rby.BodyComponentBasedCommandBuilder()
                    .set_torso_command(
                        rby.JointImpedanceControlCommandBuilder()
                        .set_command_header(
                            rby.CommandHeaderBuilder().set_control_hold_time(_HOLD_TIME)
                        )
                        .set_position(torso_q)
                        .set_minimum_time(_MIN_TIME)
                        .set_stiffness(_TORSO_STIFFNESS)
                        .set_damping_ratio(_TORSO_DAMPING)
                        .set_torque_limit(_TORSO_TORQUE_LIM)
                    )
                    .set_right_arm_command(
                        rby.JointImpedanceControlCommandBuilder()
                        .set_command_header(
                            rby.CommandHeaderBuilder().set_control_hold_time(_HOLD_TIME)
                        )
                        .set_position(right_start.tolist())
                        .set_minimum_time(_MIN_TIME)
                        .set_stiffness(_ARM_STIFFNESS)
                        .set_damping_ratio(_ARM_DAMPING)
                        .set_torque_limit(_ARM_TORQUE_LIM)
                    )
                    .set_left_arm_command(
                        rby.JointImpedanceControlCommandBuilder()
                        .set_command_header(
                            rby.CommandHeaderBuilder().set_control_hold_time(_HOLD_TIME)
                        )
                        .set_position(left_start.tolist())
                        .set_minimum_time(_MIN_TIME)
                        .set_stiffness(_ARM_STIFFNESS)
                        .set_damping_ratio(_ARM_DAMPING)
                        .set_torque_limit(_ARM_TORQUE_LIM)
                    )
                )
            )
        )

        stream.send_command(rc)

        if stop_requested[0] and t_since_stop >= _DECEL_TIME:
            logging.info("Deceleration complete. Cancelling stream.")
            stream.cancel()
            break

        time.sleep(_DT)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="39_wiggle_motion")
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
