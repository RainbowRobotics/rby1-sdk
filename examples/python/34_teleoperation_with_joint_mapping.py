# Teleoperation Example
#
# This example initializes the robot, gripper, and master arm connected to a UPC, moves the robot to a
# ready pose, and streams teleoperation commands that map master arm joint motion and trigger input to
# robot arm and gripper control. See --help for arguments.
#
# Usage example:
#     python 34_teleoperation_with_joint_mapping.py --address 192.168.30.1:50051 --model a --power '.*' --servo 'torso_.*|right_arm_.*|left_arm_.*' --mode position
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


# Run this example on a UPC to which the master arm and gripper are connected.
import importlib
import rby1_sdk as rby
import numpy as np
import os
import time
import logging
import argparse
import signal
import threading
import datetime
from typing import *
from dataclasses import dataclass

helper = importlib.import_module("00_helper")
initialize_robot = helper.initialize_robot

GRIPPER_DIRECTION = False

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@dataclass
class Pose:
    torso: np.typing.NDArray
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


class Settings:
    leader_arm_loop_period = 1 / 100
    impedance_stiffness = 50
    impedance_damping_ratio = 1.0
    impedance_torque_limit = 30.0
    startup_align_timeout = 15.0
    startup_align_tolerance = np.deg2rad(5.0)
    startup_align_command_step = np.deg2rad(0.1)


READY_POSE = Pose(
        torso=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -20.0, 0.0, 20.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -20.0, 0.0, 20.0, 0.0]),
    )

class Gripper:
    """
    Class for gripper
    """

    def __init__(self):
        self.bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self.bus.open_port()
        self.bus.set_baud_rate(2_000_000)
        self.bus.set_torque_constant([1, 1])
        self.min_q = np.array([np.inf, np.inf])
        self.max_q = np.array([-np.inf, -np.inf])
        self.target_q: np.typing.NDArray = None
        self._running = False
        self._thread = None

    def initialize(self, verbose=False):
        rv = True
        for dev_id in [0, 1]:
            if not self.bus.ping(dev_id):
                if verbose:
                    logging.error(f"Dynamixel ID {dev_id} is not active")
                rv = False
            else:
                if verbose:
                    logging.info(f"Dynamixel ID {dev_id} is active")
        if rv:
            logging.info("Servo on gripper")
            self.bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])
        return rv

    def set_operating_mode(self, mode):
        self.bus.group_sync_write_torque_enable([(dev_id, 0) for dev_id in [0, 1]])
        self.bus.group_sync_write_operating_mode([(dev_id, mode) for dev_id in [0, 1]])
        self.bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])

    def homing(self):
        self.set_operating_mode(rby.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0, 0], dtype=np.float64)
        prev_q = np.array([0, 0], dtype=np.float64)
        counter = 0
        while direction < 2:
            self.bus.group_sync_write_send_torque(
                [(dev_id, 0.5 * (1 if direction == 0 else -1)) for dev_id in [0, 1]]
            )
            rv = self.bus.group_fast_sync_read_encoder([0, 1])
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)
            if np.array_equal(prev_q, q):
                counter += 1
            prev_q = q.copy()
            if counter >= 15:
                direction += 1
                counter = 0
            time.sleep(0.1)
        return True

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self.loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def loop(self):
        self.set_operating_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque([(dev_id, 5) for dev_id in [0, 1]])
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(
                    [(dev_id, q) for dev_id, q in enumerate(self.target_q.tolist())]
                )
            time.sleep(0.1)

    def set_target(self, normalized_q):
        if not np.isfinite(self.min_q).all() or not np.isfinite(self.max_q).all():
            logging.error("Cannot set target. min_q or max_q is not valid.")
            return

        if GRIPPER_DIRECTION:
            self.target_q = normalized_q * (self.max_q - self.min_q) + self.min_q
        else:
            self.target_q = (1 - normalized_q) * (self.max_q - self.min_q) + self.min_q


def joint_position_command_builder(
    pose: Pose, minimum_time, control_hold_time=0, position_mode=True
):
    right_arm_builder = (
        rby.JointPositionCommandBuilder()
        if position_mode
        else rby.JointImpedanceControlCommandBuilder()
    )
    (
        right_arm_builder.set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )
        .set_position(pose.right_arm)
        .set_minimum_time(minimum_time)
    )
    if not position_mode:
        (
            right_arm_builder.set_stiffness(
                [Settings.impedance_stiffness] * len(pose.right_arm)
            )
            .set_damping_ratio(Settings.impedance_damping_ratio)
            .set_torque_limit([Settings.impedance_torque_limit] * len(pose.right_arm))
        )

    left_arm_builder = (
        rby.JointPositionCommandBuilder()
        if position_mode
        else rby.JointImpedanceControlCommandBuilder()
    )
    (
        left_arm_builder.set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )
        .set_position(pose.left_arm)
        .set_minimum_time(minimum_time)
    )
    if not position_mode:
        (
            left_arm_builder.set_stiffness(
                [Settings.impedance_stiffness] * len(pose.left_arm)
            )
            .set_damping_ratio(Settings.impedance_damping_ratio)
            .set_torque_limit([Settings.impedance_torque_limit] * len(pose.left_arm))
        )

    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
                )
                .set_position(pose.torso)
                .set_minimum_time(minimum_time)
            )
            .set_right_arm_command(right_arm_builder)
            .set_left_arm_command(left_arm_builder)
        )
    )


def move_j(robot: Union[rby.Robot_A, rby.Robot_M], pose: Pose, minimum_time=5.0):
    handler = robot.send_command(joint_position_command_builder(pose, minimum_time))
    return handler.get() == rby.RobotCommandFeedback.FinishCode.Ok


def main(address, model, power, servo, control_mode):
    # ===== SETUP ROBOT =====
    robot = initialize_robot(address, model, power, servo)

    supported_model = ["A", "M"]
    supported_control_mode = ["position", "impedance"]
    model = robot.model()
    dyn_model = robot.get_dynamics()
    dyn_state = dyn_model.make_state([], model.robot_joint_names)
    robot_q = None
    robot_max_q = dyn_model.get_limit_q_upper(dyn_state)
    robot_min_q = dyn_model.get_limit_q_lower(dyn_state)
    robot_max_qdot = dyn_model.get_limit_qdot_upper(dyn_state)
    robot_max_qddot = dyn_model.get_limit_qddot_upper(dyn_state)

    if control_mode == "impedance":
        robot_max_qdot[model.right_arm_idx[-1]] *= 10
        robot_max_qdot[model.left_arm_idx[-1]] *= 10

    if not model.model_name in supported_model:
        logging.error(
            f"Model {model.model_name} not supported (Current supported model is {supported_model})"
        )
        exit(1)
    if not control_mode in supported_control_mode:
        logging.error(
            f"Control mode {control_mode} not supported (Current supported control mode is {supported_control_mode})"
        )
        exit(1)
    position_mode = control_mode == "position"

    for arm in ["right", "left"]:
        if not robot.set_tool_flange_output_voltage(arm, 12):
            logging.error(f"Failed to set tool flange output voltage ({arm}) as 12v")
            exit(1)
    robot.set_parameter("joint_position_command.cutoff_frequency", "3")
    move_j(robot, READY_POSE, 5)

    def robot_state_callback(state: rby.RobotState_A):
        nonlocal robot_q
        robot_q = state.position

    robot.start_state_update(robot_state_callback, 1 / Settings.leader_arm_loop_period)

    # ===== SETUP GRIPPER =====
    gripper = Gripper()
    if not gripper.initialize():
        logging.error("Failed to initialize gripper")
        robot.stop_state_update()
        robot.power_off("12v")
        exit(1)
    gripper.homing()
    gripper.start()

    # ===== SETUP LEADER ARM =====
    rby.upc.initialize_device(rby.upc.LeaderArmDeviceName)
    leader_arm_model = f"{os.path.dirname(os.path.realpath(__file__))}/../../models/leader_arm/model.urdf"
    leader_arm = rby.upc.LeaderArm(rby.upc.LeaderArmDeviceName)
    leader_arm.set_model_path(leader_arm_model)
    leader_arm.set_control_period(Settings.leader_arm_loop_period)
    active_ids = leader_arm.initialize(verbose=False)
    if len(active_ids) != rby.upc.LeaderArm.DeviceCount:
        logging.error(
            f"Mismatch in the number of devices detected for RBY Leader Arm (active devices: {active_ids})"
        )
        exit(1)

    ma_q_limit_barrier = 0.5
    ma_min_q = np.deg2rad(
        [-360, -30, 0, -135, -90, 35, -360, -360, 10, -90, -135, -90, 35, -360]
    )
    ma_max_q = np.deg2rad(
        [360, -10, 90, -60, 90, 80, 360, 360, 30, 0, -60, 90, 80, 360]
    )
    ma_torque_limit = np.array([1.46, 1.46, 1.46, 1.46, 0.6, 0.6, 0.6] * 2)
    ma_viscous_gain = np.array([0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.002] * 2)
    startup_align_phase = True
    startup_align_initialized = False
    startup_align_failed = False
    startup_align_failure_message = ""
    startup_align_start_time = time.monotonic()
    right_q = READY_POSE.right_arm.copy()
    left_q = READY_POSE.left_arm.copy()
    right_minimum_time = 1.0
    left_minimum_time = 1.0
    stream = robot.create_command_stream(priority=1)  # TODO
    stream.send_command(
        joint_position_command_builder(
            READY_POSE,
            minimum_time=5,
            control_hold_time=1e6,
            position_mode=position_mode,
        )
    )
    logging.info("Aligning the leader arm to READY_POSE before enabling teleoperation.")

    def cleanup(exit_code=1):
        print(f"\n[{signum}] stop. safe stop procedure.")

        try: robot.stop_state_update()
        except: pass

        try: master_arm.stop_control()
        except: pass

        try: robot.cancel_control()
        except: pass

        time.sleep(0.5)
        try: robot.disable_control_manager()
        except: pass

        try: robot.power_off("12v")
        except: pass

        try: gripper.stop()
        except: pass

        try: robot.disconnect()
        except: pass

        print("end.")
        raise SystemExit(exit_code)

    log_count = 0

    def leader_arm_control_loop(state: rby.upc.LeaderArm.State):
        nonlocal position_mode, right_q, left_q, right_minimum_time, left_minimum_time, log_count, startup_align_phase, startup_align_initialized, startup_align_failed, startup_align_failure_message

        if right_q is None:
            right_q = state.q_joint[0:7]
        if left_q is None:
            left_q = state.q_joint[7:14]

        ma_input = rby.upc.LeaderArm.ControlInput()

        if startup_align_phase:
            if not startup_align_initialized:
                right_q = state.q_joint[0:7].copy()
                left_q = state.q_joint[7:14].copy()
                startup_align_initialized = True

            right_q += np.clip(
                READY_POSE.right_arm - right_q,
                -Settings.startup_align_command_step,
                Settings.startup_align_command_step,
            )
            left_q += np.clip(
                READY_POSE.left_arm - left_q,
                -Settings.startup_align_command_step,
                Settings.startup_align_command_step,
            )

            ma_input.target_operating_mode[0:7].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[0:7] = ma_torque_limit[0:7]
            ma_input.target_position[0:7] = right_q
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[7:14] = ma_torque_limit[7:14]
            ma_input.target_position[7:14] = left_q

            right_error = np.max(np.abs(state.q_joint[0:7] - READY_POSE.right_arm))
            left_error = np.max(np.abs(state.q_joint[7:14] - READY_POSE.left_arm))
            if max(right_error, left_error) <= Settings.startup_align_tolerance:
                startup_align_phase = False
                right_q = READY_POSE.right_arm.copy()
                left_q = READY_POSE.left_arm.copy()
                logging.info("Leader arm aligned to READY_POSE. Teleoperation unlocked.")
            elif (
                time.monotonic() - startup_align_start_time
                > Settings.startup_align_timeout
            ):
                startup_align_failed = True
                startup_align_failure_message = (
                    "Failed to align the leader arm to READY_POSE within the startup timeout."
                )
            return ma_input

        if startup_align_phase:
            if not startup_align_initialized:
                right_q = state.q_joint[0:7].copy()
                left_q = state.q_joint[7:14].copy()
                startup_align_initialized = True

            right_q += np.clip(
                READY_POSE.right_arm - right_q,
                -Settings.startup_align_command_step,
                Settings.startup_align_command_step,
            )
            left_q += np.clip(
                READY_POSE.left_arm - left_q,
                -Settings.startup_align_command_step,
                Settings.startup_align_command_step,
            )

            ma_input.target_operating_mode[0:7].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[0:7] = ma_torque_limit[0:7]
            ma_input.target_position[0:7] = right_q
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[7:14] = ma_torque_limit[7:14]
            ma_input.target_position[7:14] = left_q

            right_error = np.max(np.abs(state.q_joint[0:7] - READY_POSE.right_arm))
            left_error = np.max(np.abs(state.q_joint[7:14] - READY_POSE.left_arm))
            if max(right_error, left_error) <= Settings.startup_align_tolerance:
                startup_align_phase = False
                right_q = READY_POSE.right_arm.copy()
                left_q = READY_POSE.left_arm.copy()
                logging.info("Master arm aligned to READY_POSE. Teleoperation unlocked.")
            elif (
                time.monotonic() - startup_align_start_time
                > Settings.startup_align_timeout
            ):
                startup_align_failed = True
                startup_align_failure_message = (
                    "Failed to align the master arm to READY_POSE within the startup timeout."
                )
            return ma_input

        log_count += 1
        if log_count % round(1 / Settings.leader_arm_loop_period) == 0:
            print(f"--- {datetime.datetime.now().time()} ---")
            print(f"Button: {state.button_right.button}, {state.button_left.button}")
            print(f"Trigger: {state.button_right.trigger}, {state.button_left.trigger}")
            log_count = 0
        gripper.set_target(
            np.array(
                [state.button_right.trigger / 1000, state.button_left.trigger / 1000]
            )
        )
        # ===== CALCULATE LEADER ARM COMMAND =====
        torque = (
            state.gravity_term
            + ma_q_limit_barrier
            * (
                np.maximum(ma_min_q - state.q_joint, 0)
                + np.minimum(ma_max_q - state.q_joint, 0)
            )
            + ma_viscous_gain * state.qvel_joint
        )
        torque = np.clip(torque, -ma_torque_limit, ma_torque_limit)
        if state.button_right.button == 1:
            ma_input.target_operating_mode[0:7].fill(
                rby.DynamixelBus.CurrentControlMode
            )
            ma_input.target_torque[0:7] = torque[0:7] * 0.6
            right_q = state.q_joint[0:7]
        else:
            ma_input.target_operating_mode[0:7].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[0:7] = ma_torque_limit[0:7]
            ma_input.target_position[0:7] = right_q

        if state.button_left.button == 1:
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentControlMode
            )
            ma_input.target_torque[7:14] = torque[7:14] * 0.6
            left_q = state.q_joint[7:14]
        else:
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[7:14] = ma_torque_limit[7:14]
            ma_input.target_position[7:14] = left_q

        # Check whether the target configuration is in collision
        if robot_q is None:
            return ma_input

        q = robot_q.copy()
        q[model.right_arm_idx] = right_q
        q[model.left_arm_idx] = left_q
        dyn_state.set_q(q)
        dyn_model.compute_forward_kinematics(dyn_state)
        is_collision = (
            dyn_model.detect_collisions_or_nearest_links(dyn_state, 1)[0].distance
            < 0.02
        )

        # ===== BUILD ROBOT COMMAND =====
        rc = rby.BodyComponentBasedCommandBuilder()
        if state.button_right.button and not is_collision:
            right_minimum_time -= Settings.leader_arm_loop_period
            right_minimum_time = max(
                right_minimum_time, Settings.leader_arm_loop_period * 1.01
            )
            right_arm_builder = (
                rby.JointPositionCommandBuilder()
                if position_mode
                else rby.JointImpedanceControlCommandBuilder()
            )
            (
                right_arm_builder.set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1e6)
                )
                .set_position(
                    np.clip(
                        right_q,
                        robot_min_q[model.right_arm_idx],
                        robot_max_q[model.right_arm_idx],
                    )
                )
                .set_velocity_limit(robot_max_qdot[model.right_arm_idx])
                .set_acceleration_limit(robot_max_qddot[model.right_arm_idx] * 30)
                .set_minimum_time(right_minimum_time)
            )
            if not position_mode:
                (
                    right_arm_builder.set_stiffness(
                        [Settings.impedance_stiffness] * len(model.right_arm_idx)
                    )
                    .set_damping_ratio(Settings.impedance_damping_ratio)
                    .set_torque_limit(
                        [Settings.impedance_torque_limit] * len(model.right_arm_idx)
                    )
                )
            rc.set_right_arm_command(right_arm_builder)
        else:
            right_minimum_time = 0.8

        if state.button_left.button and not is_collision:
            left_minimum_time -= Settings.leader_arm_loop_period
            left_minimum_time = max(
                left_minimum_time, Settings.leader_arm_loop_period * 1.01
            )
            left_arm_builder = (
                rby.JointPositionCommandBuilder()
                if position_mode
                else rby.JointImpedanceControlCommandBuilder()
            )
            (
                left_arm_builder.set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1e6)
                )
                .set_position(
                    np.clip(
                        left_q,
                        robot_min_q[model.left_arm_idx],
                        robot_max_q[model.left_arm_idx],
                    )
                )
                .set_velocity_limit(robot_max_qdot[model.left_arm_idx])
                .set_acceleration_limit(robot_max_qddot[model.left_arm_idx] * 30)
                .set_minimum_time(left_minimum_time)
            )
            if not position_mode:
                (
                    left_arm_builder.set_stiffness(
                        [Settings.impedance_stiffness] * len(model.left_arm_idx)
                    )
                    .set_damping_ratio(Settings.impedance_damping_ratio)
                    .set_torque_limit(
                        [Settings.impedance_torque_limit] * len(model.left_arm_idx)
                    )
                )
            rc.set_left_arm_command(left_arm_builder)
        else:
            left_minimum_time = 0.8
        stream.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(rc)
            )
        )

        return ma_input

    leader_arm.start_control(leader_arm_control_loop)

    # ===== SETUP SIGNAL =====
    def handler(signum, frame):
        cleanup(1)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    while True:
        if startup_align_failed:
            logging.error(startup_align_failure_message)
            cleanup(1)
        time.sleep(0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="34_teleoperation_with_joint_mapping")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Regex pattern for power device names (default: '.*')",
    )
    parser.add_argument(
        "--servo",
        type=str,
        default="torso_.*|right_arm_.*|left_arm_.*",
        help="Regex pattern for servo names (default: 'torso_.*|right_arm_.*|left_arm_.*')",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="position",
        choices=["position", "impedance"],
        help="Control mode to use: 'position' or 'impedance' (default: 'position')",
    )
    args = parser.parse_args()

    main(args.address, args.model, args.power, args.servo, args.mode)
