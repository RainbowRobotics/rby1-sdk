"""
Teleoperation Example

Run this example on UPC to which the master arm and hands are connected
"""

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

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


@dataclass
class Pose:
    toros: np.typing.NDArray
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


class Settings:
    master_arm_loop_period = 1 / 20


READY_POSE = {
    "A": Pose(
        toros=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0])
    ),
    "T5": Pose(
        toros=np.deg2rad([45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0])
    ),
    "M": Pose(
        toros=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0])
    )
}


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
            self.bus.bulk_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])
        return rv

    def set_operation_mode(self, mode):
        self.bus.bulk_write_torque_enable([(dev_id, 0) for dev_id in [0, 1]])
        self.bus.bulk_write_operation_mode([(dev_id, mode) for dev_id in [0, 1]])
        self.bus.bulk_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])

    def homing(self):
        self.set_operation_mode(rby.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0, 0])
        prev_q = np.array([0, 0])
        counter = 0
        while direction < 2:
            self.bus.bulk_write_send_torque([(dev_id, 0.5 * (1 if direction == 0 else -1)) for dev_id in [0, 1]])
            rv = self.bus.bulk_read_encoder([0, 1])
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)
            if prev_q == q:
                counter += 1
            prev_q = q
            if counter >= 5:
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
        self.set_operation_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.bulk_write_send_torque([(dev_id, 5) for dev_id in [0, 1]])
        while self._running:
            if self.target_q is not None:
                self.bus.bulk_write_send_position([(dev_id, q) for dev_id, q in enumerate(self.target_q.tolist())])
            time.sleep(0.1)

    def set_target(self, normalized_q):
        self.target_q = normalized_q * (self.max_q - self.min_q) + self.min_q


def joint_position_command_builder(pose: Pose, minimum_time):
    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder()
        .set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_position(pose.toros)
                .set_minimum_time(minimum_time)
            )
            .set_right_arm_command(
                rby.JointPositionCommandBuilder()
                .set_position(pose.right_arm)
                .set_minimum_time(minimum_time)
            )
            .set_left_arm_command(
                rby.JointPositionCommandBuilder()
                .set_position(pose.left_arm)
                .set_minimum_time(minimum_time)
            )
        )
    )


def move_j(robot: Union[rby.Robot_A, rby.Robot_T5, rby.Robot_M], pose: Pose, minimum_time=5.0):
    handler = robot.send_command(joint_position_command_builder(pose, minimum_time))
    return handler.get() == rby.RobotCommandFeedback.FinishCode.Ok


def main(address, model, power, servo):
    # ===== SETUP ROBOT =====
    robot = rby.create_robot(address, model)
    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)
    supported_model = ["A", "T5", "M"]
    model = robot.model()

    if not model.model_name in supported_model:
        logging.error(f"Model {model.model_name} not supported (Current supported model is {supported_model})")
        exit(1)
    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            logging.error(f"Failed to servo ({servo}) on")
            exit(1)
    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        logging.error(f"Failed to enable control manager")
        exit(1)
    for arm in ['right', 'left']:
        if not robot.set_tool_flange_output_voltage(arm, 12):
            logging.error(f"Failed to set tool flange output voltage ({arm}) as 12v")
            exit(1)
    move_j(robot, READY_POSE, 5)

    is_collision = False

    def robot_state_callback(state: rby.RobotState_A):
        nonlocal is_collision
        is_collision = state.collisions[0].distance < 0.02

    robot.start_state_update(robot_state_callback, 1 / Settings.master_arm_loop_period)

    # ===== SETUP GRIPPER =====
    gripper = Gripper()
    if not gripper.initialize():
        logging.error("Failed to initialize gripper")
        exit(1)
    gripper.homing()
    gripper.start()

    # ===== SETUP SIGNAL =====
    def handler(signum, frame):
        robot.power_off("12v")
        gripper.stop()
        exit(1)

    signal.signal(signal.SIGINT, handler)

    # ===== SETUP MASTER ARM =====
    rby.upc.initialize_device(rby.upc.MasterArmDeviceName)
    master_arm_model = f"{os.path.dirname(os.path.realpath(__file__))}/../../models/master_arm/model.urdf"
    master_arm = rby.upc.MasterArm(rby.upc.MasterArmDeviceName)
    master_arm.set_model_path(master_arm_model)
    master_arm.set_control_period(Settings.master_arm_loop_period)
    active_ids = master_arm.initialize(verbose=False)
    if len(active_ids) != rby.upc.MasterArm.DeviceCount:
        logging.error(f"Mismatch in the number of devices detected for RBY Master Arm (active devices: {active_ids})")
        exit(1)

    ma_q_limit_barrier = 0.5
    ma_min_q = np.deg2rad([-360, -30, 0, -135, -90, 35, -360, -360, 10, -90, -135, -90, 35, -360])
    ma_max_q = np.deg2rad([360, -10, 90, -60, 90, 80, 360, 360, 30, 0, -60, 90, 80, 360])
    ma_torque_limit = np.array([3.] * 14)
    ma_viscous_gain = np.array([0.01, 0.01, 0.01, 0.01, 0.005, 0.005, 0.001] * 2)
    right_q = None
    left_q = None
    right_minimum_time = 1.
    left_minimum_time = 1.

    def master_arm_control_loop(state: rby.upc.MasterArm.State):
        nonlocal right_q, left_q, right_minimum_time, left_minimum_time

        if right_q is None:
            right_q = state.q_joint[0:7]
        if left_q is None:
            left_q = state.q_joint[7:14]

        ma_input = rby.upc.MasterArm.ControlInput()

        print(f"--- {datetime.datetime.now().time()} ---")
        print(f"Button: {state.button_right.button}, {state.button_left.button}")
        print(f"Trigger: {state.button_right.trigger}, {state.button_left.trigger}")

        # ===== CALCULATE MASTER ARM COMMAND =====
        torque = (
                state.gravity_term +
                ma_q_limit_barrier * (
                        np.maximum(ma_min_q - state.q_joint, 0) + np.minimum(ma_max_q - state.q_joint, 0)) +
                ma_viscous_gain * state.qvel_joint
        )
        torque = np.clip(torque, -ma_torque_limit, ma_torque_limit)
        if state.button_right:
            ma_input.target_operation_mode[0:7].fill(rby.DynamixelBus.CurrentControlMode)
            ma_input.target_torque[0:7] = torque[0:7]
            right_q = state.q_joint[0:7]
        else:
            ma_input.target_operation_mode[0:7].fill(rby.DynamixelBus.CurrentBasedPositionControlMode)
            ma_input.target_torque[0:7].fill(5)
            ma_input.target_position[0:7] = right_q

        if state.button_left:
            ma_input.target_operation_mode[7:14].fill(rby.DynamixelBus.CurrentControlMode)
            ma_input.target_torque[7:14] = torque[7:14]
            left_q = state.q_joint[7:14]
        else:
            ma_input.target_operation_mode[7:14].fill(rby.DynamixelBus.CurrentBasedPositionControlMode)
            ma_input.target_torque[7:14].fill(5)
            ma_input.target_position[7:14] = left_q

        # ===== BUILD ROBOT COMMAND =====
        # TODO: rc = rby.BodyComponentBasedCommandBuilder()
        if state.button_right and not is_collision:
            right_minimum_time -= Settings.master_arm_loop_period
            right_minimum_time = max(right_minimum_time, Settings.master_arm_loop_period * 1.1)
            # TODO: rc.set_right_arm_command(...)
        else:
            right_minimum_time = 1

        if state.button_left and not is_collision:
            left_minimum_time -= Settings.master_arm_loop_period
            left_minimum_time = max(left_minimum_time, Settings.master_arm_loop_period * 1.1)
        else:
            left_minimum_time = 1
        # TODO: stream.send_command

        return ma_input

    master_arm.start_control(master_arm_control_loop)

    time.sleep(100)

    master_arm.stop_control()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="20_teleoperation")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument('--model', type=str, default='a', help="Robot Model Name (default: 'a')")
    parser.add_argument('--power', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default="^(?!head|.*_wheel).*$",
                        help="Servo name regex pattern (default: '^(?!head|.*_wheel).*$'")
    args = parser.parse_args()

    main(args.address, args.model, args.power, args.servo)
