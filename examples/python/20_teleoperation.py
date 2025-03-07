"""
Teleoperation Example

Run this example on UPC to which the master arm and hands are connected
"""

import rby1_sdk as rby
import numpy as np
import time
import logging
import argparse
import signal
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


ZERO_POSE = {
    "A": Pose(
        toros=np.deg2rad([0.] * 6),
        right_arm=np.deg2rad([0.] * 7),
        left_arm=np.deg2rad([0.] * 7)
    ),
    "T5": Pose(
        toros=np.deg2rad([0.] * 5),
        right_arm=np.deg2rad([0.] * 7),
        left_arm=np.deg2rad([0.] * 7)
    ),
    "M": Pose(
        toros=np.deg2rad([0.] * 6),
        right_arm=np.deg2rad([0.] * 7),
        left_arm=np.deg2rad([0.] * 7)
    )
}
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

    def handler(signum, frame):
        robot.power_off("12v")
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

    # ===== SETUP MASTER ARM =====
    def master_arm_control_loop(state: rby.upc.MasterArm.State):
        with np.printoptions(suppress=True, precision=3, linewidth=300):
            print(f"--- {datetime.datetime.now().time()} ---")
            print(f"q: {state.q_joint}")
            print(f"g: {state.gravity_term}")
            print(
                f"right: {state.button_right.button}, left: {state.button_left.button}"
            )

        input = rby.upc.MasterArm.ControlInput()

        input.target_operation_mode.fill(rby.DynamixelBus.CurrentControlMode)
        input.target_torque = state.gravity_term

        return input

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
