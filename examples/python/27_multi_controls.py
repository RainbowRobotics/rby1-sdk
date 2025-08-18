import rby1_sdk as rby
from helper import *
import numpy as np
import time
import argparse
import logging
from dataclasses import dataclass

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@dataclass
class Pose:
    torso: np.typing.NDArray
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


READY_POSE = {
    "A": Pose(
        torso=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    ),
    "T5": Pose(
        torso=np.deg2rad([45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    ),
    "M": Pose(
        torso=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    ),
}


def main(address, model, power, servo):
    robot: rby.Robot_A = initialize_robot(address, model, power, servo)
    model: rby.Model_A = robot.model()
    minimum_time = 2
    robot_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_position(np.zeros(len(model.body_idx)))
                .set_minimum_time(minimum_time)
            )
        ),
        priority=1,
    )
    time.sleep(1)

    right_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(READY_POSE[model.model_name].right_arm)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        priority=10,
    )
    time.sleep(1)

    left_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(READY_POSE[model.model_name].left_arm)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        priority=1,
    )

    print(f"{robot_handle.get().finish_code = }")
    print(f"{right_handle.get().finish_code = }")
    print(f"{left_handle.get().finish_code = }")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="27_multi_controls")
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
