import rby1_sdk as rby
from helper import *
import numpy as np
import time
import logging
import argparse

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model, power, servo):
    logging.info("===== Cartesian Command Stream Example =====")

    robot = initialize_robot(address, model, power, servo)

    robot.set_parameter("cartesian_command.cutoff_frequency", "5")

    model = robot.model()
    torso_dof = len(model.torso_idx)
    right_arm_dof = len(model.right_arm_idx)
    left_arm_dof = len(model.left_arm_idx)
    movej(
        robot,
        np.zeros(torso_dof),
        np.zeros(right_arm_dof),
        np.zeros(left_arm_dof),
        minimum_time=3,
    )
    movej(
        robot,
        torso=None if model.model_name == "UB" else np.deg2rad(
            [0.0, 45.0, -90.0, 45.0, 0.0, 0.0]
            if model.model_name != "T5"
            else [45.0, -90.0, 45.0, 0.0, 0.0]
        ),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        minimum_time=3,
    )

    dyn_robot = robot.get_dynamics()
    dyn_state = dyn_robot.make_state(["base", "ee_right"], model.robot_joint_names)
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
                )
            )
        )
        return rc

    stream = robot.create_command_stream()
    for pos_diff in [
        [0, 0, -0.08],
        [0, 0, 0.08],
        [0.1, -0.16, 0.08],
        [0.1, -0.16, -0.08],
        [0, 0, -0.08],
        [0, 0, 0.08],
        [0, -0.2, 0.1],
        [0, -0.2, -0.1],
        [-0.05, 0.1, -0.08],
        [-0.05, 0.1, 0.08],
        [0, -0.2, 0.1],
        [0, -0.2, -0.1],
        [0, 0, -0.08],
        [0, 0, 0.08],
        [0, -0.16, 0.08],
        [0, -0.16, -0.08],
        [0, 0, -0.08],
        [0, 0, 0.08],
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
    parser = argparse.ArgumentParser(description="07_impedance_control")
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
