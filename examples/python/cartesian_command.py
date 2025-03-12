import rby1_sdk as rby
import numpy as np
import argparse


def main(address, model):
    robot = rby.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("ERROR: Robot connection failed.")
        exit(1)
    model = robot.model()

    if not robot.is_power_on(".*"):
        rv = robot.power_on(".*")
        if not rv:
            print("ERROR: Failed to power on.")
            exit(1)

    if not robot.is_servo_on(".*"):
        rv = robot.servo_on(".*")
        if not rv:
            print("ERROR: Failed to servo on.")
            exit(1)

    robot.set_parameter(
        "default.cartesian_command.stop_joint_position_tracking_error", "0.001"
    )

    robot.reset_fault_control_manager()
    robot.enable_control_manager()

    def move_j(position, minimum_time, command_type="jpc"):
        if command_type == "jpc":
            rc = (
                rby.JointPositionCommandBuilder()
                .set_minimum_time(minimum_time)
                .set_position(position)
            )
        else:
            rc = rby.CartesianCommandBuilder().set_minimum_time(minimum_time)
            for i, idx in enumerate(model.body_idx):
                rc.add_joint_position_target(model.robot_joint_names[idx], position[i])

        rv = robot.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(rc)
            )
        ).get()
        return rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok

    zero = [0] * len(model.body_idx)
    ready = np.deg2rad(
        [0.0, 45.0, -90.0, 45.0, 0.0, 0.0]
        + [0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]
        + [0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]
    )

    print(f"Motion result: {move_j(zero, 1, command_type='jpc')}")
    print(f"Motion result: {move_j(ready, 1, command_type='jpc')}")

    print(f"Motion result: {move_j(zero, 1, command_type='cc')}")
    print(f"Motion result: {move_j(ready, 1, command_type='cc')}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="04_robot_state_stream")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
