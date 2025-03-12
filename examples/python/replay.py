import rby1_sdk
import numpy as np
import time
import sys
import argparse


def pre_processing(address):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()

    if not robot.is_power_on(".*"):
        print("Power is currently OFF. Attempting to power on...")
        if not robot.power_on(".*"):
            print("Error: Failed to power on the robot.")
            sys.exit(1)
        print("Robot powered on successfully.")
    else:
        print("Power is already ON.")

    if not robot.is_servo_on(".*"):
        print("Servo is currently OFF. Attempting to activate servo...")
        if not robot.servo_on(".*"):
            print("Error: Failed to activate servo.")
            sys.exit(1)
        print("Servo activated successfully.")
    else:
        print("Servo is already ON.")

    control_manager_state = robot.get_control_manager_state()

    if (
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault
        or control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault
    ):
        if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
            print("Warning: Detected a Major Fault in the Control Manager.")
        else:
            print("Warning: Detected a Minor Fault in the Control Manager.")

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)

    return robot


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    args = parser.parse_args()
    robot = pre_processing(args.address)
    stream = robot.create_command_stream(10)

    saved_traj = np.load("recorded.npz", allow_pickle=True)["data"]
    for i, traj_step in enumerate(saved_traj):
        # First step: dt = 5, others: dt = 0.1
        dt = 0.1 if i > 0 else 5
        traj_step_body = traj_step[2:-2]
        rc = rby1_sdk.RobotCommandBuilder().set_command(
            rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
                rby1_sdk.JointPositionCommandBuilder()
                .set_command_header(
                    rby1_sdk.CommandHeaderBuilder().set_control_hold_time(1)
                )
                .set_minimum_time(dt)
                .set_position(traj_step_body)
            )
        )
        rv = stream.send_command(rc)

        time.sleep(dt * 0.95)
