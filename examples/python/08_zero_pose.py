import time
import rby1_sdk
from rby1_sdk import *
import argparse
import numpy as np
import sys

MINIMUM_TIME = 10

def zero_pose(robot):
    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(
                JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_waist)
            )
            .set_right_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_right_arm)
            )
            .set_left_arm_command(
                JointPositionCommandBuilder()
                .set_minimum_time(MINIMUM_TIME)
                .set_position(q_joint_left_arm)
            )
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct zero pose motion.")
        return 1

    return 0

def main(address, power_device, servo):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()
    
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
        
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)
    
    if not robot.is_servo_on(servo):
        rv = robot.servo_on(servo)
        if not rv:
            print("Fail to servo on")
            exit(1)

    control_manager_state = robot.get_control_manager_state()            
    if (control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault or \
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault):

        if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
            print("Warning: Detected a Major Fault in the Control Manager.")
        else:
            print("Warning: Detected a Minor Fault in the Control Manager.")

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    print("Control Manager state is normal. No faults detected.")

    print("Enabling the Control Manager...")
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled successfully.")

    if not zero_pose(robot):
        print("Finish Joint")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="07_impedance_control")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo name regex pattern (default: '.*')")
    args = parser.parse_args()

    main(address=args.address,
         power_device=args.device,
         servo = args.servo)
