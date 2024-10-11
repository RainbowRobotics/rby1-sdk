import time
import rby1_sdk
from rby1_sdk import *
import argparse
import numpy as np
import math
import sys

CONTROL_HOLD_TIME = 300
MINIMUM_TIME = 5

angular_velocity_limit = np.pi * 1.5
linear_velocity_limit = 1.5
acceleration_limit = 1.0
stop_orientation_tracking_error = 1e-5
stop_position_tracking_error = 1e-5

def example_joint_position_command_1(robot):
    print("joint position command example 1")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.deg2rad(np.array([30, -10, 0, -100, 0, 20, 0]))
    q_joint_left_arm = np.deg2rad(np.array([30, 10, 0, -100, 0, 20, 0]))

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
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0

def example_cartesian_command_1(robot):
    print("Cartesian command example 1")

    # Initialize transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    # Define transformation matrices
    angle = np.pi / 6
    T_torso[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
                                [0, 1, 0],
                                [-np.sin(angle), 0, np.cos(angle)]])
    T_torso[:3, 3] = [0.1, 0, 1.2]

    angle = -np.pi / 4
    T_right[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
                                [0, 1, 0],
                                [-np.sin(angle), 0, np.cos(angle)]])
    T_right[:3, 3] = [0.4, -0.4, 0]

    T_left[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
                               [0, 1, 0],
                               [-np.sin(angle), 0, np.cos(angle)]])
    T_left[:3, 3] = [0.4, 0.4, 0]

    # Build command
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            CartesianCommandBuilder()
            .add_target("base", "link_torso_5", T_torso, linear_velocity_limit, angular_velocity_limit,
                        acceleration_limit)
            .add_target("link_torso_5", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit,
                        acceleration_limit)
            .add_target("link_torso_5", "ee_left", T_left, linear_velocity_limit, angular_velocity_limit,
                        acceleration_limit)
            .set_stop_position_tracking_error(stop_position_tracking_error)
            .set_stop_orientation_tracking_error(stop_orientation_tracking_error)
            .set_minimum_time(MINIMUM_TIME)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0



def example_impedance_control_command_1(robot):
    print("Impedance control command example 1")

    # Initialize transformation matrices
    T_torso = np.eye(4)
    T_right = np.eye(4)
    T_left = np.eye(4)

    # Define transformation matrices
    angle = np.pi / 6
    T_torso[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
                                [0, 1, 0],
                                [-np.sin(angle), 0, np.cos(angle)]])
    T_torso[:3, 3] = [0.1, 0, 1.2]

    angle = -np.pi / 4
    T_right[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
                                [0, 1, 0],
                                [-np.sin(angle), 0, np.cos(angle)]])
    T_right[:3, 3] = [0.4, -0.4, 0]

    T_left[:3, :3] = np.array([[np.cos(angle), 0, np.sin(angle)],
                               [0, 1, 0],
                               [-np.sin(angle), 0, np.cos(angle)]])
    T_left[:3, 3] = [0.4, 0.4, 0]

    # Build commands
    torso_command = rby1_sdk.ImpedanceControlCommandBuilder().set_command_header(
        rby1_sdk.CommandHeaderBuilder().set_control_hold_time(CONTROL_HOLD_TIME)
    ).set_reference_link_name("base").set_link_name("link_torso_5").set_translation_weight(
        [1000, 1000, 1000]).set_rotation_weight([100, 100, 100]).set_transformation(T_torso)

    right_arm_command = rby1_sdk.ImpedanceControlCommandBuilder().set_command_header(
        rby1_sdk.CommandHeaderBuilder().set_control_hold_time(CONTROL_HOLD_TIME)
    ).set_reference_link_name("link_torso_5").set_link_name("ee_right").set_translation_weight(
        [3000, 3000, 0]).set_rotation_weight([50, 50, 50]).set_transformation(T_right)

    left_arm_command = rby1_sdk.ImpedanceControlCommandBuilder().set_command_header(
        rby1_sdk.CommandHeaderBuilder().set_control_hold_time(CONTROL_HOLD_TIME)
    ).set_reference_link_name("link_torso_5").set_link_name("ee_left").set_translation_weight(
        [1000, 1000, 1000]).set_rotation_weight([50, 50, 50]).set_transformation(T_left)

    # Send command
    rc = rby1_sdk.RobotCommandBuilder().set_command(
        rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
            rby1_sdk.BodyComponentBasedCommandBuilder()
            .set_right_arm_command(right_arm_command)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct impedance motion.")
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

    robot.set_parameter("default.acceleration_limit_scaling", "0.8")
    robot.set_parameter("joint_position_command.cutoff_frequency", "5")
    robot.set_parameter("cartesian_command.cutoff_frequency", "5")
    robot.set_parameter("default.linear_acceleration_limit", "5")

    if not example_joint_position_command_1(robot):
        print("Finish Joint")
    if not example_cartesian_command_1(robot):
        print("Finish Impedance")        
    if not example_impedance_control_command_1(robot):
        print("Finish Impedance")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="07_impedance_control")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo name regex pattern (default: '.*')")
    args = parser.parse_args()

    main(address=args.address,
         power_device=args.device,
         servo = args.servo)
