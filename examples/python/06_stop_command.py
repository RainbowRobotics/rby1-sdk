import time

import rby1_sdk
import argparse


def callback(robot_state: rby1_sdk.RobotState_A):
    if robot_state.collisions:
        collision = robot_state.collisions[0]
        if collision.distance < 0:
            print(">>>>> Collision detected!")
        print(collision)


def main(address):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    robot.send_command(rby1_sdk.RobotCommandBuilder().set_command(rby1_sdk.WholeBodyCommandBuilder().set_command(rby1_sdk.StopCommandBuilder())), priority=99).get()
    robot.power_off(".*")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="05_collisions")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
