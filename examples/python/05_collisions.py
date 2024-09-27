import time

import rby1_sdk
import argparse


def callback(robot_state: rby1_sdk.RobotState_A):
    if robot_state.collisions:
        collision = robot_state.collisions[0]
        if collision.distance < 0:
            print(">>>>> Collision detected!")
        print(collision)


def main(address, power_device):
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

    robot.start_state_update(callback,
                             rate=10  # Hz
                             )
    try:
        time.sleep(100)
    except KeyboardInterrupt:
        print("Stopping state update...")
    finally:
        robot.stop_state_update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="05_collisions")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    args = parser.parse_args()

    main(address=args.address,
         power_device=args.device)
