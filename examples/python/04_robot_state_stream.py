import time

import rby1_sdk
import argparse
import numpy as np


def callback(robot_state, control_manager_state: rby1_sdk.ControlManagerState):
    np.set_printoptions(precision=3, suppress=True, floatmode="fixed")

    print(robot_state)
    print(control_manager_state)


# or
# def callback(robot_state):
#     print(robot_state)


def main(address, model, power):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on(power):
        rv = robot.power_on(power)
        if not rv:
            print("Failed to power on")
            exit(1)

    robot.start_state_update(callback, rate=10)  # Hz
    try:
        time.sleep(100)
    except KeyboardInterrupt:
        print("Stopping state update...")
    finally:
        robot.stop_state_update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="04_robot_state_stream")
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
    args = parser.parse_args()

    main(address=args.address, model=args.model, power=args.power)
