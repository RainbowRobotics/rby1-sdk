import rby1_sdk
import argparse
import numpy as np

np.set_printoptions(precision=3, suppress=True, floatmode="fixed")


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

    robot_state = robot.get_state()
    with rby1_sdk.printoptions(linewidth=10**9, multiline_repr=True):
        print(repr(robot_state))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="03_robot_state")
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
