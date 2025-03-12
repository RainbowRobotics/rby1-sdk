import rby1_sdk
import argparse


def main(address, model, power_device):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="02_power")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--device",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model, power_device=args.device)
