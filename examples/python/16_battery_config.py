import rby1_sdk
import argparse
import time
from datetime import datetime
from zoneinfo import ZoneInfo


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Failed to connect robot")
        exit(1)

    print("# Set battery level as 50")
    print(f" -- {'SUCCESS' if robot.set_battery_level(50) else 'FAIL'}")

    time.sleep(1)

    print("# Reset battery configuration")
    print(f" -- {'SUCCESS' if robot.reset_battery_config() else 'FAIL'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="16_battery_config")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
