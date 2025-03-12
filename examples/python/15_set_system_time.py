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

    dt, localtime_string = robot.get_system_time()
    print(f"Robot System Time: {dt}, {localtime_string}")

    print("# Change To TimeZone(EST)")
    print(
        f" -- {'SUCCESS' if robot.set_system_time(datetime.now(), 'EST') else 'FAIL'}"
    )
    time.sleep(0.5)
    print(f"Robot System Time: {robot.get_system_time()}")

    print("# Change to TimeZone")
    dt = dt.astimezone(ZoneInfo("Asia/Seoul"))
    print(f" -- {'SUCCESS' if robot.set_system_time(dt) else 'FAIL'}")
    time.sleep(0.5)  # Need for changing timezone
    print(f"Robot System Time: {robot.get_system_time()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="15_set_system_time")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
