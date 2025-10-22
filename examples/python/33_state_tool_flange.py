import rby1_sdk
import time
import argparse


def callback(rs):
    print("---")
    print(f"{rs.tool_flange_right = }")
    print(f"{rs.tool_flange_left = }")


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on('48v'):
        rv = robot.power_on('48v')
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
    parser = argparse.ArgumentParser(description="33_state_tool_flange")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
