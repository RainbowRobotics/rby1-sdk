import rby1_sdk
import argparse


def main(address, model, num_entries):
    robot = rby1_sdk.create_robot(address, model)

    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)

    logs = robot.get_last_log(num_entries)
    for log in logs:
        print(f"{log}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="31_log")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "-n",
        "--num-entries",
        type=int,
        default=10,
        help="Number of log entries to retrieve (default: 10)",
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model, num_entries=args.num_entries)
