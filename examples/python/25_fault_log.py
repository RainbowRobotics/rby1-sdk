import rby1_sdk as rby
import argparse


def main(address, model):
    robot = rby.create_robot(address, model)
    robot.connect()

    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)

    fault_log_list = robot.get_fault_log_list()
    print(f"Fault log list: {fault_log_list}")

    if len(fault_log_list) == 0:
        print("No fault logs found.")
        return

    print(f"Downloading fault log: {fault_log_list[0]}")
    with open("fault.csv", "wb") as f:
        robot.download_file(fault_log_list[0], f)
    print("Fault log downloaded successfully.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="25_fault_log")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
