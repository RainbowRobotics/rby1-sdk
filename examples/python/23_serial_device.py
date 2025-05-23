import rby1_sdk as rby
import argparse


def main(address):
    robot = rby.create_robot_a(address)
    robot.connect()

    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)

    serial_devices = robot.get_serial_device_list()
    for [i, device] in enumerate(serial_devices):
        print(f"[{i}] {device}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="23_serial_device")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
