import rby1_sdk
import argparse


def main(address):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()
    if not robot.is_connected():
        print("Failed to connect robot")
        exit(1)

    robot.set_led_color(rby1_sdk.Color(0, 255, 0), 1, False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="13_robot_model")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
