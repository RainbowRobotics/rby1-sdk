import rby1_sdk
import argparse


def main(address):
    robot = rby1_sdk.create_robot(address)

    # Model
    m = robot.model()

    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
    robot_info = robot.get_robot_info()
    print(f"Hello, RB-Y1! (Robot model name: {m.model_name})")
    print(robot_info)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="01_hello_rby1")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
