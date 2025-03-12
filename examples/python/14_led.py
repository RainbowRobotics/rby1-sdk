import rby1_sdk
import argparse
import time


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Failed to connect robot")
        exit(1)

    print("#1 Red 0.5s")
    robot.set_led_color(
        rby1_sdk.Color(255, 0, 0), duration=0.5, transition_time=0.1, blinking=False
    )
    time.sleep(0.5)

    print("#2 Green 0.5s")
    robot.set_led_color(
        rby1_sdk.Color(0, 255, 0), duration=0.5, transition_time=0.1, blinking=False
    )
    time.sleep(0.5)

    print("#3 Blue 0.5s")
    robot.set_led_color(
        rby1_sdk.Color(0, 0, 255), duration=0.5, transition_time=0.1, blinking=False
    )
    time.sleep(0.5)

    print("#4 White Blinking 1s")
    robot.set_led_color(
        rby1_sdk.Color(200, 200, 200),
        duration=1,
        transition_time=0.1,
        blinking=True,
        blinking_freq=4,
    )
    time.sleep(1)

    # Rainbow colors
    rainbow_colors = [
        rby1_sdk.Color(255, 0, 0),  # Red
        rby1_sdk.Color(255, 127, 0),  # Orange
        rby1_sdk.Color(255, 255, 0),  # Yellow
        rby1_sdk.Color(0, 255, 0),  # Green
        rby1_sdk.Color(0, 0, 255),  # Blue
        rby1_sdk.Color(75, 0, 130),  # Indigo
        rby1_sdk.Color(148, 0, 211),  # Violet
    ]

    print("#5 Rainbow")
    for color in rainbow_colors:
        robot.set_led_color(color, duration=0.5, transition_time=0.2, blinking=False)
        time.sleep(0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="14_led")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
