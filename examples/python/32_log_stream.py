import rby1_sdk
import time
import argparse


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)

    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
        
    def cb(logs):
        for log in logs:
            if int(log.level) >= int(rby1_sdk.Log.Level.Info):
                print(f"{log}")
        
    robot.sync_time()
    robot.start_log_stream(cb, 1)
    time.sleep(60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="31_log")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
