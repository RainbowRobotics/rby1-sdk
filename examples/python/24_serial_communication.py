import rby1_sdk as rby
import argparse


def main(address, device_path):
    robot = rby.create_robot_a(address)
    robot.connect()

    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)

    dev = robot.open_serial_stream(device_path, 115200)
    connected = dev.connect(verbose=True)

    if not connected:
        print("Failed to connect")
        exit(1)

    print("Listening for incoming data...\n")
    dev.set_read_callback(lambda data: print(data, end="", flush=True))

    try:
        while True:
            c = input("\nEnter data to send (or type 'exit' to quit): ")
            if c.strip().lower() == "exit":
                break
            dev.write(c)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    print("Disconnecting serial stream...")
    dev.disconnect()
    print("Disconnected. Exiting.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="24_serial_communication")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--device_path", type=str, default="/dev/ttyUSB1", help="Serial port in RPC")
    args = parser.parse_args()

    main(address=args.address, device_path=args.device_path)
