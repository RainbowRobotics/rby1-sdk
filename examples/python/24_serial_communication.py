import time
import rby1_sdk as rby
import argparse
import threading


def main(address, device_path, baudrate):
    robot = rby.create_robot_a(address)
    robot.connect()

    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)

    dev = robot.open_serial_stream(device_path, baudrate)
    connected = dev.connect(verbose=True)

    if not connected:
        print("Failed to connect")
        exit(1)

    print("Listening for incoming data...\n")

    def print_hex(data):
        hex_string = "".join(f"{int(ch):02X}" for ch in data)
        print(f"<< {hex_string}", flush=True)

    dev.set_read_callback(print_hex)

    def user_input_loop():
        try:
            while True:
                c = input("\n>> Enter hex data to send (e.g., 'B7B8010401C5C6'), or type 'exit': ")
                if c.strip().lower() == "exit":
                    break
                try:
                    hex_str = c.strip()
                    if len(hex_str) % 2 != 0:
                        print("Hex string must have even number of characters.")
                        continue
                    byte_values = bytes.fromhex(hex_str)
                    dev.write(byte_values)
                except ValueError:
                    print("Invalid hex input. Please enter valid hexadecimal characters (0-9, A-F).")
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            print("Disconnecting serial stream...")
            dev.disconnect()
            print("Disconnected. Exiting.")

    # 스레드 시작
    input_thread = threading.Thread(target=user_input_loop)
    input_thread.start()

    input_thread.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="24_serial_communication")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--device_path", type=str, default="/dev/ttyUSB1", help="Serial port in RPC")
    parser.add_argument("--baudrate", type=int, default=19200, help="Baudrate")
    args = parser.parse_args()

    main(address=args.address, device_path=args.device_path, baudrate=args.baudrate)
