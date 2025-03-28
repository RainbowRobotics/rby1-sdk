import curses
import rby1_sdk
import argparse
import time
import sys
import threading


def wifi_setup_ui(stdscr, networks):
    curses.curs_set(1)
    selected_idx = 0
    input_text = ""
    current_step = "network"
    selected_network = None
    password = ""
    use_dhcp = True
    ip_address, gateway, dns = "", "", []

    while True:
        stdscr.clear()

        if current_step == "network":
            stdscr.addstr("Select a WiFi network:\n", curses.A_BOLD)
            for idx, network in enumerate(networks):
                prefix = "> " if idx == selected_idx else "  "
                line = f"{prefix}{network.ssid} (Signal: {network.signal_strength}, Secured: {network.secured})\n"
                if idx == selected_idx:
                    stdscr.addstr(line, curses.A_REVERSE)
                else:
                    stdscr.addstr(line)
        elif current_step == "password":
            stdscr.addstr(f"Selected: {selected_network.ssid}\n", curses.A_BOLD)
            stdscr.addstr("Enter password (press Enter to confirm):\n")
            stdscr.addstr(f"{'*' * len(input_text)}", curses.A_REVERSE)  # 비밀번호 마스킹
        elif current_step == "dhcp":
            stdscr.addstr(f"Selected: {selected_network.ssid}\n", curses.A_BOLD)
            stdscr.addstr("Use DHCP? (y/n): ")
            stdscr.addstr(f"{input_text}", curses.A_REVERSE)
        elif current_step == "manual_ip":
            stdscr.addstr(f"Selected: {selected_network.ssid}\n", curses.A_BOLD)
            stdscr.addstr("Enter IP address: ")
            stdscr.addstr(f"{input_text}", curses.A_REVERSE)
        elif current_step == "manual_gateway":
            stdscr.addstr(f"Selected: {selected_network.ssid}\n", curses.A_BOLD)
            stdscr.addstr(f"IP: {ip_address}\n")
            stdscr.addstr("Enter Gateway: ")
            stdscr.addstr(f"{input_text}", curses.A_REVERSE)
        elif current_step == "manual_dns":
            stdscr.addstr(f"Selected: {selected_network.ssid}\n", curses.A_BOLD)
            stdscr.addstr(f"IP: {ip_address}, Gateway: {gateway}\n")
            stdscr.addstr("Enter DNS (comma separated): ")
            stdscr.addstr(f"{input_text}", curses.A_REVERSE)
        elif current_step == "confirm":
            stdscr.addstr(f"Selected: {selected_network.ssid}\n", curses.A_BOLD)
            stdscr.addstr(f"Password: {'*' * len(password)}\n" if selected_network.secured else "")
            stdscr.addstr(f"Use DHCP: {use_dhcp}\n")
            if not use_dhcp:
                stdscr.addstr(f"IP: {ip_address}, Gateway: {gateway}, DNS: {', '.join(dns)}\n")
            stdscr.addstr("\nPress Enter to connect or 'q' to cancel.")

        stdscr.refresh()

        key = stdscr.getch()

        if current_step == "network":
            if key == curses.KEY_UP and selected_idx > 0:
                selected_idx -= 1
            elif key == curses.KEY_DOWN and selected_idx < len(networks) - 1:
                selected_idx += 1
            elif key in [curses.KEY_ENTER, 10, 13]:
                selected_network = networks[selected_idx]
                if selected_network.secured:
                    current_step = "password"
                    input_text = ""
                else:
                    current_step = "dhcp"
                    input_text = ""
        elif current_step in ["password", "dhcp", "manual_ip", "manual_gateway", "manual_dns"]:
            if key in [curses.KEY_BACKSPACE, 127]:
                input_text = input_text[:-1]
            elif key in [curses.KEY_ENTER, 10, 13]:
                if current_step == "password":
                    password = input_text
                    current_step = "dhcp"
                    input_text = ""
                elif current_step == "dhcp":
                    use_dhcp = input_text.lower() == "y"
                    if use_dhcp:
                        current_step = "confirm"
                    else:
                        current_step = "manual_ip"
                    input_text = ""
                elif current_step == "manual_ip":
                    ip_address = input_text
                    current_step = "manual_gateway"
                    input_text = ""
                elif current_step == "manual_gateway":
                    gateway = input_text
                    current_step = "manual_dns"
                    input_text = ""
                elif current_step == "manual_dns":
                    dns = input_text.split(",")
                    current_step = "confirm"
                    input_text = ""
            elif 32 <= key <= 126:  # 일반적인 키 입력 (ASCII)
                input_text += chr(key)
        elif current_step == "confirm":
            if key in [curses.KEY_ENTER, 10, 13]:
                return selected_network.ssid, password, use_dhcp, ip_address, gateway, dns
            elif key == ord("q"):
                return None, None, None, None, None, None


def connect_wifi_interactive(robot, networks):
    ssid, password, use_dhcp, ip_address, gateway, dns = curses.wrapper(wifi_setup_ui, networks)

    if ssid is None:
        print("WiFi connection canceled.")
        return

    print(f"Wait for connecting to {ssid}...")

    success = robot.connect_wifi(
        ssid=ssid,
        password=password,
        use_dhcp=use_dhcp,
        ip_address=ip_address,
        gateway=gateway,
        dns=dns
    )

    if success:
        print(f"✅ Successfully connected to {ssid}!")
    else:
        print(f"❌ Failed to connect to {ssid}. Please try again.")


def loading_animation(event):
    chars = ["|", "/", "-", "\\"]
    idx = 0
    while not event.is_set():
        sys.stdout.write(f"\rScanning Wi-Fi {chars[idx]} ")
        sys.stdout.flush()
        idx = (idx + 1) % len(chars)
        time.sleep(0.2)
    sys.stdout.write("\rScanning completed!     \n")


def main(address, model):
    robot = rby1_sdk.create_robot(address, model)
    robot.connect()
    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)

    event = threading.Event()
    animation_thread = threading.Thread(target=loading_animation, args=(event,))
    animation_thread.start()

    wifi_networks = robot.scan_wifi()

    event.set()
    animation_thread.join()

    if not wifi_networks:
        print("No WiFi networks found.")
        exit(1)

    print("\nAvailable Wi-Fi networks:")
    for idx, network in enumerate(wifi_networks):
        print(f"{idx + 1}. {network.ssid} (Signal: {network.signal_strength}, Secured: {network.secured})")

    print()

    connect_wifi_interactive(robot, wifi_networks)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="20_wifi")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--model", type=str, default="a", help="Robot Model Name (default: 'a')")
    args = parser.parse_args()

    main(args.address, args.model)
