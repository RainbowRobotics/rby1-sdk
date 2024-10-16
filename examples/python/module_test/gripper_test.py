import time
import os, sys
# sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'third-party', 'DynamixelSDK', 'python', 'src'))
import threading
import dynamixel_sdk as dxl
import numpy as np

import rby1_sdk
import argparse

# Dynamixel Control Table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_CURRENT = 102
ADDR_OPERATING_MODE = 11

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
BAUDRATE = 2000000
DEVICENAME = '/dev/rby1_gripper'  # Check device name

# Define constants
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
CURRENT_CONTROL_MODE = 0
D2R = np.pi / 180
R2D = 180 / np.pi

isInitFinish = False

def torque_enable(port_handler, packet_handler, dxl_id, enable):
    packet_handler.write1ByteTxOnly(port_handler, dxl_id, ADDR_TORQUE_ENABLE, enable)
    time.sleep(0.5)

def read_operation_mode(port_handler, packet_handler, dxl_id):
    dxl_comm_result, operation_mode, dxl_error = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE)
    if dxl_comm_result == dxl.COMM_SUCCESS:
        return operation_mode
    else:
        return None

def read_encoder(port_handler, packet_handler, dxl_id):
    dxl_comm_result, position, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result == dxl.COMM_SUCCESS:
        return (position / 4096.0) * 2 * np.pi  # radian
    else:
        return None

def send_operation_mode(port_handler, packet_handler, dxl_id, mode):
    packet_handler.write1ByteTxOnly(port_handler, dxl_id, ADDR_OPERATING_MODE, mode)
    time.sleep(0.5)

def send_current(port_handler, packet_handler, dxl_id, current):
    current_value = int(current / 2.69 * 1000)
    packet_handler.write2ByteTxOnly(port_handler, dxl_id, ADDR_GOAL_CURRENT, current_value)

def control_loop_for_gripper(port_handler, packet_handler, active_ids):
    q_min_max_vector = [np.zeros(2) for _ in range(2)]
    
    cnt = 0
    while True:
        is_init = True
        for i, dxl_id in enumerate(active_ids):
            # Ensure the control mode is in current control mode
            while True:
                operation_mode = read_operation_mode(port_handler, packet_handler, dxl_id)
                if operation_mode is not None:
                    if operation_mode != CURRENT_CONTROL_MODE:
                        torque_enable(port_handler, packet_handler, dxl_id, TORQUE_DISABLE)
                        send_operation_mode(port_handler, packet_handler, dxl_id, CURRENT_CONTROL_MODE)
                        torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                    else:
                        break
                time.sleep(0.1)

            # Enable the torque if disabled
            while True:
                _, torque_enable_val, _ = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE)
                if torque_enable_val == 0:
                    torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                else:
                    break
                time.sleep(0.1)

            # Control current and read encoder
            q = read_encoder(port_handler, packet_handler, dxl_id)
            if q is not None:
                q_min_max_vector[i][cnt % 2] = q

            if cnt % 2 == 0:
                send_current(port_handler, packet_handler, dxl_id, 0.5)
            else:
                send_current(port_handler, packet_handler, dxl_id, -0.5)

            # Check if initialization is complete
            if np.abs(q_min_max_vector[i][1] - q_min_max_vector[i][0]) * R2D < 540 * 0.9:
                is_init = False

            print(f"id: {dxl_id}")
            print(f"q_min_max_vector : {q_min_max_vector[i][0] * R2D}")
            print(f"q_min_max_vector : {q_min_max_vector[i][1] * R2D}")
            print(f"is_init: {is_init}")
        
        if is_init:
            for i, dxl_id in enumerate(active_ids):
                q_min_max_vector[i].sort()
                send_current(port_handler, packet_handler, dxl_id, 0)
            time.sleep(3)
            global isInitFinish
            isInitFinish = True
            break

        cnt += 1
        time.sleep(3)

def pre_process(address):
    print("Attempting to connect to the robot...")

    robot = rby1_sdk.create_robot_a(address)

    if not robot.connect():
        print("Error: Unable to establish connection to the robot at")
        sys.exit(1)

    print("Successfully connected to the robot")
    
    if not robot.is_power_on('.*'):
        if not robot.power_on('.*'):
            print("Error")
            return 1
    else:
        print("Power is already ON")
    
    if robot.is_power_on('48v'):
        robot.set_tool_flange_output_voltage('right', 12)
        robot.set_tool_flange_output_voltage('left', 12)
        print('Attempting to 12V power on for gripper')
        time.sleep(1)

def main(address):
    pre_process(address)
    # Initialize PortHandler and PacketHandler
    port_handler_gripper = dxl.PortHandler(DEVICENAME)
    packet_handler_gripper = dxl.PacketHandler(PROTOCOL_VERSION)

    if not port_handler_gripper.openPort():
        print("Failed to open port")
        quit()

    if not port_handler_gripper.setBaudRate(BAUDRATE):
        print("Failed to set baud rate")
        quit()

    active_ids_gripper = []
    for dxl_id in range(2):
        dxl_comm_result, _, _ = packet_handler_gripper.ping(port_handler_gripper, dxl_id)
        if dxl_comm_result == dxl.COMM_SUCCESS:
            print(f"Dynamixel ID {dxl_id} is active")
            active_ids_gripper.append(dxl_id)
        else:
            print(f"Dynamixel ID {dxl_id} is not active")

    if len(active_ids_gripper) != 2:
        print("Unable to ping all devices for grippers")
        quit()

    for dxl_id in active_ids_gripper:
        send_operation_mode(port_handler_gripper, packet_handler_gripper, dxl_id, CURRENT_CONTROL_MODE)
        torque_enable(port_handler_gripper, packet_handler_gripper, dxl_id, TORQUE_ENABLE)

    control_thread = threading.Thread(target=control_loop_for_gripper, args=(port_handler_gripper, packet_handler_gripper, active_ids_gripper))
    control_thread.start()

    while not isInitFinish:
        time.sleep(0.005)

    control_thread.join()
    port_handler_gripper.closePort()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="07_impedance_control")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
