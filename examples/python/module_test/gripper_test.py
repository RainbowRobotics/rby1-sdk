import time
import os, sys
# sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'third-party', 'DynamixelSDK', 'python', 'src'))
import threading
import dynamixel_sdk as dxl
import ctypes
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

DEVICENAME = "/dev/rby1_gripper"

# Define constants
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
CURRENT_CONTROL_MODE = 0
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3

MIN_INDEX = 0
MAX_INDEX = 1

DEG_PER_PULSE = 360/4096 #0.088

REV_RANGE_MIN = -256
REV_RANGE_MAX = 256

PULSE_RANGE_MIN = REV_RANGE_MIN*360/DEG_PER_PULSE
PULSE_RANGE_MAX = REV_RANGE_MAX*360/DEG_PER_PULSE

IS_INIT_FINISH = False

def read_operation_mode(port_handler, packet_handler, dxl_id):
    # data_read, result, error
    operation_mode, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE)
    if dxl_comm_result == dxl.COMM_SUCCESS:
        return operation_mode
    else:
        return None

def read_encoder(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id):
    
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_POSITION)
    dxl_present_position = ctypes.c_int32(dxl_present_position).value
    print(f"id: {dxl_id}, Encoder(pulse): {dxl_present_position}, Encoder(deg): {dxl_present_position*DEG_PER_PULSE}, dxl_error: {dxl_error}")
    if dxl_comm_result == dxl.COMM_SUCCESS:
        return dxl_present_position*DEG_PER_PULSE
    else:
        print("NONE NONE NONE NONE")
        return None

def send_torque_enable(port_handler, packet_handler, dxl_id, enable):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, enable)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
    else:
        print(f"ID {dxl_id} has been successfully connected")
    time.sleep(1)

def send_operation_mode(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id, mode):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, mode)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
    else:
        print(f"(ID: {dxl_id}, Mode: {mode}) has been successfully changed")
    time.sleep(0.5)

def send_current(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id, current):
    current_value = int(current / 2.69 * 1000)
    # packet_handler.write2ByteTxOnly(port_handler, dxl_id, ADDR_GOAL_CURRENT, current_value)
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, ADDR_GOAL_CURRENT, current_value)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))

def control_loop_for_gripper(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, active_ids):
    q_min_max_vector = [np.zeros(2) for _ in range(2)]

    for dxl_id in active_ids:
        q_min_max_vector[dxl_id][MIN_INDEX] = REV_RANGE_MAX*360
        q_min_max_vector[dxl_id][MAX_INDEX] = REV_RANGE_MIN*360

    cnt = 0
    while True:
        is_init = True
        for dxl_id in active_ids:
            # Ensure the control mode is in current control mode
            
            while True:
                operation_mode = read_operation_mode(port_handler, packet_handler, dxl_id)
                if operation_mode is not None:
                    if operation_mode != CURRENT_CONTROL_MODE:
                        send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_DISABLE)
                        send_operation_mode(port_handler, packet_handler, dxl_id, CURRENT_CONTROL_MODE)
                        send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                    else:
                        break
                time.sleep(0.1)
            
            # Enable the torque if disabled
            while True:
                torque_enable_val, result, _ = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE)
                if torque_enable_val == 0:
                    send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                else:
                    break
                time.sleep(0.1)

            q = read_encoder(port_handler, packet_handler, dxl_id)
            if q is not None:
                if q_min_max_vector[dxl_id][MIN_INDEX] > q :
                    q_min_max_vector[dxl_id][MIN_INDEX] = q
                if q_min_max_vector[dxl_id][MAX_INDEX] < q :
                    q_min_max_vector[dxl_id][MAX_INDEX] = q

            if cnt % 2 == 0:
                send_current(port_handler, packet_handler, dxl_id, 0.5)
            else:
                send_current(port_handler, packet_handler, dxl_id, -0.5)

            # Check if initialization is complete
            if abs(q_min_max_vector[dxl_id][MAX_INDEX] - q_min_max_vector[dxl_id][MIN_INDEX])< 540 * 0.9:
                is_init = False

            print(f" id: {dxl_id}")
            print(f" min val: {q_min_max_vector[dxl_id][MIN_INDEX]}")
            print(f" max val : {q_min_max_vector[dxl_id][MAX_INDEX]}")
            print(f" is_init: {is_init}")
        
        if is_init:
            for dxl_id in active_ids:
                send_current(port_handler, packet_handler, dxl_id, 0)                
                
            time.sleep(3)
            global IS_INIT_FINISH
            IS_INIT_FINISH = True
            break

        cnt += 1
        time.sleep(3)
    print("Ok")

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
    time.sleep(3)
    if robot.is_power_on('48v'):
        robot.set_tool_flange_output_voltage('right', 12)
        robot.set_tool_flange_output_voltage('left', 12)
        print('Attempting to 12V power on for gripper')
    return robot
    

def main(address):
    robot = pre_process(address)
    time.sleep(3)
    print("tool_flange")
    # Initialize PortHandler and PacketHandler
    port_handler_gripper = dxl.PortHandler(DEVICENAME)
    packet_handler_gripper = dxl.PacketHandler(PROTOCOL_VERSION)
    print(f"Protocol Version: {packet_handler_gripper.getProtocolVersion()}")
    if not port_handler_gripper.openPort():
        print("Failed to open port")
        quit()

    if not port_handler_gripper.setBaudRate(BAUDRATE):
        print("Failed to set baud rate")
        quit()

    active_ids_gripper = []
    
    for dxl_id in range(2):
        model_num, dxl_comm_result, err = packet_handler_gripper.ping(port_handler_gripper, dxl_id)
        if dxl_comm_result == dxl.COMM_SUCCESS:
            print(f"Dynamixel ID {dxl_id} is active")
            active_ids_gripper.append(dxl_id)
        else:
            print(f"Dynamixel ID {dxl_id} is not active")

    if len(active_ids_gripper) != 2:
        print("Unable to ping all devices for grippers")
        quit()
    
    for dxl_id in active_ids_gripper:
        send_torque_enable(port_handler_gripper, packet_handler_gripper, dxl_id, TORQUE_DISABLE)
        send_operation_mode(port_handler_gripper, packet_handler_gripper, dxl_id, CURRENT_CONTROL_MODE)
        send_torque_enable(port_handler_gripper, packet_handler_gripper, dxl_id, TORQUE_ENABLE)

    control_thread = threading.Thread(target=control_loop_for_gripper, args=(port_handler_gripper, packet_handler_gripper, active_ids_gripper))
    control_thread.start()
    global IS_INIT_FINISH
    while not IS_INIT_FINISH:
        time.sleep(0.005)

    control_thread.join()
    port_handler_gripper.closePort()

    robot.set_tool_flange_output_voltage("left", 0)
    robot.set_tool_flange_output_voltage("right", 0)
    if not robot.is_servo_on(".*"):
        robot.power_off(".*")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="module_test/gripper_test")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
