import os, sys, time
import argparse
import threading

import numpy as np

import dynamixel_sdk as dxl
import ctypes

import rby1_sdk
from rby1_sdk import *

GRIPPER_DIRECTION = False

DEVICENAME_MASTER_ARM = "/dev/rby1_master_arm"
DEVICENAME_GRIPPER = "/dev/rby1_gripper"

# Dynamixel Control Table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_OPERATING_MODE = 11

ADDR_PRESENT_BUTTON_STATUS = 132

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
BAUDRATE = 2000000

# Define constants
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

D2R = 0.017453288888888
R2D = 57.29579143313326

CURRENT_CONTROL_MODE = 0
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
CURRENT_BASED_POSITION_CONTROL_MODE = 5

MIN_INDEX = 0
MAX_INDEX = 1

DEG_PER_PULSE = 360/4096 #0.088

REV_RANGE_MIN = -256
REV_RANGE_MAX = 256

PULSE_RANGE_MIN = REV_RANGE_MIN*360/DEG_PER_PULSE
PULSE_RANGE_MAX = REV_RANGE_MAX*360/DEG_PER_PULSE


LINK_NAMES = [
    "Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R", 
    "Link_0L", "Link_1L", "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"
]

JOINT_NAMES = [
    "J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R", 
    "J4_Wrist_Yaw1_R", "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R", 
    "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L", 
    "J10_Elbow_L", "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"
]

M_SF = 0.4

MTX_Q_JOINT_MASTER_ARM_INFO = threading.Lock()
MTX_HAND_CONTROLLER_INFO = threading.Lock()

Q_JOINT_MASTER_ARM = np.zeros(14, dtype=np.float64)

TEMP_HAND_CONTROLLER_TRIGGER = np.full(2, 0.5)

HAND_CONTROLLER_TRIGGER = np.full(2, 0.5)
HAND_CONTROLLER_BUTTON = np.zeros(2)

HAND_CONTROLLER_TRIGGER_MIN_MAX = [
    np.array([0, 1000], dtype=np.float64),
    np.array([0, 1000], dtype=np.float64)
]

VERBOSE_MASTER_ARM_INFO = False

TORQUE_CONSTANT = np.array([
    1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
    1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043
], dtype=np.float64)

###
### Robot Status
###
def cb(rs):
    print(f"Timestamp: {rs.timestamp.tv_sec}.{rs.timestamp.tv_nsec}")
    position = rs.position * 180 / 3.141592
    print(f"torso [deg]: {position[2:2 + 6]}")
    print(f"right arm [deg]: {position[8:8 + 7]}")
    print(f"left arm [deg]: {position[15:15 + 7]}")
    
###
### Robot Computation
###
def compute_gravity_torque(robot_config: rby1_sdk.dynamics.Robot_24, state: rby1_sdk.dynamics.State_24, q_joint):
    state.set_q(q_joint)
    robot_config.compute_forward_kinematics(state)
    robot_config.compute_diff_forward_kinematics(state)
    robot_config.compute_2nd_diff_forward_kinematics(state)
    robot_config.compute_inverse_dynamics(state)
    
    return state.get_tau()

def calc_torque_for_limit_avoid(q_joint):
    torque_add = np.zeros(14, dtype=np.float64)

    arm_dof = 7

    n_joint = 1
    if q_joint[n_joint] > -10.0 * D2R:
        torque_add[n_joint] += (-10.0 * D2R - q_joint[n_joint]) * 4.0

    n_joint = arm_dof + 1
    if q_joint[n_joint] < 10.0 * D2R:
        torque_add[n_joint] += (10.0 * D2R - q_joint[n_joint]) * 4.0

    n_joint = 2
    if q_joint[n_joint] > 90.0 * D2R:
        torque_add[n_joint] += (90.0 * D2R - q_joint[n_joint]) * 0.5
    if q_joint[n_joint] < 0.0:
        torque_add[n_joint] += (0.0 - q_joint[n_joint]) * 0.5

    n_joint = arm_dof + 2
    if q_joint[n_joint] < -90.0 * D2R:
        torque_add[n_joint] += (-90.0 * D2R - q_joint[n_joint]) * 0.5
    if q_joint[n_joint] > 0.0:
        torque_add[n_joint] += (0.0 - q_joint[n_joint]) * 0.5

    n_joint = 5
    if q_joint[n_joint] > 90.0 * D2R:
        torque_add[n_joint] += (90.0 * D2R - q_joint[n_joint]) * 1.0
    if q_joint[n_joint] < 0.0 * D2R:
        torque_add[n_joint] += (0.0 * D2R - q_joint[n_joint]) * 1.0

    n_joint = arm_dof + 5
    if q_joint[n_joint] > 90.0 * D2R:
        torque_add[n_joint] += (90.0 * D2R - q_joint[n_joint]) * 1.0
    if q_joint[n_joint] < 0.0 * D2R:
        torque_add[n_joint] += (0.0 * D2R - q_joint[n_joint]) * 1.0

    n_joint = 4
    if q_joint[n_joint] > 90.0 * D2R:
        torque_add[n_joint] += (90.0 * D2R - q_joint[n_joint]) * 0.5
    if q_joint[n_joint] < -90.0 * D2R:
        torque_add[n_joint] += (-90.0 * D2R - q_joint[n_joint]) * 0.5

    n_joint = arm_dof + 4
    if q_joint[n_joint] > 90.0 * D2R:
        torque_add[n_joint] += (90.0 * D2R - q_joint[n_joint]) * 0.5
    if q_joint[n_joint] < -90.0 * D2R:
        torque_add[n_joint] += (-90.0 * D2R - q_joint[n_joint]) * 0.5

    n_joint = 3
    if q_joint[n_joint] < -135.0 * D2R:
        torque_add[n_joint] += (-135.0 * D2R - q_joint[n_joint]) * 6.0
    if q_joint[n_joint] > -20.0 * D2R:
        torque_add[n_joint] += (-20.0 * D2R - q_joint[n_joint]) * 6.0

    n_joint = arm_dof + 3
    if q_joint[n_joint] < -135.0 * D2R:
        torque_add[n_joint] += (-135.0 * D2R - q_joint[n_joint]) * 6.0
    if q_joint[n_joint] > -20.0 * D2R:
        torque_add[n_joint] += (-20.0 * D2R - q_joint[n_joint]) * 6.0

    torque_add_limit = np.full(14, 300.0 / 1000.0, dtype=np.float64)
    torque_add_limit[5] = 500.0 / 1000.0
    torque_add_limit[arm_dof + 5] = 500.0 / 1000.0

    torque_add = np.minimum(torque_add, torque_add_limit)
    torque_add = np.maximum(torque_add, -torque_add_limit)

    torque_add *= TORQUE_CONSTANT

    return torque_add

###
### Dxl Write
###
def send_goal_position(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id: int, goal_position: int):
    packet_handler.write4ByteTxRx(port_handler, dxl_id, ADDR_GOAL_POSITION, goal_position)
    time.sleep(0.0005)

def send_torque_enable(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id, enable):
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, enable)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
    else:
        print(f"ID {dxl_id} has been successfully connected")
    time.sleep(5/1000000) # 5 us

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
    


###
### Dxl Read
###
def read_operation_mode(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id):
    # data_read, result, error
    operation_mode, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE)
    if dxl_comm_result == dxl.COMM_SUCCESS:
        return operation_mode
    else:
        return None

def read_encoder(port_handler, packet_handler, dxl_id):
    try:
        dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, dxl_id, ADDR_PRESENT_POSITION
        )
        
        if dxl_comm_result == dxl.COMM_SUCCESS:
            # Convert to signed 32-bit value
            dxl_present_position = ctypes.c_int32(dxl_present_position).value
            
            # Convert pulse to radians (C++ logic: position / 4096 * 2 * pi)
            position_in_radians = dxl_present_position / 4096.0 * 2.0 * 3.141592
            
            print(f"id: {dxl_id}, Encoder (pulses): {dxl_present_position}, Encoder (radians): {position_in_radians}")
            return position_in_radians
        else:
            print(f"Failed to read encoder for ID {dxl_id}. Communication result: {dxl_comm_result}")
            return None
    except Exception as e:
        print(f"Error reading encoder for ID {dxl_id}: {e}")
        return None
    
def read_torque_enable(port_handler, packet_handler, dxl_id):
    try:
        dxl_error = 0
        onoff, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE)
        
        if dxl_comm_result == dxl.COMM_SUCCESS:
            return onoff
        else:
            return None
    except Exception as e:
        # print(f"Error reading torque enable for ID {dxl_id}: {e}")
        return None
    
def read_button_status(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_id: int):
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_BUTTON_STATUS)
    dxl_present_position = ctypes.c_int32(dxl_present_position).value
    if dxl_comm_result == dxl.COMM_SUCCESS:
        button = (dxl_present_position >> 8) & 0xFF
        trigger = ((dxl_present_position >> 16) & 0xFF) | (((dxl_present_position >> 24) & 0xFF) << 8)
        return dxl_id, (button, trigger)
    else:
        return None 

###
### Dxl Bulk Write
###
def bulk_write_torque_enable_individual(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, id_and_enable_vector):
    group_bulk_write = dxl.GroupBulkWrite(port_handler, packet_handler)

    for dxl_id, enable in id_and_enable_vector:
        if dxl_id < 0x80:
            param = [enable]  # Single byte parameter
            group_bulk_write.addParam(dxl_id, ADDR_TORQUE_ENABLE, 1, param)

    dxl_comm_result = group_bulk_write.txPacket()
    # if dxl_comm_result != dxl.COMM_SUCCESS:
    #     print(f"Bulk write torque enable failed: {dxl_comm_result}")
    time.sleep(0.0005)  # 500 microseconds
    
def bulk_write_torque_enable(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_ids, enable):
    group_bulk_write = dxl.GroupBulkWrite(port_handler, packet_handler)

    for dxl_id in dxl_ids:
        if dxl_id < 0x80:
            param = [enable]  # Single byte parameter
            group_bulk_write.addParam(dxl_id, ADDR_TORQUE_ENABLE, 1, param)

    dxl_comm_result = group_bulk_write.txPacket()
    # if dxl_comm_result != dxl.COMM_SUCCESS:
    #     print(f"Bulk write torque enable failed: {dxl_comm_result}")
    time.sleep(0.0005)  # 500 microseconds

def bulk_write_operation_mode(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, id_and_mode_vector):
    group_bulk_write = dxl.GroupBulkWrite(port_handler, packet_handler)

    for dxl_id, mode in id_and_mode_vector:
        if dxl_id < 0x80:
            param = [mode]  # Single byte parameter
            group_bulk_write.addParam(dxl_id, ADDR_OPERATING_MODE, 1, param)

    dxl_comm_result = group_bulk_write.txPacket()
    # if dxl_comm_result != dxl.COMM_SUCCESS:
    #     print(f"Bulk write operation mode failed: {dxl_comm_result}")
    time.sleep(0.0005)  # 500 microseconds

def bulk_write_send_torque(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, id_and_torque_vector):
    group_bulk_write = dxl.GroupBulkWrite(port_handler, packet_handler)

    for dxl_id, torque in id_and_torque_vector:
        if dxl_id < 0x80:
            # Convert torque to the format required by the Dynamixel
            goal_current = int(torque / TORQUE_CONSTANT[dxl_id] * 1000.0 / 2.69)
            param = [goal_current & 0xFF, (goal_current >> 8) & 0xFF]  # Convert to 2 bytes
            group_bulk_write.addParam(dxl_id, ADDR_GOAL_CURRENT, 2, param)

    dxl_comm_result = group_bulk_write.txPacket()
    # if dxl_comm_result != dxl.COMM_SUCCESS:
    #     print(f"Bulk write send torque failed: {dxl_comm_result}")
    time.sleep(0.0005)  # 500 microseconds
    
###
### Dxl Bulk Read
###
def bulk_read_operation_mode(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_ids):
    operation_mode_vector = []
    group_bulk_read = dxl.GroupBulkRead(port_handler, packet_handler)

    for dxl_id in dxl_ids:
        if dxl_id < 0x80:
            group_bulk_read.addParam(dxl_id, ADDR_OPERATING_MODE, 1)

    dxl_comm_result = group_bulk_read.txRxPacket()
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print(f"Bulk read communication failed: {dxl_comm_result}")
        return None

    for dxl_id in dxl_ids:
        if dxl_id < 0x80:
            if group_bulk_read.isAvailable(dxl_id, ADDR_OPERATING_MODE, 1):
                operation_mode = group_bulk_read.getData(dxl_id, ADDR_OPERATING_MODE, 1)
                operation_mode_vector.append((dxl_id, operation_mode))
                
    if len(operation_mode_vector) == 0:
        return None
    else:
        return operation_mode_vector    
    
def bulk_read_encoder(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_ids):
    group_bulk_read = dxl.GroupBulkRead(port_handler, packet_handler)
    
    for dxl_id in dxl_ids:
        if dxl_id < 0x80: 
            group_bulk_read.addParam(dxl_id, ADDR_PRESENT_POSITION, 4)
    
    dxl_comm_result = group_bulk_read.txRxPacket()
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print(f"Bulk read communication failed: {dxl_comm_result}")
        return None  # 통신 실패 시 None 반환
    
    position_vector = []

    # 각 ID에 대해 데이터 확인 및 처리
    for dxl_id in dxl_ids:
        if dxl_id < 0x80:
            if group_bulk_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, 4):
                dxl_present_position = group_bulk_read.getData(dxl_id, ADDR_PRESENT_POSITION, 4)
                dxl_present_position = ctypes.c_int32(dxl_present_position).value
                position_rad = dxl_present_position * 2.0 * 3.141592 / 4096.0
                
                position_vector.append((dxl_id, position_rad))
    
    if len(position_vector) == 0:
        return None
    else:
        return position_vector
    
def bulk_read_torque_enable(port_handler: dxl.PortHandler, packet_handler: dxl.PacketHandler, dxl_ids):
    torque_enable_vector = []
    group_bulk_read = dxl.GroupBulkRead(port_handler, packet_handler)

    for dxl_id in dxl_ids:
        if dxl_id < 0x80:
            group_bulk_read.addParam(dxl_id, ADDR_TORQUE_ENABLE, 1)

    dxl_comm_result = group_bulk_read.txRxPacket()
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print(f"Bulk read communication failed: {dxl_comm_result}")
        return None

    for dxl_id in dxl_ids:
        if dxl_id < 0x80:
            if group_bulk_read.isAvailable(dxl_id, ADDR_TORQUE_ENABLE, 1):
                torque_enable = group_bulk_read.getData(dxl_id, ADDR_TORQUE_ENABLE, 1)
                torque_enable_vector.append((dxl_id, torque_enable))
                
    if len(torque_enable_vector) == 0:
        return None
    else:
        return torque_enable_vector

def control_loop_for_master_arm(port_handler, packet_handler, active_ids):
    robot_dyn = rby1_sdk.dynamics.Robot(rby1_sdk.dynamics.load_robot_from_urdf("/home/nvidia/ws/rby1-sdk/models/master_arm/model.urdf", "Base"))    
    state = robot_dyn.make_state(LINK_NAMES, JOINT_NAMES)
    
    gravity_vector = np.array([0, 0, 0, 0, 0, -9.81])
    state.set_gravity(gravity_vector)

    q_joint = np.zeros(14, dtype=np.float64)
    tau_joint = np.zeros(14, dtype=np.float64)
    operation_mode = np.full(14, -1, dtype=np.int32)
    torque_enable = np.zeros(14, dtype=np.int32)
    button_status_vector = []
    temp_eigen = np.full(14, -1, dtype=np.float64)
    button_info = np.zeros(2, dtype=np.float64)
    trigger_info = np.zeros(2, dtype=np.float64)

    while True:
        start_time = time.time()

        button_status_vector.clear()
        operation_mode.fill(-1)
        torque_enable.fill(0)
        temp_eigen.fill(-1)

        # Read button status
        for dxl_id in active_ids:
            if dxl_id >= 0x80:
                temp_button_status = read_button_status(port_handler, packet_handler, dxl_id)
                button_status_vector.append(temp_button_status)

        # Read joint positions
        temp_q_joint_vector = bulk_read_encoder(port_handler, packet_handler, active_ids)
        if temp_q_joint_vector is not None:
            for ret in temp_q_joint_vector:
                q_joint[ret[0]] = ret[1]

        # Compute torques
        tau_joint = compute_gravity_torque(robot_dyn, state, q_joint) * M_SF
        add_torque = calc_torque_for_limit_avoid(q_joint)
        tau_joint += add_torque

        # Save q_joint to shared variable
        with MTX_Q_JOINT_MASTER_ARM_INFO:
            global Q_JOINT_MASTER_ARM
            Q_JOINT_MASTER_ARM = q_joint

        # Read operation mode
        temp_operation_mode_vector = bulk_read_operation_mode(port_handler, packet_handler, active_ids)
        if temp_operation_mode_vector is not None:
            for ret in temp_operation_mode_vector:
                operation_mode[ret[0]] = ret[1]

        # Read torque enable
        temp_torque_enable_vector = bulk_read_torque_enable(port_handler, packet_handler, active_ids)
        if temp_torque_enable_vector is not None:
            for ret in temp_torque_enable_vector:
                torque_enable[ret[0]] = ret[1]

        # Update torque enable
        id_and_enable_vector = []
        for i in active_ids:
            if i <0x80 and not torque_enable[i]:
                id_and_enable_vector.append((i, 1))

        bulk_write_torque_enable_individual(port_handler, packet_handler, id_and_enable_vector)

        # Button control logic
        id_and_mode_vector = []
        id_torque_onoff_vector = []
        id_send_torque_vector = []

        for button_status in button_status_vector:
            if button_status is not None:
                id_hand_controller = button_status[0]
                button, trigger = button_status[1]

                trigger_info[id_hand_controller - 0x80] = trigger
                button_info[id_hand_controller - 0x80] = button

                with MTX_HAND_CONTROLLER_INFO:
                    global HAND_CONTROLLER_BUTTON
                    global HAND_CONTROLLER_TRIGGER
                    HAND_CONTROLLER_TRIGGER[id_hand_controller - 0x80] = (
                        (trigger - HAND_CONTROLLER_TRIGGER_MIN_MAX[id_hand_controller - 0x80][MIN_INDEX])
                        / (HAND_CONTROLLER_TRIGGER_MIN_MAX[id_hand_controller - 0x80][MAX_INDEX]
                        - HAND_CONTROLLER_TRIGGER_MIN_MAX[id_hand_controller - 0x80][MIN_INDEX])
                    )
                    HAND_CONTROLLER_BUTTON[id_hand_controller - 0x80] = button

                if id_hand_controller == 0x80:
                    for i in range(7):  # Right arm
                        if button == 0:
                            if operation_mode[i] != CURRENT_BASED_POSITION_CONTROL_MODE:
                                id_and_mode_vector.append((i, CURRENT_BASED_POSITION_CONTROL_MODE))
                                id_torque_onoff_vector.append(i)
                        elif button == 1:
                            if operation_mode[i] != CURRENT_CONTROL_MODE:
                                id_and_mode_vector.append((i, CURRENT_CONTROL_MODE))
                                id_torque_onoff_vector.append(i)
                            else:
                                id_send_torque_vector.append((i, tau_joint[i]))

                elif id_hand_controller == 0x81:
                    for i in range(7, 14):  # Left arm
                        if button == 0:
                            if operation_mode[i] != CURRENT_BASED_POSITION_CONTROL_MODE:
                                id_and_mode_vector.append((i, CURRENT_BASED_POSITION_CONTROL_MODE))
                                id_torque_onoff_vector.append(i)
                        elif button == 1:
                            if operation_mode[i] != CURRENT_CONTROL_MODE:
                                id_and_mode_vector.append((i, CURRENT_CONTROL_MODE))
                                id_torque_onoff_vector.append(i)
                            else:
                                id_send_torque_vector.append((i, tau_joint[i]))

        # Bulk write operations
        bulk_write_torque_enable(port_handler, packet_handler, id_torque_onoff_vector, 0)
        bulk_write_operation_mode(port_handler, packet_handler, id_and_mode_vector)
        bulk_write_torque_enable(port_handler, packet_handler, id_torque_onoff_vector, 1)
        bulk_write_send_torque(port_handler, packet_handler, id_send_torque_vector)

        # Log outputs periodically
        if VERBOSE_MASTER_ARM_INFO and int(time.time() * 1000) % 5 == 0:
            print(f"button_info: {button_info}")
            print(f"trigger_info: {trigger_info}")
            print(f"right q_joint [deg]: {q_joint[:7] * (180. / np.pi)}")
            print(f"left q_joint [deg]: {q_joint[7:] * (180. / np.pi)}")

        # Ensure loop timing
        elapsed_time = time.time() - start_time
        if elapsed_time < 0.001:
            time.sleep(0.001 - elapsed_time)        
                
def control_loop_for_gripper(port_handler, packet_handler, active_ids):
    q_min_max_vector = [np.zeros(2) for _ in range(2)]

    cnt = 0
    while True:
        is_init = True
        print(f"{is_init=}")
        print(active_ids)
        if len(active_ids) != 2:
            print("The number of Dynamixels for hand gripper does not match the configuration")
            return

        for dxl_id in active_ids:
            # Ensure the control mode is in CURRENT_CONTROL_MODE
            while True:
                operation_mode = read_operation_mode(port_handler, packet_handler, dxl_id)
                if operation_mode is not None:
                    if operation_mode != CURRENT_CONTROL_MODE:
                        send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_DISABLE)
                        send_operation_mode(port_handler, packet_handler, dxl_id, CURRENT_CONTROL_MODE)
                        send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                        print(f"Try to change control mode, id: {dxl_id}")
                    else:
                        break
                time.sleep(0.1)

            # Enable the torque if disabled
            while True:
                torque_enable_val = read_torque_enable(port_handler, packet_handler, dxl_id)
                print(f"{torque_enable_val=}")
                if torque_enable_val is not None:
                    if not torque_enable_val:
                        send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                        print(f"Try to enable torque, id: {dxl_id}")
                    else:
                        break
                time.sleep(0.1)

            # Update min/max encoder values
            if cnt % 2 == 0:
                q = read_encoder(port_handler, packet_handler, dxl_id)
                if q is not None:
                    q_min_max_vector[dxl_id][MIN_INDEX] = q
                send_current(port_handler, packet_handler, dxl_id, 0.5)
            else:
                q = read_encoder(port_handler, packet_handler, dxl_id)
                if q is not None:
                    q_min_max_vector[dxl_id][MAX_INDEX] = q
                send_current(port_handler, packet_handler, dxl_id, -0.5)

            # Check if initialization is complete
            if abs(q_min_max_vector[dxl_id][MAX_INDEX] - q_min_max_vector[dxl_id][MIN_INDEX]) * 180 / 3.141592 < (540 * 0.9):
                is_init = False

        if is_init:
            for dxl_id in active_ids:
                if q_min_max_vector[dxl_id][MIN_INDEX] > q_min_max_vector[dxl_id][MAX_INDEX]:
                    temp = q_min_max_vector[id][MIN_INDEX]
                    q_min_max_vector[id][MIN_INDEX] = q_min_max_vector[id][MAX_INDEX];
                    q_min_max_vector[id][MAX_INDEX] = temp
                send_current(port_handler, packet_handler, dxl_id, 0.5)
            time.sleep(3)
            break
        

        cnt += 1
        time.sleep(3)

    print("Finish init")

    while True:
        for dxl_id in active_ids:
            operation_mode = read_operation_mode(port_handler, packet_handler, dxl_id)
            if operation_mode is not None:
                if operation_mode != CURRENT_BASED_POSITION_CONTROL_MODE:
                    send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_DISABLE)
                    send_operation_mode(port_handler, packet_handler, dxl_id, CURRENT_BASED_POSITION_CONTROL_MODE)
                    send_torque_enable(port_handler, packet_handler, dxl_id, TORQUE_ENABLE)
                else:
                    goal_position = 0
                    with MTX_HAND_CONTROLLER_INFO:
                        global TEMP_HAND_CONTROLLER_TRIGGER, HAND_CONTROLLER_TRIGGER
                        TEMP_HAND_CONTROLLER_TRIGGER[dxl_id] = (
                            TEMP_HAND_CONTROLLER_TRIGGER[dxl_id] * 0.9 +
                            HAND_CONTROLLER_TRIGGER[dxl_id] * 0.1
                        )
                        TEMP_HAND_CONTROLLER_TRIGGER[dxl_id] = np.clip(TEMP_HAND_CONTROLLER_TRIGGER[dxl_id], 0, 1)
                        HAND_CONTROLLER_TRIGGER[dxl_id] = round(TEMP_HAND_CONTROLLER_TRIGGER[dxl_id] * 100) / 100.0

                        if GRIPPER_DIRECTION:
                            goal_position = (
                                HAND_CONTROLLER_TRIGGER[dxl_id] * q_min_max_vector[dxl_id][MAX_INDEX] +
                                (1.0 - HAND_CONTROLLER_TRIGGER[dxl_id]) * q_min_max_vector[dxl_id][MIN_INDEX]
                            )
                        else:
                            goal_position = (
                                HAND_CONTROLLER_TRIGGER[dxl_id] * q_min_max_vector[dxl_id][MIN_INDEX] +
                                (1.0 - HAND_CONTROLLER_TRIGGER[dxl_id]) * q_min_max_vector[dxl_id][MAX_INDEX]
                            )
                    send_goal_position(port_handler, packet_handler, dxl_id, int(goal_position * 4096 / 3.141592 / 2))
        time.sleep(0.01)   

def main(address, power_device, servo):
    robot = rby1_sdk.create_robot_a(address)
    if not robot.connect():
        print("Error: Unable to establish connection to the robot at")
        sys.exit(1)

    robot.start_state_update(cb, 0.1)
    robot.set_parameter("joint_position_command.cutoff_frequency", "10.0", False)

    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)
        
    if not robot.is_power_on(power_device):
        rv = robot.power_on(power_device)
        if not rv:
            print("Failed to power on")
            exit(1)

    if not robot.is_servo_on(servo):
        rv = robot.servo_on(servo)
        if not rv:
            print("Fail to servo on")
            exit(1)
    
    control_manager_state = robot.get_control_manager_state()
    if control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault or \
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:

        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    print("Enabling the Control Manager...")
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled successfully.")
    
    if robot.is_power_on("48v"):
        robot.set_tool_flange_output_voltage("right", 12)
        robot.set_tool_flange_output_voltage("left", 12)
        print("Attempting to 12V power on for gripper")
        time.sleep(1.0)

    
    # Master Arm
    port_handler_master_arm = dxl.PortHandler(DEVICENAME_MASTER_ARM)
    packet_handler_master_arm = dxl.PacketHandler(PROTOCOL_VERSION)
    print(f"Protocol Version: {packet_handler_master_arm.getProtocolVersion()}")
    if not port_handler_master_arm.openPort():
        print("Failed to open port")
        quit()

    if not port_handler_master_arm.setBaudRate(BAUDRATE):
        print("Failed to set baud rate")
        quit()

    active_ids_master_arm = []
    
    for dxl_id in range(14):
        model_num, dxl_comm_result, err = packet_handler_master_arm.ping(port_handler_master_arm, dxl_id)
        if dxl_comm_result == dxl.COMM_SUCCESS:
            print(f"Dynamixel ID {dxl_id} is active")
            active_ids_master_arm.append(dxl_id)
        else:
            print(f"Dynamixel ID {dxl_id} is not active")
    
    for dxl_id in range(0x80, 0x80 + 2):
        model_num, dxl_comm_result, err = packet_handler_master_arm.ping(port_handler_master_arm, dxl_id)
        if dxl_comm_result == dxl.COMM_SUCCESS:
            print(f"Dynamixel ID {dxl_id} is active")
            active_ids_master_arm.append(dxl_id)
        else:
            print(f"Dynamixel ID {dxl_id} is not active")

    if len(active_ids_master_arm) != 16:
        print("Unable to ping all devices for master arm")
        quit()
    
    for dxl_id in active_ids_master_arm:
        if dxl_id < 0x80:
            dxl_comm_result, dxl_error = packet_handler_master_arm.write1ByteTxRx(port_handler_master_arm, dxl_id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Failed to write current control mode value: {packet_handler_master_arm.getTxRxResult(dxl_comm_result)}")
                quit()            
            send_torque_enable(port_handler_master_arm, packet_handler_master_arm, dxl_id, TORQUE_ENABLE)

    master_arm_handler = threading.Thread(target = control_loop_for_master_arm, args=(port_handler_master_arm, packet_handler_master_arm, active_ids_master_arm))
    master_arm_handler.start()
    
    # Initialize PortHandler and PacketHandler
    port_handler_gripper = dxl.PortHandler(DEVICENAME_GRIPPER)
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
        dxl_comm_result, dxl_error = packet_handler_gripper.write1ByteTxRx(port_handler_gripper, dxl_id, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL_MODE)
        if dxl_comm_result != dxl.COMM_SUCCESS:
                print(f"Failed to write current control mode value:  {packet_handler_gripper.getTxRxResult(dxl_comm_result)}")
                quit()            
        
        send_torque_enable(port_handler_gripper, packet_handler_gripper, dxl_id, TORQUE_ENABLE)
    
    gripper_handler = threading.Thread(target = control_loop_for_gripper, args=(port_handler_gripper, packet_handler_gripper, active_ids_gripper))
    gripper_handler.start()
    
    q_joint_torso = np.deg2rad(np.array([0, 30, -60, 30, 0, 0]))
    rc = RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            BodyComponentBasedCommandBuilder()
            .set_torso_command(
                JointPositionCommandBuilder()
                .set_command_header(CommandHeaderBuilder().set_control_hold_time(1.0))
                .set_minimum_time(5)
                .set_position(q_joint_torso)
            )
        )
    )
    rv = robot.send_command(rc).get()
    
    q_joint_ref = np.zeros(14)
    
    with MTX_Q_JOINT_MASTER_ARM_INFO:
        q_joint_ref = Q_JOINT_MASTER_ARM
    
    dyn = robot.get_dynamics()
    dyn_state = dyn.make_state(["base"], Model_A().robot_joint_names)
    
    q_upper_limit = dyn.get_limit_q_upper(dyn_state)[8:8 + 14]
    q_lower_limit = dyn.get_limit_q_lower(dyn_state)[8:8 + 14]
    
    stream = robot.create_command_stream()
    
    time.sleep(5)
    
    right_arm_minimum_time = 1.0
    left_arm_minimum_time = 1.0
    lpf_update_ratio = 0.1
    
    print("Start to send command to robot")
    
    q_joint_ref_20 = np.zeros(20, dtype=np.float64)
    q_joint_ref_20[0:6] = np.deg2rad(np.array([0, 30, -60, 30, 0, 0]))
    
    while True:
        with MTX_Q_JOINT_MASTER_ARM_INFO:
            with MTX_HAND_CONTROLLER_INFO:
                q_joint_ref_20[6:6+14] = q_joint_ref_20[6:6+14] * (1 - lpf_update_ratio) + Q_JOINT_MASTER_ARM * lpf_update_ratio
                q_joint_rby1 = np.zeros(24, dtype = np.float64)
                q_joint_rby1[2:2+20] = q_joint_ref_20
                
                dyn_state.set_q(q_joint_rby1)
                dyn.compute_forward_kinematics(dyn_state)
                res_col = dyn.detect_collisions_or_nearest_links(dyn_state, 1)
                is_collision = False
                
                if res_col[0].distance < 0.02:
                    is_collision = True
                    
                if HAND_CONTROLLER_BUTTON[0] and not is_collision:
                    q_joint_ref[0:7] = q_joint_ref[0:7] * (1-lpf_update_ratio) + Q_JOINT_MASTER_ARM[0:7] * lpf_update_ratio
                else:
                    right_arm_minimum_time = 1.0
                
                if HAND_CONTROLLER_BUTTON[1] and not is_collision:
                    q_joint_ref[7:7+7] = q_joint_ref[7:7+7] * (1-lpf_update_ratio) + Q_JOINT_MASTER_ARM[7:7+7] *lpf_update_ratio
                else:
                    left_arm_minimum_time = 1.0
        
        
        q_joint_ref = np.clip(q_joint_ref, q_lower_limit, q_upper_limit)
        target_position_right = q_joint_ref[0:7]
        target_position_left = q_joint_ref[7:7+7]
        
        acc_limit = np.full(7, 1200.0, dtype=np.float64)
        acc_limit = np.deg2rad(acc_limit) 

        vel_limit = np.array([160, 160, 160, 160, 330, 330, 330], dtype=np.float64)
        vel_limit = np.deg2rad(vel_limit)
        
        right_arm_minimum_time *= 0.99
        right_arm_minimum_time = max(right_arm_minimum_time, 0.01)
        
        left_arm_minimum_time *= 0.99
        left_arm_minimum_time = max(left_arm_minimum_time, 0.01)
        
        rc = RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(
                BodyComponentBasedCommandBuilder()
                .set_right_arm_command(
                    JointPositionCommandBuilder()
                    .set_command_header(CommandHeaderBuilder().set_control_hold_time(4.0))
                    .set_minimum_time(right_arm_minimum_time)
                    .set_position(target_position_right)
                    .set_velocity_limit(vel_limit)
                    .set_acceleration_limit(acc_limit)
                )
                .set_left_arm_command(
                    JointPositionCommandBuilder()
                    .set_command_header(CommandHeaderBuilder().set_control_hold_time(4.0))
                    .set_minimum_time(left_arm_minimum_time)
                    .set_position(target_position_left)
                    .set_velocity_limit(vel_limit)
                    .set_acceleration_limit(acc_limit)
                )
            )
        )
        
        stream.send_command(rc)
        time.sleep(0.005) # 5ms
    
    master_arm_handler.join()
    gripper_handler.join()
    
    port_handler_master_arm.closePort()
    port_handler_gripper.closePort()
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="17_teleoperation_with_joint_mapping")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo name regex pattern (default: '.*')")
    args = parser.parse_args()

    main(address=args.address,
         power_device=args.device,
         servo = args.servo)

