import os, sys

sys.path.append(
    os.path.join(
        os.path.dirname(__file__), "..", "third-party", "DynamixelSDK", "python", "src"
    )
)

import dynamixel_sdk as dxl
import numpy as np
import threading
import time

# Constants
PROTOCOL_VERSION = 2.0
BAUDRATE = 2000000
DEVICENAME_MASTER_ARM = "/dev/rby1_master_arm"
DEVICENAME_GRIPPER = "/dev/rby1_gripper"
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_OPERATING_MODE = 11
CURRENT_CONTROL_MODE = 0
CURRENT_BASED_POSITION_CONTROL_MODE = 5
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
D2R = np.pi / 180
R2D = 180 / np.pi

# Shared variables
q_joint_ma_mtx = threading.Lock()
q_joint_ma = np.zeros(14)
gripper_mtx = threading.Lock()
gripper_reference = np.full(2, 0.5)
gripper_reference_min_max = [np.array([11450, 12500]), np.array([5440, 6450])]

# Create a port handler and packet handler
portHandler = dxl.PortHandler(DEVICENAME_MASTER_ARM)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port and set baud rate
if not portHandler.openPort():
    print("Failed to open the port!")
    exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate!")
    exit(1)


# Functions
def send_vibration(portHandler, packetHandler, id, level):
    if id > 0x80:
        packetHandler.write2ByteTxOnly(
            portHandler, id, ADDR_GOAL_VIBRATION_LEVEL, level
        )
        time.sleep(0.0005)


def read_button_status(portHandler, packetHandler, id):
    result, position, dxl_error = packetHandler.read4ByteTxRx(
        portHandler, id, ADDR_PRESENT_BUTTON_STATUS
    )
    if result == dxl.COMM_SUCCESS:
        button = (position >> 8) & 0xFF
        trigger = ((position >> 16) & 0xFF) | (((position >> 24) & 0xFF) << 8)
        return (id, (button, trigger))
    return None


# Additional code will be needed to translate other functions such as control_loop, etc.


# Main logic
def main():
    if not portHandler.openPort():
        print("Failed to open the port!")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate!")
        return

    # Example: Enable Torque
    for id in range(14):
        packetHandler.write1ByteTxOnly(
            portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        time.sleep(0.0005)

    # Example control loop
    while True:
        with q_joint_ma_mtx:
            # Do some control tasks
            pass
        time.sleep(0.01)


if __name__ == "__main__":
    main()
