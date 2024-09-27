import rby1_sdk
import numpy as np
import time
import sys

ROBOT_ADDRESS = "192.168.30.1:50051"

current_state = None
recorded_traj = []
def cb(state):
    print("---")
    print(f"current position: {state.position}")
    
    current_state = state.position.copy()
    recorded_traj.append(current_state)

def pre_processing():
    robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
    robot.connect()
    
    if not robot.is_power_on(".*"):
        print("Power is currently OFF. Attempting to power on...")
        if not robot.power_on(".*"):
            print("Error: Failed to power on the robot.")
            sys.exit(1)
        print("Robot powered on successfully.")
    else:
        print("Power is already ON.")

    if not robot.is_servo_on(".*"):
        print("Servo is currently OFF. Attempting to activate servo...")
        if not robot.servo_on(".*"):
            print("Error: Failed to activate servo.")
            sys.exit(1)
        print("Servo activated successfully.")
    else:
        print("Servo is already ON.")

    control_manager_state = robot.get_control_manager_state()

    if (control_manager_state.state == rby1_sdk.ControlManagerState.State.MinorFault or \
        control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault):
        if control_manager_state.state == rby1_sdk.ControlManagerState.State.MajorFault:
            print("Warning: Detected a Major Fault in the Control Manager.")
        else:
            print("Warning: Detected a Minor Fault in the Control Manager.")

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    return robot

if __name__ == '__main__':
    robot = pre_processing()
    
    robot.start_state_update(cb, 10) #10Hz
    time.sleep(10)
    
    np.savez_compressed('recorded.npz', data=recorded_traj)