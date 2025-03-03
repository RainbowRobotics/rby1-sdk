import rby1_sdk as rby
import argparse
import time

def main(address):
    robot = rby.create_robot_a(address)
    robot.connect()
    
    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)
        
    if not robot.power_on("12v"):
        print("Error: Failed to power on 12V.")
        exit(1)
        
    master_arm = rby.upc.MasterArm(rby.upc.MasterArmDeviceName)
    active_ids = master_arm.initialize()
    if len(active_ids) != rby.upc.MasterArm.DeviceCount:
        print("Error: Mismatch in the number of devices detected for RBY Master Arm.")
        exit(1)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="19_master_arm")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
