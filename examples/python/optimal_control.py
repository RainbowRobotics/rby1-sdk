import os
import sys
import argparse
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'python'))
from robot_manager import RobotManager

async def main(address: str):
    robot_manager = RobotManager(address)
    joint_states = []
    robot_info = await robot_manager.get_robot_info()
    
    for j in robot_info.joint_infos:
        joint_states += [
        {"name": j.name, "fet": False, "run": False, "init": False, "motor_type": 0, "motor_state": 0,
            "position": 0., "velocity": 0., "current": 0., "torque": 0.}]
    
    
    q = np.zeros(22)
    q[:6] = 0.01

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Robot Manager script')
    parser.add_argument('--address', type=str, default='localhost:50051',
                        help='The address of the robot manager server (default: localhost:50051)')
    
    args = parser.parse_args()
    main(args.address)

