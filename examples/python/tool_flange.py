import time
import rby1_sdk

ROBOT_ADDRESS = "192.168.30.1:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

if not robot.is_power_on("48v"):
    robot.power_on("48v")


def cb(rs):
    print("---")
    print("right")
    print(f"  imu")
    print(f"    gyro: {rs.tool_flange_right.gyro}")
    print(f"    acceleration: {rs.tool_flange_right.acceleration}")
    print(f"  switch: {rs.tool_flange_right.switch_A}")
    print(f"  output voltage: {rs.tool_flange_right.output_voltage}")
    print("left")
    print(f"  imu")
    print(f"    gyro: {rs.tool_flange_left.gyro}")
    print(f"    acceleration: {rs.tool_flange_left.acceleration}")
    print(f"  switch: {rs.tool_flange_left.switch_A}")
    print(f"  output voltage: {rs.tool_flange_left.output_voltage}")


robot.start_state_update(cb, 10)  # (Hz)

time.sleep(100)
