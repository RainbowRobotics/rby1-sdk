import time
import rby1_sdk

ROBOT_ADDRESS = "192.168.30.1:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

if not robot.is_power_on("48v"):
    robot.power_on("48v")


def cb(rs):
    print("---")
    print(
        f"right ft sensor: [{rs.timestamp - rs.ft_sensor_right.time_since_last_update}] force {rs.ft_sensor_right.force}, torque {rs.ft_sensor_right.torque}"
    )
    print(
        f"left ft sensor: [{rs.timestamp - rs.ft_sensor_left.time_since_last_update}] force {rs.ft_sensor_left.force}, torque {rs.ft_sensor_left.torque}"
    )
    print(f"{rs.is_ready=}")


robot.start_state_update(cb, 10)  # (Hz)

time.sleep(100)
