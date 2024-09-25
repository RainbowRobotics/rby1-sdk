import rby1_sdk

ROBOT_ADDRESS = "192.168.30.1:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

rv = robot.power_on("12v")
print(f"Result: {rv}")
