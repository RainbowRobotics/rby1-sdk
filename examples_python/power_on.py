import rby1_sdk

ROBOT_ADDRESS = "192.168.1.229:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

print("Result:", robot.power_on("48v"))
