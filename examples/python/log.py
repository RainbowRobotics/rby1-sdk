import rby1_sdk

ROBOT_ADDRESS = "localhoast:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

logs = robot.get_last_log(5)
print(logs)