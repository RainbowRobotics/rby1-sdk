import rby1_sdk

ROBOT_ADDRESS = "localhost:50051"

robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

robot.sync_time()
logs = robot.get_last_log(5)

for log in logs:
    print(f"{log}")
    
print("---")

with rby1_sdk.printoptions(array_mode="full", multiline_repr=True):
    print(logs)
