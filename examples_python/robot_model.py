import rby1_sdk

ROBOT_ADDRESS = "localhost:50051"
robot = rby1_sdk.create_robot_a(ROBOT_ADDRESS)
robot.connect()

model = robot.get_robot_model()
print(model)
# model = model + "\n" + "<!-- ABC -->"
# print(robot.import_robot_model("abc", model))
#
robot.set_parameter("model_name", "\"\"")
