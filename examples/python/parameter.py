import rby1_sdk

robot = rby1_sdk.create_robot_a("192.168.30.1:50051")

robot.connect()

parameter_list = robot.get_parameter_list()
print(parameter_list)
print("------------------")

PARAMETER_NAME = "joint_position_command.cutoff_frequency"
print(f"Current parameter value")
print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
print("------------------")

# Reset
robot.reset_parameter_to_default(PARAMETER_NAME)

# Get
print(f"Parameter value after reset")
print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
print("------------------")

###

# Set
rv = robot.set_parameter(PARAMETER_NAME, "1")
print(f"Parameter value after set valid value")
print(f"Set parameter result: {rv}")

# Get
print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
print("------------------")

###

# Set invalid value
rv = robot.set_parameter(PARAMETER_NAME, "1000")
print(f"Parameter value after set invalid value")
print(f"Set parameter result: {rv}")

# Get
print(f"{PARAMETER_NAME}: {robot.get_parameter(PARAMETER_NAME)}")
print("------------------")
