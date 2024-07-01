import rby1
import numpy as np

robot = rby1('10.0.0.1', 5051)

# Power On
robot.power_command('mobility', rby1.POWER_ON)
robot.power_command('manipulator', rby1.POWER_ON)

# Motor On
robot.motor_command('*', rby1.SERVO_ON)

# Start controller
robot.control_manager_command(rby1.ENABLE)


def controller(t, robot_state, robot_command) -> [rby1.VelocityControl, rby1.PositionControl]:
    return [np.array([0, 1]), np.array([0] * 20)]


robot.control(controller)
