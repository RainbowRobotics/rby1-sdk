import asyncio
import numpy as np
import argparse

from rby1_sdk import *

from sample_action_form import Ui_MainWindow

import PySide6.QtAsyncio as QtAsyncio
from PySide6.QtWidgets import *
from PySide6.QtCore import *

GREEN_COLOR_CODE = '#57965C'
RED_COLOR_CODE = '#C94F4F'


def make_joint_control_command(joint_target, minimum_time):
    return RobotCommandBuilder().set_command(
        ComponentBasedCommandBuilder().set_body_command(
            JointPositionCommandBuilder()
            .set_command_header(CommandHeaderBuilder().set_control_hold_time(1))
            .set_minimum_time(minimum_time)
            .set_position(joint_target)
        )
    )


class SampleGui(QMainWindow, Ui_MainWindow):

    def __init__(self, robot: Robot_A, parent=None):
        super(SampleGui, self).__init__(parent)
        self.setupUi(self)

        self.robot = robot
        self.switch = [False, False]
        self.cancel_command = None
        self.robot.start_state_update(self.callback, rate=10)
        self.PB_Zero.clicked.connect(lambda: asyncio.ensure_future(self.goto_zero()))
        self.PB_Ready.clicked.connect(lambda: asyncio.ensure_future(self.goto_ready()))
        self.PB_Tab1Start.clicked.connect(lambda: asyncio.ensure_future(self.tab1_start()))
        self.PB_Tab1Stop.clicked.connect(lambda: asyncio.ensure_future(self.tab1_stop()))

        self.timer = QTimer(self)
        self.timer.start(200)
        self.timer.timeout.connect(lambda: asyncio.ensure_future(self.cm_state_update()))

    def callback(self, robot_state: RobotState_A):
        self.switch = [robot_state.tool_flange_right.switch_A, robot_state.tool_flange_left.switch_A]

    async def cm_state_update(self):
        cm_state: ControlManagerState = await asyncio.to_thread(Robot_A.get_control_manager_state,
                                                                self.robot)
        color = RED_COLOR_CODE
        if cm_state.control_state == ControlManagerState.ControlState.Executing or ControlManagerState.ControlState.Switching:
            color = GREEN_COLOR_CODE
        self.LE_State.setStyleSheet(
            f"background-color: {GREEN_COLOR_CODE if cm_state.control_state == ControlManagerState.ControlState.Executing else RED_COLOR_CODE};")
        self.LE_State.setText(cm_state.control_state.name)

    async def goto_zero(self):
        handler = self.robot.send_command(make_joint_control_command([0.] * 20, 5))
        await asyncio.to_thread(Robot_A_CommandHandler.get, handler)

    async def goto_ready(self):
        target_joint = np.array([0, 35, -70, 35, 0, 0] + [-30, -10, 0, -100, 0, 0, 0] + [-30, 10, 0, -100, 0, 0, 0])
        handler = self.robot.send_command(make_joint_control_command(np.deg2rad(target_joint), 5))
        await asyncio.to_thread(Robot_A_CommandHandler.get, handler)

    async def tab1_start(self):
        self.PB_Tab1Start.setEnabled(False)
        self.PB_Tab1Stop.setEnabled(True)

        robot_state: RobotState_A = await asyncio.to_thread(Robot_A.get_state, self.robot)

        stream: Robot_A_CommandStreamHandler = self.robot.create_command_stream()
        self.cancel_command = lambda: stream.cancel()
        try:
            while not stream.is_done():
                rc = RobotCommandBuilder().set_command(
                    ComponentBasedCommandBuilder().set_body_command(
                        BodyComponentBasedCommandBuilder()
                        .set_torso_command(OptimalControlCommandBuilder()
                                           .set_command_header(CommandHeaderBuilder().set_control_hold_time(1.))
                                           .set_center_of_mass_target("base", robot_state.center_of_mass, 0.001)
                                           .set_velocity_tracking_gain(0.01)
                                           .set_stop_cost(0.)
                                           .set_min_delta_cost(0.))
                        .set_right_arm_command(GravityCompensationCommandBuilder()
                                               .set_command_header(CommandHeaderBuilder().set_control_hold_time(1.))
                                               .set_on(self.switch[0]))
                        .set_left_arm_command(GravityCompensationCommandBuilder()
                                              .set_command_header(CommandHeaderBuilder().set_control_hold_time(1.))
                                              .set_on(self.switch[1]))
                    )
                )
                await asyncio.to_thread(Robot_A_CommandStreamHandler.send_command, stream, rc)
                await asyncio.sleep(0.1)
        except:
            print("cancelled stream")
        self.cancel_command = None

        self.PB_Tab1Start.setEnabled(True)
        self.PB_Tab1Stop.setEnabled(False)

    async def tab1_stop(self):
        if self.cancel_command:
            self.cancel_command()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="sample_action")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()
    rb = create_robot_a(args.address)
    rb.connect()

    app = QApplication()
    window = SampleGui(rb)
    window.show()

    QtAsyncio.run(handle_sigint=True)
