import asyncio
import numpy as np
import argparse

from rby1_sdk import *
from common import Rx, Ry, Rz

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
        self.dyn_robot = self.robot.get_dynamics()
        self.dyn_state = self.dyn_robot.make_state(["base", "link_right_arm_5", "link_left_arm_5", "link_torso_5", "ee_right", "ee_left"], Model_A().robot_joint_names)
        self.callback_count = 0
        self.switch = [False, False]
        self.position = []
        self.cancel_command = None
        self.robot.start_state_update(self.callback, rate=50)
        self.PB_Zero.clicked.connect(lambda: asyncio.ensure_future(self.goto_zero()))
        self.PB_Ready.clicked.connect(lambda: asyncio.ensure_future(self.goto_ready()))
        self.PB_Tab1Start.clicked.connect(lambda: asyncio.ensure_future(self.tab1_start()))
        self.PB_Tab1Stop.clicked.connect(lambda: asyncio.ensure_future(self.tab1_stop()))
        self.PB_Tab2Start.clicked.connect(lambda: asyncio.ensure_future(self.tab2_start()))
        self.PB_Tab2Stop.clicked.connect(lambda: asyncio.ensure_future(self.tab2_stop()))
        self.PB_Tab2Reset.clicked.connect(lambda: asyncio.ensure_future(self.tab2_reset()))
        self.PB_Tab3Start.clicked.connect(lambda: asyncio.ensure_future(self.tab3_start()))
        self.PB_Tab3Stop.clicked.connect(lambda: asyncio.ensure_future(self.tab3_stop()))

        self.timer = QTimer(self)
        self.timer.start(200)
        self.timer.timeout.connect(lambda: asyncio.ensure_future(self.update_display()))

    def callback(self, robot_state: RobotState_A):
        self.switch = [robot_state.tool_flange_right.switch_A, robot_state.tool_flange_left.switch_A]
        self.position = robot_state.position
        self.callback_count = self.callback_count + 1

    async def update_display(self):
        cm_state: ControlManagerState = await asyncio.to_thread(Robot_A.get_control_manager_state,
                                                                self.robot)
        color = RED_COLOR_CODE
        if cm_state.control_state == ControlManagerState.ControlState.Executing or ControlManagerState.ControlState.Switching:
            color = GREEN_COLOR_CODE
        self.LE_State.setStyleSheet(
            f"background-color: {GREEN_COLOR_CODE if cm_state.control_state == ControlManagerState.ControlState.Executing else RED_COLOR_CODE};")
        self.LE_State.setText(cm_state.control_state.name)

        self.LE_Tab2XAxis.setText(str(self.S_Tab2XAxis.value()) + "%")
        self.LE_Tab2YAxis.setText(str(self.S_Tab2YAxis.value()) + "%")
        self.LE_Tab2ZAxis.setText(str(self.S_Tab2ZAxis.value()) + "%")
        self.LE_Tab2Roll.setText(str(self.D_Tab2Roll.value()) + "%")
        self.LE_Tab2Yaw.setText(str(self.D_Tab2Yaw.value()) + "%")

    async def goto_zero(self):
        handler = self.robot.send_command(make_joint_control_command([0.] * 20, 5))
        await asyncio.to_thread(Robot_A_CommandHandler.get, handler)

    async def goto_ready(self):
        target_joint = np.array([0, 35, -70, 35, 0, 0] + [-30, -10, 0, -100, 0, 40, 0] + [-30, 10, 0, -100, 0, 40, 0])
        handler = self.robot.send_command(make_joint_control_command(np.deg2rad(target_joint), 5))
        await asyncio.to_thread(Robot_A_CommandHandler.get, handler)

    async def tab1_start(self):
        self.PB_Tab1Start.setEnabled(False)
        self.PB_Tab1Stop.setEnabled(True)

        robot_state = await asyncio.to_thread(Robot_A.get_state, self.robot)

        stream = self.robot.create_command_stream()
        self.cancel_command = lambda: stream.cancel()
        try:
            while not stream.is_done():
                rc = RobotCommandBuilder().set_command(
                    ComponentBasedCommandBuilder().set_body_command(
                        BodyComponentBasedCommandBuilder()
                        .set_torso_command(OptimalControlCommandBuilder()
                                           .set_command_header(CommandHeaderBuilder().set_control_hold_time(1.))
                                           .add_cartesian_target("base", "link_torso_5", np.eye(4), 0, 0.3)
                                           .set_center_of_mass_target("base", robot_state.center_of_mass, 0.3)
                                           .set_velocity_tracking_gain(0.02)
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

    async def tab2_start(self):
        await self.tab2_reset()

        self.PB_Tab2Start.setEnabled(False)
        self.PB_Tab2Stop.setEnabled(True)

        self.last_count = self.callback_count
        await asyncio.sleep(0.1)

        if self.last_count < self.callback_count:
            await asyncio.sleep(0)

            self.dyn_state.set_q(self.position)
            self.dyn_robot.compute_forward_kinematics(self.dyn_state)
            init_right_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 1)
            init_left_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 2)
            init_torso_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 3)

            X_Length = 0.08 # (m)
            Y_Length = 0.08 # (m)
            Z_Length = 0.15 # (m)
            Roll_Angle = 15 / 180. * np.pi
            Yaw_Angle = 20 / 180. * np.pi

            filter_constant = 0.15

            x_value = self.S_Tab2XAxis.value()
            y_value = self.S_Tab2YAxis.value()
            z_value = self.S_Tab2ZAxis.value()
            roll_value = self.D_Tab2Roll.value()
            yaw_value = self.D_Tab2Yaw.value()

            stream = self.robot.create_command_stream()
            self.cancel_command = lambda: stream.cancel()
            try:
                while not stream.is_done():
                    x_value = (1. - filter_constant) * x_value + filter_constant * self.S_Tab2XAxis.value()
                    y_value = (1. - filter_constant) * y_value + filter_constant * self.S_Tab2YAxis.value()
                    z_value = (1. - filter_constant) * z_value + filter_constant * self.S_Tab2ZAxis.value()
                    roll_value = (1. - filter_constant) * roll_value + filter_constant * self.D_Tab2Roll.value()
                    yaw_value = (1. - filter_constant) * yaw_value + filter_constant * self.D_Tab2Yaw.value()

                    offset = np.eye(4)
                    offset[0, 3] = x_value / 100. * X_Length
                    offset[1, 3] = y_value / 100. * Y_Length
                    offset[2, 3] = z_value / 100. * Z_Length
                    offset[0:3, 0:3] = Rz(yaw_value / 100 * Yaw_Angle) * Rx(roll_value / 100 * Roll_Angle) 
                    torso_T = init_torso_T @ offset

                    rc = RobotCommandBuilder().set_command(
                        ComponentBasedCommandBuilder().set_body_command(
                            BodyComponentBasedCommandBuilder()
                            .set_torso_command(CartesianCommandBuilder()
                                                   .add_target("base", "link_torso_5", torso_T, 
                                                               0.8, # (m/2)
                                                               np.pi * 0.3, # (rad/s)
                                                               0.1)
                                                   .set_minimum_time(0.2)
                                                   .set_stop_orientation_tracking_error(0)
                                                   .set_stop_position_tracking_error(0))
                            .set_right_arm_command(CartesianCommandBuilder()
                                                   .add_target("base", "link_right_arm_5", init_right_T, 
                                                               0.3, # (m/2)
                                                               np.pi * 1, # (rad/s)
                                                               0.8)
                                                   .set_minimum_time(0.2)
                                                   .set_stop_orientation_tracking_error(0)
                                                   .set_stop_position_tracking_error(0))
                            .set_left_arm_command(CartesianCommandBuilder()
                                                   .add_target("base", "link_left_arm_5", init_left_T, 
                                                               0.3, # (m/2)
                                                               np.pi * 1, # (rad/s)
                                                               0.8)
                                                   .set_minimum_time(0.2)
                                                   .set_stop_orientation_tracking_error(0)
                                                   .set_stop_position_tracking_error(0))
                        )
                    )
                    await asyncio.to_thread(Robot_A_CommandStreamHandler.send_command, stream, rc)
                    await asyncio.sleep(0.1)
            except:
                print("cancelled stream")
            self.cancel_command = None
        
        self.PB_Tab2Start.setEnabled(True)
        self.PB_Tab2Stop.setEnabled(False)
        

    async def tab2_stop(self):
        if self.cancel_command:
            self.cancel_command()

    async def tab2_reset(self):
        self.S_Tab2XAxis.setValue(0)
        self.S_Tab2YAxis.setValue(0)
        self.S_Tab2ZAxis.setValue(0)
        self.D_Tab2Roll.setValue(0)
        self.D_Tab2Yaw.setValue(0)

    async def tab3_start(self):
        self.PB_Tab3Start.setEnabled(False)
        self.PB_Tab3Stop.setEnabled(True)

        self.last_count = self.callback_count
        await asyncio.sleep(0.1)

        if self.last_count < self.callback_count:
            await asyncio.sleep(0)

            self.dyn_state.set_q(self.position)
            self.dyn_robot.compute_forward_kinematics(self.dyn_state)
            init_right_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 4)
            init_left_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 5)

            stream = self.robot.create_command_stream()
            self.cancel_command = lambda: stream.cancel()
            try:
                while not stream.is_done():
                    self.dyn_state.set_q(self.position)
                    self.dyn_robot.compute_forward_kinematics(self.dyn_state)
                    right_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 4)
                    left_T = self.dyn_robot.compute_transformation(self.dyn_state, 0, 5)
                    delta_right_T = np.linalg.inv(init_right_T) @ right_T
                    delta_left_T = np.linalg.inv(init_left_T) @ left_T
                    delta_right_T[1, 3] = -delta_right_T[1, 3]
                    delta_left_T[1, 3] = -delta_left_T[1, 3]

                    rc = RobotCommandBuilder().set_command(
                        ComponentBasedCommandBuilder().set_body_command(
                            BodyComponentBasedCommandBuilder()
                            .set_right_arm_command(ImpedanceControlCommandBuilder()
                                                   .set_command_header(CommandHeaderBuilder().set_control_hold_time(0.5))
                                                   .set_reference_link_name("base")
                                                   .set_link_name("ee_right")
                                                   .set_translation_weight([1000, 1000, 1000])
                                                   .set_rotation_weight([50, 50, 50])
                                                   .set_transformation(init_right_T @ delta_left_T))
                            # .set_left_arm_command(ImpedanceControlCommandBuilder()
                            #                       .set_command_header(CommandHeaderBuilder().set_control_hold_time(0.5))
                            #                       .set_reference_link_name("base")
                            #                       .set_link_name("ee_left")
                            #                       .set_translation_weight([1000, 1000, 1000])
                            #                       .set_rotation_weight([100, 100, 100])
                            #                       .set_transformation(init_left_T @ delta_right_T))
                            .set_left_arm_command(GravityCompensationCommandBuilder()
                                                   .set_command_header(CommandHeaderBuilder().set_control_hold_time(1.))
                                                   .set_on(self.switch[1]))
                        )
                    )
                    await asyncio.to_thread(Robot_A_CommandStreamHandler.send_command, stream, rc)
                    await asyncio.sleep(0.01)
            except:
                print("cancelled stream")
            self.cancel_command = None
        
        self.PB_Tab3Start.setEnabled(True)
        self.PB_Tab3Stop.setEnabled(False)
        

    async def tab3_stop(self):
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
