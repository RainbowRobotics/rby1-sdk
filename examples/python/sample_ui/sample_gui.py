import math
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'generated', 'python'))

import grpc
import asyncio
from enum import Enum

from robot_state import RobotState
from power_states_table_model import PowerStatesTableModel
from joint_states_table_model import JointStatesTableModel
from joint_position_table_model import JointPositionTableModel, SpinBoxDelegate
from common import *
import PySide6.QtAsyncio as QtAsyncio
from PySide6.QtWidgets import QWidget, QApplication, QHeaderView
from PySide6.QtCore import Qt, Slot, QTimer, QAbstractTableModel, QModelIndex
from PySide6.QtWidgets import (QApplication, QMainWindow, QTableView, QSpinBox,
                               QStyledItemDelegate, QVBoxLayout, QWidget, QAbstractItemView, QHeaderView)
from PySide6.QtCore import Qt, QAbstractTableModel, QModelIndex
from sample_form import Ui_MainForm
import rb.api.header_pb2 as header_pb2
import rb.api.power_pb2 as power_pb2
import rb.api.power_service_pb2_grpc as power_service_pb2_grpc
import rb.api.control_manager_pb2 as control_manager_pb2
import rb.api.control_manager_service_pb2_grpc as control_manager_service_pb2_grpc
import rb.api.robot_state_pb2 as robot_state_pb2
import rb.api.robot_state_service_pb2_grpc as robot_state_service_pb2_grpc
import rb.api.robot_info_pb2 as robot_info_pb2
import rb.api.robot_info_service_pb2_grpc as robot_info_service_prb2_grpc
import rb.api.robot_command_pb2 as robot_command_pb2
import rb.api.robot_command_service_pb2_grpc as robot_command_service_pb2_grpc
import rb.api.whole_body_command_pb2 as whole_body_command_pb2
import rb.api.body_command_pb2 as body_command_pb2
import rb.api.basic_command_pb2 as basic_command_pb2
import rb.api.command_header_pb2 as command_header_pb2
import rb.api.component_based_command_pb2 as component_based_command_pb2
import google.protobuf.duration_pb2 as duration_pb2


def make_request_header():
    return header_pb2.RequestHeader(request_timestamp=get_current_time_in_timestamp())


class ControlCommand(Enum):
    Enable = 1
    Disable = 2
    ResetFault = 3


class SampleGui(QWidget, Ui_MainForm):

    def __init__(self, parent=None):
        super(SampleGui, self).__init__(parent)
        self.setupUi(self)
        self.setStyleSheet("QPushButton:disabled { color: gray }")

        self.connected = False
        self.retry_connect = 0

        self.channel = None
        self.power_service = None
        self.robot_stat_service = None
        self.control_manager_service = None
        self.robot_info_service = None
        self.robot_command_service = None
        self.time_scale_init = False

        self.power_table_model = PowerStatesTableModel()
        self.TV_PowerStates.setModel(self.power_table_model)
        self.TV_PowerStates.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        self.joint_table_model = JointStatesTableModel()
        self.TV_JointStates.setModel(self.joint_table_model)
        self.TV_JointStates.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.TV_JointStates_width_ratios = [4, 1, 1, 1, 1, 1, 1, 4, 4, 4, 4]

        self.connection_check_timer = QTimer()
        self.connection_check_timer.timeout.connect(lambda: asyncio.ensure_future(self.connection_check()))
        self.connection_check_timer.setInterval(500)
        self.connection_check_timer.start()

        self.robot_disconnected()

        self.BTN_RobotConnect.clicked.connect(lambda: asyncio.ensure_future(self.connect_robot()))
        self.PB_PowerOn.clicked.connect(lambda: asyncio.ensure_future(self.power_command(True)))
        self.PB_PowerOff.clicked.connect(lambda: asyncio.ensure_future(self.power_command(False)))
        self.PB_JointServoOn.clicked.connect(lambda: asyncio.ensure_future(self.joint_servo_on()))
        self.PB_BrakeEngage.clicked.connect(lambda: asyncio.ensure_future(self.brake_command(True)))
        self.PB_BrakeRelease.clicked.connect(lambda: asyncio.ensure_future(self.brake_command(False)))
        self.PB_ZPReset.clicked.connect(lambda: asyncio.ensure_future(self.home_offset_reset()))
        self.PB_CMEnable.clicked.connect(
            lambda: asyncio.ensure_future(self.control_manager_command(ControlCommand.Enable)))
        self.PB_CMDisable.clicked.connect(
            lambda: asyncio.ensure_future(self.control_manager_command(ControlCommand.Disable)))
        self.PB_CMResetFault.clicked.connect(
            lambda: asyncio.ensure_future(self.control_manager_command(ControlCommand.ResetFault)))
        self.HS_TimeScale.valueChanged.connect(
            lambda value: asyncio.ensure_future(self.time_scale_value_changed(value)))
        self.PB_JogPositive.clicked.connect(lambda: asyncio.ensure_future(self.jog_positive()))
        self.PB_JogNegative.clicked.connect(lambda: asyncio.ensure_future(self.jog_negative()))
        self.PB_JogRelative.clicked.connect(lambda: asyncio.ensure_future(self.jog_relative()))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_TV_JointStates_column_widths()

    async def jp_reflect(self):
        for i, idx in enumerate(self.joint_position_table_model.target_index):
            self.joint_position_table_model.target_position[i] = \
                np.rad2deg(RobotState.joint_states[idx]["position"])
        self.joint_position_table_model.emitAllDataChanged()

    def update_TV_JointStates_column_widths(self):
        total_width = self.TV_JointStates.viewport().width()
        total_ratio = sum(self.TV_JointStates_width_ratios)
        for i, ratio in enumerate(self.TV_JointStates_width_ratios):
            self.TV_JointStates.horizontalHeader().resizeSection(i, total_width * ratio / total_ratio)

    async def connection_check(self):
        if self.channel:
            if self.channel.get_state(try_to_connect=True) == grpc.ChannelConnectivity.READY:
                self.retry_connect = 0
                if not self.connected:
                    self.robot_connected()
                return
            else:
                self.retry_connect += 1
                print(f"retry (iteration: {self.retry_connect})")
                if self.retry_connect >= 10:
                    self.retry_connect = 0
                    self.channel = None
                    self.BTN_RobotConnect.setEnabled(True)
        self.robot_disconnected()

    async def initialize(self):
        robot_info = await self.robot_info_service.GetRobotInfo(robot_info_pb2.GetRobotInfoRequest())
        # print(robot_info)

        # Power
        RobotState.power_states = []
        for p in robot_info.robot_info.power_infos:
            RobotState.power_states += [{"name": p.name, "voltage": 0., "state": False}]
        self.power_table_model.layoutChanged.emit()

        self.CB_PowerList.clear()
        self.CB_PowerList.addItem(".*")
        self.CB_PowerList.addItems([power["name"] for power in RobotState.power_states])

        # Joint
        RobotState.joint_states = []
        for j in robot_info.robot_info.joint_infos:
            RobotState.joint_states += [
                {"name": j.name, "fet": False, "run": False, "init": False, "motor_type": 0, "motor_state": 0,
                 "position": 0., "velocity": 0., "current": 0., "torque": 0.}]
        self.joint_table_model.layoutChanged.emit()

        self.CB_JointList.clear()
        self.CB_JointList.addItem(".*")
        self.CB_JointList.addItems([joint["name"] for joint in RobotState.joint_states])

        self.joint_position_table_model = JointPositionTableModel(robot_info.robot_info.body_joint_idx)
        self.joint_position_table_model.layoutChanged.emit()

        # await self.set_time_scale(self.HS_TimeScale.value() / 100.)

        asyncio.ensure_future(self.update_state(), loop=asyncio.get_event_loop())

    def robot_connected(self):
        self.LE_RobotConnection.setStyleSheet(f"background-color: {GREEN_CODE};")
        self.PB_PowerOn.setEnabled(True)
        self.PB_PowerOff.setEnabled(True)
        self.PB_JointServoOn.setEnabled(True)
        self.PB_BrakeEngage.setEnabled(True)
        self.PB_BrakeRelease.setEnabled(True)
        self.PB_CMEnable.setEnabled(True)
        self.PB_CMDisable.setEnabled(True)
        self.PB_CMResetFault.setEnabled(True)
        self.PB_ZPReset.setEnabled(True)
        self.LE_TimeScale.setEnabled(True)
        self.HS_TimeScale.setEnabled(True)
        self.connected = True

    def robot_disconnected(self):
        self.LE_RobotConnection.setStyleSheet(f"background-color: {RED_CODE};")
        self.PB_PowerOn.setEnabled(False)
        self.PB_PowerOff.setEnabled(False)
        self.PB_JointServoOn.setEnabled(False)
        self.PB_BrakeEngage.setEnabled(False)
        self.PB_BrakeRelease.setEnabled(False)
        self.PB_CMEnable.setEnabled(False)
        self.PB_CMDisable.setEnabled(False)
        self.PB_CMResetFault.setEnabled(False)
        self.PB_ZPReset.setEnabled(False)
        self.LE_TimeScale.setEnabled(False)
        self.HS_TimeScale.setEnabled(False)
        self.PB_JogPositive.setEnabled(False)
        self.PB_JogNegative.setEnabled(False)
        self.SB_JogRelative.setEnabled(False)
        self.PB_JogRelative.setEnabled(False)
        self.connected = False

    async def connect_robot(self):
        self.BTN_RobotConnect.setEnabled(False)

        self.channel = grpc.aio.insecure_channel(self.LE_RobotAddress.text())
        await self.channel.channel_ready()

        self.power_service = power_service_pb2_grpc.PowerServiceStub(self.channel)
        self.robot_stat_service = robot_state_service_pb2_grpc.RobotStateServiceStub(self.channel)
        self.control_manager_service = control_manager_service_pb2_grpc.ControlManagerServiceStub(self.channel)
        self.robot_info_service = robot_info_service_prb2_grpc.RobotInfoServiceStub(self.channel)
        self.robot_command_service = robot_command_service_pb2_grpc.RobotCommandServiceStub(self.channel)
        self.robot_connected()

        await self.initialize()

    async def power_command(self, command):
        await self.power_service.PowerCommand(power_pb2.PowerCommandRequest(request_header=make_request_header(),
                                                                            name=self.CB_PowerList.currentText(),
                                                                            command=power_pb2.PowerCommandRequest.COMMAND_POWER_ON if command else power_pb2.PowerCommandRequest.COMMAND_POWER_OFF))

    async def joint_servo_on(self):
        await self.power_service.JointCommand(
            power_pb2.JointCommandRequest(request_header=make_request_header(), name=self.CB_JointList.currentText(),
                                          command=power_pb2.JointCommandRequest.COMMAND_SERVO_ON))

    async def brake_command(self, command):
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=self.CB_JointList.currentText(),
                                                                            command=power_pb2.JointCommandRequest.COMMAND_BRAKE_ENGAGE if command else power_pb2.JointCommandRequest.COMMAND_BRAKE_RELEASE))

    async def home_offset_reset(self):
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=self.CB_JointList.currentText(),
                                                                            command=power_pb2.JointCommandRequest.COMMAND_HOME_OFFSET_RST))

    async def jog_positive(self):
        if self.robot_command_service is None:
            return

        await self.robot_command_service.RobotCommand(
            robot_command_pb2.RobotCommandRequest(
                robot_command=robot_command_pb2.RobotCommand.Request(jog_command=basic_command_pb2.JogCommand.Request(
                    joint_name=self.CB_JointList.currentText(),
                    one_step=True
                ))))

    async def jog_relative(self):
        if self.robot_command_service is None:
            return

        await self.robot_command_service.RobotCommand(
            robot_command_pb2.RobotCommandRequest(
                robot_command=robot_command_pb2.RobotCommand.Request(jog_command=basic_command_pb2.JogCommand.Request(
                    joint_name=self.CB_JointList.currentText(),
                    relative_position=np.deg2rad(self.SB_JogRelative.value())
                ))))

    async def jog_negative(self):
        if self.robot_command_service is None:
            return

        await self.robot_command_service.RobotCommand(
            robot_command_pb2.RobotCommandRequest(
                robot_command=robot_command_pb2.RobotCommand.Request(jog_command=basic_command_pb2.JogCommand.Request(
                    joint_name=self.CB_JointList.currentText(),
                    one_step=False
                ))))

    async def control_manager_command(self, command: ControlCommand):
        if command == ControlCommand.Enable:
            cmd = control_manager_pb2.ControlManagerCommandRequest.COMMAND_ENABLE
        elif command == ControlCommand.Disable:
            cmd = control_manager_pb2.ControlManagerCommandRequest.COMMAND_DISABLE
        elif command == ControlCommand.ResetFault:
            cmd = control_manager_pb2.ControlManagerCommandRequest.COMMAND_RESET_FAULT
        else:
            return
        await self.control_manager_service.ControlManagerCommand(
            control_manager_pb2.ControlManagerCommandRequest(request_header=make_request_header(), command=cmd))

    async def update_state(self):
        async for state in self.robot_stat_service.GetRobotStateStream(
                robot_state_pb2.GetRobotStateStreamRequest(request_header=make_request_header(), update_rate=10)):
            self.LE_BatteryVoltage.setText(f"{state.robot_state.battery_state.voltage:2.3f} V")
            self.LE_BatteryCurrent.setText(f"{state.robot_state.battery_state.current:2.3f} Amp")

            for i, ps in enumerate(state.robot_state.power_states):
                RobotState.power_states[i]["voltage"] = ps.voltage
                RobotState.power_states[i]["state"] = (ps.state == robot_state_pb2.PowerState.STATE_POWER_ON)
            self.power_table_model.emitAllDataChanged()

            for i, js in enumerate(state.robot_state.joint_states):
                RobotState.joint_states[i]["fet"] = (js.fet_state == robot_state_pb2.JointState.FET_STATE_ON)
                RobotState.joint_states[i]["run"] = (js.run_state == robot_state_pb2.JointState.RUN_STATE_CONTROL_ON)
                RobotState.joint_states[i]["init"] = (
                        js.init_state == robot_state_pb2.JointState.INIT_STATE_INITIALIZED)
                RobotState.joint_states[i]["motor_type"] = js.motor_type
                RobotState.joint_states[i]["motor_state"] = js.motor_state
                RobotState.joint_states[i]["position"] = js.position
                RobotState.joint_states[i]["velocity"] = js.velocity
                RobotState.joint_states[i]["current"] = js.current
                RobotState.joint_states[i]["torque"] = js.torque
            self.joint_table_model.emitAllDataChanged()

            if not self.time_scale_init:
                self.HS_TimeScale.setValue(state.control_manager_state.time_scale * 100)
                self.time_scale_init = True

            self.LE_CMState.setText(CM_STATE_STRING[state.control_manager_state.state])
            self.LE_CMState.setStyleSheet(
                f"background-color: {GREEN_CODE if state.control_manager_state.state == control_manager_pb2.ControlManagerState.CONTROL_MANAGER_STATE_ENABLED else RED_CODE};")
            self.LE_TimeScale.setText(f"{state.control_manager_state.time_scale:2.3f}")

            self.LE_OdometryX.setText(f"{state.robot_state.odometry.position.x:2.3f}")
            self.LE_OdometryY.setText(f"{state.robot_state.odometry.position.y:2.3f}")
            self.LE_OdometryAngle.setText(f"{state.robot_state.odometry.angle:2.3f}")

            if state.control_manager_state.state == control_manager_pb2.ControlManagerState.CONTROL_MANAGER_STATE_ENABLED:
                self.PB_PowerOn.setEnabled(False)
                self.PB_PowerOff.setEnabled(False)
                self.PB_JointServoOn.setEnabled(False)
                self.PB_JogPositive.setEnabled(True)
                self.PB_JogNegative.setEnabled(True)
                self.SB_JogRelative.setEnabled(True)
                self.PB_JogRelative.setEnabled(True)
            else:
                self.PB_PowerOn.setEnabled(True)
                self.PB_PowerOff.setEnabled(True)
                self.PB_JointServoOn.setEnabled(True)
                self.PB_JogPositive.setEnabled(False)
                self.PB_JogNegative.setEnabled(False)
                self.SB_JogRelative.setEnabled(False)
                self.PB_JogRelative.setEnabled(False)

    async def time_scale_value_changed(self, value):
        await self.set_time_scale(value / 100)

    async def set_time_scale(self, value):
        await self.control_manager_service.SetTimeScale(
            control_manager_pb2.SetTimeScaleRequest(request_header=make_request_header(),
                                                    time_scale=value))


if __name__ == '__main__':
    app = QApplication()
    window = SampleGui()
    window.show()
    window.update_TV_JointStates_column_widths()

    QtAsyncio.run(handle_sigint=True)
