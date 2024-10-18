import sys
import os
import signal
import argparse
import asyncio
import json
import numpy as np
import cv2

from ui_form import Ui_MainWindow
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QMainWindow, QApplication, QMessageBox, \
    QGraphicsOpacityEffect
from PySide6.QtCore import QSocketNotifier, QTimer
from PySide6.QtGui import QIntValidator, QPixmap, Qt, QFont, QPainter, QColor

import zmq

COLOR_GREEN = '#55a194'
COLOR_BLUE = '#1e85f7'
COLOR_RED = '#f16a6f'


class CountdownOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setWindowModality(Qt.ApplicationModal)  # 다른 창 비활성화

        self.label = QLabel("", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("color: red;")
        self.label.setFont(QFont('Arial', 100))

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_countdown)
        self.count = -1

    def start_countdown(self):
        self.count = 5
        self.label.setText(str(self.count))
        self.resize(self.parent().size())
        self.show()
        self.timer.start(1000)

    def update_countdown(self):
        if self.count > 0:
            self.count -= 1
            self.label.setText(str(self.count))
        else:
            self.timer.stop()
            self.hide()
            self.parent().execute_task()

    def paintEvent(self, event):
        super().paintEvent(event)

        if self.count >= 0:
            # 검은색 반투명 배경 그리기
            painter = QPainter(self)
            painter.setBrush(QColor(0, 0, 0, 150))  # 알파값을 설정하여 투명도 조절
            painter.setPen(Qt.NoPen)
            painter.drawRect(self.rect())


class DataCollectorGui(QMainWindow, Ui_MainWindow):

    def __init__(self, address, cmd_port, data_port, parent=None):
        super(DataCollectorGui, self).__init__(parent)
        self.setupUi(self)

        self.address = address
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.ctx = None
        self.sub = None
        self.dealer = None
        self.notifier = None
        self.setupZMQ()

        self.overlay_count = CountdownOverlay(self)

    def setupUi(self, main_windows):
        super(DataCollectorGui, self).setupUi(self)

        self.LE_EpisodeNumber.setValidator(QIntValidator(bottom=0, parent=self))

        self.PB_Close.clicked.connect(self.quit)
        self.PB_Zero.clicked.connect(self.zero)
        self.PB_Ready.clicked.connect(self.ready)
        self.PB_PowerOn.clicked.connect(self.power_on)
        self.PB_PowerOff.clicked.connect(self.power_off)
        self.PB_ServoOn.clicked.connect(self.servo_on)
        self.PB_InitControlManager.clicked.connect(self.init_control_manager)
        self.PB_StartTeleoperation.clicked.connect(self.start_teleoperation)
        self.PB_StopMotion.clicked.connect(self.stop_motion)
        self.PB_PositionZero.clicked.connect(self.position_zero)
        self.PB_RotationZero.clicked.connect(self.rotation_zero)
        self.PB_StartRecording.clicked.connect(self.start_recording)
        self.PB_StopRecording.clicked.connect(self.stop_recording)
        self.PB_EpisodeNumberReset.clicked.connect(lambda: self.LE_EpisodeNumber.setText('0'))
        self.S_Yaw.sliderReleased.connect(lambda: self.S_Yaw.setValue(0))
        self.S_Yaw.actionTriggered.connect(lambda e: self.S_Yaw.setValue(0) if 0 < e < 7 else 0)
        self.S_Pitch.sliderReleased.connect(lambda: self.S_Pitch.setValue(0))
        self.S_Pitch.actionTriggered.connect(lambda e: self.S_Pitch.setValue(0) if 0 < e < 7 else 0)
        self.S_Y.sliderReleased.connect(lambda: self.S_Y.setValue(0))
        self.S_Y.actionTriggered.connect(lambda e: self.S_Y.setValue(0) if 0 < e < 7 else 0)
        self.S_Z.sliderReleased.connect(lambda: self.S_Z.setValue(0))
        self.S_Z.actionTriggered.connect(lambda e: self.S_Z.setValue(0) if 0 < e < 7 else 0)

        self.showFullScreen()

    def setupZMQ(self):
        self.ctx = zmq.Context.instance()

        cmd_url = f"tcp://{self.address}:{self.cmd_port}"
        # print(cmd_url)
        self.dealer = self.ctx.socket(zmq.DEALER)
        self.dealer.setsockopt(zmq.IDENTITY, b'client')
        self.dealer.connect(cmd_url)

        sub_url = f"tcp://{self.address}:{self.data_port}"
        # print(sub_url)
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(sub_url)
        self.sub.setsockopt(zmq.SUBSCRIBE, b'')

        self.notifier = QSocketNotifier(self.sub.getsockopt(zmq.FD), QSocketNotifier.Type.Read, self)
        self.notifier.activated.connect(self.subscribe)

    def quit(self):
        self.close()

    def position_zero(self):
        pass

    def rotation_zero(self):
        pass

    def robot_common_command(self, text: str, command: str):
        msg_box = QMessageBox()
        msg_box.setText(text)
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
        msg_box.setDefaultButton(QMessageBox.StandardButton.Ok)
        ret = msg_box.exec()

        if ret != QMessageBox.StandardButton.Ok:
            return

        if self.dealer is None:
            return  # ZMQ is not initialized yet

        command = {'command': command}
        self.dealer.send_multipart([json.dumps(command).encode('utf-8')])

    def power_on(self):
        self.robot_common_command("Do you want the robot to power on?", "power_on")

    def power_off(self):
        self.robot_common_command("Do you want the robot to power off?", "power_off")

    def servo_on(self):
        self.robot_common_command("Do you want the robot to servo on?", "servo_on")

    def init_control_manager(self):
        self.robot_common_command("Do you want the robot to initialize control manager?", "init_control_manager")

    def zero(self):
        self.robot_common_command("Do you want the robot to go zero pose?", "zero_pose")

    def ready(self):
        self.robot_common_command("Do you want the robot to go ready pose?", "ready_pose")

    def start_teleoperation(self):
        self.robot_common_command("Do you want to start tele-operation?", "start_teleoperation")

    def stop_motion(self):
        self.robot_common_command("Do you want to stop robot?", "stop_motion")

    def subscribe(self):
        self.notifier.setEnabled(False)

        flags = self.sub.getsockopt(zmq.EVENTS)
        # print("[Socket] socket.getsockopt(zmq.EVENTS): " + repr(flags))

        while flags:
            if flags & zmq.POLLIN:
                [topic, msg] = self.sub.recv_multipart()

                if topic == b"data":
                    data = json.loads(msg)

                    if data["power"] is not None:
                        self.set_background_color(self.LE_Power, COLOR_GREEN if data["power"] else COLOR_RED)

                    if data["servo"] is not None:
                        self.set_background_color(self.LE_Servo, COLOR_GREEN if data["servo"] else COLOR_RED)

                    if data["control_manager"] is not None:
                        cm = data["control_manager"]
                        s = cm["state"]
                        detail = cm["detail"]
                        self.set_background_color(self.LE_ControlManager, COLOR_GREEN if s else COLOR_RED)
                        self.LE_ControlManager.setText(detail)

                        if "Idle" in detail:
                            self.PB_Zero.setEnabled(True)
                            self.PB_Ready.setEnabled(True)
                            self.PB_StartTeleoperation.setEnabled(True)
                        else:
                            self.PB_Zero.setEnabled(False)
                            self.PB_Ready.setEnabled(False)
                            self.PB_StartTeleoperation.setEnabled(False)

                    if data["robot"] is not None:
                        self.set_background_color(self.LE_Robot, COLOR_GREEN if data["robot"] else COLOR_RED)

                    if data["camera"] is not None:
                        self.set_background_color(self.LE_Camera, COLOR_GREEN if data["camera"] else COLOR_RED)

                    if data["master_arm"] is not None:
                        self.set_background_color(self.LE_MasterArm, COLOR_GREEN if data["master_arm"] else COLOR_RED)

                    if data["storage_available"] is not None:
                        self.LE_StorageAvailable.setText(f"{data['storage_available']} MB")

                    if data["storage_free"] is not None:
                        self.LE_StorageFree.setText(f"{data['storage_free']} MB")

                    if data["storage_capacity"] is not None:
                        self.LE_StorageCapacity.setText(f"{data['storage_capacity']} MB")

                    if data["recording"] is not None:
                        recording = data["recording"]

                        if recording:
                            self.set_background_color(self.LE_Recording, COLOR_GREEN)
                            self.PB_StartRecording.setEnabled(False)
                            self.PB_StopRecording.setEnabled(True)
                        else:
                            self.set_background_color(self.LE_Recording, COLOR_RED)
                            self.PB_StartRecording.setEnabled(True)
                            self.PB_StopRecording.setEnabled(False)
                        # self.set_background_color(self.LE_Recording, COLOR_GREEN if data["recording"] else COLOR_RED)

                    if data["recording_count"] is not None:
                        self.L_RecordingCount.setText(f"{data['recording_count']}")

                if topic == b"image":
                    data = json.loads(msg)

                    if data["cam0_rgb"] is not None:
                        pixmap = QPixmap()
                        pixmap.loadFromData(bytes(data["cam0_rgb"]['bytes']))
                        scaled_pixmap = pixmap.scaled(self.L_Cam0RGB.size(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation)
                        self.L_Cam0RGB.setPixmap(scaled_pixmap)

                    if data["cam1_rgb"] is not None:
                        pixmap = QPixmap()
                        pixmap.loadFromData(bytes(data["cam1_rgb"]['bytes']))
                        scaled_pixmap = pixmap.scaled(self.L_Cam1RGB.size(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation)
                        self.L_Cam1RGB.setPixmap(scaled_pixmap)

                    if data["cam2_rgb"] is not None:
                        pixmap = QPixmap()
                        pixmap.loadFromData(bytes(data["cam2_rgb"]['bytes']))
                        scaled_pixmap = pixmap.scaled(self.L_Cam2RGB.size(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation)
                        self.L_Cam2RGB.setPixmap(scaled_pixmap)

                    if data["cam0_depth"] is not None:
                        pixmap = QPixmap()
                        pixmap.loadFromData(bytes(data["cam0_depth"]['bytes']))
                        scaled_pixmap = pixmap.scaled(self.L_Cam0Depth.size(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation)
                        self.L_Cam0Depth.setPixmap(scaled_pixmap)

                    if data["cam1_depth"] is not None:
                        pixmap = QPixmap()
                        pixmap.loadFromData(bytes(data["cam1_depth"]['bytes']))
                        scaled_pixmap = pixmap.scaled(self.L_Cam1Depth.size(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation)
                        self.L_Cam1Depth.setPixmap(scaled_pixmap)

                    if data["cam2_depth"] is not None:
                        pixmap = QPixmap()
                        pixmap.loadFromData(bytes(data["cam2_depth"]['bytes']))
                        scaled_pixmap = pixmap.scaled(self.L_Cam2Depth.size(), Qt.KeepAspectRatio,
                                                      Qt.SmoothTransformation)
                        self.L_Cam2Depth.setPixmap(scaled_pixmap)

                # print("[Socket] zmq.POLLIN")
                # print("[Socket] received: " + repr(msg))
            # elif flags & zmq.POLLOUT:
            #     print("[Socket] zmq.POLLOUT")
            # elif flags & zmq.POLLERR:
            #     print("[Socket] zmq.POLLERR")
            else:
                print("[Socket] FAILURE")

            # Check for more messages
            flags = self.sub.getsockopt(zmq.EVENTS)
            # print("[Socket] socket.getsockopt(zmq.EVENTS): " + repr(flags))

        self.notifier.setEnabled(True)

    def start_recording(self):
        if self.LE_EpisodeName.text() == "":
            self.LE_EpisodeName.setText('episode')

        if self.LE_EpisodeNumber.text() == "":
            self.LE_EpisodeNumber.setText('0')

        if self.dealer is None:
            return  # ZMQ is not initialized yet

        self.overlay_count.start_countdown()

    def stop_recording(self):
        if self.dealer is None:
            return  # ZMQ is not initialized yet

        command = {'command': 'stop_recording'}
        self.dealer.send_multipart([json.dumps(command).encode('utf-8')])

        if self.LE_EpisodeNumber.text() == "":
            self.LE_EpisodeNumber.setText('0')
        else:
            self.LE_EpisodeNumber.setText(f"{int(self.LE_EpisodeNumber.text()) + 1}")

    def execute_task(self):
        name = f"{self.LE_EpisodeName.text()}_{self.LE_EpisodeNumber.text()}"
        command = {'command': 'start_recording', 'name': name}
        self.dealer.send_multipart([json.dumps(command).encode('utf-8')])

    @staticmethod
    def set_background_color(le, color):
        le.setStyleSheet(f"background-color: {color}")


if __name__ == '__main__':
    # os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"

    parser = argparse.ArgumentParser(description="data_collector_gui")
    parser.add_argument('--address', type=str, required=True, help="UPC address")
    parser.add_argument('--cmd_port', type=int, default=5000, help="UPC address")
    parser.add_argument('--data_port', type=int, default=5001, help="UPC address")
    args = parser.parse_args()

    app = QApplication()
    window = DataCollectorGui(address=args.address,
                              cmd_port=args.cmd_port,
                              data_port=args.data_port)
    window.show()

    # Ensure that the application quits using CTRL-C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app.exec()
