import sys
import os
import signal
import argparse
import asyncio
import json

from ui_form import Ui_MainWindow
# from PySide6.QtGui import Qt
from PySide6.QtWidgets import QMainWindow, QApplication, QMessageBox
from PySide6.QtCore import QSocketNotifier

import zmq

COLOR_GREEN = '#55a194'
COLOR_BLUE = '#1e85f7'
COLOR_RED = '#f16a6f'


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

    def setupUi(self, main_windows):
        super(DataCollectorGui, self).setupUi(self)

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
        self.S_Yaw.sliderReleased.connect(lambda: self.S_Yaw.setValue(0))
        self.S_Yaw.actionTriggered.connect(lambda e: self.S_Yaw.setValue(0) if 0 < e < 7 else 0)
        self.S_Pitch.sliderReleased.connect(lambda: self.S_Pitch.setValue(0))
        self.S_Pitch.actionTriggered.connect(lambda e: self.S_Pitch.setValue(0) if 0 < e < 7 else 0)
        self.S_Y.sliderReleased.connect(lambda: self.S_Y.setValue(0))
        self.S_Y.actionTriggered.connect(lambda e: self.S_Y.setValue(0) if 0 < e < 7 else 0)
        self.S_Z.sliderReleased.connect(lambda: self.S_Z.setValue(0))
        self.S_Z.actionTriggered.connect(lambda e: self.S_Z.setValue(0) if 0 < e < 7 else 0)

        # self.showFullScreen()

    def setupZMQ(self):
        self.ctx = zmq.Context.instance()

        cmd_url = f"tcp://{self.address}:{self.cmd_port}"
        print(cmd_url)
        self.dealer = self.ctx.socket(zmq.DEALER)
        self.dealer.setsockopt(zmq.IDENTITY, b'client')
        self.dealer.connect(cmd_url)

        sub_url = f"tcp://{self.address}:{self.data_port}"
        print(sub_url)
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

                    if data["robot"] is not None:
                        self.set_background_color(self.LE_Robot, COLOR_GREEN if data["robot"] else COLOR_RED)

                    if data["camera"] is not None:
                        self.set_background_color(self.LE_Camera, COLOR_GREEN if data["camera"] else COLOR_RED)

                    if data["master_arm"] is not None:
                        self.set_background_color(self.LE_MasterArm, COLOR_GREEN if data["master_arm"] else COLOR_RED)

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

    @staticmethod
    def set_background_color(le, color):
        le.setStyleSheet(f"background-color: {color}")


if __name__ == '__main__':
    os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"

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
