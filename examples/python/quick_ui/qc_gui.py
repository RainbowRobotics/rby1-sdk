# -*- coding: utf-8 -*-
import grpc
import asyncio
import PySide6.QtAsyncio as QtAsyncio
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QLineEdit, QDoubleSpinBox, QPushButton, QApplication, \
    QTextEdit, QMessageBox, QSplashScreen, QVBoxLayout, QSlider
from PySide6.QtCore import Qt, Slot, QTimer, QThread, Signal
import os, sys, time, math
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'generated', 'python'))
import rb.api.power_pb2 as power_pb2
import rb.api.power_service_pb2_grpc as power_service_pb2_grpc
from functools import partial

import numpy as np
import subprocess
import argparse
import rby1_sdk
from rby1_sdk import *

IS_READY = None
CUR_POSITION =None
CONTROL_MANAGER_STATE = None
IS_POWER = None

def STATE_CALL_BACK(robot_state, control_manager_state: rby1_sdk.ControlManagerState):
    global IS_READY, CUR_POSITION, CONTROL_MANAGER_STATE, IS_POWER
    IS_READY = robot_state.is_ready
    CUR_POSITION = robot_state.position
    CONTROL_MANAGER_STATE = control_manager_state.state
    IS_POWER = True if robot_state.power_states[-1].voltage > 40 else False  

    
class SplashScreen(QSplashScreen):
    def __init__(self):
        super().__init__()
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.setStyleSheet("background-color: #2d2d2d; color: white; font-size: 48px;")  # 폰트 크기를 48로 설정

        # 텍스트 레이블 추가
        self.label = QLabel("Loading... Please wait", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 64px;")  # 폰트 크기를 48로 설정

        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        # 창의 크기 설정
        self.setFixedSize(800, 400)  # 스플래쉬 화면 크기 설정 (가로 800, 세로 600)

    def show_message(self, message):
        self.label.setText(message)


TITLE_STYLE_SHEET = "font-size: 16px; font-weight: bold; margin-top: 10;font-size: 14px;"
IDLE_STATE = "background-color: lightgray;"
ACTIVATE_STATE = "background-color: #A6D256;"
FAULT_STATE = "background-color: #ED325A;"

class ButtonGrid(QWidget):
    def __init__(self, address, device, model, servo):
        super().__init__()
        # 각 프로세스를 관리할 딕셔너리
        self.processes = {}
        self.jog_task = None  # 비동기 작업을 관리할 변수를 추가

        self.address = address
        self.device = device
        self.model = model
        self.servo = servo
        self.robot = create_robot(self.address, self.model)
        self.splash_screen = SplashScreen()

        # 전체 레이아웃
        self.layout = QGridLayout()

        # 공통 섹션
        common_title = QLabel("공통 작업")
        common_title.setStyleSheet(TITLE_STYLE_SHEET)
        self.layout.addWidget(common_title, 0, 0, 1, 4)        

        self.time_scale_label = QLabel("time_scale: 0.0")
        self.time_scale_slider = QSlider(Qt.Horizontal)
        self.time_scale_slider.setRange(0, 10)  # 0에서 10까지 설정 (0.1 단위로 만들기 위해 10배)
        self.time_scale_slider.setSingleStep(1)  # 0.1 단위로 변경
        # self.time_scale_slider.valueChanged.connect(lambda value: asyncio.ensure_future(self.time_scale_value_changed(value)))
        self.time_scale_slider.valueChanged.connect(self.time_scale_value_changed)
        
        self.layout.addWidget(self.time_scale_label, 1, 4, 1, 2)
        self.layout.addWidget(self.time_scale_slider, 2, 4, 1, 2)
        
        # Connect state 설정
        self.connect_state = QLineEdit("None")
        self.connect_state.setReadOnly(True)  # 읽기 전용
        self.connect_state.setFocusPolicy(Qt.NoFocus)  # 선택 안되게 설정
        self.connect_state.setStyleSheet(IDLE_STATE)  # 배경색 설정
        self.layout.addWidget(self.connect_state, 1, 0, 1, 1)

        # Control state 설정
        self.control_state = QLineEdit("Idle")
        self.control_state.setReadOnly(True)  # 읽기 전용
        self.control_state.setFocusPolicy(Qt.NoFocus)  # 선택 안되게 설정
        self.control_state.setStyleSheet(IDLE_STATE)  # 배경색 설정
        self.layout.addWidget(self.control_state, 3, 0, 1, 1)
        # 공통 버튼
        common_buttons = [
            ("Connect", 1, 1, lambda: asyncio.ensure_future(self.connect())),
            ("Power On", 1, 2, self.power_on),
            ("Power Off", 1, 3, self.power_off),
            ("Control Reset", 3, 1, self.control_reset),
            ("Control Enable", 3, 2, self.control_enable),
            ("Control Disable", 3, 3, self.control_disable)
        ]
        for text, row, col, slot in common_buttons:
            self.create_button(self.layout, text, row, col, slot)

        # Jog Value 라벨 및 Line Edit 추가
        jog_label = QLabel("Jog Value")
        jog_label.setStyleSheet("font-size: 14px;")
        self.layout.addWidget(jog_label, 2, 0)
        
        self.jog_value_spinbox = QDoubleSpinBox()  # QDoubleSpinBox 사용
        self.jog_value_spinbox.setDecimals(1)  # 소수점 자릿수 설정
        self.jog_value_spinbox.setSingleStep(0.1)  # 0.1 단위로 조정
        self.jog_value_spinbox.setMinimum(0.1)  # 최소값 0.1로 설정
        self.jog_value_spinbox.setMaximum(10.0)
        self.jog_value_spinbox.setValue(1.0)
        self.layout.addWidget(self.jog_value_spinbox, 2, 1)
        
        self.create_button(self.layout, "팔+다리 Servo On", 2, 2, self.leg_arm_servo_on, 1, background_color="#589FEF")
        self.create_button(self.layout, "홈 자세", 4, 0, lambda: asyncio.ensure_future(self.go_to_zero_pose()) , 1)
        self.create_button(self.layout, "패킹 자세", 4, 1, lambda: asyncio.ensure_future(self.go_to_packing_pose()), 1)
        self.create_button(self.layout, "브레이크 테스트 자세1", 4, 2, lambda: asyncio.ensure_future(self.go_to_break1()), 1)
        self.create_button(self.layout, "브레이크 테스트 자세2", 4, 3, lambda: asyncio.ensure_future(self.go_to_break2()), 1)

        # 다리 QC 섹션
        leg_title = QLabel("다리 QC")
        leg_title.setStyleSheet(TITLE_STYLE_SHEET)
        self.layout.addWidget(leg_title, 5, 0, 1, 4)

        # 다리 관련 버튼 (reset, servo on)
        
        self.create_button(self.layout, "다리 전체 Servo On(순차)", 6, 0, self.leg_servo_on, col_span=2, background_color="#589FEF")
        self.create_button(self.layout, "다리 전체 Servo On(한번에)", 6, 2, self.leg_servo_on2, col_span=2, background_color="#589FEF")

        # 다리 플러스/마이너스 버튼 (T0, T1, ...)
        # 다리 플러스/마이너스 버튼 (T0, T1, ...)
        leg_buttons_torso = [
            ("T0 +", 7, 0, 0, True), ("T0 -", 7, 2, 0, False),
            ("T1 +", 8, 0, 1, True), ("T1 -", 8, 2, 1, False),
            ("T2 +", 9, 0, 2, True), ("T2 -", 9, 2, 2, False),
            ("T3 +", 10, 0, 3, True), ("T3 -", 10, 2, 3, False),
            ("T4 +", 11, 0, 4, True), ("T4 -", 11, 2, 4, False),
            ("T5 +", 12, 0, 5, True), ("T5 -", 12, 2, 5, False)
        ]
        for text, row, col, idx, direction in leg_buttons_torso:
            joint_name = f"torso_{idx}"
            label = QLabel(joint_name)
            self.layout.addWidget(label, row, 4)
            self.create_button(self.layout, text, row, col, lambda joint_name=joint_name, direction=direction: asyncio.ensure_future(self.jog_execute(joint_name, direction)), col_span=2)

        

        self.create_button(self.layout, "다리 전체 홈 리셋", 13, 0, lambda: asyncio.ensure_future(self.leg_reset()), col_span=4, background_color="#ED325A")

        # 팔 QC 섹션
        arm_title = QLabel("팔 조립체 QC")
        arm_title.setStyleSheet(TITLE_STYLE_SHEET)
        self.layout.addWidget(arm_title, 14, 0, 1, 4)
        
        self.create_button(self.layout, "팔 전체 Servo On(순차)", 15, 0, self.arm_servo_on, col_span=2, background_color="#589FEF")
        self.create_button(self.layout, "팔 전체 Servo On(한번에)", 15, 2, self.arm_servo_on2, col_span=2, background_color="#589FEF")
                    
        # 오른팔 플러스/마이너스 버튼 (R0, R1, ...)
        right_arm_buttons = [
            ("R0 +", 16, 0, 0, True),   ("R0 -", 16, 1, 0, False),
            ("R1 +", 17, 0, 1, True),   ("R1 -", 17, 1, 1, False),
            ("R2 +", 18, 0, 2, True),   ("R2 -", 18, 1, 2, False),
            ("R3 +", 19, 0, 3, True),   ("R3 -", 19, 1, 3, False),
            ("R4 +", 20, 0, 4, True),   ("R4 -", 20, 1, 4, False),
            ("R5 +", 21, 0, 5, True),   ("R5 -", 21, 1, 5, False),
            ("R6 +", 22, 0, 6, True),   ("R6 -", 22, 1, 6, False)
        ]
        # for text, row, col, idx, direction in right_arm_buttons:
        #     joint_name = f"right_arm_{idx}"
        #     self.create_button(layout, text, row, col, partial(asyncio.ensure_future, self.jog_execute(joint_name, direction)))
        for text, row, col, idx, direction in right_arm_buttons:
            joint_name = f"right_arm_{idx}"
            label = QLabel(joint_name)
            self.layout.addWidget(label, row, 4)
            # lambda를 사용하여 각각의 버튼에 대한 비동기 호출 처리
            self.create_button(self.layout, text, row, col, lambda joint_name=joint_name, direction=direction: asyncio.ensure_future(self.jog_execute(joint_name, direction)))


        # 팔 플러스/마이너스 버튼 (L0, L1, ...)
        left_arm_buttons = [
            ("L0 +", 16, 2, 0, True),   ("L0 -", 16, 3, 0, False),
            ("L1 +", 17, 2, 1, True),   ("L1 -", 17, 3, 1, False),
            ("L2 +", 18, 2, 2, True),   ("L2 -", 18, 3, 2, False),
            ("L3 +", 19, 2, 3, True),   ("L3 -", 19, 3, 3, False),
            ("L4 +", 20, 2, 4, True),   ("L4 -", 20, 3, 4, False),
            ("L5 +", 21, 2, 5, True),   ("L5 -", 21, 3, 5, False),
            ("L6 +", 22, 2, 6, True),   ("L6 -", 22, 3, 6, False)
        ]
        for text, row, col, idx, direction in left_arm_buttons:
            joint_name = f"left_arm_{idx}"
            label = QLabel(joint_name)
            self.layout.addWidget(label, row, 6)
            self.create_button(self.layout, text, row, col, lambda joint_name=joint_name, direction=direction: asyncio.ensure_future(self.jog_execute(joint_name, direction)))

        self.create_button(self.layout, "팔 전체 홈 리셋", 23, 0, lambda : asyncio.ensure_future(self.arm_reset()), col_span=4, background_color="#ED325A")
        
        self.create_button(self.layout, "그리퍼 QC", 24, 0, self.gripper_test, 1)
        self.create_button(self.layout, "FT 센서 QC 시작", 24, 1, self.start_ft_sensor_test, 1)
        self.create_button(self.layout, "FT 센서 QC 종료", 25, 1, self.stop_ft_sensor_test, 1)
        
        self.create_button(self.layout, "툴플렌지 QC 시작", 24, 2, self.start_tool_flange_test, 1)
        self.create_button(self.layout, "툴플렌지 QC 종료", 25, 2, self.start_tool_flange_test, 1)
        
        self.create_button(self.layout, "마스터암 QC 시작", 24, 3, self.start_master_arm_test, 1)
        self.create_button(self.layout, "마스터암 QC 종료", 25, 3, self.stop_master_arm_test, 1)
        
        
        home_offset_rst = QLabel("전체 QC")
        home_offset_rst.setStyleSheet(TITLE_STYLE_SHEET)
        self.layout.addWidget(home_offset_rst, 0, 9, 1, 4)
        self.create_button(self.layout, "브레이크 풀기(Troso_5, Arm_0 to 6)", 1, 9, lambda:asyncio.ensure_future(self.break_release_for_cal()) , 2)
        self.create_button(self.layout, "홈 리셋(Troso_5, Arm_0 to 6)", 1, 11, lambda:asyncio.ensure_future(self.home_offset_reset_for_cal()), 2, background_color="#ED325A")
        
        self.create_button(self.layout, "브레이크 풀기(r_arm_6)", 2, 9, lambda:asyncio.ensure_future(self.r_arm_6_break_release_for_cal()), 1)
        self.create_button(self.layout, "홈 리셋(r_arm_6)", 2, 10, lambda:asyncio.ensure_future(self.r_arm_6_home_offset_reset_for_cal()), 1, background_color="#ED325A")
        
        self.create_button(self.layout, "브레이크 풀기(l_arm_6)", 2, 11, lambda:asyncio.ensure_future(self.l_arm_6_break_release_for_cal()), 1)
        self.create_button(self.layout, "홈 리셋(l_arm_6)", 2, 12, lambda:asyncio.ensure_future(self.l_arm_6_home_offset_reset_for_cal()), 1, background_color="#ED325A")
        
        if self.model == "a":
            for idx in range(24):
                lamp = QLineEdit("")
                lamp.setFixedSize(65,20)
                lamp.setReadOnly(True)  # 읽기 전용
                lamp.setFocusPolicy(Qt.NoFocus)  # 선택 안되게 설정
                lamp.setStyleSheet(IDLE_STATE)
                
                if idx >=2 and idx<=7:
                    self.layout.addWidget(lamp, 7+idx-2, 5)
                elif idx >=8 and idx<=14:
                    self.layout.addWidget(lamp, 16+idx-8, 5)
                elif idx >=15 and idx<=21:
                    self.layout.addWidget(lamp, 16+idx-15, 7)
                    
        elif self.model == "t5":
            for idx in range(23):
                lamp = QLineEdit("")
                lamp.setFixedSize(65,20)
                lamp.setReadOnly(True)  # 읽기 전용
                lamp.setFocusPolicy(Qt.NoFocus)  # 선택 안되게 설정
                lamp.setStyleSheet(IDLE_STATE)
                
                if idx >=2 and idx<=6:
                    self.layout.addWidget(lamp, 7+idx-1, 5)
                elif idx >=7 and idx<=13:
                    self.layout.addWidget(lamp, 16+idx-7, 5)
                elif idx >=14 and idx<=20:
                    self.layout.addWidget(lamp, 16+idx-14, 7)
                    
        elif self.model == "m":
            for idx in range(26):
                lamp = QLineEdit("")
                lamp.setFixedSize(65,20)
                lamp.setReadOnly(True)  # 읽기 전용
                lamp.setFocusPolicy(Qt.NoFocus)  # 선택 안되게 설정
                lamp.setStyleSheet(IDLE_STATE)
                
                if idx >=4 and idx<=9:
                    self.layout.addWidget(lamp, 7+idx-4, 5)
                elif idx >=10 and idx<=16:
                    self.layout.addWidget(lamp, 16+idx-10, 5)
                elif idx >=17 and idx<=23:
                    self.layout.addWidget(lamp, 16+idx-17, 7)
                    
        self.create_button(self.layout, "데모모션(비인증)", 3, 9, lambda : self.demo_motion(False), 2)
        self.create_button(self.layout, "데모모션(인증)", 3, 11, lambda: self.demo_motion(True), 2)
        
        self.create_button(self.layout, "텔레오퍼레이션(비인증)", 4, 9, lambda : self.start_teleoperation(False), 2)
        self.create_button(self.layout, "텔레오퍼레이션(인증)", 4, 11, lambda: self.start_teleoperation(True), 2)
        self.create_button(self.layout, "텔레오퍼레이션 종료", 5, 9, lambda:asyncio.ensure_future(self.stop_teleoperation), 4)
        
        how_to_use_title = QLabel("Cheat Sheet")
        how_to_use_title.setStyleSheet(TITLE_STYLE_SHEET)
        self.layout.addWidget(how_to_use_title, 6, 9, 1, 4)
        
        how_to_use_te = QTextEdit()
        how_to_use_te.setReadOnly(True)
        how_to_use_te.append("<b># FONT INSTALL</b>")
        how_to_use_te.append("sudo apt install fonts-noto-cjk")
        how_to_use_te.append("")
        how_to_use_te.append("<b># 빌드 무조건 해야함</b>")
        how_to_use_te.append("cmake --build --preset conan-release")
        how_to_use_te.append("")
        how_to_use_te.append("<b># USB 하나만 꽂고 확인. 스티커 붙은게 그리퍼</b>")
        how_to_use_te.append("udevadm info --query=all --name=/dev/ttyUSB0 | grep SHORT")
        how_to_use_te.append("")
        how_to_use_te.append("<b># U2D2 세팅</b>")
        how_to_use_te.append("sudo gedit /etc/udev/rules.d/99-u2d2.rules")
        how_to_use_te.append('SUBSYSTEM=="tty", ATTRS{serial}=="123456", SYMLINK+="rby1_gripper"')
        how_to_use_te.append('SUBSYSTEM=="tty", ATTRS{serial}=="123456", SYMLINK+="rby1_master_arm"')
        how_to_use_te.append("")
        how_to_use_te.append("sudo udevadm control --reload-rules")
        how_to_use_te.append("sudo udevadm trigger")
        how_to_use_te.append("")
        how_to_use_te.append("<b># rby1_gripper, rby1_master_arm 인식 확인</b>")
        how_to_use_te.append("ls /dev/rb*")
        how_to_use_te.append("")
        how_to_use_te.append("<b># 자주쓰는 regex</b>")
        how_to_use_te.append("^(?!.*_wheel$).+$")
        how_to_use_te.append("^(right_arm_[0-5]|left_arm_[0-5]|torso_5)$")
        self.layout.addWidget(how_to_use_te, 7, 9, 18, 4)
        # 최종 레이아웃 설정
        self.setLayout(self.layout)
        self.setWindowTitle("Robot QC Controls")

    
    def time_scale_value_changed(self, value):
        disp_value = value/10
        ret = self.robot.set_time_scale(disp_value)
        print(ret)
        self.time_scale_label.setText(f"time_scale: {disp_value}")
        
    def show_splash(self):
        """스플래쉬 화면을 보여줌"""
        self.splash_screen.show()

    def close_splash(self):
        """스플래쉬 화면을 닫음"""
        self.splash_screen.close()


    def create_button(self, layout, text, row, col, slot, col_span=1, background_color=None, text_color=None, font_size:float=None):
        """버튼을 생성하고 레이아웃에 추가하며, 슬롯을 연결."""
        button = QPushButton(text)
        button.setMinimumSize(120, 30)
        
        button.clicked.connect(slot)

        # 스타일 시트를 설정하여 색상 지정
        style = ""
        if background_color:
            style += f"background-color: {background_color};"
        if text_color:
            style += f"color: {text_color};"
        if font_size:
            style += f"font-size: {font_size}px;"
        button.setStyleSheet(style)
        self.layout.addWidget(button, row, col, 1, col_span)

    def start_process(self, key, command):
        self.cleanup_process(key)  
        """특정 key에 해당하는 프로세스를 시작."""
        if key not in self.processes or self.processes[key] is None:
            process = subprocess.Popen(
                command,
                cwd=".",  # 현재 경로 기준으로 실행
            )
            self.processes[key] = process
            print(f"{key} started")

    def cleanup_process(self, key):
        """종료된 프로세스를 딕셔너리에서 제거."""
        if key in self.processes and self.processes[key] is not None:
            process = self.processes[key]
            if process.poll() is not None:  # 프로세스가 종료된 경우
                print(f"{key} process finished. Cleaning up.")
                del self.processes[key]  # 딕셔너리에서 제거
                
    def stop_process(self, key):
        """특정 key에 해당하는 프로세스를 종료."""
        process = self.processes.get(key)
        if process:
            process.terminate()
            process.wait()  # 프로세스가 완전히 종료될 때까지 대기
            self.processes[key] = None
            print(f"{key} stopped")

    def start_process_with_sudo(self, key, command):
        self.cleanup_process(key)  
        """특정 key에 해당하는 프로세스를 sudo로 실행."""
        if key not in self.processes or self.processes[key] is None:
            # sudo 비밀번호를 묻지 않고 명령을 실행
            sudo_command = ['sudo'] + command
            process = subprocess.Popen(
                sudo_command,
                cwd=".",  # 현재 경로 기준으로 실행
            )
            self.processes[key] = process
            print(f"{key} started with sudo")

    def stop_process_with_sudo(self, key):
        """특정 key에 해당하는 sudo로 실행된 프로세스를 종료."""
        process = self.processes.get(key)
        if process:
            sudo_kill_command = f"sudo kill -9 {process.pid}"
            subprocess.run(sudo_kill_command, shell=True)
            process.wait()  # 프로세스가 완전히 종료될 때까지 대기
            self.processes[key] = None
            print(f"{key} stopped with sudo")
            

    def make_joint_control_command(self, joint_target, minimum_time):
        return RobotCommandBuilder().set_command(
            ComponentBasedCommandBuilder().set_body_command(
                JointPositionCommandBuilder()
                .set_command_header(CommandHeaderBuilder().set_control_hold_time(1))
                .set_minimum_time(minimum_time)
                .set_position(joint_target)
            )
        )
            
    async def jog_execute(self, joint_name, direction):
        """jog 슬롯: 인덱스와 방향을 받아 처리."""
        try:
            # 이전 작업이 끝날 때까지 대기
            if self.jog_task and not self.jog_task.done():
                print("Previous jog operation still running. Ignoring new request.")
                return

            print(joint_name)
            jog_value = float(self.jog_value_spinbox.value()) if direction else -float(self.jog_value_spinbox.value())
            jog_value = math.radians(jog_value)
            print(f"jog_value: {jog_value}")

            # 비동기 작업 시작
            self.jog_task = asyncio.ensure_future(self._send_jog_command(joint_name, jog_value))

            # 비동기 작업 완료 후 처리
            await self.jog_task  # 작업이 완료될 때까지 기다림
            self.jog_task = None  # 작업이 완료되면 리셋

        except Exception as e:
            print(f"An exception occurred during task execution: {e}")
            if self.jog_task:
                self.jog_task.cancel()  # 예외 발생 시 작업을 취소
            self.jog_task = None  # 작업 리셋


    async def _send_jog_command(self, joint_name, jog_value):
        """실제 jog 명령을 보내는 함수"""
        handler = self.robot.send_command(
            rby1_sdk.RobotCommandBuilder().set_command(
                rby1_sdk.JogCommandBuilder().set_joint_name(joint_name).set_command(rby1_sdk.JogCommandBuilder().RelativePosition(jog_value))
            )
        )
        if self.model =="a":
            await asyncio.to_thread(Robot_A_CommandHandler.get, handler)
        elif self.model =="t5":
            await asyncio.to_thread(Robot_T5_CommandHandler.get, handler)
        elif self.model =="m":
            await asyncio.to_thread(Robot_M_CommandHandler.get, handler)
        

    async def update_display(self):
        global IS_READY, CUR_POSITION, CONTROL_MANAGER_STATE, IS_POWER
        if CONTROL_MANAGER_STATE == ControlManagerState.State.Enabled:
            self.control_state.setText("Enabled")
            self.control_state.setStyleSheet(ACTIVATE_STATE)
        elif CONTROL_MANAGER_STATE == ControlManagerState.State.MajorFault or CONTROL_MANAGER_STATE == ControlManagerState.State.MinorFault:
            self.control_state.setText("Fault")
            self.control_state.setStyleSheet(FAULT_STATE)
        else:
            self.control_state.setText("idle")
            self.control_state.setStyleSheet(IDLE_STATE)
            
        if IS_POWER:
            self.connect_state.setText("Power On")
            self.connect_state.setStyleSheet(ACTIVATE_STATE)
        else:
            self.connect_state.setText("Power Off")
            self.connect_state.setStyleSheet(FAULT_STATE)
        
        for idx in range(IS_READY.__len__()):
            lamp = QLineEdit("")
            lamp.setFixedSize(65,20)
            
            lamp.setFocusPolicy(Qt.NoFocus)  # 선택 안되게 설정
            if IS_READY[idx]:
                lamp.setStyleSheet(ACTIVATE_STATE)
            else:
                lamp.setStyleSheet(FAULT_STATE)
            lamp.setText(f"{math.degrees(CUR_POSITION[idx]):.2f}")
            lamp.setReadOnly(True)  # 읽기 전용
            # if CUR_POSITION and CUR_POSITION.__len__()>0:
            #     print(CUR_POSITION[idx])
                # lamp.setText(CUR_POSITION[idx])
            if self.model == "a":
                if idx >=2 and idx<=7:
                    self.layout.addWidget(lamp, 7+idx-2, 5)
                elif idx >=8 and idx<=14:
                    self.layout.addWidget(lamp, 16+idx-8, 5)
                elif idx >=15 and idx<=21:
                    self.layout.addWidget(lamp, 16+idx-15, 7)
                    
            elif self.model == "t5":                
                if idx >=2 and idx<=6:
                    self.layout.addWidget(lamp, 7+idx-1, 5)
                elif idx >=7 and idx<=13:
                    self.layout.addWidget(lamp, 16+idx-7, 5)
                elif idx >=14 and idx<=20:
                    self.layout.addWidget(lamp, 16+idx-14, 7)
                    
            elif self.model == "m":
                if idx >=4 and idx<=9:
                    self.layout.addWidget(lamp, 7+idx-4, 5)
                elif idx >=10 and idx<=16:
                    self.layout.addWidget(lamp, 16+idx-10, 5)
                elif idx >=17 and idx<=23:
                    self.layout.addWidget(lamp, 16+idx-17, 7)
                
    @Slot()
    async def connect(self):
        print("connect")
        ret = self.robot.connect()
        print(ret)
        self.channel = grpc.aio.insecure_channel(self.address)
        await self.channel.channel_ready()

        self.power_service = power_service_pb2_grpc.PowerServiceStub(self.channel)
        if ret:
            self.timer = QTimer(self)
            self.timer.timeout.connect(lambda: asyncio.ensure_future(self.update_display()))  # asyncio.ensure_future()로 호출
            self.timer.setInterval(300)
            self.timer.start()
            # self.robot.set_time_scale(0.5)
            self.robot.start_state_update(STATE_CALL_BACK, rate=10)
            
            self.connect_state.setText("Connected")
            self.connect_state.setStyleSheet(IDLE_STATE)
            disp_value = self.robot.get_time_scale()
            self.time_scale_label.setText(f"time_scale: {disp_value}")
            self.time_scale_slider.setValue(disp_value*10)
            
        else:
            self.connect_state.setText("Power Off")
            self.connect_state.setStyleSheet(FAULT_STATE)

    @Slot()
    def power_on(self):
        if not self.robot.is_power_on(self.device):
            ret = self.robot.power_on(self.device)
            # self.set_robot_state("power_on") if ret else self.set_robot_state("power_off")
        print("Power On clicked")

    @Slot()
    def power_off(self):
        ret = self.robot.power_off(self.device)
        # self.set_robot_state("power_off")
        print("Power Off clicked")

    @Slot()
    def control_reset(self):
        self.robot.reset_fault_control_manager()
        print("Control Reset clicked")

    @Slot()
    def control_enable(self):
        if all(not state for state in IS_READY):
            QMessageBox.critical(self, "Error", "Servo On된 모터가 하나도 없습니다.")
            return
        ret = self.robot.enable_control_manager()
        print("Control Enable clicked")

    @Slot()
    def control_disable(self):
        self.robot.disable_control_manager()
        print("Control Disable clicked")
        
    @Slot()
    def leg_arm_servo_on(self):
        self.robot.servo_on(f"^(torso_[0-5]|left_arm_[0-6]|right_arm_[0-6])$")
        print("Leg, Arm Servo ON clicked")

    @Slot()
    async def leg_reset(self):
        name = "^(torso_[0-5])$"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_HOME_OFFSET_RST))
        
        print("Leg Reset clicked")
    

    @Slot()
    def leg_servo_on(self):
        for i in range(6):
            self.robot.servo_on(f"torso_{i}")
            # time.sleep(0.2)
        print("Leg Servo ON clicked")
        
    @Slot()
    def leg_servo_on2(self):
        self.robot.servo_on(f"^(torso_[0-5])$")
        print("Leg Servo ON clicked")

    @Slot()
    async def arm_reset(self):
        name = "^(left|right)_arm_[0-6]$"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_HOME_OFFSET_RST))
        
        print("Arm Reset clicked")

    @Slot()
    def arm_servo_on(self):
        for i in range(7):
            self.robot.servo_on(f"right_arm_{i}")
            # time.sleep(0.2)
        for i in range(7):
            self.robot.servo_on(f"left_arm_{i}")
            # time.sleep(0.2)
        print("Arm Servo ON clicked")
        
    @Slot()
    def arm_servo_on2(self):
        self.robot.servo_on(f"^(right_arm_[0-6]|left_arm_[0-6])$")
        print("Arm Servo ON clicked")
    
    @Slot()
    async def break_release_for_cal(self):
        name = "^(left_arm_[0-6]|right_arm_[0-6]|torso_5)$"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_BRAKE_RELEASE))
        
        print("Break Rel Clicked")
        
    @Slot()
    async def home_offset_reset_for_cal(self):
        name = "^(left_arm_[0-6]|right_arm_[0-6]|torso_5)$"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_HOME_OFFSET_RST))
        print("Home Offset Reset Clicked")
        
    @Slot()
    async def r_arm_6_break_release_for_cal(self):
        name = "right_arm_6"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_BRAKE_RELEASE))
        print("Break Rel Clicked")
        
    @Slot()
    async def r_arm_6_home_offset_reset_for_cal(self):
        name = "right_arm_6"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_HOME_OFFSET_RST))
        print("Home Offset Reset Clicked")
        
    @Slot()
    async def l_arm_6_break_release_for_cal(self):
        name = "left_arm_6"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_BRAKE_RELEASE))
        print("Break Rel Clicked")
        
    @Slot()
    async def l_arm_6_home_offset_reset_for_cal(self):
        name = "left_arm_6"
        await self.power_service.JointCommand(power_pb2.JointCommandRequest(name=name, 
                                                                            command=power_pb2.JointCommandRequest.COMMAND_HOME_OFFSET_RST))
        print("Home Offset Reset Clicked")
    
    @Slot()
    def demo_motion(self, is_certification:bool):
        print("데모 모션 ", is_certification)
        if is_certification:
            self.start_process("demo_motion", ["python", "examples/python/09_demo_motion.py", "--address", self.address, "--model", self.model, "--servo", "torso_.*|left_arm_.*|right_arm_.*"])
        else:
            self.start_process("demo_motion", ["python", "examples/python/09_demo_motion.py", "--address", self.address, "--model", self.model,])
            
    @Slot()
    def start_teleoperation(self, is_samsung:bool):
        if is_samsung:
            self.start_process_with_sudo("teleoperation", ["./build/examples/cpp/example_teleoperation_with_joint_mapping", self.address, "^(?!.*_wheel$).+$"])
        else:
            self.start_process_with_sudo("teleoperation", ["./build/examples/cpp/example_teleoperation_with_joint_mapping", self.address])
    
    @Slot()
    async def stop_teleoperation(self):
        self.stop_process_with_sudo("teleoperation")
        
        ret = await self.robot.disable_control_manager()
        if ret:
            await self.robot.power_off("12v")
            
    @Slot()
    def gripper_test(self):
        self.start_process_with_sudo("gripper", ["./build/examples/cpp/module_test/module_test_gripper", self.address])
        
    @Slot()
    def start_ft_sensor_test(self):
        self.start_process("ft_sensor", ["./build/examples/cpp/module_test/module_test_ft_sensor", self.address])
    
    @Slot()
    def stop_ft_sensor_test(self):
        self.stop_process("ft_sensor")

    @Slot()
    def start_tool_flange_test(self):
        self.start_process("tool_flange", ["./build/examples/cpp/module_test/module_test_tool_flange", self.address])

    @Slot()
    def stop_tool_flange_test(self):
        self.stop_process("tool_flange")

    @Slot()
    def start_master_arm_test(self):
        self.start_process_with_sudo("master_arm", ["./build/examples/cpp/module_test/module_test_master_arm", self.address])
    
    @Slot()
    def stop_master_arm_test(self):
        self.stop_process_with_sudo("master_arm")

    def set_buttons_enabled(self, enabled: bool):
        """모든 버튼을 활성화 또는 비활성화."""
        for button in self.findChildren(QPushButton):
            button.setEnabled(enabled)

    @Slot()
    async def go_to_zero_pose(self):
        QTimer.singleShot(0, self.show_splash)
        self.set_buttons_enabled(False)
        
        if self.model =="a": 
            handler = self.robot.send_command(self.make_joint_control_command([0.] * 20, 4))
            ret = await asyncio.to_thread(Robot_A_CommandHandler.get, handler)
        elif self.model =="t5": 
            handler = self.robot.send_command(self.make_joint_control_command([0.] * 19, 4))
            ret = await asyncio.to_thread(Robot_T5_CommandHandler.get, handler)
        elif self.model == "m":
            handler = self.robot.send_command(self.make_joint_control_command([0.] * 20, 4))
            ret = await asyncio.to_thread(Robot_M_CommandHandler.get, handler)
        if ret.finish_code == RobotCommandFeedback.FinishCode.Canceled:
            QMessageBox.critical(self, "Error", "Canceled(이전 명령 중복?)")
        elif ret.finish_code == RobotCommandFeedback.FinishCode.InitializedFailed:
            QMessageBox.critical(self, "Error", "InitializedFailed(컨트롤 확인)")
        
         # 스플래쉬 스크린을 닫고 버튼 활성화
        QTimer.singleShot(0, self.close_splash)
        self.set_buttons_enabled(True)
  
    @Slot()
    async def go_to_packing_pose(self):
        QTimer.singleShot(0, self.show_splash)
        self.set_buttons_enabled(False)
        

        if self.model == "a":
            target_joint = np.array([0, 80, -140, 60, 0, 0] + [0, 0, 0, -150, 0, 105, 0] + [0, 0, 0, -150, 0, 105, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_A_CommandHandler.get, handler) 
            
        if self.model == "t5":
            target_joint = np.array([80, -140, 60, 0, 0] + [0, 0, 0, -150, 0, 105, 0] + [0, 0, 0, -150, 0, 105, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_T5_CommandHandler.get, handler)    
                     
        elif self.model == "m":
            target_joint = np.array([0, 80, -140, 60, 0, 0] + [0, 0, 0, -150, 0, 105, 0] + [0, 0, 0, -150, 0, 105, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_M_CommandHandler.get, handler) 
            
        if ret.finish_code == RobotCommandFeedback.FinishCode.Canceled:
            QMessageBox.critical(self, "Error", "Canceled(이전 명령 중복?)")
        elif ret.finish_code == RobotCommandFeedback.FinishCode.InitializedFailed:
            QMessageBox.critical(self, "Error", "InitializedFailed(컨트롤 확인)")
        
        # 스플래쉬 스크린을 닫고 버튼 활성화
        QTimer.singleShot(0, self.close_splash)
        self.set_buttons_enabled(True)

    @Slot()
    async def go_to_break1(self):
        splash_screen = SplashScreen()
    
        # 스플래쉬 스크린을 메인 스레드에서 실행
        def show_splash():
            splash_screen.show()

        def close_splash():
            splash_screen.close()

        # 메인 스레드에서 스플래쉬 스크린을 표시
        QTimer.singleShot(0, show_splash)
        
        # 버튼 비활성화
        self.set_buttons_enabled(False)
        
        
        if self.model == "a":
            target_joint = np.array([0, 35, -70, 35, 0, 0] + [0, -90, 0, -100, 0, 90, 0] + [0, 90, 0, -100, 0, 90, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_A_CommandHandler.get, handler) 
        elif self.model == "t5":
            target_joint = np.array([35, -70, 35, 0, 0] + [0, -90, 0, -100, 0, 90, 0] + [0, 90, 0, -100, 0, 90, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_T5_CommandHandler.get, handler) 
        elif self.model == "m":
            target_joint = np.array([0, 35, -70, 35, 0, 0] + [0, -90, 0, -100, 0, 90, 0] + [0, 90, 0, -100, 0, 90, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_M_CommandHandler.get, handler) 
        
        if ret.finish_code == RobotCommandFeedback.FinishCode.Canceled:
            QMessageBox.critical(self, "Error", "Canceled(이전 명령 중복?)")
        elif ret.finish_code == RobotCommandFeedback.FinishCode.InitializedFailed:
            QMessageBox.critical(self, "Error", "InitializedFailed(컨트롤 확인)")
         # 스플래쉬 스크린을 닫고 버튼 활성화
        QTimer.singleShot(0, close_splash)
        self.set_buttons_enabled(True)
        
    @Slot()
    async def go_to_break2(self):
        splash_screen = SplashScreen()
    
        # 스플래쉬 스크린을 메인 스레드에서 실행
        def show_splash():
            splash_screen.show()

        def close_splash():
            splash_screen.close()

        # 메인 스레드에서 스플래쉬 스크린을 표시
        QTimer.singleShot(0, show_splash)
        
        
        if self.model == "a":
            target_joint = np.array([0, 35, -70, 35, 0, 0] + [-30, -10, 0, -100, 0, 0, 0] + [-30, 10, 0, -100, 0, 0, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_A_CommandHandler.get, handler)
        elif self.model == "t5":
            target_joint = np.array([35, -70, 35, 0, 0] + [-30, -10, 0, -100, 0, 0, 0] + [-30, 10, 0, -100, 0, 0, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_T5_CommandHandler.get, handler)  
        elif self.model == "m":
            target_joint = np.array([0, 35, -70, 35, 0, 0] + [-30, -10, 0, -100, 0, 0, 0] + [-30, 10, 0, -100, 0, 0, 0])
            handler = self.robot.send_command(self.make_joint_control_command(np.deg2rad(target_joint), 4))
            ret = await asyncio.to_thread(Robot_M_CommandHandler.get, handler)  
        
        if ret.finish_code == RobotCommandFeedback.FinishCode.Canceled:
            QMessageBox.critical(self, "Error", "Canceled(이전 명령 중복?)")
        elif ret.finish_code == RobotCommandFeedback.FinishCode.InitializedFailed:
            QMessageBox.critical(self, "Error", "InitializedFailed(컨트롤 확인)")
        # 스플래쉬 스크린을 닫고 버튼 활성화
        QTimer.singleShot(0, close_splash)
        self.set_buttons_enabled(True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="quick_ui_cal")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--model', type=str, default='a', help="Robot Model Name (default: 'a')")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    parser.add_argument('--servo', type=str, default=".*", help="Servo name regex pattern (default: '.*')")
    args = parser.parse_args()

    app = QApplication()
    #sudo apt install fonts-noto-cjk
    font = QFont("Noto Sans CJK KR", 10) 
    app.setFont(font)
    # QtAsyncio.run(app.exec, ButtonGrid(address=args.address, device=args.device).show())
    window = ButtonGrid(address=args.address, device=args.device, model = args.model, servo = args.servo)
    window.show()
    
    
    QtAsyncio.run(handle_sigint=True)
