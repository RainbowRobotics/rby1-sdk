from PySide6.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton
from PySide6.QtCore import Slot
import subprocess
import argparse

class ButtonGrid(QWidget):
    def __init__(self, address, device):
        super().__init__()
        # 각 프로세스를 관리할 딕셔너리
        self.processes = {}
        self.address = address
        self.device = device

        # Layout 설정
        layout = QGridLayout()

        # 버튼 추가 및 레이아웃 설정
        self.create_button(layout, "Zero", 0, 0, self.zero_pose, col_span=2, background_color="#3498db")
        self.create_button(layout, "Demo Motion Start", 1, 0, self.demo_motion_start)
        self.create_button(layout, "Demo Motion Stop", 1, 1, self.demo_motion_stop)
        self.create_button(layout, "Teleoperation Start", 2, 0, self.teleoperation_start)
        self.create_button(layout, "Teleoperation Stop", 2, 1, self.teleoperation_stop)
        self.create_button(layout, "Teleoperation Record\n Start", 3, 0, self.teleoperation_record_start)
        self.create_button(layout, "Teleoperation Record\n Stop", 3, 1, self.teleoperation_record_stop)
        self.create_button(layout, "Replay Start", 4, 0, self.replay_start)
        self.create_button(layout, "Replay Stop", 4, 1, self.replay_stop)
        self.create_button(layout, "Impedance Start", 5, 0, self.impedance_start)
        self.create_button(layout, "Impedance Stop", 5, 1, self.impedance_stop)
        self.create_button(layout, "EMS Stop", 6, 0, self.ems_stop, col_span=2, background_color="#db3498")

        # 레이아웃 설정
        self.setLayout(layout)
        self.setWindowTitle("Quick UI")

    def create_button(self, layout, text, row, col, slot, col_span=1, background_color=None, text_color=None):
        """버튼을 생성하고 레이아웃에 추가하며, 슬롯을 연결."""
        button = QPushButton(text)
        button.setMinimumSize(150, 80)
        button.clicked.connect(slot)
        
        # 스타일 시트를 설정하여 색상 지정
        style = ""
        if background_color:
            style += f"background-color: {background_color};"
        if text_color:
            style += f"color: {text_color};"
        
        button.setStyleSheet(style)
        layout.addWidget(button, row, col, 1, col_span)

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

    @Slot()
    def zero_pose(self):
        self.start_process("zero_pose", ["python", "examples/python/08_zero_pose.py", "--address", self.address])

    @Slot()
    def demo_motion_start(self):
        self.start_process("demo_motion", ["python", "examples/python/09_demo_motion.py", "--address", self.address])

    @Slot()
    def demo_motion_stop(self):
        self.stop_process("demo_motion")

    @Slot()
    def teleoperation_start(self):
        self.start_process_with_sudo("teleoperation", ["./build/examples/cpp/example_teleoperation_with_joint_mapping", self.address])

    @Slot()
    def teleoperation_stop(self):
        self.stop_process_with_sudo("teleoperation")

    @Slot()
    def teleoperation_record_start(self):
        self.start_process("teleoperation_record", ["python", "examples/python/record.py", "--address", self.address])

    @Slot()
    def teleoperation_record_stop(self):
        self.stop_process("teleoperation_record")

    @Slot()
    def replay_start(self):
        self.start_process("replay", ["python", "examples/python/replay.py", "--address", self.address])

    @Slot()
    def replay_stop(self):
        self.stop_process("replay")
        
    @Slot()
    def impedance_start(self):
        self.start_process("impedance", ["python", "examples/python/07_impedance_control.py", "--address", self.address])

    @Slot()
    def impedance_stop(self):
        self.stop_process("impedance")
    
    @Slot()
    def ems_stop(self):
        self.start_process("ems_stop", ["python", "examples/python/06_stop_command.py", "--address", self.address])

# Application 실행
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="quick_ui")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    parser.add_argument('--device', type=str, default=".*", help="Power device name regex pattern (default: '.*')")
    args = parser.parse_args()
    
    app = QApplication([])
    window = ButtonGrid(address=args.address, device=args.device)
    window.show()
    app.exec()
