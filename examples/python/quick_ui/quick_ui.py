from PySide6.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton
from PySide6.QtCore import Slot
import subprocess
import argparse

class ButtonGrid(QWidget):
    def __init__(self, address):
        super().__init__()
        # 각 프로세스를 관리할 딕셔너리
        self.processes = {}
        self.address = address

        # Layout 설정
        layout = QGridLayout()

        # 버튼 추가 및 레이아웃 설정
        self.create_button(layout, "Demo Motion Start", 0, 0, self.demo_motion_start)
        self.create_button(layout, "Demo Motion Stop", 0, 1, self.demo_motion_stop)
        self.create_button(layout, "Teleoperation Start", 1, 0, self.teleoperation_start)
        self.create_button(layout, "Teleoperation Stop", 1, 1, self.teleoperation_stop)
        self.create_button(layout, "Teleoperation Record\n Start", 2, 0, self.teleoperation_record_start)
        self.create_button(layout, "Teleoperation Record\n Stop", 2, 1, self.teleoperation_record_stop)
        self.create_button(layout, "Replay Start", 3, 0, self.replay_start)
        self.create_button(layout, "Replay Stop", 3, 1, self.replay_stop)

        # 레이아웃 설정
        self.setLayout(layout)
        self.setWindowTitle("Quick UI")

    def create_button(self, layout, text, row, col, slot):
        """버튼을 생성하고 레이아웃에 추가하며, 슬롯을 연결."""
        button = QPushButton(text)
        button.setFixedSize(150, 100)
        button.clicked.connect(slot)
        layout.addWidget(button, row, col)

    def start_process(self, key, command):
        """특정 key에 해당하는 프로세스를 시작."""
        if key not in self.processes or self.processes[key] is None:
            self.processes[key] = subprocess.Popen(
                command,
                cwd=".",  # 현재 경로 기준으로 실행
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f"{key} started")

    def stop_process(self, key):
        """특정 key에 해당하는 프로세스를 종료."""
        process = self.processes.get(key)
        if process:
            process.terminate()
            process.wait()  # 프로세스가 완전히 종료될 때까지 대기
            self.processes[key] = None
            print(f"{key} stopped")

    # 슬롯 메서드 정의
    @Slot()
    def demo_motion_start(self):
        self.start_process("demo_motion", ["./build/examples/cpp/example_demo_motion", self.address])

    @Slot()
    def demo_motion_stop(self):
        self.stop_process("demo_motion")

    @Slot()
    def teleoperation_start(self):
        self.start_process("teleoperation", ["./build/examples/cpp/example_teleoperation_with_joint_mapping", self.address])

    @Slot()
    def teleoperation_stop(self):
        self.stop_process("teleoperation")

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

# Application 실행
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="quick_ui")
    parser.add_argument('--address', type=str, required=True, help="Robot address")
    args = parser.parse_args()
    
    app = QApplication([])
    window = ButtonGrid(address=args.address)
    window.show()
    app.exec()
