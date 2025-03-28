import rby1_sdk
import numpy as np
import time
import sys
import argparse
import signal

recorded_traj = []  # 데이터를 저장할 리스트
recording = False  # 녹화 상태를 관리할 변수


def cb(state):
    global recorded_traj, recording
    if recording:
        print("---")
        print(f"current position: {state.target_position}")

        current_state = state.position.copy()
        recorded_traj.append(current_state)


def pre_processing(address):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()

    if not robot.is_power_on(".*"):
        print("Power is currently OFF. Attempting to power on...")
        if not robot.power_on(".*"):
            print("Error: Failed to power on the robot.")
            sys.exit(1)
        print("Robot powered on successfully.")
    else:
        print("Power is already ON.")

    return robot


def start_recording():
    global recording, recorded_traj
    recorded_traj = []  # 녹화를 시작할 때 리스트 초기화
    recording = True
    print("Recording started...")


def stop_recording():
    global recording
    recording = False
    np.savez_compressed("recorded.npz", data=recorded_traj)
    print("Recording stopped and data saved to 'recorded.npz'.")


def handle_exit(sig, frame):
    """프로그램 종료 시 호출되는 핸들러"""
    stop_recording()
    sys.exit(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    args = parser.parse_args()

    # 종료 시 호출될 핸들러 등록
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    robot = pre_processing(args.address)

    robot.start_state_update(cb, 10)

    start_recording()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        handle_exit(None, None)  # Ctrl+C로 종료할 때도 파일을 저장하도록 처리
