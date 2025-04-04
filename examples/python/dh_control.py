import time
import rby1_sdk
import numpy as np

robot = rby1_sdk.create_robot_a("192.168.30.1:50051")
robot.connect()

recorded_position = []
recorded_velocity = []

def cb(rs):
    global recorded_position, recorded_velocity
    recorded_position = recorded_position + [rs.target_position[3]]
    recorded_velocity = recorded_velocity + [rs.target_position[11]]

robot.start_state_update(cb, 0.1)  # (Hz)

robot.power_on(".*")
robot.servo_on(".*")
robot.reset_fault_control_manager()
robot.enable_control_manager()

flag = 0
count = 0
max_count = 1000  # 0~1 범위로 변환하기 위한 최대값
first_state_q = np.zeros(24, dtype=np.float64)  # 초기 관절 state 저장 변수

def control(state: rby1_sdk.Robot_A_ControlState):
    global count, flag, first_state_q
    if count == 1:
        print(f"current time: {time.time_ns() / 1_000_000_000}")
        print(f"time: {state.t}")
        # print(f"position: {state.position}")

    i = rby1_sdk.Robot_A_ControlInput()

    # 최초 한 번만 초기 상태 저장
    if flag == 0:
        first_state_q = state.position.copy()  # 최초 상태 저장
        flag = 1

    # count 값을 0~1 사이로 정규화
    t = count / max_count  # t는 0~1 사이 값

    # 코사인 함수 적용 (first_state_q 기준으로 궤적 생성)
    i.target = first_state_q - 0.5 * first_state_q * (1 - np.cos(np.pi * t))
    i.feedback_gain.fill(10)
    i.feedforward_torque.fill(0)
    i.finish = count > 1000

    count += 1  # 카운트 증가

    return i

print(robot.control(control))




