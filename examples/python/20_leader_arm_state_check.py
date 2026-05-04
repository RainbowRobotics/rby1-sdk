################################ Note ################################
# This example does not run in simulation.
######################################################################
# Leader arm state check
# Check leader arm's q,torque,current,etc.
# if connect failed continuously and it's occured more than N times, operate safety function and close node.
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
# Usage example:
#     python 20_leader_arm_state_check.py --address 192.168.30.1:50051 --model m
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.
import os
import rby1_sdk as rby
import numpy as np
import argparse
import time
import datetime
import signal
import threading
import logging
import copy
import queue

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "../../models/master_arm", "model.urdf") # /../../models/master_arm/model.urdf"
LEADER_ARM_DEVICE_NAME = rby.upc.MasterArmDeviceName

class LeaderArm:
    DOF = 14
    DEVICE_COUNT = 16
    RIGHT_TOOL_ID = 0x80
    LEFT_TOOL_ID = 0x81
    MAXIMUM_TORQUE = 0.5
    TORQUE_SCALING = 0.5
    kBaseLinkId = 0
    kRightLinkId = 7
    kLeftLinkId = 14

    class ButtonState: # 버튼 상태 구조체
        __slots__ = ['button', 'trigger']
        def __init__(self, b, t):
            self.button = b
            self.trigger = t

    class State: # 기본적으로 저장되는 변수들
        __slots__ = [
            'q_joint', 'qvel_joint', 'torque_joint', 'gravity_term', 
            'operating_mode', 'target_position', 'button_right', 
            'button_left', 'T_right', 'T_left', 'temperatures',
            'fault_ids', 'joint_fault_ids', 'tool_fault_ids', 'current', 'tool_error_counts', 'joint_error_counts',
            'check_status_duration', 'fault_ids_history'
        ]
        def __init__(self, dof=14):
            self.q_joint = np.zeros(dof, dtype=np.float64)
            self.qvel_joint = np.zeros(dof, dtype=np.float64)
            self.torque_joint = np.zeros(dof, dtype=np.float64)
            self.gravity_term = np.zeros(dof, dtype=np.float64)
            self.operating_mode = np.full(dof, -1, dtype=np.int64)
            self.target_position = np.zeros(dof, dtype=np.float64)
            self.button_right = rby.DynamixelBus.ButtonState()
            self.button_left = rby.DynamixelBus.ButtonState()
            self.T_right = np.eye(4)
            self.T_left = np.eye(4)
            self.temperatures = np.zeros(dof, dtype=np.float64)
            self.fault_ids = [] # 통신실패한 id
            self.joint_fault_ids = []
            self.tool_fault_ids = [] # 통신실패한 tool id 
            self.fault_ids_history = np.zeros(dof +2, dtype=np.int64)
            self.current = np.zeros(dof, dtype=np.float64)
            self.tool_error_counts = 0
            self.joint_error_counts = 0
            self.check_status_duration = 0.0

        # 메모리 접근충돌을 막기 위해 데이터를 복사해서 사용
        def copy(self):
            # Create a shallow copy of the object structure
            snapshot = copy.copy(self)
            
            # Re-allocate unique arrays for the snapshot to prevent thread race conditions
            dof = len(self.q_joint)
            snapshot.q_joint = np.zeros(dof, dtype=np.float64)
            snapshot.qvel_joint = np.zeros(dof, dtype=np.float64)
            snapshot.torque_joint = np.zeros(dof, dtype=np.float64)
            snapshot.gravity_term = np.zeros(dof, dtype=np.float64)
            snapshot.operating_mode = np.full(dof, -1, dtype=np.int64)
            snapshot.target_position = np.zeros(dof, dtype=np.float64)
            snapshot.temperatures = np.zeros(dof, dtype=np.float64)
            snapshot.fault_ids = []
            snapshot.joint_fault_ids = []
            snapshot.tool_fault_ids = []
            snapshot.current = np.zeros(dof, dtype=np.float64)
            snapshot.tool_error_counts = 0
            snapshot.joint_error_counts = 0
            snapshot.check_status_duration = 0.0
            snapshot.fault_ids_history = np.zeros(dof + 2, dtype=np.int64)
            
            # Copy data into new arrays
            self.copy_to(snapshot)
            return snapshot

        def copy_to(self, target):
            """Efficiently copies data into an existing State object to avoid allocations."""
            target.q_joint[:] = self.q_joint
            target.qvel_joint[:] = self.qvel_joint
            target.torque_joint[:] = self.torque_joint
            target.gravity_term[:] = self.gravity_term
            target.operating_mode[:] = self.operating_mode
            target.target_position[:] = self.target_position
            target.temperatures[:] = self.temperatures
            target.fault_ids = list(self.fault_ids)
            target.joint_fault_ids = list(self.joint_fault_ids)
            target.tool_fault_ids = list(self.tool_fault_ids)
            target.current[:] = self.current
            target.tool_error_counts = self.tool_error_counts
            target.joint_error_counts = self.joint_error_counts
            target.check_status_duration = self.check_status_duration
            target.fault_ids_history[:] = self.fault_ids_history

            # Handle button snapshots (always create a new frozen snapshot for the state)
            target.button_right = LeaderArm.ButtonState(self.button_right.button, self.button_right.trigger)
            target.button_left = LeaderArm.ButtonState(self.button_left.button, self.button_left.trigger)

            # Transformation matrices (4x4 numpy arrays)
            if target.T_right is not None:
                target.T_right[:] = self.T_right
            else:
                target.T_right = self.T_right.copy()
            
            if target.T_left is not None:
                target.T_left[:] = self.T_left
            else:
                target.T_left = self.T_left.copy()

    class ControlInput:
        def __init__(self, dof=14):
            self.target_operating_mode = np.full(dof, -1, dtype=int)
            self.target_position = np.zeros(dof, dtype=np.float64)
            self.target_torque = np.zeros(dof, dtype=np.float64)
    
    # 기존 멀티스레드 관리방식이랑 동일하게 구현해놓음
    class EventLoop:
        def __init__(self):
            self._tasks = queue.Queue()
            self._running = False
            self._paused = True
            self._thread = None
            self._lock = threading.Lock()

        def start(self):
            with self._lock:
                if self._thread is None or not self._thread.is_alive():
                    self._running = True
                    self._paused = False
                    self._thread = threading.Thread(target=self._worker, daemon=True)
                    self._thread.start()

        def stop(self):
            self._running = False
            self._tasks.put(None)  # Wake up worker
            if self._thread:
                self._thread.join()
                self._thread = None

        def pause(self):
            self._paused = True

        def unpause(self):
            self._paused = False

        def push_task(self, task):
            self._tasks.put(task)

        def push_cyclic_task(self, task, period_sec):
            def cyclic_wrapper():
                if not self._running:
                    return
                
                start_time = time.time()
                result = task()
                
                if not self._running or result is False:
                    return

                elapsed = time.time() - start_time
                sleep_time = period_sec - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                if self._running:
                    self.push_task(cyclic_wrapper)

            self.push_task(cyclic_wrapper)

        def _worker(self):
            while self._running:
                task = self._tasks.get()
                if task is None or not self._running:
                    break
                if self._paused:
                    self._tasks.task_done()
                    continue
                try:
                    task()
                except Exception as e:
                    logging.error(f"Error in EventLoop task: {e}")
                finally:
                    self._tasks.task_done()

    # 리더암 클래스의 초기화 함수
    def __init__(self, dev_name=LEADER_ARM_DEVICE_NAME, control_period=0.01, check_goal_position=True):
        self.dev_name = dev_name
        self.bus = rby.DynamixelBus(dev_name)
        self.ev = self.EventLoop()
        self.ctrl_ev = self.EventLoop()
        self.control_period = control_period
        self.ctrl_session_active = False # Tracks if start_control is active
        self.ctrl_callback_busy = False   # Tracks if the callback is currently executing
        self.control_callback = None
        self.safety_function = None
        self.goal_pos_flag = check_goal_position

        self.torque_constant = np.array([1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
                                         1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043])
        self.active_ids = []
        self.motor_ids = list(range(self.DOF))
        self.tool_ids = [self.RIGHT_TOOL_ID, self.LEFT_TOOL_ID]
        self.active_joint_ids = [] # Dynamically set during initialize
        self.active_tool_ids = []  # Dynamically set during initialize

        self.initialized = False
        self.operating_mode_init = False
        self.state = self.State(self.DOF)
        self.model_path = URDF_PATH
        self.is_running = False
        self.tool_error_counts = 0
        self.joint_error_counts = 0
        self.MAX_TOOL_RETRIES = 5 # 10 consecutive fails (~0.1s at 100Hz)
        self.MAX_JOINT_RETRIES = 5 # 10 consecutive fails (~0.1s at 100Hz)
        self.recovery_sync_flag = False

    
    def SetControlPeriod(self, control_period):
        self.control_period = control_period

    def check_goal_position_state(self, enable: bool):
        self.goal_pos_flag = enable

    def SetModelPath(self, model_path):
        self.model_path = model_path
        if self.is_running:
            self._init_dynamics()

    def SetTorqueConstant(self, torque_constant):
        self.torque_constant = np.array(torque_constant)
        if self.initialized:
            self.bus.set_torque_constant(self.torque_constant.tolist())
    
    # 동역학 모델 정의
    def _init_dynamics(self):
        # Initialize robot kinematics and state using the trusted factory pattern
        config = rby.dynamics.load_robot_from_urdf(self.model_path, "Base")
        self.robot = rby.dynamics.Robot(config)
        self.dyn_state = self.robot.make_state(
            ["Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R", "Link_0L", "Link_1L",
             "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"],
            ["J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R", "J4_Wrist_Yaw1_R",
             "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R", "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L",
             "J10_Elbow_L", "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"]
        )
        self.dyn_state.set_gravity([0, 0, 0, 0, 0, -9.81])

    # 기존 초기화 기능과 동일. cpp에는 좀 더 긴데, 그건 함수로 모듈화해놓고 사용중
    def initialize(self, verbose=False):
        # 내부 쓰레드 오류를 터미널에 표시하기 위해 기본 로깅을 설정
        if verbose:
            logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
        else:
            logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

        try:
            rby.upc.initialize_device(self.dev_name)
        except Exception as e:
            if verbose:
                logging.warning(f"Failed to initialize device latency: {e}")

        if not self.bus.open_port():
            print("Failed to open the port!")
            return []
        if not self.bus.set_baud_rate(self.bus.DefaultBaudrate):
            print("Failed to change the baudrate!")
            return []

        self.initialized = True
        self.active_ids = self.check_motor_status(verbose)
        
        # Categorize detected IDs
        self.active_joint_ids = [mid for mid in self.motor_ids if mid in self.active_ids]
        self.active_tool_ids = [tid for tid in self.tool_ids if tid in self.active_ids]
        
        self.bus.set_torque_constant(self.torque_constant.tolist())
        return self.active_ids

    def check_motor_status(self, verbose=True):
        start_time = time.time()
        active_ids = []
        # Check Motor 0~13 and Tool Motor 0x80, 0x81
        self.motor_ids = list(range(self.DOF))
        check_ids = self.motor_ids + self.tool_ids

        for dev_id in check_ids:
            if self.bus.ping(dev_id):
                active_ids.append(dev_id)
                if verbose:
                    logging.info(f"Dynamixel ID {dev_id} is active")
            else:
                if verbose and dev_id < self.DOF:
                    logging.warning(f"Dynamixel ID {dev_id} is NOT active.")
        
        self.state.check_status_duration = time.time() - start_time
        return active_ids

    def EnableTorque(self):
        self.bus.group_sync_write_torque_enable(self.motor_ids, 1)

    def DisableTorque(self):
        self.bus.group_sync_write_torque_enable(self.motor_ids, 0)

    def set_max_retries(self, max_tool_retries = 5, max_joint_retries = 1):
        self.MAX_TOOL_RETRIES = max_tool_retries
        self.MAX_JOINT_RETRIES = max_joint_retries

    # qc를 위해 자세들을 입력해서 모터를 움직이는 코드
    def set_target_position(self, q_target, goal_current=0.5):
        if len(q_target) != self.DOF:
            logging.error(f"Target position length mismatch: expected {self.DOF}, got {len(q_target)}")
            return False
        
        def task():
            target_mode = rby.DynamixelBus.CurrentBasedPositionControlMode
            
            needs_mode_change = []
            for i in self.motor_ids:
                if self.state.operating_mode[i] != target_mode:
                    needs_mode_change.append(i)
            
            if needs_mode_change:
                # 1. Disable Torque only for motors needing mode change
                self.bus.group_sync_write_torque_enable(needs_mode_change, 0)
                # 2. Set Operating Mode
                self.bus.group_sync_write_operating_mode([(i, target_mode) for i in needs_mode_change])
                # 3. Enable Torque
                self.bus.group_sync_write_torque_enable(needs_mode_change, 1)
                
                # Update local operating mode state
                for i in needs_mode_change:
                    self.state.operating_mode[i] = target_mode

            # 4. Set Goal Current (Torque Limit)
            self.bus.group_sync_write_send_torque([(i, goal_current) for i in self.motor_ids])

            # 5. Send Position
            # SDK handles radian -> tick conversion internally. No manual scaling needed.
            self.bus.group_sync_write_send_position([(i, q) for i, q in enumerate(q_target)])

        if self.is_running:
            self.ev.push_task(task)
        else:
            task()
        return True

    # 기존 start_control 함수. 테스크의 내용은 아래에 있음
    def start_control(self, callback, safety_function=None):
        if not self.initialized:
            return False
        if self.ctrl_session_active:
            return False

        self.control_callback = callback
        self.safety_function = safety_function
        self.ctrl_session_active = True
        self.ctrl_callback_busy = False

        # Load robot kinematics
        self._init_dynamics()

        # Initial operating mode setup
        if not self.operating_mode_init:
            self.bus.group_sync_write_torque_enable(self.motor_ids, 0)
            self.bus.group_sync_write_operating_mode([(i, rby.DynamixelBus.CurrentControlMode) for i in self.motor_ids])
            self.bus.group_sync_write_torque_enable(self.motor_ids, 1)
            self.operating_mode_init = True

        self.ev.unpause()
        self.ctrl_ev.unpause()
        self.ev.start()
        self.ctrl_ev.start()
        self.is_running = True

        self.ev.push_cyclic_task(self._ev_task, self.control_period)
        return True

    def stop_control(self, torque_disable=False):
        if not self.ctrl_session_active:
            return False

        # 1. Disable torque first if requested (Highest priority)
        if torque_disable:
            self.DisableTorque()

        # 2. Shutdown threads
        self.ctrl_session_active = False
        self.is_running = False
        self.ev.stop()
        self.ctrl_ev.stop()

        self.control_callback = None
        return True

    # 데이터 읽는 테스크
    def _ev_task(self):
        try:
            # 0. Reset/Initialize faults for the current cycle
            # joint_fault_ids and tool_fault_ids initially contain IDs that are missing from the active lists
            self.state.joint_fault_ids = sorted(list(set(self.motor_ids) - set(self.active_joint_ids)))
            self.state.tool_fault_ids = sorted(list(set(self.tool_ids) - set(self.active_tool_ids)))
            self.state.fault_ids = sorted(list(set(self.state.joint_fault_ids) | set(self.state.tool_fault_ids)))
            self.state.gravity_term.fill(0.0)
            
            for tid in self.active_tool_ids:
                res = self.bus.read_button_status(tid)
                if res:
                    _, bstate = res
                    if tid == self.RIGHT_TOOL_ID:
                        self.state.button_right = bstate
                    else:
                        self.state.button_left = bstate
                else:
                    self.state.tool_fault_ids.append(tid)
                    # Increment history (Tools start after DOF joints: 0x80->14, 0x81->15)
                    self.state.fault_ids_history[tid - 0x80 + self.DOF] += 1
                    logging.warning(f"Tool ID {tid} skipped communication step (Count: {self.tool_error_counts+1})")
            
            self.state.tool_fault_ids = sorted(list(set(self.state.tool_fault_ids)))
            
            if not self.state.tool_fault_ids:
                self.tool_error_counts = 0
            else:
                self.tool_error_counts += 1

            # 2. Read Operating Modes (Joints Only - Critical)
            # Sequential check for immediate daisy-chain fault isolation.
            # We probe the full motor_ids list to find where the signal physically breaks.
            for i, mid in enumerate(self.motor_ids):
                # Check each ID individually to pinpoint the first break in the chain.
                mode = self.bus.read_operating_mode(mid, False)
                if mode is not None:
                    if mid < self.DOF:
                        self.state.operating_mode[mid] = mode
                else:
                    # FAILURE: Mark this ID and all downstream IDs as faulted.
                    new_faults = self.motor_ids[i:]
                    self.state.joint_fault_ids = sorted(list(set(self.state.joint_fault_ids) | set(new_faults)))
                    logging.warning(f"[LeaderArm] Communication break detected at ID {mid}. "
                                    f"Marking {len(new_faults)} IDs as faulted: {new_faults}")
                    break
            
            
            if not self.state.joint_fault_ids:
                # 3. Read Motor States (Use active IDs to avoid bulk-read failure from missing hardware)
                ms_list = self.bus.get_motor_states(self.active_joint_ids)
                if ms_list:
                    for mid, mstate in ms_list:
                        if mid < self.DOF:
                            self.state.q_joint[mid] = mstate.position
                            self.state.qvel_joint[mid] = mstate.velocity
                            self.state.current[mid] = mstate.current
                            self.state.torque_joint[mid] = mstate.current * self.torque_constant[mid]
                            self.state.temperatures[mid] = mstate.temperature
                else:
                    # Fallback for bulk-read failure
                    self.state.joint_fault_ids = sorted(list(set(self.state.joint_fault_ids) | set(self.active_joint_ids)))

                # 4. Read Goal Positions (Only if goal_pos_flag is True and no joint faults in Step 3)
                if self.goal_pos_flag and not self.state.joint_fault_ids:
                    temp_gp = self.bus.group_fast_sync_read(self.active_joint_ids, rby.DynamixelBus.AddrGoalPosition, 4)
                    if temp_gp:
                        for mid, val in temp_gp:
                            if mid < self.DOF:
                                self.state.target_position[mid] = val / 4096.0 * 2.0 * np.pi
                    else:
                        self.state.joint_fault_ids = sorted(list(set(self.state.joint_fault_ids) | set(self.active_joint_ids)))

            # 5. Compute Kinematics & Dynamics (Only if no joint faults in Step 4)
            if not self.state.joint_fault_ids:
                # 5-1. Map Natural Order (R-then-L) to URDF Order (L-then-R)
                q_urdf = np.concatenate([self.state.q_joint[self.DOF//2:], self.state.q_joint[:self.DOF//2]])
                self.dyn_state.set_q(q_urdf)
                self.robot.compute_forward_kinematics(self.dyn_state)
                
                # Compute gravity and un-map back to Natural Order
                grav_urdf = self.robot.compute_gravity_term(self.dyn_state)
                self.state.gravity_term = np.concatenate([grav_urdf[self.DOF//2:], grav_urdf[:self.DOF//2]]) * self.TORQUE_SCALING
                
                self.state.T_right = self.robot.compute_transformation(self.dyn_state, self.kBaseLinkId, self.kRightLinkId)
                self.state.T_left = self.robot.compute_transformation(self.dyn_state, self.kBaseLinkId, self.kLeftLinkId)
            
            # 6. Hardware Recovery & Fault Consolidation
            # Trigger recovery block if we have ANY joint or tool faults (missing or newly failed)
            if self.state.joint_fault_ids or self.state.tool_fault_ids:
                # Record recently detected faults to history before re-pinging
                for fid in self.state.joint_fault_ids:
                    if fid < self.DOF:
                        self.state.fault_ids_history[fid] += 1
                for fid in self.state.tool_fault_ids:
                    if fid in self.tool_ids:
                        self.state.fault_ids_history[fid - 0x80 + self.DOF] += 1
                
                active_ids = self.check_motor_status(verbose=False)
                # Redefine fault_ids based on actual current active IDs
                self.state.joint_fault_ids = sorted(list(set(self.motor_ids) - (set(active_ids) & set(self.motor_ids))))
                self.state.tool_fault_ids = sorted(list(set(self.tool_ids) - (set(active_ids) & set(self.tool_ids))))
                
                # Update active joint/tool lists to include newly reconnected hardware
                self.active_joint_ids = [mid for mid in self.motor_ids if mid in active_ids]
                self.active_tool_ids = [tid for tid in self.tool_ids if tid in active_ids]
                
                # Update counters
                self.joint_error_counts += 1
                if not self.state.joint_fault_ids:
                    self.joint_error_counts = 0
                    self.recovery_sync_flag = True
                
                if not self.state.tool_fault_ids:
                    self.tool_error_counts = 0
            
            # Consolidate Faults for Debugging/Monitoring (Final update for the cycle)
            self.state.fault_ids = sorted(list(set(self.state.joint_fault_ids) | set(self.state.tool_fault_ids)))

            # 7. Safety Check & Control
            # Treat both joint faults and tool faults as critical safety events
            if self.joint_error_counts > self.MAX_JOINT_RETRIES or self.tool_error_counts > self.MAX_TOOL_RETRIES:
                if self.safety_function:
                    # Run safety_function in a separate thread to avoid deadlock
                    # (safety_function may call stop_control which joins ev thread)
                    fault_state = self.state.copy()
                    safety_thread = threading.Thread(target=self.safety_function, args=(fault_state,), daemon=True)
                    safety_thread.start()
                else:
                    # Fallback: Print combined error and skip cycle
                    print(f"[LeaderArm] ERROR: Hardware fault detected (IDs: {self.state.fault_ids}) but no safety_function is registered! Skipping cycle.")
                return
            
            # 8. Sync Counters to State for Monitoring
            self.state.joint_error_counts = self.joint_error_counts
            self.state.tool_error_counts = self.tool_error_counts

            if self.control_callback and self.ctrl_session_active and not self.ctrl_callback_busy:
                self.ctrl_callback_busy = True
                
                # Capturing a snapshot of the state ensures that the control task 
                # works with a consistent snapshot, avoiding race conditions 
                # when _ev_task starts updating the state for the next cycle.
                captured_state = self.state.copy()
                self.ctrl_ev.push_task(lambda: self._ctrl_task(captured_state))
        except Exception as e:
            logging.error(f"[LeaderArm] UNEXPECTED ENGINE EXCEPTION: {e}")
            if self.safety_function:
                # Force safety shutdown on any software exception
                self.safety_function(self.state)
            raise e

    # 유저가 정의한 콜백 함수를 실행하는 테스크
    def _ctrl_task(self, state):
        try:
            if state.joint_fault_ids:
                state.gravity_term = np.zeros(self.DOF)
            user_input = self.control_callback(state)
            if user_input:
                self._handle_control_input(user_input, state)
        except Exception as e:
            logging.error(f"[LeaderArm] EXCEPTION IN CONTROL CALLBACK: {e}")
            if self.safety_function:
                self.safety_function(state)
            raise e
        finally:
            self.ctrl_callback_busy = False

    def _handle_control_input(self, user_input, state):

        changed_ids = []
        changed_id_modes = []
        id_position = []
        id_torque = []

        for i in range(self.DOF):
            # 1. Determine if a mode change or full recovery sync is needed
            if self.recovery_sync_flag or state.operating_mode[i] != user_input.target_operating_mode[i]:
                changed_ids.append(i)
                changed_id_modes.append((i, user_input.target_operating_mode[i]))
            
            # 2. Always prepare target values (Torque/Position) for active control modes
            # Note: Send values regardless of whether mode is changing this cycle; 
            # the _write_task handles the sequencing (Mode/Torque Enable -> Values).
            target_mode = user_input.target_operating_mode[i]
            if target_mode == rby.DynamixelBus.CurrentControlMode:
                id_torque.append((i, user_input.target_torque[i]))
            elif target_mode == rby.DynamixelBus.CurrentBasedPositionControlMode:
                id_torque.append((i, user_input.target_torque[i]))
                id_position.append((i, user_input.target_position[i]))
        
        self.recovery_sync_flag = False

        # Push write task back to ev thread
        self.ev.push_task(lambda: self._write_task(changed_ids, changed_id_modes, id_torque, id_position))

    def _write_task(self, changed_ids, changed_id_modes, id_torque, id_position):
        try:
            if changed_ids:
                self.bus.group_sync_write_torque_enable(changed_ids, 0)
                self.bus.group_sync_write_operating_mode(changed_id_modes)
                self.bus.group_sync_write_torque_enable(changed_ids, 1)

            if id_torque:
                self.bus.group_sync_write_send_torque(id_torque)
            if id_position:
                self.bus.group_sync_write_send_position(id_position)
        except Exception as e:
            logging.error(f"[LeaderArm] EXCEPTION IN WRITE TASK: {e}")
            if self.safety_function:
                self.safety_function(self.state)
            raise e

    def close(self):
        self.stop_control(torque_disable=True)


def main(address, model):
    robot = rby.create_robot(address, model)
    robot.connect()

    if not robot.is_connected():
        print("Error: Robot connection failed.")
        exit(1)

    if not robot.power_on("12v"):
        print("Error: Failed to power on 12V.")
        exit(1)

    leader_arm = LeaderArm(control_period=0.01)
    leader_arm.set_max_retries(max_tool_retries=100, max_joint_retries=100)
    
    if not leader_arm.initialize(verbose=True):
        print("Failed to initialize Leader Arm")
        exit(1)
        
    if len(leader_arm.active_ids) != leader_arm.DEVICE_COUNT:
        print(f"Error: Mismatch in the number of devices detected. Expected {leader_arm.DEVICE_COUNT}, got {len(leader_arm.active_ids)}")
        exit(1)

    # 모니터링을 위한 함수
    def fmt(arr):
        return ", ".join([f"{x:7.3f}" for x in arr])

    def fmt_int(arr):
        return ", ".join([f"{int(x):7d}" for x in arr])


    # 사용자 정의함수
    # 1. 상태 모니터링
    # 2. 중력보상값 모터에 입력
    # 3. 통신문제(조인트가 문제발생, 혹은 버튼 트리거 관련 신호가 5번 연속으로 문제생길경우) 발생 시 12v 공급 차단
    def control(state: LeaderArm.State):
        header = f"--- Leader Arm state Monitor | {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} ---"
        line_idx = "index:        " + ", ".join([f"{i:7d}" for i in range(len(state.q_joint))])
        line_q = f"q (rad):      {fmt(state.q_joint)}"
        line_current = f"current (A):  {fmt(state.current)}"
        line_temp = f"temp (C):     {fmt(state.temperatures)}"
        line_torque = f"torque (Nm):  {fmt(state.torque_joint)}"
        line_grav = f"gravity (Nm): {fmt(state.gravity_term)}"
        line_btn = f"BTN   | L: {state.button_left.button:1d} TRG: {state.button_left.trigger:4d} | R: {state.button_right.button:1d} TRG: {state.button_right.trigger:4d}"
        line_fault = f"Fault IDs:    {state.fault_ids}, (check time : {state.check_status_duration * 1000.0:6.1f}ms)"
        history_joints = state.fault_ids_history[:14]
        history_tools = state.fault_ids_history[14:]
        line_hist_j = f"Joint Fault count:  {fmt_int(history_joints)}"
        line_hist_t = f"Tool Fault count:   right: {int(history_tools[0]):d} | left: {int(history_tools[1]):d}"
        
        # 5. Status & Alarm Section
        if state.fault_ids:
            status_line = f"\033[1;33mSTATUS: [ WARNING - Communication Issues on IDs: {state.fault_ids} ]\033[0m"
        else:
            status_line = f"\033[1;32mSTATUS: [ NORMAL ]\033[0m"

        print("\033[H\033[J", end="", flush=True)  # Clear terminal and move cursor to top
        print(header, flush=True)
        print("-" * len(header), flush=True)
        print(line_idx, flush=True)
        print(line_q, flush=True)
        print(line_current, flush=True)
        print(line_temp, flush=True)
        print(line_torque, flush=True)
        print(line_grav, flush=True)
        print(line_btn, flush=True)
        print(line_fault, flush=True)
        print(line_hist_j, flush=True)
        print(line_hist_t, flush=True)
        print("\n" + status_line, flush=True)
        
        input = LeaderArm.ControlInput()
        input.target_operating_mode.fill(rby.DynamixelBus.CurrentControlMode)
        input.target_torque = state.gravity_term

        return input

    # safety_function 중복 실행 방지용 플래그
    shutdown_lock = threading.Lock()
    shutdown_done = False

    def safety_function(state: LeaderArm.State):
        nonlocal shutdown_done
        import sys

        # 중복 실행 방지: 첫 번째 호출만 실행, 나머지는 무시
        with shutdown_lock:
            if shutdown_done:
                return
            shutdown_done = True
        
        # Flush all pending logs first
        sys.stdout.flush()
        sys.stderr.flush()
        
        error_msg = f"\n\n\033[1;31m[CRITICAL ERROR / EXCEPTION DETECTED]\033[0m\n"
        if state.fault_ids:
            error_msg += f"Communication failure on IDs: {state.fault_ids}\n"
        error_msg += "ACTION: Immediate Emergency Shutdown (Power Off 12V).\n"
        
        print(error_msg, flush=True)
        
        # Priority 1: Cleanup (제어 루프 중지, 토크 비활성화)
        if leader_arm:
            leader_arm.stop_control(torque_disable=True)

        # Priority 2: 12V 차단
        try:
            robot.power_off("12v")
        except:
            pass

        # Priority 3: 분리된 프로세스로 ping test 예약 (부모 종료 후 자동 실행됨)
        # run_final_ping_test()
            
        print("Shutdown complete. Exiting.", flush=True)
        sys.stdout.flush()
        time.sleep(0.5) # Critical delay to ensure terminal displays the message
        os._exit(1)

    try:
        leader_arm.start_control(control, safety_function=safety_function)
        while leader_arm.ctrl_session_active:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping...")
    finally:
        # safety_function이 이미 처리한 경우 중복 실행 방지
        with shutdown_lock:
            if shutdown_done:
                return
            shutdown_done = True
        print("Shutting down Leader Arm engine...")
        leader_arm.stop_control(torque_disable=True)
        print("Powering off 12V...")
        try:
            if robot.get_control_manager_state().state == rby.ControlManagerState.State.Enabled:
                print("Disabling control manager...")
                robot.disable_control_manager()
        except Exception:
            pass
        try:
            robot.power_off("12v")
        except:
            pass
        # run_final_ping_test()
        print("System shutdown complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="19_master_arm")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
