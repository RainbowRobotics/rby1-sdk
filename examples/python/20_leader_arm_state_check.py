################################ Note ################################
# This example does not run in simulation.
######################################################################
# Leader arm state check
# Check leader arm's q,torque,current,etc.
# Optimized version:
# 1. 100Hz high-speed control loop using 1-shot Bulk/Sync Read during normal operation.
# 2. Fast fault isolation & auto-recovery synchronization upon communication failure.
# 3. Decoupled configurable Hz monitoring console rendering to eliminate stdout I/O jitter.
# 4. Soft torque ramp-down (3.0s, 10 steps) with control cancel/disable before 12V power off.
# 5. Configurable safety function: Hardware faults follow USE_SOFT_STOP; Ctrl+C / normal exits use soft stop.
# 6. Smart 12V power state validation & control manager management on start/shutdown.
# 7. Unified safety shutdown on critical hardware faults and Ctrl+C (SIGINT).
#
# Usage example:
#     python 20_leader_arm_state_check.py --address 192.168.30.1:50051 --model m
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.
#
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
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# URDF_PATH = os.path.join(SCRIPT_DIR, "models", "leader_arm", "model.urdf")
URDF_PATH = os.path.join(SCRIPT_DIR, "../../models/leader_arm", "model.urdf") # /../../models/leader_arm/model.urdf
LEADER_ARM_DEVICE_NAME = rby.upc.resolve_leader_arm_device_name()

# ============================================================
# Global Configuration & User Customization Parameters
# ============================================================
DECAY_TIME = 3.0              # Soft torque ramp-down time in seconds
MONITOR_HZ = 20.0             # Monitoring UI refresh rate (Hz)
MAX_RETRY_COUNT_TOOL = 10     # Maximum retry count for tool communication
MAX_RETRY_COUNT_JOINT = 10    # Maximum retry count for joint communication
USE_SOFT_STOP = True          # Hardware fault behavior: True (Soft ramp-down) / False (Instant power off)


# ============================================================
# LeaderArm
# ============================================================
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

    class State:
        __slots__ = [
            'q_joint', 'qvel_joint', 'torque_joint', 'gravity_term',
            'operating_mode', 'target_position', 'button_right',
            'button_left', 'T_right', 'T_left', 'temperatures',
            'fault_ids', 'joint_fault_ids', 'tool_fault_ids', 'current',
            'tool_error_counts', 'joint_error_counts',
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
            self.fault_ids = []
            self.joint_fault_ids = []
            self.tool_fault_ids = []
            self.fault_ids_history = np.zeros(dof + 2, dtype=np.int64)
            self.current = np.zeros(dof, dtype=np.float64)
            self.tool_error_counts = 0
            self.joint_error_counts = 0
            self.check_status_duration = 0.0

        def copy(self):
            snapshot = copy.copy(self)
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
            snapshot.fault_ids_history = np.zeros(dof + 2, dtype=np.int64)
            self.copy_to(snapshot)
            return snapshot

        def copy_to(self, target):
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

            # ButtonState is an immutable/readonly binding object, reference assignment is safe
            target.button_right = self.button_right
            target.button_left = self.button_left

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
            self._tasks.put(None)
            if self._thread and threading.current_thread() != self._thread:
                self._thread.join(timeout=1.0)
            self._thread = None

        def pause(self):
            self._paused = True

        def unpause(self):
            self._paused = False

        def push_task(self, task):
            if self._running:
                self._tasks.put(task)

        def push_cyclic_task(self, task, period_sec):
            next_time = [time.time() + period_sec]

            def cyclic_wrapper():
                if not self._running:
                    return

                try:
                    result = task()
                except Exception as e:
                    logging.error(f"Error in cyclic task: {e}")
                    result = False

                if not self._running or result is False:
                    return

                now = time.time()
                sleep_time = next_time[0] - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    next_time[0] += period_sec
                else:
                    next_time[0] = time.time() + period_sec

                if self._running:
                    self.push_task(cyclic_wrapper)

            self.push_task(cyclic_wrapper)

        def _worker(self):
            while self._running:
                try:
                    task = self._tasks.get(timeout=0.1)
                except queue.Empty:
                    continue

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

    def __init__(self, dev_name=LEADER_ARM_DEVICE_NAME, control_period=0.01, check_goal_position=True):
        self.dev_name = dev_name
        self.bus = rby.DynamixelBus(dev_name)
        self.ev = self.EventLoop()
        self.ctrl_ev = self.EventLoop()
        self.control_period = control_period
        self.ctrl_session_active = False
        self.ctrl_callback_busy = False
        self.control_callback = None
        self.safety_function = None
        self.goal_pos_flag = check_goal_position
        self._safety_triggered = False

        self.torque_constant = np.array([1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
                                         1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043])
        self.active_ids = []
        self.motor_ids = list(range(self.DOF))
        self.tool_ids = [self.RIGHT_TOOL_ID, self.LEFT_TOOL_ID]
        self.active_joint_ids = []
        self.active_tool_ids = []

        self.initialized = False
        self.operating_mode_init = False
        self.state = self.State(self.DOF)
        self.model_path = URDF_PATH
        self.is_running = False
        self.tool_error_counts = 0
        self.joint_error_counts = 0
        self.MAX_TOOL_RETRIES = 5
        self.MAX_JOINT_RETRIES = 5
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
            try:
                self.bus.set_torque_constant(self.torque_constant.tolist())
            except Exception as e:
                logging.error(f"Failed to set torque constant: {e}")

    def _init_dynamics(self):
        try:
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
        except Exception as e:
            logging.error(f"Failed to initialize dynamics from URDF '{self.model_path}': {e}")
            raise e

    def initialize(self, verbose=False):
        if verbose:
            logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
        else:
            logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

        try:
            rby.upc.initialize_device(self.dev_name)
        except Exception as e:
            if verbose:
                logging.warning(f"Failed to initialize device latency: {e}")

        try:
            if not self.bus.open_port():
                print("Failed to open the port!")
                return []
            if not self.bus.set_baud_rate(self.bus.DefaultBaudrate):
                print("Failed to change the baudrate!")
                return []
        except Exception as e:
            logging.error(f"Bus initialization exception: {e}")
            return []

        self.initialized = True
        self.active_ids = self.check_motor_status(verbose)
        self.active_joint_ids = [mid for mid in self.motor_ids if mid in self.active_ids]
        self.active_tool_ids = [tid for tid in self.tool_ids if tid in self.active_ids]

        try:
            self.bus.set_torque_constant(self.torque_constant.tolist())
        except Exception as e:
            logging.warning(f"Failed to set initial torque constant: {e}")

        return self.active_ids

    def check_motor_status(self, verbose=True):
        start_time = time.time()
        active_ids = []
        self.motor_ids = list(range(self.DOF))
        check_ids = self.motor_ids + self.tool_ids

        for dev_id in check_ids:
            try:
                if self.bus.ping(dev_id):
                    active_ids.append(dev_id)
                    if verbose:
                        logging.info(f"Dynamixel ID {dev_id} is active")
                else:
                    if verbose and dev_id < self.DOF:
                        logging.warning(f"Dynamixel ID {dev_id} is NOT active.")
            except Exception as e:
                logging.warning(f"Ping exception on ID {dev_id}: {e}")

        self.state.check_status_duration = time.time() - start_time
        return active_ids

    def EnableTorque(self):
        if self.initialized:
            try:
                self.bus.group_sync_write_torque_enable(self.motor_ids, 1)
            except Exception as e:
                logging.error(f"Failed to enable torque: {e}")

    def DisableTorque(self):
        if self.initialized:
            try:
                self.bus.group_sync_write_torque_enable(self.motor_ids, 0)
            except Exception as e:
                logging.error(f"Failed to disable torque: {e}")

    def set_max_retries(self, max_tool_retries=5, max_joint_retries=5):
        self.MAX_TOOL_RETRIES = max_tool_retries
        self.MAX_JOINT_RETRIES = max_joint_retries

    def set_target_position(self, q_target, goal_current=0.5):
        if len(q_target) != self.DOF:
            logging.error(f"Target position length mismatch: expected {self.DOF}, got {len(q_target)}")
            return False

        def task():
            try:
                target_mode = rby.DynamixelBus.CurrentBasedPositionControlMode
                needs_mode_change = []
                for i in self.motor_ids:
                    if self.state.operating_mode[i] != target_mode:
                        needs_mode_change.append(i)

                if needs_mode_change:
                    self.bus.group_sync_write_torque_enable(needs_mode_change, 0)
                    self.bus.group_sync_write_operating_mode([(i, target_mode) for i in needs_mode_change])
                    self.bus.group_sync_write_torque_enable(needs_mode_change, 1)
                    for i in needs_mode_change:
                        self.state.operating_mode[i] = target_mode

                self.bus.group_sync_write_send_torque([(i, goal_current) for i in self.motor_ids])
                self.bus.group_sync_write_send_position([(i, q) for i, q in enumerate(q_target)])
            except Exception as e:
                logging.error(f"Failed in set_target_position task: {e}")

        if self.is_running:
            self.ev.push_task(task)
        else:
            task()
        return True

    def start_control(self, callback, safety_function=None):
        if not self.initialized or self.ctrl_session_active:
            return False

        self.control_callback = callback
        self.safety_function = safety_function
        self.ctrl_session_active = True
        self.ctrl_callback_busy = False
        self._safety_triggered = False

        try:
            self._init_dynamics()
        except Exception as e:
            logging.error(f"Cannot start control: dynamics initialization failed ({e})")
            self.ctrl_session_active = False
            return False

        try:
            if not self.operating_mode_init:
                self.bus.group_sync_write_torque_enable(self.motor_ids, 0)
                self.bus.group_sync_write_operating_mode([(i, rby.DynamixelBus.CurrentControlMode) for i in self.motor_ids])
                self.bus.group_sync_write_torque_enable(self.motor_ids, 1)
                for i in self.motor_ids:
                    self.state.operating_mode[i] = rby.DynamixelBus.CurrentControlMode
                self.operating_mode_init = True
        except Exception as e:
            logging.error(f"Failed to set initial operating modes: {e}")
            self.ctrl_session_active = False
            return False

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

        self.ctrl_session_active = False
        self.is_running = False

        if torque_disable:
            self.DisableTorque()

        self.ev.stop()
        self.ctrl_ev.stop()
        self.control_callback = None
        return True

    def _ev_task(self):
        """High-speed 100Hz data gathering, fault diagnosis & safety monitoring."""
        try:
            # 1. Read Tool Button Status
            new_tool_faults = []
            for tid in self.active_tool_ids:
                try:
                    res = self.bus.read_button_status(tid)
                    if res:
                        _, bstate = res
                        if tid == self.RIGHT_TOOL_ID:
                            self.state.button_right = bstate
                        else:
                            self.state.button_left = bstate
                    else:
                        new_tool_faults.append(tid)
                        self.state.fault_ids_history[tid - 0x80 + self.DOF] += 1
                except Exception as e:
                    new_tool_faults.append(tid)
                    self.state.fault_ids_history[tid - 0x80 + self.DOF] += 1
                    logging.debug(f"Exception reading tool {tid}: {e}")

            self.state.tool_fault_ids = sorted(list(set(new_tool_faults)))
            if not self.state.tool_fault_ids:
                self.tool_error_counts = 0
            else:
                self.tool_error_counts += 1

            # 2. Read Motor States (Fast 1-Shot Bulk Read)
            ms_list = None
            try:
                ms_list = self.bus.get_motor_states(self.active_joint_ids)
            except Exception as e:
                logging.debug(f"get_motor_states threw exception: {e}")

            if ms_list and len(ms_list) == len(self.active_joint_ids):
                # If recovering from a previous fault, re-synchronize operating modes and torque
                if self.joint_error_counts > 0 or len(self.state.joint_fault_ids) > 0:
                    self.recovery_sync_flag = True

                # Normal State: All active motors responded in 1 packet
                self.state.joint_fault_ids = []
                self.joint_error_counts = 0

                for mid, mstate in ms_list:
                    if mid < self.DOF:
                        self.state.q_joint[mid] = mstate.position
                        self.state.qvel_joint[mid] = mstate.velocity
                        self.state.current[mid] = mstate.current
                        self.state.torque_joint[mid] = mstate.current * self.torque_constant[mid]
                        self.state.temperatures[mid] = mstate.temperature

                # 3. Read Goal Positions if enabled
                if self.goal_pos_flag:
                    try:
                        temp_gp = self.bus.group_fast_sync_read(self.active_joint_ids, rby.DynamixelBus.AddrGoalPosition, 4)
                        if temp_gp:
                            for mid, val in temp_gp:
                                if mid < self.DOF:
                                    self.state.target_position[mid] = val / 4096.0 * 2.0 * np.pi
                    except Exception as e:
                        logging.debug(f"Error reading goal position: {e}")

                # 4. Compute Kinematics & Dynamics
                try:
                    self.dyn_state.set_q(self.state.q_joint)
                    self.robot.compute_forward_kinematics(self.dyn_state)

                    self.state.gravity_term = self.robot.compute_gravity_term(self.dyn_state) * self.TORQUE_SCALING

                    self.state.T_right = self.robot.compute_transformation(self.dyn_state, self.kBaseLinkId, self.kRightLinkId)
                    self.state.T_left = self.robot.compute_transformation(self.dyn_state, self.kBaseLinkId, self.kLeftLinkId)
                except Exception as e:
                    logging.error(f"Error computing dynamics/kinematics: {e}")
                    self.state.gravity_term.fill(0.0)

            else:
                # Failure State: Daisy-chain communication break diagnosed immediately
                self.joint_error_counts += 1
                self.state.gravity_term.fill(0.0)

                diagnosed_break = False
                for i, mid in enumerate(self.motor_ids):
                    try:
                        mode = self.bus.read_operating_mode(mid, False)
                        if mode is None:
                            broken_ids = self.motor_ids[i:]
                            self.state.joint_fault_ids = broken_ids
                            for fid in broken_ids:
                                self.state.fault_ids_history[fid] += 1
                            diagnosed_break = True
                            logging.debug(f"[LeaderArm] Daisy-chain break at ID {mid}. Faulted: {broken_ids}")
                            break
                    except Exception as e:
                        broken_ids = self.motor_ids[i:]
                        self.state.joint_fault_ids = broken_ids
                        for fid in broken_ids:
                            self.state.fault_ids_history[fid] += 1
                            diagnosed_break = True
                            break

                if not diagnosed_break:
                    self.state.joint_fault_ids = list(self.active_joint_ids)

            # Consolidate fault list
            self.state.fault_ids = sorted(list(set(self.state.joint_fault_ids) | set(self.state.tool_fault_ids)))
            self.state.joint_error_counts = self.joint_error_counts
            self.state.tool_error_counts = self.tool_error_counts

            # 5. Safety Trigger Check (Hardware Fault)
            if self.joint_error_counts > self.MAX_JOINT_RETRIES or self.tool_error_counts > self.MAX_TOOL_RETRIES:
                if self.safety_function and not self._safety_triggered:
                    self._safety_triggered = True
                    fault_state = self.state.copy()
                    threading.Thread(target=self.safety_function, args=(fault_state,), daemon=True).start()
                return

            # 6. Dispatch Control Task
            if self.control_callback and self.ctrl_session_active and not self.ctrl_callback_busy:
                self.ctrl_callback_busy = True
                captured_state = self.state.copy()
                self.ctrl_ev.push_task(lambda: self._ctrl_task(captured_state))

        except Exception as e:
            logging.error(f"[LeaderArm] UNEXPECTED EXCEPTION IN EV TASK: {e}")
            if self.safety_function and not self._safety_triggered:
                self._safety_triggered = True
                fault_state = self.state.copy()
                threading.Thread(target=self.safety_function, args=(fault_state,), daemon=True).start()

    def _ctrl_task(self, state):
        try:
            if state.joint_fault_ids:
                state.gravity_term = np.zeros(self.DOF)
            user_input = self.control_callback(state)
            if user_input and self.is_running:
                self._handle_control_input(user_input, state)
        except Exception as e:
            logging.error(f"[LeaderArm] EXCEPTION IN CONTROL CALLBACK: {e}")
            if self.safety_function and not self._safety_triggered:
                self._safety_triggered = True
                threading.Thread(target=self.safety_function, args=(state,), daemon=True).start()
        finally:
            self.ctrl_callback_busy = False

    def _handle_control_input(self, user_input, state):
        try:
            changed_ids = []
            changed_id_modes = []
            id_position = []
            id_torque = []

            for i in range(self.DOF):
                if self.recovery_sync_flag or self.state.operating_mode[i] != user_input.target_operating_mode[i]:
                    changed_ids.append(i)
                    changed_id_modes.append((i, user_input.target_operating_mode[i]))
                    self.state.operating_mode[i] = user_input.target_operating_mode[i]

                target_mode = user_input.target_operating_mode[i]
                max_t = float(self.MAXIMUM_TORQUE[i]) if isinstance(self.MAXIMUM_TORQUE, (list, tuple, np.ndarray)) else float(self.MAXIMUM_TORQUE)
                clipped_torque = float(np.clip(user_input.target_torque[i], -max_t, max_t))
                if target_mode == rby.DynamixelBus.CurrentControlMode:
                    id_torque.append((i, clipped_torque))
                elif target_mode == rby.DynamixelBus.CurrentBasedPositionControlMode:
                    id_torque.append((i, clipped_torque))
                    id_position.append((i, user_input.target_position[i]))

            self.recovery_sync_flag = False
            self.ev.push_task(lambda: self._write_task(changed_ids, changed_id_modes, id_torque, id_position))
        except Exception as e:
            logging.error(f"[LeaderArm] Error in _handle_control_input: {e}")

    def _write_task(self, changed_ids, changed_id_modes, id_torque, id_position):
        try:
            if not self.is_running:
                return
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
            if self.safety_function and not self._safety_triggered:
                self._safety_triggered = True
                threading.Thread(target=self.safety_function, args=(self.state.copy(),), daemon=True).start()

    def close(self):
        try:
            self.stop_control(torque_disable=True)
        except Exception as e:
            logging.warning(f"Error stopping control on close: {e}")

        if self.initialized:
            try:
                self.bus.close_port()
            except Exception as e:
                logging.warning(f"Error closing port: {e}")
            self.initialized = False


def main(address, model):
    robot = rby.create_robot(address, model)
    try:
        robot.connect()
    except Exception as e:
        print(f"Error: Connection to robot failed ({e})")
        sys.exit(1)

    if not robot.is_connected():
        print("Error: Robot is not connected.")
        sys.exit(1)

    # 1. Smart 12V Power & Control Manager State Validation
    try:
        is_12v_on = robot.is_power_on("12v")
        is_cm_enabled = (robot.get_control_manager_state().state == rby.ControlManagerState.State.Enabled)

        if is_cm_enabled and is_12v_on:
            print("Control manager is Enabled and 12V is ON. Using current state.")
        elif is_cm_enabled and not is_12v_on:
            print("Control manager is Enabled but 12V is OFF. Disabling control manager to power ON 12V...")
            robot.disable_control_manager()
            time.sleep(0.5)
            if not robot.power_on("12v"):
                print("Error: Failed to power on 12V.")
                sys.exit(1)
            print("Re-enabling control manager...")
            robot.enable_control_manager()
            time.sleep(0.5)
        elif not is_cm_enabled and not is_12v_on:
            print("Control manager is Disabled and 12V is OFF. Powering ON 12V only...")
            if not robot.power_on("12v"):
                print("Error: Failed to power on 12V.")
                sys.exit(1)
        else:  # not is_cm_enabled and is_12v_on
            print("Control manager is Disabled and 12V is ON. Using current state.")
    except Exception as e:
        print(f"Error configuring robot power / control manager: {e}")
        sys.exit(1)

    leader_arm = LeaderArm(control_period=0.01)
    leader_arm.set_max_retries(max_tool_retries=MAX_RETRY_COUNT_TOOL, max_joint_retries=MAX_RETRY_COUNT_JOINT)

    if not leader_arm.initialize(verbose=True):
        print("Failed to initialize Leader Arm")
        sys.exit(1)

    if len(leader_arm.active_ids) != leader_arm.DEVICE_COUNT:
        print(f"Error: Mismatch in device count. Expected {leader_arm.DEVICE_COUNT}, got {len(leader_arm.active_ids)}")
        expected_ids = leader_arm.motor_ids + leader_arm.tool_ids
        missing_ids = [dev_id for dev_id in expected_ids if dev_id not in leader_arm.active_ids]
        print(f"Missing Device IDs: {missing_ids}")
        print("Please check the connector status or motor malfunctions.")
        sys.exit(1)

    # Thread-safe container for decoupled monitor loop
    monitor_state_lock = threading.Lock()
    latest_monitor_state = leader_arm.state.copy()
    shutdown_event = threading.Event()
    shutdown_lock = threading.Lock()
    shutdown_done = False

    def fmt(arr):
        return ", ".join([f"{x:7.3f}" for x in arr])

    def fmt_deg(arr):
        return ", ".join([f"{np.rad2deg(x):7.1f}" for x in arr])

    def fmt_int(arr):
        return ", ".join([f"{int(x):7d}" for x in arr])

    # 100Hz Control Callback: Pure math & torque generation (NO print statements)
    def control(state: LeaderArm.State):
        nonlocal latest_monitor_state
        with monitor_state_lock:
            state.copy_to(latest_monitor_state)

        input_cmd = LeaderArm.ControlInput()
        input_cmd.target_operating_mode.fill(rby.DynamixelBus.CurrentControlMode)
        input_cmd.target_torque = state.gravity_term
        return input_cmd

    # Dedicated Monitoring Thread with configurable refresh rate
    def monitor_loop(refresh_hz: float = 20.0):
        interval_sec = 1.0 / max(refresh_hz, 1.0)
        local_snapshot = LeaderArm.State()
        while not shutdown_event.is_set():
            try:
                with monitor_state_lock:
                    latest_monitor_state.copy_to(local_snapshot)

                header = f"--- Leader Arm State Monitor ({refresh_hz:.0f}Hz UI / 100Hz Control) | {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} ---"
                line_idx = "index:        " + ", ".join([f"{i:7d}" for i in range(len(local_snapshot.q_joint))])
                line_q_deg = f"q (deg):      {fmt_deg(local_snapshot.q_joint)}"
                line_q_rad = f"q (rad):      {fmt(local_snapshot.q_joint)}"
                line_current = f"current (A):  {fmt(local_snapshot.current)}"
                line_temp = f"temp (C):     {fmt(local_snapshot.temperatures)}"
                line_torque = f"torque (Nm):  {fmt(local_snapshot.torque_joint)}"
                line_grav = f"gravity (Nm): {fmt(local_snapshot.gravity_term)}"
                line_btn = f"BTN   | L: {local_snapshot.button_left.button:1d} TRG: {local_snapshot.button_left.trigger:4d} | R: {local_snapshot.button_right.button:1d} TRG: {local_snapshot.button_right.trigger:4d}"
                line_fault = f"Fault IDs:    {local_snapshot.fault_ids}, (check time : {local_snapshot.check_status_duration * 1000.0:6.1f}ms)"
                history_joints = local_snapshot.fault_ids_history[:14]
                history_tools = local_snapshot.fault_ids_history[14:]
                line_hist_j = f"Joint Fault count:  {fmt_int(history_joints)}"
                line_hist_t = f"Tool Fault count:   right: {int(history_tools[0]):d} | left: {int(history_tools[1]):d}"

                if local_snapshot.fault_ids:
                    status_line = f"\033[1;33mSTATUS: [ WARNING - Communication Issues on IDs: {local_snapshot.fault_ids} ]\033[0m"
                else:
                    status_line = f"\033[1;32mSTATUS: [ NORMAL ]\033[0m"

                output_buffer = (
                    "\033[H\033[J"
                    + header + "\n"
                    + "-" * len(header) + "\n"
                    + line_idx + "\n"
                    + line_q_deg + "\n"
                    + line_q_rad + "\n"
                    + line_current + "\n"
                    + line_temp + "\n"
                    + line_torque + "\n"
                    + line_grav + "\n"
                    + line_btn + "\n"
                    + line_fault + "\n"
                    + line_hist_j + "\n"
                    + line_hist_t + "\n\n"
                    + status_line + "\n"
                )
                sys.stdout.write(output_buffer)
                sys.stdout.flush()
            except Exception as e:
                logging.warning(f"Error in monitor loop rendering: {e}")

            time.sleep(interval_sec)

    # Core Safety Function
    def safety_function(state: LeaderArm.State, use_soft: bool = True):
        nonlocal shutdown_done
        with shutdown_lock:
            if shutdown_done:
                return
            shutdown_done = True

        print(f"\n\n\033[1;31m[SAFETY / SHUTDOWN INITIATED (use_soft={use_soft})]\033[0m")
        if state.fault_ids:
            print(f"Detected Fault IDs: {state.fault_ids}")

        # 1. Stop high-speed background control loop first
        try:
            leader_arm.ctrl_session_active = False
            leader_arm.is_running = False
            leader_arm.ev.stop()
            leader_arm.ctrl_ev.stop()
        except Exception as e:
            logging.warning(f"Error stopping event loops during safety shutdown: {e}")

        if use_soft:
            print(f"Action: Soft Torque Ramp-down over {DECAY_TIME:.2f}s (10 steps) -> Cancel/Disable Control -> Power Off 12V...")
            # 2. Ramp down torque smoothly in 10 steps
            try:
                initial_torque = np.copy(state.torque_joint)
                if np.all(np.abs(initial_torque) < 1e-4):
                    initial_torque = np.copy(state.gravity_term)

                steps = 10
                step_dt = DECAY_TIME / float(steps)

                for step in range(1, steps + 1):
                    scale = 1.0 - (step / float(steps))
                    step_torque = initial_torque * scale
                    torque_cmd = [(i, float(step_torque[i])) for i in leader_arm.active_joint_ids if i < leader_arm.DOF]
                    try:
                        leader_arm.bus.group_sync_write_send_torque(torque_cmd)
                    except Exception as e:
                        logging.warning(f"Error sending ramp-down torque at step {step}: {e}")
                        break
                    time.sleep(step_dt)
            except Exception as e:
                logging.error(f"Exception during torque ramp-down: {e}")

            # 3. Disable Torque & Close serial port
            try:
                leader_arm.DisableTorque()
                leader_arm.close()
            except Exception as e:
                logging.warning(f"Error closing leader arm: {e}")

            # 4. Check Control Manager: Cancel if executing, disable, then power off 12V
            try:
                cm_state = robot.get_control_manager_state()
                if cm_state.state == rby.ControlManagerState.State.Enabled:
                    if cm_state.control_state != rby.ControlManagerState.ControlState.Idle:
                        print("Canceling active robot control...")
                        robot.cancel_control()
                        time.sleep(0.1)
                    print("Disabling control manager...")
                    robot.disable_control_manager()
                    time.sleep(0.2)
                print("Powering off 12V...")
                robot.power_off("12v")
            except Exception as e:
                logging.error(f"Error managing control manager / 12V power: {e}")

        else:
            # Instant Total Power Off (use_soft=False)
            print("Action: Immediate Emergency Shutdown (Instant Total Power Off & Torque Off)...")
            try:
                leader_arm.DisableTorque()
                leader_arm.close()
            except Exception as e:
                logging.warning(f"Error closing leader arm: {e}")

            try:
                print("Powering off all robot power lines (.*)...")
                robot.power_off(".*")
            except Exception as e:
                logging.error(f"Error powering off all power: {e}")

        shutdown_event.set()
        print("Shutdown sequence complete.")

    # Hardware fault callback: Follows USE_SOFT_STOP setting
    def hardware_fault_safety_cb(fault_state: LeaderArm.State):
        safety_function(fault_state, use_soft=USE_SOFT_STOP)

    # Register OS signal handlers (Ctrl+C / Termination: FORCED to use_soft=True)
    def signal_handler(signum, frame):
        print(f"\nSignal {signum} (Ctrl+C / Termination) received. Initiating soft safety shutdown...")
        threading.Thread(target=safety_function, args=(leader_arm.state.copy(), True), daemon=True).start()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start Control Engine with hardware fault callback
    if not leader_arm.start_control(control, safety_function=hardware_fault_safety_cb):
        print("Failed to start leader arm control.")
        leader_arm.close()
        sys.exit(1)

    # Start Monitor Thread with configured rate
    monitor_thread = threading.Thread(target=monitor_loop, args=(MONITOR_HZ,), daemon=True)
    monitor_thread.start()

    try:
        while not shutdown_event.is_set() and leader_arm.ctrl_session_active:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected in main loop.")
        safety_function(leader_arm.state.copy(), use_soft=True)
    finally:
        with shutdown_lock:
            already_done = shutdown_done

        if not already_done:
            # Normal / finally shutdown: FORCED to use_soft=True
            safety_function(leader_arm.state.copy(), use_soft=True)
            monitor_thread.join(timeout=0.5)
        else:
            # If safety shutdown was already initiated in another thread,
            # wait for it to complete.
            shutdown_event.wait()
            monitor_thread.join(timeout=0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Leader Arm State Check & Monitor")
    parser.add_argument("--address", type=str, required=True, help="Robot address (e.g. 192.168.30.1:50051)")
    parser.add_argument("--model", type=str, default="a", help="Robot Model Name (default: 'a')")
    args = parser.parse_args()

    main(address=args.address, model=args.model)
