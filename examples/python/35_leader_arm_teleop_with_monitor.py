################################ Note ################################
# This example does not run in simulation.
######################################################################
# This example initializes the robot, gripper, and leader arm connected to a UPC, moves the robot to a
# ready pose, and streams teleoperation commands that map leader arm joint motion and trigger input to
# robot arm and gripper control. See --help for arguments.
#
# Optimized version:
# 1. 100Hz high-speed control & teleop loop using 1-shot Bulk/Sync Read during normal operation.
# 2. Fast fault isolation & auto-recovery synchronization upon communication failure.
# 3. Decoupled 20Hz monitoring console rendering to eliminate stdout I/O jitter during teleoperation.
# 4. Soft torque ramp-down (3.0s, 10 steps) with control cancel/disable before 12V power off.
# 5. Configurable safety function: Hardware faults follow USE_SOFT_STOP; Ctrl+C / normal exits use soft stop.
# 6. Maximum torque clamping and clean hardware/port resource management.
#
# Usage example:
#     python 35_leader_arm_teleop_with_monitor.py --address 192.168.30.1:50051 --model m --mode impedance
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.
#
import rby1_sdk as rby
import numpy as np
import os
import time
import logging
import argparse
import signal
import threading
import datetime
from typing import *
from dataclasses import dataclass
import copy
import queue
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# URDF_PATH = os.path.join(SCRIPT_DIR, "models", "leader_arm", "model.urdf")
URDF_PATH = os.path.join(SCRIPT_DIR, "../../models/leader_arm", "model.urdf") # /../../models/leader_arm/model.urdf
LEADER_ARM_DEVICE_NAME = rby.upc.resolve_leader_arm_device_name()

GRIPPER_DIRECTION = False

# ============================================================
# Global Configuration & User Customization Parameters
# ============================================================
DECAY_TIME = 3.0              # Soft torque ramp-down time in seconds
MONITOR_HZ = 20.0             # Monitoring UI refresh rate (Hz)
MAX_RETRY_COUNT_TOOL = 10     # Maximum retry count for tool communication
MAX_RETRY_COUNT_JOINT = 10    # Maximum retry count for joint communication
USE_SOFT_STOP = True          # Hardware fault behavior: True (Soft ramp-down) / False (Instant power off)
DEFAULT_ALIGN_DURATION = 4.0   # Duration (seconds) for leader arm posture alignment to ready pose


@dataclass
class Pose:
    toros: np.typing.NDArray
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


# Robot Ready Pose Configurations (Degrees converted to Radians)
READY_POSE = {
    "A": Pose(
        toros=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    ),
    "M": Pose(
        toros=np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    ),
}


class Settings:
    leader_arm_loop_period = 1 / 100

    impedance_stiffness = 50
    impedance_damping_ratio = 1.0
    impedance_torque_limit = 30.0


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




# ============================================================
# Gripper (from leader_arm_teleop.py)
# ============================================================
class Gripper:
    def __init__(self):
        self.bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self.bus.open_port()
        self.bus.set_baud_rate(2_000_000)
        self.bus.set_torque_constant([1, 1])
        self.min_q = np.array([np.inf, np.inf])
        self.max_q = np.array([-np.inf, -np.inf])
        self.target_q: np.typing.NDArray = None
        self._running = False
        self._thread = None

    def initialize(self, verbose=True):
        rv = True
        for dev_id in [0, 1]:
            if not self.bus.ping(dev_id):
                if verbose:
                    logging.error(f"Dynamixel ID {dev_id} is not active")
                rv = False
            else:
                if verbose:
                    logging.info(f"Dynamixel ID {dev_id} is active")
        if rv:
            logging.info("Servo on gripper")
            self.bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])
        return rv

    def set_operating_mode(self, mode):
        self.bus.group_sync_write_torque_enable([(dev_id, 0) for dev_id in [0, 1]])
        self.bus.group_sync_write_operating_mode([(dev_id, mode) for dev_id in [0, 1]])
        self.bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])

    def homing(self):
        self.set_operating_mode(rby.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0, 0], dtype=np.float64)
        prev_q = np.array([0, 0], dtype=np.float64)
        counter = 0
        while direction < 2:
            self.bus.group_sync_write_send_torque(
                [(dev_id, 0.3 * (1 if direction == 0 else -1)) for dev_id in [0, 1]]
            )
            rv = self.bus.group_fast_sync_read_encoder([0, 1])
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)
            if np.array_equal(prev_q, q):
                counter += 1
            prev_q = q.copy()
            if counter >= 30:
                direction += 1
                counter = 0
            time.sleep(0.1)
        return True

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self.loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self.bus.close_port()
        except Exception:
            pass

    def loop(self):
        self.set_operating_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque([(dev_id, 5) for dev_id in [0, 1]])
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(
                    [(dev_id, q) for dev_id, q in enumerate(self.target_q.tolist())]
                )
            time.sleep(0.1)

    def set_target(self, normalized_q):
        if not np.isfinite(self.min_q).all() or not np.isfinite(self.max_q).all():
            logging.error("Cannot set target. min_q or max_q is not valid.")
            return
        normalized_q = np.clip(np.asarray(normalized_q, dtype=np.float64), 0.0, 1.0)
        if GRIPPER_DIRECTION:
            self.target_q = normalized_q * (self.max_q - self.min_q) + self.min_q
        else:
            self.target_q = (1 - normalized_q) * (self.max_q - self.min_q) + self.min_q


# ============================================================
# Robot Command Builders (from leader_arm_teleop.py)
# ============================================================
def joint_position_command_builder(
    pose: Pose, minimum_time, control_hold_time=0, position_mode=True
):
    right_arm_builder = (
        rby.JointPositionCommandBuilder()
        if position_mode
        else rby.JointImpedanceControlCommandBuilder()
    )
    (
        right_arm_builder.set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )
        .set_position(pose.right_arm)
        .set_minimum_time(minimum_time)
    )
    if not position_mode:
        (
            right_arm_builder.set_stiffness(
                [Settings.impedance_stiffness] * len(pose.right_arm)
            )
            .set_damping_ratio(Settings.impedance_damping_ratio)
            .set_torque_limit([Settings.impedance_torque_limit] * len(pose.right_arm))
        )

    left_arm_builder = (
        rby.JointPositionCommandBuilder()
        if position_mode
        else rby.JointImpedanceControlCommandBuilder()
    )
    (
        left_arm_builder.set_command_header(
            rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )
        .set_position(pose.left_arm)
        .set_minimum_time(minimum_time)
    )
    if not position_mode:
        (
            left_arm_builder.set_stiffness(
                [Settings.impedance_stiffness] * len(pose.left_arm)
            )
            .set_damping_ratio(Settings.impedance_damping_ratio)
            .set_torque_limit([Settings.impedance_torque_limit] * len(pose.left_arm))
        )

    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            .set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
                )
                .set_position(pose.toros)
                .set_minimum_time(minimum_time)
            )
            .set_right_arm_command(right_arm_builder)
            .set_left_arm_command(left_arm_builder)
        )
    )


def move_j(robot, pose: Pose, minimum_time=5.0):
    handler = robot.send_command(joint_position_command_builder(pose, minimum_time))
    return handler.get() == rby.RobotCommandFeedback.FinishCode.Ok


# ============================================================
# Leader Arm Posture Alignment (Cosine S-curve Interpolation)
# ============================================================
def move_leader_arm_to_pose(
    leader_arm: LeaderArm,
    target_q: np.ndarray,
    duration: float = 3.0,
    goal_torque: float = 0.5,
    control_period: float = 0.05,
) -> bool:
    """Smoothly moves the leader arm from its current pose to target_q using cosine interpolation.

    Args:
        leader_arm: Initialized LeaderArm instance.
        target_q: (14,) numpy array of target joint positions in radians.
        duration: Duration of the motion in seconds (<=0 to skip).
        goal_torque: Maximum torque limit (Nm) in CurrentBasedPositionControlMode.
        control_period: Loop period in seconds (default 0.01s = 100Hz).
    """
    if not leader_arm.initialized:
        logging.error("[move_leader_arm_to_pose] Leader arm is not initialized.")
        return False

    if len(target_q) != leader_arm.DOF:
        logging.error(
            f"[move_leader_arm_to_pose] target_q length mismatch: expected {leader_arm.DOF}, got {len(target_q)}"
        )
        return False

    if duration <= 0:
        return True

    print(f"[Leader Arm] Aligning posture to ready pose in progress ({duration:.1f}s)...")

    # 1. Read current motor positions
    ms_list = leader_arm.bus.get_motor_states(leader_arm.active_joint_ids)
    if not ms_list or len(ms_list) != len(leader_arm.active_joint_ids):
        logging.error("[move_leader_arm_to_pose] Failed to read initial motor states from leader arm.")
        return False

    start_q = np.zeros(leader_arm.DOF, dtype=np.float64)
    for mid, mstate in ms_list:
        if mid < leader_arm.DOF:
            start_q[mid] = mstate.position

    # 2. Switch to CurrentBasedPositionControlMode with safe torque
    target_mode = rby.DynamixelBus.CurrentBasedPositionControlMode
    leader_arm.bus.group_sync_write_torque_enable(leader_arm.active_joint_ids, 0)
    leader_arm.bus.group_sync_write_operating_mode(
        [(i, target_mode) for i in leader_arm.active_joint_ids if i < leader_arm.DOF]
    )
    leader_arm.bus.group_sync_write_torque_enable(leader_arm.active_joint_ids, 1)

    max_t = (
        float(leader_arm.MAXIMUM_TORQUE[0])
        if isinstance(leader_arm.MAXIMUM_TORQUE, (list, tuple, np.ndarray))
        else float(leader_arm.MAXIMUM_TORQUE)
    )
    safe_torque = min(goal_torque, max_t)

    # 3. Cosine S-curve interpolation loop
    t0 = time.time()
    steps = int(max(duration / control_period, 1))

    for step in range(steps + 1):
        elapsed = time.time() - t0
        ratio = min(elapsed / duration, 1.0)
        s = 0.5 * (1.0 - np.cos(np.pi * ratio))
        cmd_q = start_q + s * (target_q - start_q)

        pos_cmd = [(i, float(cmd_q[i])) for i in leader_arm.active_joint_ids if i < leader_arm.DOF]
        torque_cmd = [(i, safe_torque) for i in leader_arm.active_joint_ids if i < leader_arm.DOF]

        leader_arm.bus.group_sync_write_send_torque(torque_cmd)
        leader_arm.bus.group_sync_write_send_position(pos_cmd)

        time.sleep(control_period)

    print("[Leader Arm] Posture alignment complete.")
    return True


# ============================================================
# Main
# ============================================================
def main(address, model_name, power, servo, control_mode, align_duration=DEFAULT_ALIGN_DURATION):
    # ===== 1. CONNECT ROBOT & POWER ON 12V / SERVOS (FIRST) =====
    print("\n[Step 1/5] Connecting to Follower Robot & Powering On 12V / Servos...")
    robot = rby.create_robot(address, model_name)
    try:
        if not robot.connect():
            logging.error(f"Failed to connect robot {address}")
            sys.exit(1)
    except Exception as e:
        logging.error(f"Failed to connect robot {address}: {e}")
        sys.exit(1)

    supported_model = ["A", "M"]
    supported_control_mode = ["position", "impedance"]
    model = robot.model()
    dyn_model = robot.get_dynamics()
    dyn_state = dyn_model.make_state([], model.robot_joint_names)
    robot_q = None
    robot_max_q = dyn_model.get_limit_q_upper(dyn_state)
    robot_min_q = dyn_model.get_limit_q_lower(dyn_state)
    robot_max_qdot = dyn_model.get_limit_qdot_upper(dyn_state)
    robot_max_qddot = dyn_model.get_limit_qddot_upper(dyn_state)

    if control_mode == "impedance":
        robot_max_qdot[model.right_arm_idx[-1]] *= 10
        robot_max_qdot[model.left_arm_idx[-1]] *= 10

    if model.model_name not in supported_model:
        logging.error(
            f"Model {model.model_name} not supported (Supported: {supported_model})"
        )
        sys.exit(1)
    if control_mode not in supported_control_mode:
        logging.error(
            f"Control mode {control_mode} not supported (Supported: {supported_control_mode})"
        )
        sys.exit(1)

    position_mode = (control_mode == "position")

    # Smart 12V Power, Servo & Control Manager State Validation
    try:
        # Reset any lingering fault in control manager
        robot.reset_fault_control_manager()

        cm_state = robot.get_control_manager_state()
        is_cm_enabled = (cm_state.state == rby.ControlManagerState.State.Enabled)
        is_12v_on = robot.is_power_on("12v")

        # Major Fault prevention: Power cannot be toggled while Control Manager is Enabled
        if is_cm_enabled and not is_12v_on:
            print("Control manager is Enabled but 12V is OFF. Temporarily disabling control manager to power ON 12V safely...")
            robot.disable_control_manager()
            time.sleep(0.5)
            is_cm_enabled = False

        # Turn ON Power
        if not robot.is_power_on(power):
            print(f"Powering ON power lines ({power})...")
            if not robot.power_on(power):
                logging.error(f"Failed to turn power ({power}) on")
                sys.exit(1)
            # Power stabilization & Dynamixel MCU boot delay (1.0s)
            print("Waiting 1.0s for 12V power stabilization & Leader Arm motor bootup...")
            time.sleep(1.0)

        # Turn ON Servos
        if not robot.is_servo_on(servo):
            print(f"Turning ON servos ({servo})...")
            if not robot.servo_on(servo):
                logging.error(f"Failed to servo ({servo}) on")
                sys.exit(1)
            time.sleep(0.3)

        # Enable Control Manager
        robot.reset_fault_control_manager()
        if not robot.enable_control_manager():
            logging.error("Failed to enable control manager")
            sys.exit(1)

        for arm in ["right", "left"]:
            if not robot.set_tool_flange_output_voltage(arm, 12):
                logging.error(f"Failed to set tool flange output voltage ({arm}) as 12v")
                sys.exit(1)

        robot.set_parameter("joint_position_command.cutoff_frequency", "3")
        print("[Step 1/5] Robot 12V power & Servos are ON and Control Manager is Enabled.")
    except Exception as e:
        logging.error(f"Error configuring robot power/servos: {e}")
        sys.exit(1)

    # ===== 2. LEADER ARM SETUP & HEALTH CHECK (Now powered via 12V) =====
    print("\n[Step 2/5] Initializing Leader Arm & Checking Hardware Status...")
    leader_arm = LeaderArm(
        control_period=Settings.leader_arm_loop_period,
    )
    leader_arm.set_max_retries(max_tool_retries=MAX_RETRY_COUNT_TOOL, max_joint_retries=MAX_RETRY_COUNT_JOINT)
    active_ids = leader_arm.initialize(verbose=True)

    if len(leader_arm.active_ids) != leader_arm.DEVICE_COUNT:
        logging.error(
            f"Mismatch in the number of devices detected. "
            f"Expected {leader_arm.DEVICE_COUNT}, got {len(leader_arm.active_ids)}"
        )
        expected_ids = leader_arm.motor_ids + leader_arm.tool_ids
        missing_ids = [dev_id for dev_id in expected_ids if dev_id not in leader_arm.active_ids]
        logging.error(f"Missing Device IDs: {missing_ids}")
        logging.error("Please check the connector status or motor malfunctions.")
        try:
            robot.disable_control_manager()
            robot.power_off("12v")
        except Exception:
            pass
        leader_arm.close()
        sys.exit(1)
    print(f"[Step 2/5] Leader Arm Hardware OK (All {leader_arm.DEVICE_COUNT} devices active).")

    # ===== 3. MOVE ROBOT TO READY POSE =====
    print("\n[Step 3/5] Moving Robot to Ready Pose...")
    move_j(robot, READY_POSE[model.model_name], 5)
    print("[Step 3/5] Robot ready pose reached.")

    def robot_state_callback(state: rby.RobotState_A):
        nonlocal robot_q
        robot_q = state.position

    robot.start_state_update(robot_state_callback, 1 / Settings.leader_arm_loop_period)

    # ===== 4. SETUP GRIPPER =====
    print("\n[Step 4/5] Initializing Gripper...")
    gripper = Gripper()
    if not gripper.initialize():
        logging.error("Failed to initialize gripper")
        robot.stop_state_update()
        robot.power_off("12v")
        leader_arm.close()
        sys.exit(1)
    gripper.homing()
    gripper.start()
    print("[Step 4/5] Gripper initialized.")

    # ===== 5. LEADER ARM POSTURE ALIGNMENT TO READY POSE =====
    target_leader_ready_q = np.concatenate([
        READY_POSE[model.model_name].right_arm,
        READY_POSE[model.model_name].left_arm,
    ])
    if align_duration > 0:
        print("\n[Step 5/5] Aligning Leader Arm Posture to Match Robot Ready Pose...")
        move_leader_arm_to_pose(
            leader_arm=leader_arm,
            target_q=target_leader_ready_q,
            duration=align_duration,
            goal_torque=0.5,
            control_period=Settings.leader_arm_loop_period,
        )
        print("[Step 5/5] Leader Arm alignment complete.")
    else:
        print("\n[Step 5/5] Leader Arm posture alignment skipped (--align-duration 0).")

    # ===== TELEOP PARAMETERS =====
    ma_q_limit_barrier = 0.0
    ma_min_q = np.deg2rad(
        [-180, -60, -90, -150, -180, -90, -180, -180, 10, -90, -150, -180, -90, -180]
    )
    ma_max_q = np.deg2rad(
        [180, -10, 90, 0, 180, 90, 180, 180, 60, 90, 0, 180, 90, 180]
    )
    ma_torque_limit = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5] * 2)
    ma_viscous_gain = np.array([0.01, 0.01, 0.01, 0.01, 0.005, 0.005, 0.001] * 2)
    right_q = READY_POSE[model.model_name].right_arm.copy()
    left_q = READY_POSE[model.model_name].left_arm.copy()
    right_minimum_time = 1.0
    left_minimum_time = 1.0
    last_collision_log_time = 0.0

    stream = robot.create_command_stream(priority=1)
    stream.send_command(
        joint_position_command_builder(
            READY_POSE[model.model_name],
            minimum_time=5,
            control_hold_time=1e6,
            position_mode=position_mode,
        )
    )

    # Thread-safe container for decoupled 20Hz monitor loop
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

    # =========================================================
    # 100Hz CONTROL CALLBACK (Teleop Calculation Only - NO Print I/O)
    # =========================================================
    def leader_arm_control_loop(state: LeaderArm.State):
        nonlocal position_mode, right_q, left_q
        nonlocal right_minimum_time, left_minimum_time
        nonlocal last_collision_log_time
        nonlocal latest_monitor_state

        with monitor_state_lock:
            state.copy_to(latest_monitor_state)

        if right_q is None:
            right_q = state.q_joint[0:7]
        if left_q is None:
            left_q = state.q_joint[7:14]

        # 1. Gripper Control
        gripper_target = np.array(
            [state.button_right.trigger, state.button_left.trigger],
            dtype=np.float64,
        ) / 1000.0
        gripper.set_target(gripper_target)

        # 2. Leader Arm Torque Calculation (100% Gravity Compensation matching 20_state_check)
        ma_input = LeaderArm.ControlInput()

        torque = (
            state.gravity_term * 1.0
            + ma_q_limit_barrier
            * (
                np.maximum(ma_min_q - state.q_joint, 0)
                + np.minimum(ma_max_q - state.q_joint, 0)
            )
            + ma_viscous_gain * state.qvel_joint
        )
        torque = np.clip(torque, -ma_torque_limit, ma_torque_limit)

        # Right arm
        if state.button_right.button == 1:
            ma_input.target_operating_mode[0:7].fill(
                rby.DynamixelBus.CurrentControlMode
            )
            ma_input.target_torque[0:7] = torque[0:7] * 0.8
            right_q = state.q_joint[0:7]
        else:
            ma_input.target_operating_mode[0:7].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[0:7] = ma_torque_limit[0:7]
            ma_input.target_position[0:7] = right_q

        # Left arm
        if state.button_left.button == 1:
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentControlMode
            )
            ma_input.target_torque[7:14] = torque[7:14] * 0.8
            left_q = state.q_joint[7:14]
        else:
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[7:14] = ma_torque_limit[7:14]
            ma_input.target_position[7:14] = left_q

        # 3. Build & Send Robot Command
        if robot_q is None:
            right_minimum_time = 0.8
            left_minimum_time = 0.8
            return ma_input

        q = robot_q.copy()
        q[model.right_arm_idx] = right_q
        q[model.left_arm_idx] = left_q
        dyn_state.set_q(q)
        dyn_model.compute_forward_kinematics(dyn_state)
        nearest_collision = dyn_model.detect_collisions_or_nearest_links(dyn_state, 1)[0]
        is_collision = nearest_collision.distance < 0.02

        if is_collision and (state.button_right.button or state.button_left.button):
            now = time.monotonic()
            if now - last_collision_log_time >= 1.0:
                warning_msg = (
                    "[COLLISION BLOCK] Robot command blocked. "
                    f"nearest distance: {nearest_collision.distance:.4f} m"
                )
                logging.warning(warning_msg)
                last_collision_log_time = now

        rc = rby.BodyComponentBasedCommandBuilder()

        if state.button_right.button and not is_collision:
            right_minimum_time -= Settings.leader_arm_loop_period
            right_minimum_time = max(
                right_minimum_time, Settings.leader_arm_loop_period * 1.01
            )
            right_arm_builder = (
                rby.JointPositionCommandBuilder()
                if position_mode
                else rby.JointImpedanceControlCommandBuilder()
            )
            (
                right_arm_builder.set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1e6)
                )
                .set_position(
                    np.clip(
                        right_q,
                        robot_min_q[model.right_arm_idx],
                        robot_max_q[model.right_arm_idx],
                    )
                )
                .set_velocity_limit(robot_max_qdot[model.right_arm_idx])
                .set_acceleration_limit(robot_max_qddot[model.right_arm_idx] * 30)
                .set_minimum_time(right_minimum_time)
            )
            if not position_mode:
                (
                    right_arm_builder.set_stiffness(
                        [Settings.impedance_stiffness] * len(model.right_arm_idx)
                    )
                    .set_damping_ratio(Settings.impedance_damping_ratio)
                    .set_torque_limit(
                        [Settings.impedance_torque_limit] * len(model.right_arm_idx)
                    )
                )
            rc.set_right_arm_command(right_arm_builder)
        else:
            right_minimum_time = 0.8

        if state.button_left.button and not is_collision:
            left_minimum_time -= Settings.leader_arm_loop_period
            left_minimum_time = max(
                left_minimum_time, Settings.leader_arm_loop_period * 1.01
            )
            left_arm_builder = (
                rby.JointPositionCommandBuilder()
                if position_mode
                else rby.JointImpedanceControlCommandBuilder()
            )
            (
                left_arm_builder.set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1e6)
                )
                .set_position(
                    np.clip(
                        left_q,
                        robot_min_q[model.left_arm_idx],
                        robot_max_q[model.left_arm_idx],
                    )
                )
                .set_velocity_limit(robot_max_qdot[model.left_arm_idx])
                .set_acceleration_limit(robot_max_qddot[model.left_arm_idx] * 30)
                .set_minimum_time(left_minimum_time)
            )
            if not position_mode:
                (
                    left_arm_builder.set_stiffness(
                        [Settings.impedance_stiffness] * len(model.left_arm_idx)
                    )
                    .set_damping_ratio(Settings.impedance_damping_ratio)
                    .set_torque_limit(
                        [Settings.impedance_torque_limit] * len(model.left_arm_idx)
                    )
                )
            rc.set_left_arm_command(left_arm_builder)
        else:
            left_minimum_time = 0.8

        stream.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(rc)
            )
        )

        return ma_input

    # =========================================================
    # 20Hz Dedicated Monitoring Thread
    # =========================================================
    def monitor_loop(refresh_hz: float = 20.0):
        interval_sec = 1.0 / max(refresh_hz, 1.0)
        local_snapshot = LeaderArm.State()
        while not shutdown_event.is_set():
            try:
                with monitor_state_lock:
                    latest_monitor_state.copy_to(local_snapshot)

                header = f"--- Teleop & Leader Arm Monitor ({refresh_hz:.0f}Hz UI / 100Hz Control) | {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} ---"
                line_idx = "index:        " + ", ".join([f"{i:7d}" for i in range(len(local_snapshot.q_joint))])
                line_q_deg = f"q (deg):      {fmt_deg(local_snapshot.q_joint)}"
                line_q_rad = f"q (rad):      {fmt(local_snapshot.q_joint)}"
                line_current = f"current (A):  {fmt(local_snapshot.current)}"
                line_temp = f"temp (C):     {fmt(local_snapshot.temperatures)}"
                line_torque = f"torque (Nm):  {fmt(local_snapshot.torque_joint)}"
                line_grav = f"gravity (Nm): {fmt(local_snapshot.gravity_term)}"
                line_btn = (
                    f"BTN   | L: {local_snapshot.button_left.button:1d} TRG: {local_snapshot.button_left.trigger:4d}"
                    f" | R: {local_snapshot.button_right.button:1d} TRG: {local_snapshot.button_right.trigger:4d}"
                )
                line_teleop = (
                    f"TELEOP | R_btn: {local_snapshot.button_right.button} L_btn: {local_snapshot.button_left.button}"
                    f" | mode: {'position' if position_mode else 'impedance'}"
                )
                line_fault = f"Fault IDs:    {local_snapshot.fault_ids}, (check time : {local_snapshot.check_status_duration * 1000.0:6.1f}ms)"
                history_joints = local_snapshot.fault_ids_history[:14]
                history_tools = local_snapshot.fault_ids_history[14:]
                line_hist_j = f"Joint Fault count:  {fmt_int(history_joints)}"
                line_hist_t = f"Tool Fault count:   right: {int(history_tools[0]):d} | left: {int(history_tools[1]):d}"

                if local_snapshot.fault_ids:
                    status_line = f"\033[1;31mSTATUS: [ !! CRITICAL ALARM !! - FAILED IDs: {local_snapshot.fault_ids} ]\033[0m"
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
                    + line_teleop + "\n"
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

    # =========================================================
    # SAFETY FUNCTION (Soft Ramp-down or Instant Stop)
    # =========================================================
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

        # 2. Cancel robot command stream first to stop streaming teleoperation commands
        try:
            if stream is not None:
                print("Canceling robot command stream...")
                stream.cancel()
        except NameError:
            pass
        except Exception as e:
            logging.warning(f"Error canceling robot command stream: {e}")

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

            # 3. Disable Torque & Close leader arm port
            try:
                leader_arm.DisableTorque()
                leader_arm.close()
            except Exception as e:
                logging.warning(f"Error closing leader arm: {e}")

            # 4. Stop Gripper & Robot State Update
            try:
                gripper.stop()
            except Exception as e:
                logging.warning(f"Error stopping gripper: {e}")

            try:
                robot.stop_state_update()
            except Exception as e:
                logging.warning(f"Error stopping robot state update: {e}")

            # 5. Cancel Robot Control & Disable Control Manager & 12V Off
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
                gripper.stop()
            except Exception:
                pass

            try:
                robot.stop_state_update()
            except Exception:
                pass

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

    # =========================================================
    # START CONTROL & MONITORING
    # =========================================================
    if not leader_arm.start_control(leader_arm_control_loop, safety_function=hardware_fault_safety_cb):
        logging.error("Failed to start leader arm control.")
        gripper.stop()
        robot.stop_state_update()
        leader_arm.close()
        sys.exit(1)

    # Start 20Hz Dedicated Monitoring Thread
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
            safety_function(leader_arm.state.copy(), use_soft=True)
            monitor_thread.join(timeout=0.5)
        else:
            # If safety shutdown was already initiated in another thread,
            # wait for it to complete.
            shutdown_event.wait()
            monitor_thread.join(timeout=0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Teleoperation with Real-time Monitoring & Safety"
    )
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Regex pattern for power device names (default: '.*')",
    )
    parser.add_argument(
        "--servo",
        type=str,
        default="torso_.*|right_arm_.*|left_arm_.*",
        help="Regex pattern for servo names",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="position",
        choices=["position", "impedance"],
        help="Control mode: 'position' or 'impedance' (default: 'position')",
    )
    parser.add_argument(
        "--align-duration",
        type=float,
        default=4.0,
        help="Duration (seconds) for leader arm posture alignment to ready pose (default: 4.0, 0 to skip)",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        model_name=args.model,
        power=args.power,
        servo=args.servo,
        control_mode=args.mode,
        align_duration=args.align_duration,
    )
