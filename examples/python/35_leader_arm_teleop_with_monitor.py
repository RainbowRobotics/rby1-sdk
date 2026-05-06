################################ Note ################################
# This example does not run in simulation.
######################################################################
# This example initializes the robot, gripper, and leader arm connected to a UPC, moves the robot to a
# ready pose, and streams teleoperation commands that map leader arm joint motion and trigger input to
# robot arm and gripper control. See --help for arguments.
#
# additional function
# monitoring leader arm's q,torque,current,etc.
# if connect failed continuously and it's occurred more than N times, operate safety function and close node.
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
# Usage example:
#     python 35_leader_arm_teleop_with_monitor.py --address 192.168.30.1:50051 --model m --mode impedance
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(SCRIPT_DIR, "../../models/leader_arm", "model.urdf") # /../../models/leader_arm/model.urdf
LEADER_ARM_DEVICE_NAME = rby.upc.LeaderArmDeviceName

GRIPPER_DIRECTION = False

            
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

    class ButtonState: # Button state structure
        __slots__ = ['button', 'trigger']
        def __init__(self, b, t):
            self.button = b
            self.trigger = t

    class State: # Variables stored by default
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
            self.fault_ids = [] # IDs with communication failures
            self.joint_fault_ids = []
            self.tool_fault_ids = [] # Tool IDs with communication failures
            self.fault_ids_history = np.zeros(dof +2, dtype=np.int64)
            self.current = np.zeros(dof, dtype=np.float64)
            self.tool_error_counts = 0
            self.joint_error_counts = 0
            self.check_status_duration = 0.0

        # Copy data before use to prevent memory access conflicts
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
    
    # Implemented in the same way as the existing multithread management logic
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

    # Initialization function for the leader arm class
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
    
    # Define the dynamics model
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

    # Same as the existing initialization logic. The C++ version is longer and uses modularized functions.
    def initialize(self, verbose=False):
        # Configure basic logging to display internal thread errors in the terminal
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

    # Move the motors by entering poses for QC
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

    # Existing start_control function. The task details are below.
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

    # Data reading task
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

    # Task that runs the user-defined callback function
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

# ============================================================
# Data Structures & Settings (from leader_arm_teleop.py)
# ============================================================
@dataclass
class Pose:
    toros: np.typing.NDArray
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


class Settings:
    leader_arm_loop_period = 1 / 100

    impedance_stiffness = 50
    impedance_damping_ratio = 1.0
    impedance_torque_limit = 30.0


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
            self._thread.join()
            self._thread = None

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
# Main
# ============================================================
def main(address, model_name, power, servo, control_mode):
    # ===== SETUP ROBOT =====
    robot = rby.create_robot(address, model_name)
    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)

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
        exit(1)
    if control_mode not in supported_control_mode:
        logging.error(
            f"Control mode {control_mode} not supported (Supported: {supported_control_mode})"
        )
        exit(1)

    position_mode = control_mode == "position"

    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            logging.error(f"Failed to servo ({servo}) on")
            exit(1)
    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        logging.error("Failed to enable control manager")
        exit(1)
    for arm in ["right", "left"]:
        if not robot.set_tool_flange_output_voltage(arm, 12):
            logging.error(f"Failed to set tool flange output voltage ({arm}) as 12v")
            exit(1)
    robot.set_parameter("joint_position_command.cutoff_frequency", "3")
    move_j(robot, READY_POSE[model.model_name], 5)

    def robot_state_callback(state: rby.RobotState_A):
        nonlocal robot_q
        robot_q = state.position

    robot.start_state_update(robot_state_callback, 1 / Settings.leader_arm_loop_period)

    # ===== SETUP GRIPPER =====
    gripper = Gripper()
    if not gripper.initialize():
        logging.error("Failed to initialize gripper")
        robot.stop_state_update()
        robot.power_off("12v")
        exit(1)
    gripper.homing()
    gripper.start()

    # ===== LEADER ARM SETUP =====
    leader_arm = LeaderArm(
        control_period=Settings.leader_arm_loop_period,
    )
    active_ids = leader_arm.initialize(verbose=True)

    if len(leader_arm.active_ids) != leader_arm.DEVICE_COUNT:
        logging.error(
            f"Mismatch in the number of devices detected. "
            f"Expected {leader_arm.DEVICE_COUNT}, got {len(leader_arm.active_ids)}"
        )
        exit(1)

    # ===== TELEOP PARAMETERS =====
    ma_q_limit_barrier = 0.5
    ma_min_q = np.deg2rad(
        [-360, -30, 0, -135, -90, 35, -360, -360, 10, -90, -135, -90, 35, -360]
    )
    ma_max_q = np.deg2rad(
        [360, -10, 90, -60, 90, 80, 360, 360, 30, 0, -60, 90, 80, 360]
    )
    ma_torque_limit = np.array([3.5, 3.5, 3.5, 1.5, 1.5, 1.5, 1.5] * 2)
    ma_viscous_gain = np.array([0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.002] * 2)
    right_q = None
    left_q = None
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

    # ===== SESSION STATISTICS (from state_check) =====
    session_stats = {
        "total_warnings": 0,
        "max_streak": 0,
        "has_warned_once": False,
        "ever_warned_ids": set(),
    }

    def fmt(arr):
        return ", ".join([f"{x:7.3f}" for x in arr])

    # =========================================================
    # CONTROL CALLBACK (Teleop + Real-time Monitoring)
    # =========================================================
    def leader_arm_control_loop(state: LeaderArm.State):
        nonlocal position_mode, right_q, left_q
        nonlocal right_minimum_time, left_minimum_time
        nonlocal session_stats
        nonlocal last_collision_log_time

        if right_q is None:
            right_q = state.q_joint[0:7]
        if left_q is None:
            left_q = state.q_joint[7:14]

        # --------------------------------------------------
        # 1. Real-time Monitoring Display (from state_check)
        # --------------------------------------------------
        current_max_streak = max(state.joint_error_counts, state.tool_error_counts)
        if current_max_streak > session_stats["max_streak"]:
            session_stats["max_streak"] = current_max_streak

        header = f"--- Teleop Monitor | {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} ---"
        line_idx = "index:        " + ", ".join([f"{i:7d}" for i in range(len(state.q_joint))])
        line_q = f"q (rad):      {fmt(state.q_joint)}"
        line_current = f"current (A):  {fmt(state.current)}"
        line_temp = f"temp (C):     {fmt(state.temperatures)}"
        line_torque = f"torque (Nm):  {fmt(state.torque_joint)}"
        line_grav = f"gravity (Nm): {fmt(state.gravity_term)}"
        line_btn = (
            f"BTN   | L: {state.button_left.button:1d} TRG: {state.button_left.trigger:4d}"
            f" | R: {state.button_right.button:1d} TRG: {state.button_right.trigger:4d}"
        )
        line_teleop = (
            f"TELEOP | R_btn: {state.button_right.button} L_btn: {state.button_left.button}"
            f" | mode: {'position' if position_mode else 'impedance'}"
        )

        # Status line
        stats_part = (
            f"(Tot: {session_stats['total_warnings']}, "
            f"Max: {session_stats['max_streak']}, "
            f"Hist IDs: {sorted(list(session_stats['ever_warned_ids']))})"
        )

        if state.fault_ids or state.tool_fault_ids:
            all_faults = sorted(list(state.fault_ids) + list(state.tool_fault_ids))
            status_line = f"\033[1;31mSTATUS: [ !! CRITICAL ALARM !! - FAILED IDs: {all_faults} ] {stats_part}\033[0m"
        elif session_stats["has_warned_once"]:
            status_line = f"\033[1;33mSTATUS: [ PAST WARNINGS DETECTED ] {stats_part}\033[0m"
        else:
            status_line = f"\033[1;32mSTATUS: [ NORMAL ] {stats_part}\033[0m"

        # Display
        print("\033[H\033[J", end="", flush=True)
        print(header, flush=True)
        print("-" * len(header), flush=True)
        print(line_idx, flush=True)
        print(line_q, flush=True)
        print(line_current, flush=True)
        print(line_temp, flush=True)
        print(line_torque, flush=True)
        print(line_grav, flush=True)
        print(line_btn, flush=True)
        print(line_teleop, flush=True)
        print("\n" + status_line, flush=True)

        if state.tool_fault_ids:
            warning_msg = f"! [TOOL WARNING] Communication failure on IDs: {state.tool_fault_ids}"
            print(warning_msg, flush=True)

        # --------------------------------------------------
        # 2. Gripper Control
        # --------------------------------------------------
        gripper_target = np.array(
            [state.button_right.trigger, state.button_left.trigger],
            dtype=np.float64,
        ) / 1000.0
        gripper.set_target(gripper_target)

        # --------------------------------------------------
        # 3. Leader Arm Torque Calculation (from teleop)
        # --------------------------------------------------
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
            ma_input.target_torque[0:7] = torque[0:7] * 0.6
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
            ma_input.target_torque[7:14] = torque[7:14] * 0.6
            left_q = state.q_joint[7:14]
        else:
            ma_input.target_operating_mode[7:14].fill(
                rby.DynamixelBus.CurrentBasedPositionControlMode
            )
            ma_input.target_torque[7:14] = ma_torque_limit[7:14]
            ma_input.target_position[7:14] = left_q

        # --------------------------------------------------
        # 4. Build & Send Robot Command (from teleop)
        # --------------------------------------------------
        if robot_q is None:
            right_minimum_time = 0.8
            left_minimum_time = 0.8
            return ma_input

        q = robot_q.copy()
        q[model.right_arm_idx] = right_q
        q[model.left_arm_idx] = left_q
        dyn_state.set_q(q)
        dyn_model.compute_forward_kinematics(dyn_state)
        nearest_collision = dyn_model.detect_collisions_or_nearest_links(dyn_state, 1)[
            0
        ]
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
    # SAFETY FUNCTION (from leader_arm_state_check.py)
    # Immediately cut 12V power and exit the process if a communication fault occurs
    # =========================================================
    def safety_function(state: LeaderArm.State):
        import sys

        sys.stdout.flush()
        sys.stderr.flush()

        all_faults = sorted(list(state.fault_ids) + list(state.tool_fault_ids))
        error_msg = f"\n\n\033[1;31m[CRITICAL ERROR / EXCEPTION DETECTED]\033[0m\n"
        if all_faults:
            error_msg += f"Communication failure on IDs: {all_faults}\n"
        error_msg += "ACTION: Immediate Emergency Shutdown (Power Off 12V).\n"

        print(error_msg, flush=True)

        # Priority 1: Hardware safety - immediately turn off leader arm torque; call directly because stop_control can deadlock
        try:
            leader_arm.DisableTorque()
        except Exception:
            pass

        # Priority 2: Immediately cut 12V power
        try:
            robot.power_off("12v")
        except Exception:
            pass

        # Priority 3: Clean up robot control
        try:
            robot.stop_state_update()
        except Exception:
            pass
        try:
            robot.cancel_control()
        except Exception:
            pass
        try:
            robot.disable_control_manager()
        except Exception:
            pass

        # Priority 4: Leader arm & gripper cleanup
        try:
            leader_arm.stop_control(torque_disable=False)  # Torque is already off
        except Exception:
            pass
        try:
            gripper.stop()
        except Exception:
            pass

        print("Shutdown complete. Exiting.", flush=True)
        sys.stdout.flush()
        time.sleep(0.5)
        os._exit(1)

    # =========================================================
    # START CONTROL LOOP
    # =========================================================
    leader_arm.start_control(leader_arm_control_loop, safety_function=safety_function)

    # ===== SIGNAL HANDLER (Ctrl+C) =====
    def handler(signum, frame):
        print("\n\nInterrupt received. Stopping...")
        robot.stop_state_update()
        if leader_arm:
            leader_arm.close()
        robot.cancel_control()
        time.sleep(0.5)
        robot.disable_control_manager()
        robot.power_off("12v")
        gripper.stop()
        print("System shutdown complete.")
        os._exit(0)

    signal.signal(signal.SIGINT, handler)

    # Main thread waits
    while leader_arm.ctrl_session_active:
        time.sleep(1)


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
    args = parser.parse_args()

    main(
        address=args.address,
        model_name=args.model,
        power=args.power,
        servo=args.servo,
        control_mode=args.mode,
    )
