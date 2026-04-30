// Teleoperation Example

//  This example initializes the robot, gripper, and leader arm connected to a UPC, moves the robot to a
//  ready pose, and streams teleoperation commands that map leader arm joint motion and trigger input to
//  robot arm and gripper control. See --help for arguments.

//  Usage example:
//      ./example_34_teleoperation_with_joint_mapping.py --address 192.168.30.1:50051 --model a --servo 'torso_.*|right_arm_.*|left_arm_.*' --mode position --mode position

//  Copyright (c) 2025 Rainbow Robotics. All rights reserved.

//  DISCLAIMER:
//  This is a sample code provided for educational and reference purposes only.
//  Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
//  the use or misuse of this demo code. Please use with caution and at your own discretion.


//  Run this example on a UPC to which the leader arm and gripper are connected.
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__s
#endif

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <type_traits>
#include "dynamixel_sdk.h"

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/dynamics/state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"
#include "rby1-sdk/upc/device.h"

using namespace rb;
using namespace std::chrono_literals;

const std::string kAll = ".*";

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m] [--power <regex>] [--servo <regex>] [--mode position|impedance]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex] [mode]" << std::endl;
}

// Protocol version
#define PROTOCOL_VERSION 2.0  // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE 2000000

#define ADDR_TORQUE_ENABLE 64
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116
#define ADDR_OPERATING_MODE 11  // Address for operating mode

#define CURRENT_CONTROL_MODE 0
#define CURRENT_BASED_POSITION_CONTROL_MODE 5

#define ADDR_PRESENT_BUTTON_STATUS 132
#define ADDR_GOAL_VIBRATION_LEVEL 102

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define D2R 0.017453288888888
#define R2D 57.29579143313326

double m_sf = 0.4;
constexpr double kImpedanceStiffness = 50.0;
constexpr double kImpedanceDampingRatio = 1.0;
constexpr double kImpedanceTorqueLimit = 30.0;

using namespace rb::dyn;
using namespace rb;

#define MIN_INDEX 0
#define MAX_INDEX 1

std::mutex mtx_q_joint_ma_info;
Eigen::Matrix<double, 14, 1> q_joint_ma = Eigen::Matrix<double, 14, 1>::Zero();

std::mutex mtx_hand_controller_info;
Eigen::Matrix<double, 2, 1> hand_controller_trigger = Eigen::Matrix<double, 2, 1>::Constant(0.5);
Eigen::Matrix<double, 2, 1> hand_controller_button = Eigen::Matrix<double, 2, 1>::Constant(0);

//XXX: CAUTION CHECK YOUR TRIGGER FIRMWARE
std::vector<Eigen::Matrix<double, 2, 1>> hand_controller_trigger_min_max = {Eigen::Matrix<double, 2, 1>({0, 1000}),
                                                                            Eigen::Matrix<double, 2, 1>({0, 1000})};
int gripper_direction = 0;
bool ma_info_verbose = false;

volatile std::sig_atomic_t g_stop_requested = 0;
void SignalHandler(int) { g_stop_requested = 1; }

std::vector<double> torque_constant = {1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
                                       1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043};

void SendVibration(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, int level) {

  if (id > 0x80) {
    packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_VIBRATION_LEVEL, level);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
}

std::optional<std::pair<int, std::pair<int, int>>> ReadButtonStatus(dynamixel::PortHandler* portHandler,
                                                                    dynamixel::PacketHandler* packetHandler, int id) {

  int32_t position = 0;
  uint8_t dxl_error = 0;
  int dxl_comm_result =
      packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_BUTTON_STATUS, (uint32_t*)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    int button = (position >> 8) & 0xff;
    int trigger = ((position >> 16) & 0xff) | (((position >> 24) & 0xff) << 8);
    return std::make_pair(id, std::make_pair(button, trigger));
  } else {
    return {};
  }
}

void TorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, int onoff) {
  packetHandler->write1ByteTxOnly(portHandler, id, ADDR_TORQUE_ENABLE, onoff);
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

std::optional<int> ReadTorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                                    int id) {
  int8_t onoff = -1;
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, (uint8_t*)&onoff, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
    return onoff;
  } else {
    return {};
  }
}

std::optional<double> ReadEncoder(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                                  int id) {
  int32_t position = 0;
  uint8_t dxl_error = 0;
  int dxl_comm_result =
      packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, (uint32_t*)&position, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
    return (double)position / 4096. * 2. * 3.141592;  // unit [rad]
  } else {
    return {};
  }
}

std::optional<std::vector<std::pair<int, double>>> BulkReadEncoder(dynamixel::PortHandler* portHandler,
                                                                   dynamixel::PacketHandler* packetHandler,
                                                                   std::vector<int> ids) {

  std::vector<std::pair<int, double>> position_vector;
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_PRESENT_POSITION, 4);
    }
  }

  groupBulkRead.txRxPacket();

  for (auto const& id : ids) {
    if (id < 0x80) {
      if (groupBulkRead.isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
        int position = groupBulkRead.getData(id, ADDR_PRESENT_POSITION, 4);
        position_vector.push_back(std::make_pair(id, (double)position / 4096. * 2. * 3.141592));
      }
    }
  }

  if (position_vector.size() == 0) {
    return {};
  } else {
    return position_vector;
  }
}

void SendGoalPosition(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id,
                      int goal_position) {
  packetHandler->write4ByteTxOnly(portHandler, id, ADDR_GOAL_POSITION, goal_position);
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

std::optional<int> ReadOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                                     int id) {
  int8_t operation_mode = -1;
  uint8_t dxl_error = 0;
  int dxl_comm_result =
      packetHandler->read1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, (uint8_t*)&operation_mode, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS) {
    return operation_mode;
  } else {
    return {};
  }
}

std::optional<std::vector<std::pair<int, int>>> BulkReadOperationMode(dynamixel::PortHandler* portHandler,
                                                                      dynamixel::PacketHandler* packetHandler,
                                                                      std::vector<int> ids) {

  std::vector<std::pair<int, int>> operation_mode_vector;
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_OPERATING_MODE, 1);
    }
  }

  groupBulkRead.txRxPacket();

  for (auto const& id : ids) {
    if (id < 0x80) {
      if (groupBulkRead.isAvailable(id, ADDR_OPERATING_MODE, 1)) {
        int operation_mode = groupBulkRead.getData(id, ADDR_OPERATING_MODE, 1);
        operation_mode_vector.push_back(std::make_pair(id, operation_mode));
      }
    }
  }

  if (operation_mode_vector.size() == 0) {
    return {};
  } else {
    return operation_mode_vector;
  }
}

std::optional<std::vector<std::pair<int, int>>> BulkReadTorqueEnable(dynamixel::PortHandler* portHandler,
                                                                     dynamixel::PacketHandler* packetHandler,
                                                                     std::vector<int> ids) {

  std::vector<std::pair<int, int>> torque_enable_vector;
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_TORQUE_ENABLE, 1);
    }
  }

  groupBulkRead.txRxPacket();

  for (auto const& id : ids) {
    if (id < 0x80) {
      if (groupBulkRead.isAvailable(id, ADDR_TORQUE_ENABLE, 1)) {
        int operation_mode = groupBulkRead.getData(id, ADDR_TORQUE_ENABLE, 1);
        torque_enable_vector.push_back(std::make_pair(id, operation_mode));
      }
    }
  }

  if (torque_enable_vector.size() == 0) {
    return {};
  } else {
    return torque_enable_vector;
  }
}

void BulkWriteTorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                           std::vector<std::pair<int, int>> id_and_enable_vector) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  uint8_t param[1];

  for (auto const& id_and_enable : id_and_enable_vector) {
    if (id_and_enable.first < 0x80) {
      param[0] = id_and_enable.second;
      groupBulkWrite.addParam(id_and_enable.first, ADDR_TORQUE_ENABLE, 1, param);
    }
  }

  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void BulkWriteTorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                           std::vector<int> ids, int enable) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  uint8_t param[1];

  for (auto const& id : ids) {
    if (id < 0x80) {
      param[0] = enable;
      groupBulkWrite.addParam(id, ADDR_TORQUE_ENABLE, 1, param);
    }
  }

  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void BulkWriteOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                            std::vector<std::pair<int, int>> id_and_mode_vector) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  uint8_t param[1];

  for (auto const& id_and_mode : id_and_mode_vector) {
    if (id_and_mode.first < 0x80) {
      param[0] = id_and_mode.second;
      groupBulkWrite.addParam(id_and_mode.first, ADDR_OPERATING_MODE, 1, param);
    }
  }

  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void BulkWriteSendTorque(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                         std::vector<std::pair<int, double>> id_and_torque_vector) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  uint16_t param[1];

  for (auto const& id_and_mode : id_and_torque_vector) {
    if (id_and_mode.first < 0x80) {
      param[0] = (int16_t)(id_and_mode.second / torque_constant[id_and_mode.first] * 1000. / 2.69);
      groupBulkWrite.addParam(id_and_mode.first, ADDR_GOAL_CURRENT, 2, reinterpret_cast<uint8_t*>(&param));
    }
  }

  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void SendOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id,
                       int operation_mode) {
  packetHandler->write1ByteTxOnly(portHandler, id, ADDR_OPERATING_MODE, operation_mode);
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void SendTorque(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id,
                double joint_torque) {
  int32_t torque_value = (int32_t)(joint_torque / torque_constant[id] * 1000. / 2.69);
  packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, torque_value);
}

void SendCurrent(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, double current) {
  //current unit is [A]
  int32_t current_value = (int)(current / 2.69 * 1000.);
  packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, current_value);
}

Eigen::Matrix<double, 14, 1> ComputeGravityTorque(std::shared_ptr<rb::dyn::Robot<14>> robot,
                                                  std::shared_ptr<State<14>> state,
                                                  Eigen::Matrix<double, 14, 1> q_joint) {

  state->SetQ(q_joint);
  robot->ComputeForwardKinematics(state);
  robot->ComputeDiffForwardKinematics(state);
  robot->Compute2ndDiffForwardKinematics(state);
  robot->ComputeInverseDynamics(state);
  return state->GetTau();  // / unit [Nm]
}

Eigen::Matrix<double, 14, 1> calc_torque_for_limit_avoid(Eigen::Matrix<double, 14, 1> q_joint) {

  Eigen::Matrix<double, 14, 1> torque_add;
  torque_add.setZero();

  int arm_dof = 7;

  int n_joint = 1;
  if (q_joint(n_joint) > -10. * D2R) {
    torque_add(n_joint) += (-10. * D2R - q_joint(n_joint)) * 4.;
  }

  n_joint = arm_dof + 1;
  if (q_joint(n_joint) < 10. * D2R) {
    torque_add(n_joint) += (10. * D2R - q_joint(n_joint)) * 4.;
  }

  n_joint = 2;
  if (q_joint(n_joint) > 90 * D2R) {
    torque_add(n_joint) += (90. * D2R - q_joint(n_joint)) * 0.5;
  }
  if (q_joint(n_joint) < 0) {
    torque_add(n_joint) += (0. - q_joint(n_joint)) * 0.5;
  }

  n_joint = arm_dof + 2;
  if (q_joint(n_joint) < -90 * D2R) {
    torque_add(n_joint) += (-90. * D2R - q_joint(n_joint)) * 0.5;
  }
  if (q_joint(n_joint) > 0) {
    torque_add(n_joint) += (0. - q_joint(n_joint)) * 0.5;
  }

  n_joint = 5;
  if (q_joint(n_joint) > 90 * D2R) {
    torque_add(n_joint) += (90. * D2R - q_joint(n_joint)) * 1.;
  }
  if (q_joint(n_joint) < 0. * D2R) {
    torque_add(n_joint) += (0. * D2R - q_joint(n_joint)) * 1.;
  }

  n_joint = arm_dof + 5;
  if (q_joint(n_joint) > 90 * D2R) {
    torque_add(n_joint) += (90. * D2R - q_joint(n_joint)) * 1.;
  }
  if (q_joint(n_joint) < 0. * D2R) {
    torque_add(n_joint) += (0. * D2R - q_joint(n_joint)) * 1.;
  }

  n_joint = 4;
  if (q_joint(n_joint) > 90 * D2R) {
    torque_add(n_joint) += (90. * D2R - q_joint(n_joint)) * 0.5;
  }
  if (q_joint(n_joint) < -90 * D2R) {
    torque_add(n_joint) += (-90. * D2R - q_joint(n_joint)) * 0.5;
  }

  n_joint = arm_dof + 4;
  if (q_joint(n_joint) > 90 * D2R) {
    torque_add(n_joint) += (90. * D2R - q_joint(n_joint)) * 0.5;
  }
  if (q_joint(n_joint) < -90 * D2R) {
    torque_add(n_joint) += (-90. * D2R - q_joint(n_joint)) * 0.5;
  }

  n_joint = 3;
  if (q_joint(n_joint) < -135. * D2R) {
    torque_add(n_joint) += (-135. * D2R - q_joint(n_joint)) * 6.;
  }

  if (q_joint(n_joint) > -20. * D2R) {
    torque_add(n_joint) += (-20. * D2R - q_joint(n_joint)) * 6.;
  }

  n_joint = arm_dof + 3;
  if (q_joint(n_joint) < -135. * D2R) {
    torque_add(n_joint) += (-135. * D2R - q_joint(n_joint)) * 6.;
  }

  if (q_joint(n_joint) > -20. * D2R) {
    torque_add(n_joint) += (-20. * D2R - q_joint(n_joint)) * 6.;
  }

  Eigen::Matrix<double, 14, 1> torque_add_limit;
  torque_add_limit.setConstant(300. / 1000.);

  torque_add_limit(5) = 500. / 1000.;
  torque_add_limit(arm_dof + 5) = 500 / 1000.;

  torque_add = torque_add.cwiseMin(torque_add_limit);
  torque_add = torque_add.cwiseMax(-torque_add_limit);

  for (int i = 0; i < 14; i++) {
    torque_add(i) *= torque_constant[i];
  }

  return torque_add;
}

void control_loop_for_leader_arm(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                                 std::vector<int> activeIDs) {

  auto robot = std::make_shared<rb::dyn::Robot<14>>(LoadRobotFromURDF(PATH "/leader_arm.urdf", "Base"));
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>(
      {"Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R", "Link_0L", "Link_1L",
       "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"},
      {"J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R", "J4_Wrist_Yaw1_R",
       "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R", "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L",
       "J10_Elbow_L", "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"});

  state->SetGravity({0, 0, 0, 0, 0, -9.81});

  Eigen::Matrix<double, 14, 1> q_joint, tau_joint;
  Eigen::Matrix<double, 14, 1> ma_torque_limit;
  Eigen::Matrix<int, 14, 1> operation_mode, torque_enable;
  std::vector<std::optional<std::pair<int, std::pair<int, int>>>> button_status_vector;
  q_joint.setZero();
  tau_joint.setZero();
  ma_torque_limit << 1.46, 1.46, 1.46, 1.46, 0.6, 0.6, 0.6, 1.46, 1.46, 1.46, 1.46, 0.6, 0.6, 0.6;

  Eigen::Matrix<double, 14, 1> temp_eigen;
  Eigen::Matrix<double, 2, 1> button_info, trigger_info;
  temp_eigen.setZero();
  button_info.setZero();
  trigger_info.setZero();

  Eigen::Matrix<double, 14, 1> ready_pose;
  ready_pose << 0.0, -5.0, 0.0, -20.0, 0.0, 20.0, 0.0, 0.0, 5.0, 0.0, -20.0, 0.0, 20.0, 0.0;
  ready_pose *= D2R;
  constexpr double kStartupAlignTimeout = 15.0;
  constexpr double kStartupAlignTolerance = 5.0 * D2R;
  constexpr double kStartupAlignCommandStep = 0.1 * D2R;
  bool startup_align_phase = true;
  bool startup_align_initialized = false;
  bool startup_align_failed = false;
  Eigen::Matrix<double, 14, 1> startup_align_q = ready_pose;
  const auto startup_align_start_time = std::chrono::steady_clock::now();
  std::cout << "Aligning leader arm to READY_POSE before teleoperation..." << std::endl;

  while (!g_stop_requested) {
    auto start = std::chrono::steady_clock::now();

    button_status_vector.clear();
    operation_mode.setConstant(-1);
    torque_enable.setZero();
    temp_eigen.setConstant(-1);

    for (int id : activeIDs) {
      if (id >= 0x80) {
        //for hand board
        std::optional<std::pair<int, std::pair<int, int>>> temp_button_status =
            ReadButtonStatus(portHandler, packetHandler, id);
        button_status_vector.push_back(temp_button_status);
      }
    }

    std::optional<std::vector<std::pair<int, double>>> temp_q_joint_vector =
        BulkReadEncoder(portHandler, packetHandler, activeIDs);
    if (temp_q_joint_vector.has_value()) {
      for (auto const& ret : temp_q_joint_vector.value()) {
        q_joint(ret.first) = ret.second;
      }
    }

    tau_joint = ComputeGravityTorque(robot, state, q_joint) * m_sf;

    Eigen::Matrix<double, 14, 1> add_torque = calc_torque_for_limit_avoid(q_joint);

    tau_joint = tau_joint + add_torque;
    tau_joint = tau_joint.cwiseMin(ma_torque_limit).cwiseMax(-ma_torque_limit);

    {
      std::lock_guard<std::mutex> lg(mtx_q_joint_ma_info);
      q_joint_ma = q_joint;
    }

    auto temp_operation_mode_vector = BulkReadOperationMode(portHandler, packetHandler, activeIDs);
    if (temp_operation_mode_vector.has_value()) {
      for (auto const& ret : temp_operation_mode_vector.value()) {
        operation_mode(ret.first) = ret.second;
      }
    }

    auto temp_torque_enable_vector = BulkReadTorqueEnable(portHandler, packetHandler, activeIDs);
    if (temp_torque_enable_vector.has_value()) {
      for (auto const& ret : temp_torque_enable_vector.value()) {
        torque_enable(ret.first) = ret.second;
      }
    }

    std::vector<std::pair<int, int>> id_and_enable_vector;
    for (auto const& id : activeIDs) {
      if (!torque_enable(id)) {
        id_and_enable_vector.push_back(std::make_pair(id, 1));
      }
    }

    BulkWriteTorqueEnable(portHandler, packetHandler, id_and_enable_vector);

    std::vector<std::pair<int, int>> id_and_mode_vector;
    std::vector<int> id_torque_onoff_vector;
    std::vector<std::pair<int, double>> id_send_torque_vector;

    if (startup_align_phase) {
      if (!startup_align_initialized) {
        startup_align_q = q_joint;
        startup_align_initialized = true;
      }

      for (int id = 0; id < 14; id++) {
        const double delta = std::clamp(ready_pose(id) - startup_align_q(id), -kStartupAlignCommandStep,
                                        kStartupAlignCommandStep);
        startup_align_q(id) += delta;
      }

      for (int id = 0; id < 14; id++) {
        if (operation_mode(id) != CURRENT_BASED_POSITION_CONTROL_MODE) {
          id_and_mode_vector.emplace_back(id, CURRENT_BASED_POSITION_CONTROL_MODE);
          id_torque_onoff_vector.push_back(id);
        }
      }

      BulkWriteTorqueEnable(portHandler, packetHandler, id_torque_onoff_vector, 0);
      BulkWriteOperationMode(portHandler, packetHandler, id_and_mode_vector);
      BulkWriteTorqueEnable(portHandler, packetHandler, id_torque_onoff_vector, 1);

      for (int id = 0; id < 14; id++) {
        SendGoalPosition(portHandler, packetHandler, id, static_cast<int>(startup_align_q(id) * 4096. / 3.141592 / 2.));
      }

      const double right_error = (q_joint.block(0, 0, 7, 1) - ready_pose.block(0, 0, 7, 1)).cwiseAbs().maxCoeff();
      const double left_error = (q_joint.block(7, 0, 7, 1) - ready_pose.block(7, 0, 7, 1)).cwiseAbs().maxCoeff();
      if (std::max(right_error, left_error) <= kStartupAlignTolerance) {
        startup_align_phase = false;
        std::cout << "Leader arm aligned to READY_POSE. Teleoperation unlocked." << std::endl;
      } else {
        const double elapsed_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - startup_align_start_time).count();
        if (elapsed_sec > kStartupAlignTimeout) {
          startup_align_failed = true;
          std::cerr << "Failed to align leader arm to READY_POSE within startup timeout." << std::endl;
          g_stop_requested = 1;
        }
      }

      if (startup_align_phase && !startup_align_failed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
    }

    for (auto& button_status : button_status_vector) {

      if (button_status.has_value()) {
        int id_hand_controlelr = button_status.value().first;
        std::pair<int, int> temp_button_status = button_status.value().second;
        int button = temp_button_status.first;
        int trigger = temp_button_status.second;
        trigger_info(id_hand_controlelr - 0x80) = trigger;
        button_info(id_hand_controlelr - 0x80) = button;

        {
          std::lock_guard<std::mutex> lg(mtx_hand_controller_info);

          hand_controller_trigger(id_hand_controlelr - 0x80) =
              (trigger - hand_controller_trigger_min_max[id_hand_controlelr - 0x80](MIN_INDEX)) /
              (hand_controller_trigger_min_max[id_hand_controlelr - 0x80](MAX_INDEX) -
               hand_controller_trigger_min_max[id_hand_controlelr - 0x80](MIN_INDEX));

          hand_controller_button(id_hand_controlelr - 0x80) = button;
        }

        if (id_hand_controlelr == 0x80) {
          // right arm
          for (int id = 0; id < 7; id++) {

            if (button == 0) {
              // position control
              if (operation_mode(id) != CURRENT_BASED_POSITION_CONTROL_MODE) {
                id_and_mode_vector.push_back(std::make_pair(id, CURRENT_BASED_POSITION_CONTROL_MODE));
                id_torque_onoff_vector.push_back(id);
              }
            } else if (button == 1) {
              // current control
              if (operation_mode(id) != CURRENT_CONTROL_MODE) {
                id_and_mode_vector.push_back(std::make_pair(id, CURRENT_CONTROL_MODE));
                id_torque_onoff_vector.push_back(id);
              } else {
                id_send_torque_vector.push_back(std::make_pair(id, tau_joint(id)));
              }
            }
          }
        }

        if (id_hand_controlelr == 0x81) {
          //left arm
          for (int id = 7; id < 7 + 7; id++) {

            if (button == 0) {
              // position control
              if (operation_mode(id) != CURRENT_BASED_POSITION_CONTROL_MODE) {
                id_and_mode_vector.push_back(std::make_pair(id, CURRENT_BASED_POSITION_CONTROL_MODE));
                id_torque_onoff_vector.push_back(id);
              }
            } else if (button == 1) {
              // current control
              if (operation_mode(id) != CURRENT_CONTROL_MODE) {
                id_and_mode_vector.push_back(std::make_pair(id, CURRENT_CONTROL_MODE));
                id_torque_onoff_vector.push_back(id);
              } else {
                id_send_torque_vector.push_back(std::make_pair(id, tau_joint(id)));
              }
            }
          }
        }
      }
    }

    BulkWriteTorqueEnable(portHandler, packetHandler, id_torque_onoff_vector, 0);
    BulkWriteOperationMode(portHandler, packetHandler, id_and_mode_vector);
    BulkWriteTorqueEnable(portHandler, packetHandler, id_torque_onoff_vector, 1);

    BulkWriteSendTorque(portHandler, packetHandler, id_send_torque_vector);

    static int cnt = 0;
    if (ma_info_verbose) {
      if (cnt++ % 5 == 0) {

        std::cout << "button_info : " << button_info.transpose() << std::endl;
        std::cout << "trigger_info : " << trigger_info.transpose() << std::endl;
        std::cout << "right q_joint [deg]: " << q_joint.block(0, 0, 7, 1).transpose() * 180. / 3.141592 << std::endl;
        std::cout << "left q_joint [deg]: " << q_joint.block(7, 0, 7, 1).transpose() * 180. / 3.141592 << std::endl;

        std::cout << "Duration: "
                  << (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
                                                                                  start)
                             .count() /
                         1.e6
                  << " ms" << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void control_loop_for_gripper(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                              std::vector<int> activeIDs) {

  std::vector<Eigen::Matrix<double, 2, 1>> q_min_max_vector;
  q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());
  q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());

  while (!g_stop_requested) {
    static int cnt = 0;
    int is_init = true;

    if (activeIDs.size() != 2) {
      std::cout << "The number of Dynamixels for hand gripper does not match the configuration\n";
      return;
    }

    // total moving angle 540 deg

    for (auto const& id : activeIDs) {
      while (!g_stop_requested) {

        std::optional<int> operation_mode = ReadOperationMode(portHandler, packetHandler, id);

        if (operation_mode.has_value()) {
          if (operation_mode.value() != CURRENT_CONTROL_MODE) {
            TorqueEnable(portHandler, packetHandler, id, 0);
            SendOperationMode(portHandler, packetHandler, id, CURRENT_CONTROL_MODE);
            TorqueEnable(portHandler, packetHandler, id, 1);
            std::cout << "try to change control mode, id : " << id << std::endl;
          } else {
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      while (!g_stop_requested) {

        std::optional<int> torque_enable = ReadTorqueEnable(portHandler, packetHandler, id);

        if (torque_enable.has_value()) {
          if (!torque_enable.value()) {
            TorqueEnable(portHandler, packetHandler, id, 1);
            std::cout << "try to enable torque, id : " << id << std::endl;
          } else {
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      if (cnt % 2 == 0) {
        std::optional<double> q = ReadEncoder(portHandler, packetHandler, id);
        if (q.has_value()) {
          q_min_max_vector[id](cnt % 2) = q.value();
          std::cout<<"q.has value() 0 "<<std::endl;
        }
        SendCurrent(portHandler, packetHandler, id, 0.5);
      } else {
        std::optional<double> q = ReadEncoder(portHandler, packetHandler, id);
        if (q.has_value()) {
          q_min_max_vector[id](cnt % 2) = q.value();
          std::cout<<"q.has value() 1 "<<std::endl;
        }
        SendCurrent(portHandler, packetHandler, id, -0.5);
      }

      // 이전 그리퍼 540
      // 고속 그리퍼 229.183
      if ((double)(abs(q_min_max_vector[id](MAX_INDEX) - q_min_max_vector[id](MIN_INDEX))) * 180 / 3.141592 <
          230 * 0.9) {
        is_init = false;
        std::cout<< "id : "<< id << std::endl;
        std::cout<< "vlaue : "<< (double)(abs(q_min_max_vector[id](MAX_INDEX) - q_min_max_vector[id](MIN_INDEX))) * 180 / 3.141592 <<std::endl;
        std::cout<< "is_init false"<<std::endl;
      }
    }

    if (is_init) {
      std::cout<< "is_init true"<<std::endl;
      for (auto const& id : activeIDs) {
        if (q_min_max_vector[id](MIN_INDEX) > q_min_max_vector[id](MAX_INDEX)) {
          double temp = q_min_max_vector[id](MIN_INDEX);
          q_min_max_vector[id](MIN_INDEX) = q_min_max_vector[id](MAX_INDEX);
          q_min_max_vector[id](MAX_INDEX) = temp;
        }

        SendCurrent(portHandler, packetHandler, id, 0.5);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      break;
    }

    cnt++;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  std::cout << "finish init\n";

  while (!g_stop_requested) {

    for (auto const& id : activeIDs) {
      std::optional<int> operation_mode = ReadOperationMode(portHandler, packetHandler, id);
      if (operation_mode.has_value()) {
        if (operation_mode.value() != CURRENT_BASED_POSITION_CONTROL_MODE) {
          TorqueEnable(portHandler, packetHandler, id, 0);
          SendOperationMode(portHandler, packetHandler, id, CURRENT_BASED_POSITION_CONTROL_MODE);
          TorqueEnable(portHandler, packetHandler, id, 1);
        } else {
          double goal_position = 0;

          {
            std::lock_guard<std::mutex> lg(mtx_hand_controller_info);

            static Eigen::Vector<double, 2> temp_hand_controller_trigger = Eigen::Matrix<double, 2, 1>::Constant(0.5);

            temp_hand_controller_trigger(id) =
                temp_hand_controller_trigger(id) * 0.9 + hand_controller_trigger(id) * 0.1;

            temp_hand_controller_trigger = (temp_hand_controller_trigger.array().max(0.).min(1.)).matrix();

            hand_controller_trigger(id) = (double)((int)(temp_hand_controller_trigger(id) * 100)) / 100.;

            if (gripper_direction) {
              goal_position = hand_controller_trigger(id) * q_min_max_vector[id](MAX_INDEX) +
                              (1. - hand_controller_trigger(id)) * q_min_max_vector[id](MIN_INDEX);

            } else {
              goal_position = hand_controller_trigger(id) * q_min_max_vector[id](MIN_INDEX) +
                              (1. - hand_controller_trigger(id)) * q_min_max_vector[id](MAX_INDEX);
            }
          }

          SendGoalPosition(portHandler, packetHandler, id, (int)(goal_position * 4096. / 3.141592 / 2.));
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template <typename ModelT>
int RunTeleoperation(const std::string& address, const std::string& power, const std::string& servo,
                     const std::string& mode) {
  const std::string mode_lower = ToLower(mode);
  if (mode_lower != "position" && mode_lower != "impedance") {
    std::cerr << "Unsupported mode: " << mode << " (supported: position, impedance)" << std::endl;
    return 1;
  }
  const bool position_mode = (mode_lower == "position");

  std::signal(SIGINT, SignalHandler);

  try {
    // Latency timer setting
    upc::InitializeDevice(upc::kGripperDeviceName);
    upc::InitializeDevice(upc::kLeaderArmDeviceName);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  auto robot = rb::Robot<ModelT>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Starting state update..." << std::endl;

  robot->StartStateUpdate(
      [](const auto& state) {
        std::cout << "State Update Received:" << std::endl;
        std::cout << "  Timestamp: " << state.timestamp.tv_sec << ".";
        std::cout << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
        if constexpr (std::is_same_v<ModelT, y1_model::A>) {
          std::cout << "  waist [deg]     : " << state.position.block(2, 0, 6, 1).transpose() * R2D << std::endl;
          std::cout << "  right arm [deg] : " << state.position.block(2 + 6, 0, 7, 1).transpose() * R2D << std::endl;
          std::cout << "  left arm [deg]  : " << state.position.block(2 + 6 + 7, 0, 7, 1).transpose() * R2D << std::endl;
        } else {
          std::cout << "  waist [deg]     : " << state.position.block(4, 0, 6, 1).transpose() * R2D << std::endl;
          std::cout << "  right arm [deg] : " << state.position.block(4 + 6, 0, 7, 1).transpose() * R2D << std::endl;
          std::cout << "  left arm [deg]  : " << state.position.block(4 + 6 + 7, 0, 7, 1).transpose() * R2D << std::endl;
        }
      },
      0.1 /* Hz */);

  std::cout << "!!\n";

  std::this_thread::sleep_for(1s);

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn(power)) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(power)) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  std::cout << "Checking servo status..." << std::endl;
  if (!robot->IsServoOn(servo)) {
    std::cout << "Servo is currently OFF. Attempting to activate servo..." << std::endl;
    if (!robot->ServoOn(servo)) {
      std::cerr << "Error: Failed to activate servo." << std::endl;
      return 1;
    }
    std::cout << "Servo activated successfully." << std::endl;
  } else {
    std::cout << "Servo is already ON." << std::endl;
  }

  const auto& control_manager_state = robot->GetControlManagerState();
  if (control_manager_state.state == ControlManagerState::State::kMajorFault ||
      control_manager_state.state == ControlManagerState::State::kMinorFault) {
    std::cerr << "Warning: Detected a "
              << (control_manager_state.state == ControlManagerState::State::kMajorFault ? "Major" : "Minor")
              << " Fault in the Control Manager." << std::endl;

    std::cout << "Attempting to reset the fault..." << std::endl;
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
      return 1;
    }
    std::cout << "Fault reset successfully." << std::endl;
  }
  std::cout << "Control Manager state is normal. No faults detected." << std::endl;

  std::cout << "Enabling the Control Manager..." << std::endl;
  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    return 1;
  }
  std::cout << "Control Manager enabled successfully." << std::endl;

  if (robot->IsPowerOn("48v")) {
    robot->SetToolFlangeOutputVoltage("right", 12);
    robot->SetToolFlangeOutputVoltage("left", 12);
    std::cout << "Attempting to 12V power on for gripper." << std::endl;
    std::this_thread::sleep_for(1s);
  }

  robot->SetParameter("joint_position_command.cutoff_frequency", "10.0");
  std::cout << robot->GetParameter("joint_position_command.cutoff_frequency") << std::endl;

  const char* devicename_leader_arm = "/dev/rby1_leader_arm";

  dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(devicename_leader_arm);
  dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    std::cerr << "Failed to open the port!" << std::endl;
    return 1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    std::cerr << "Failed to change the baudrate!" << std::endl;
    return 1;
  }

  std::vector<int> activeIDs;

  for (int id = 0; id < 14; ++id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->ping(portHandler, id, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      activeIDs.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }

  for (int id = 0x80; id < 0x80 + 2; id++) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->ping(portHandler, id, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      activeIDs.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }

  if (activeIDs.size() != 16) {
    std::cerr << "Unable to ping all devices for leader arm" << std::endl;
    Eigen::Map<Eigen::VectorXi> ids(activeIDs.data(), activeIDs.size());
    std::cerr << "active ids: " << ids.transpose() << std::endl;
    return 1;
  }

  for (int id : activeIDs) {
    if (id < 0x80) {
      int dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
      if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to write current control mode value: " << packetHandler->getTxRxResult(dxl_comm_result)
                  << std::endl;
        return 1;
      }

      TorqueEnable(portHandler, packetHandler, id, 1);
    }
  }

  std::thread leader_arm_handler(control_loop_for_leader_arm, portHandler, packetHandler, activeIDs);

  auto shutdown_after_leader_started = [&](const std::string& reason) -> int {
    std::cerr << reason << std::endl;
    g_stop_requested = 1;
    if (leader_arm_handler.joinable()) {
      leader_arm_handler.join();
    }
    portHandler->closePort();
    robot->CancelControl();
    robot->DisableControlManager();
    robot->StopStateUpdate();
    return 1;
  };

  const char* devicename_gripper = "/dev/rby1_gripper";

  dynamixel::PortHandler* portHandler_gripper = dynamixel::PortHandler::getPortHandler(devicename_gripper);
  dynamixel::PacketHandler* packetHandler_gripper = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_gripper->openPort()) {
    return shutdown_after_leader_started("Failed to open the gripper port!");
  }

  if (!portHandler_gripper->setBaudRate(BAUDRATE)) {
    portHandler_gripper->closePort();
    return shutdown_after_leader_started("Failed to change the gripper baudrate!");
  }

  std::vector<int> activeIDs_gripper;

  for (int id = 0; id < 2; ++id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_gripper->ping(portHandler_gripper, id, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      activeIDs_gripper.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }

  if (activeIDs_gripper.size() != 2) {
    std::cerr << "Unable to ping all devices for grippers" << std::endl;
    Eigen::Map<Eigen::VectorXi> ids(activeIDs_gripper.data(), activeIDs_gripper.size());
    std::cerr << "active ids: " << ids.transpose() << std::endl;
    portHandler_gripper->closePort();
    return shutdown_after_leader_started("Gripper initialization failed.");
  }

  for (int id : activeIDs_gripper) {
    int dxl_comm_result =
        packetHandler_gripper->write1ByteTxOnly(portHandler_gripper, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
    if (dxl_comm_result != COMM_SUCCESS) {
      std::cerr << "Failed to write current control mode value: "
                << packetHandler_gripper->getTxRxResult(dxl_comm_result) << std::endl;
      portHandler_gripper->closePort();
      return shutdown_after_leader_started("Failed to configure gripper control mode.");
    }

    TorqueEnable(portHandler_gripper, packetHandler_gripper, id, 1);
  }

  std::thread gripper_handler(control_loop_for_gripper, portHandler_gripper, packetHandler_gripper, activeIDs_gripper);

  if (1) {
    const Eigen::Vector<double, 6> q_torso = Eigen::Vector<double, 6>{0, 45, -90, 45, 0, 0} * D2R;
    const Eigen::Vector<double, 7> q_right = Eigen::Vector<double, 7>{0, -5, 0, -20, 0, 20, 0} * D2R;
    const Eigen::Vector<double, 7> q_left = Eigen::Vector<double, 7>{0, 5, 0, -20, 0, 20, 0} * D2R;

    BodyComponentBasedCommandBuilder ready_body_builder;
    ready_body_builder.SetTorsoCommand(JointPositionCommandBuilder()
                                           .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                                           .SetMinimumTime(5)
                                           .SetPosition(q_torso));
    if (position_mode) {
      ready_body_builder.SetRightArmCommand(JointPositionCommandBuilder()
                                                .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                                                .SetMinimumTime(5)
                                                .SetPosition(q_right));
      ready_body_builder.SetLeftArmCommand(JointPositionCommandBuilder()
                                               .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                                               .SetMinimumTime(5)
                                               .SetPosition(q_left));
    } else {
      ready_body_builder.SetRightArmCommand(JointImpedanceControlCommandBuilder()
                                                .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                                                .SetMinimumTime(5)
                                                .SetPosition(q_right)
                                                .SetStiffness(Eigen::VectorXd::Constant(7, kImpedanceStiffness))
                                                .SetDampingRatio(kImpedanceDampingRatio)
                                                .SetTorqueLimit(Eigen::VectorXd::Constant(7, kImpedanceTorqueLimit)));
      ready_body_builder.SetLeftArmCommand(JointImpedanceControlCommandBuilder()
                                               .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.))
                                               .SetMinimumTime(5)
                                               .SetPosition(q_left)
                                               .SetStiffness(Eigen::VectorXd::Constant(7, kImpedanceStiffness))
                                               .SetDampingRatio(kImpedanceDampingRatio)
                                               .SetTorqueLimit(Eigen::VectorXd::Constant(7, kImpedanceTorqueLimit)));
    }

    robot->SendCommand(
             RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(ready_body_builder)))
        ->Get();

    Eigen::Matrix<double, 14, 1> q_joint_ref;
    q_joint_ref.setZero();

    {
      std::lock_guard<std::mutex> lg(mtx_q_joint_ma_info);
      q_joint_ref = q_joint_ma;
    }

    auto dyn = robot->GetDynamics();
    std::vector<std::string> joint_names;
    if constexpr (std::is_same_v<ModelT, y1_model::A>) {
      joint_names = {y1_model::A::kRobotJointNames.begin(), y1_model::A::kRobotJointNames.end()};
    } else {
      joint_names = {y1_model::M::kRobotJointNames.begin(), y1_model::M::kRobotJointNames.end()};
    }
    auto dyn_state = dyn->MakeState({"base"}, joint_names);

    Eigen::Vector<double, 14> q_lower_limit, q_upper_limit;
    if constexpr (std::is_same_v<ModelT, y1_model::A>) {
      q_upper_limit = dyn->GetLimitQUpper(dyn_state).block(2 + 6, 0, 14, 1);
      q_lower_limit = dyn->GetLimitQLower(dyn_state).block(2 + 6, 0, 14, 1);
    } else {
      q_upper_limit = dyn->GetLimitQUpper(dyn_state).block(4 + 6, 0, 14, 1);
      q_lower_limit = dyn->GetLimitQLower(dyn_state).block(4 + 6, 0, 14, 1);
    }

    auto stream = robot->CreateCommandStream();

    std::this_thread::sleep_for(5s);

    double right_arm_minimum_time = 1.;
    double left_arm_minimum_time = 1.;
    double lpf_update_ratio = 0.1;

    std::cout << "Start to send command to robot." << std::endl;

    Eigen::Matrix<double, 20, 1> q_joint_ref_20x1;
    q_joint_ref_20x1.setZero();
    q_joint_ref_20x1.block(0, 0, 6, 1) << 0, 30, -60, 30, 0, 0;
    q_joint_ref_20x1 *= D2R;

    while (!g_stop_requested) {
      {
        std::lock_guard<std::mutex> lg(mtx_q_joint_ma_info);
        {
          std::lock_guard<std::mutex> lg(mtx_hand_controller_info);

          q_joint_ref_20x1.block(6, 0, 14, 1) =
              q_joint_ref_20x1.block(6, 0, 14, 1) * (1 - lpf_update_ratio) + q_joint_ma * lpf_update_ratio;

          if constexpr (std::is_same_v<ModelT, y1_model::A>) {
            Eigen::Matrix<double, 24, 1> q_joint_rby1;
            q_joint_rby1.setZero();
            q_joint_rby1.block(2, 0, 20, 1) = q_joint_ref_20x1;
            dyn_state->SetQ(q_joint_rby1);
          } else {
            Eigen::Matrix<double, 26, 1> q_joint_rby1;
            q_joint_rby1.setZero();
            q_joint_rby1.block(4, 0, 20, 1) = q_joint_ref_20x1;
            dyn_state->SetQ(q_joint_rby1);
          }
          
          dyn->ComputeForwardKinematics(dyn_state);
          auto res_col = dyn->DetectCollisionsOrNearestLinks(dyn_state, 1);
          bool is_coliision = false;

          if (res_col[0].distance < 0.02) {
            std::cout<<"is collision ture"<<std::endl;
            is_coliision = true;
          }

          if (hand_controller_button(0) && !is_coliision) {
            //right hand position control mode
            q_joint_ref.block(0, 0, 7, 1) = q_joint_ref.block(0, 0, 7, 1) * (1 - lpf_update_ratio) +
                                            q_joint_ma.block(0, 0, 7, 1) * lpf_update_ratio;
          } else {
            right_arm_minimum_time = 1.0;
          }

          if (hand_controller_button(1) && !is_coliision) {
            //left hand position control mode
            q_joint_ref.block(7, 0, 7, 1) = q_joint_ref.block(7, 0, 7, 1) * (1 - lpf_update_ratio) +
                                            q_joint_ma.block(7, 0, 7, 1) * lpf_update_ratio;
          } else {
            left_arm_minimum_time = 1.0;
          }
        }
      }

      for (int i = 0; i < 14; i++) {
        q_joint_ref(i) = std::clamp(q_joint_ref(i), q_lower_limit(i), q_upper_limit(i));
      }
      Eigen::Vector<double, 7> target_position_left = q_joint_ref.block(7, 0, 7, 1);
      Eigen::Vector<double, 7> target_position_right = q_joint_ref.block(0, 0, 7, 1);
      Eigen::Vector<double, 7> acc_limit, vel_limit;

      acc_limit.setConstant(1200.0);
      acc_limit *= D2R;

      vel_limit << 160, 160, 160, 160, 330, 330, 330;
      vel_limit *= D2R;

      RobotCommandBuilder command_builder;

      right_arm_minimum_time *= 0.99;
      right_arm_minimum_time = std::max(right_arm_minimum_time, 0.01);

      left_arm_minimum_time *= 0.99;
      left_arm_minimum_time = std::max(left_arm_minimum_time, 0.01);

      BodyComponentBasedCommandBuilder body_builder;
      if (position_mode) {
        body_builder.SetLeftArmCommand(JointPositionCommandBuilder()
                                           .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                           .SetMinimumTime(left_arm_minimum_time)
                                           .SetPosition(target_position_left)
                                           .SetVelocityLimit(vel_limit)
                                           .SetAccelerationLimit(acc_limit));
        body_builder.SetRightArmCommand(JointPositionCommandBuilder()
                                            .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                            .SetMinimumTime(right_arm_minimum_time)
                                            .SetPosition(target_position_right)
                                            .SetVelocityLimit(vel_limit)
                                            .SetAccelerationLimit(acc_limit));
      } else {
        body_builder.SetLeftArmCommand(JointImpedanceControlCommandBuilder()
                                           .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                           .SetMinimumTime(left_arm_minimum_time)
                                           .SetPosition(target_position_left)
                                           .SetVelocityLimit(vel_limit)
                                           .SetAccelerationLimit(acc_limit)
                                           .SetStiffness(Eigen::VectorXd::Constant(7, kImpedanceStiffness))
                                           .SetDampingRatio(kImpedanceDampingRatio)
                                           .SetTorqueLimit(Eigen::VectorXd::Constant(7, kImpedanceTorqueLimit)));
        body_builder.SetRightArmCommand(JointImpedanceControlCommandBuilder()
                                            .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(4.))
                                            .SetMinimumTime(right_arm_minimum_time)
                                            .SetPosition(target_position_right)
                                            .SetVelocityLimit(vel_limit)
                                            .SetAccelerationLimit(acc_limit)
                                            .SetStiffness(Eigen::VectorXd::Constant(7, kImpedanceStiffness))
                                            .SetDampingRatio(kImpedanceDampingRatio)
                                            .SetTorqueLimit(Eigen::VectorXd::Constant(7, kImpedanceTorqueLimit)));
      }
      command_builder.SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(body_builder));

      stream->SendCommand(command_builder);

      std::this_thread::sleep_for(5ms);
    }

    // Stop command streaming first so Control Manager can be disabled cleanly.
    stream->Cancel();
    stream.reset();
  }

  g_stop_requested = 1;
  robot->CancelControl();

  leader_arm_handler.join();
  gripper_handler.join();
  portHandler->closePort();
  portHandler_gripper->closePort();

  bool disabled = false;
  for (int i = 0; i < 20; ++i) {
    if (robot->DisableControlManager()) {
      disabled = true;
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  if (!disabled) {
    std::cerr << "Warning: Failed to disable Control Manager during shutdown." << std::endl;
  }

  robot->StopStateUpdate();

  if (disabled) {
    robot->PowerOff("12v");
  }
  
  return 0;
}

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = kAll;
  std::string servo = kAll;
  std::string mode = "position";

  if (argc >= 2 && std::string(argv[1]).rfind("--", 0) != 0) {
    address = argv[1];
    if (argc >= 3) model = argv[2];
    if (argc >= 4) power = argv[3];
    if (argc >= 5) servo = argv[4];
    if (argc >= 6) mode = argv[5];
    if (argc > 6) {
      PrintUsage(argv[0]);
      return 1;
    }
  } else {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        PrintUsage(argv[0]);
        return 0;
      }
      if (arg == "--address" && i + 1 < argc) {
        address = argv[++i];
      } else if (arg == "--model" && i + 1 < argc) {
        model = argv[++i];
      } else if (arg == "--power" && i + 1 < argc) {
        power = argv[++i];
      } else if (arg == "--servo" && i + 1 < argc) {
        servo = argv[++i];
      } else if (arg == "--mode" && i + 1 < argc) {
        mode = argv[++i];
      } else {
        std::cerr << "Unknown or incomplete argument: " << arg << std::endl;
        PrintUsage(argv[0]);
        return 1;
      }
    }
  }

  if (address.empty()) {
    std::cerr << "--address is required." << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);
  mode = ToLower(mode);

  if (model == "a") return RunTeleoperation<y1_model::A>(address, power, servo, mode);
  if (model == "m") return RunTeleoperation<y1_model::M>(address, power, servo, mode);

  std::cerr << "Unsupported model: " << model << " (supported: a, m)" << std::endl;
  return 1;
}
