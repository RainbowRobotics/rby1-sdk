#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include "dynamixel_sdk.h"  // Uses Dynamixel SDK library
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/dynamics/state.h"

// Protocol version
#define PROTOCOL_VERSION 2.0  // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE 2000000
#define DEVICENAME \
  "/dev/ttyUSB0"  // Check which port is being used on your controller \
                  // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define DEVICENAME_HAND \
  "/dev/ttyUSB1"  // Check which port is being used on your controller \
                  // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

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

double m_sf = 0.4;

using namespace rb::dyn;
using namespace rb;

#define MIN_INDEX 0
#define MAX_INDEX 1

std::mutex gripper_mtx;
Eigen::Matrix<double, 2, 1> gripper_reference = Eigen::Matrix<double, 2, 1>::Constant(0.5);
std::vector<Eigen::Matrix<double, 2, 1>> gripper_reference_min_max = {Eigen::Matrix<double, 2, 1>({1050, 1150}),
                                                                      Eigen::Matrix<double, 2, 1>({950, 1250})};

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

  // Add parameters for bulk read

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_PRESENT_POSITION, 4);
    }
  }
  // Execute bulk read
  groupBulkRead.txRxPacket();

  // Print results
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

  // Add parameters for bulk read

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_OPERATING_MODE, 1);
    }
  }
  // Execute bulk read
  groupBulkRead.txRxPacket();

  // Print results
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

  // Add parameters for bulk read

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_TORQUE_ENABLE, 1);
    }
  }
  // Execute bulk read
  groupBulkRead.txRxPacket();

  // Print results
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

  // Add parameters for bulk read

  uint8_t param[1];

  for (auto const& id_and_enable : id_and_enable_vector) {
    if (id_and_enable.first < 0x80) {
      param[0] = id_and_enable.second;
      groupBulkWrite.addParam(id_and_enable.first, ADDR_TORQUE_ENABLE, 1, param);
    }
  }
  // Execute bulk read
  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void BulkWriteTorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                           std::vector<int> ids, int enable) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Add parameters for bulk read

  uint8_t param[1];

  for (auto const& id : ids) {
    if (id < 0x80) {
      param[0] = enable;
      groupBulkWrite.addParam(id, ADDR_TORQUE_ENABLE, 1, param);
    }
  }
  // Execute bulk read
  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void BulkWriteOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                            std::vector<std::pair<int, int>> id_and_mode_vector) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Add parameters for bulk read

  uint8_t param[1];

  for (auto const& id_and_mode : id_and_mode_vector) {
    if (id_and_mode.first < 0x80) {
      param[0] = id_and_mode.second;
      groupBulkWrite.addParam(id_and_mode.first, ADDR_OPERATING_MODE, 1, param);
    }
  }
  // Execute bulk read
  groupBulkWrite.txPacket();
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void BulkWriteSendTorque(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                         std::vector<std::pair<int, double>> id_and_torque_vector) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Add parameters for bulk read

  uint16_t param[1];

  for (auto const& id_and_mode : id_and_torque_vector) {
    if (id_and_mode.first < 0x80) {
      param[0] = (int16_t)(id_and_mode.second / torque_constant[id_and_mode.first] * 1000. / 2.69 * m_sf);
      groupBulkWrite.addParam(id_and_mode.first, ADDR_GOAL_CURRENT, 2, reinterpret_cast<uint8_t*>(&param));
    }
  }
  // Execute bulk read
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
  int32_t torque_value = (int32_t)(joint_torque / torque_constant[id] * 1000. / 2.69 * m_sf);
  packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, torque_value);
}

void SendCurrent(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, double current) {
  //current unit is [A]
  int32_t current_value = (int)(current / 2.69 * 1000.);
  packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, current_value);
}

Eigen::Matrix<double, 14, 1> ComputeGravityTorque(std::shared_ptr<Robot<14>> robot, std::shared_ptr<State<14>> state,
                                                  Eigen::Matrix<double, 14, 1> q_joint) {

  state->SetQ(q_joint);
  robot->ComputeForwardKinematics(state);
  robot->ComputeDiffForwardKinematics(state);
  robot->Compute2ndDiffForwardKinematics(state);
  robot->ComputeInverseDynamics(state);
  return state->GetTau();  // / unit [Nm]
}

void control_loop(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                  std::vector<int> activeIDs) {

  auto robot = std::make_shared<Robot<14>>(LoadRobotFromURDF(PATH "/master_arm.urdf", "Base"));
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>(
      {"Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R", "Link_0L", "Link_1L",
       "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"},
      {"J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R", "J4_Wrist_Yaw1_R",
       "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R", "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L",
       "J10_Elbow_L", "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"});

  state->SetGravity({0, 0, 0, 0, 0, -9.81});

  Eigen::Matrix<double, 14, 1> q_joint, tau_joint;
  Eigen::Matrix<int, 14, 1> operation_mode, torque_enable;
  std::vector<std::optional<std::pair<int, std::pair<int, int>>>> button_status_vector;
  q_joint.setZero();
  tau_joint.setZero();

  Eigen::Matrix<double, 14, 1> temp_eigen;
  Eigen::Matrix<double, 2, 1> button_info, trigger_info;
  temp_eigen.setZero();
  button_info.setZero();
  trigger_info.setZero();

  while (true) {
    auto start = std::chrono::steady_clock::now();

    if (activeIDs.size() != 16) {
      std::cout << "The number of Dynamixels and Hand Controller does not match the configuration\n";
      return;
    }

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

    tau_joint = ComputeGravityTorque(robot, state, q_joint);

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

    for (auto& button_status : button_status_vector) {

      if (button_status.has_value()) {
        int id_hand_controlelr = button_status.value().first;
        std::pair<int, int> temp_button_status = button_status.value().second;
        int button = temp_button_status.first;
        int trigger = temp_button_status.second;
        trigger_info(id_hand_controlelr - 0x80) = trigger;
        button_info(id_hand_controlelr - 0x80) = button;

        {
          std::lock_guard<std::mutex> lg(gripper_mtx);

          // if (trigger < gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX) ||
          //     gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX) < 0) {
          //   gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX) = trigger;
          // }

          // if (trigger > gripper_reference_min_max[id_hand_controlelr - 0x80](MAX_INDEX) ||
          //     gripper_reference_min_max[id_hand_controlelr - 0x80](MAX_INDEX) < 0) {
          //   gripper_reference_min_max[id_hand_controlelr - 0x80](MAX_INDEX) = trigger;
          // }

          // if (abs(gripper_reference_min_max[id_hand_controlelr - 0x80](MAX_INDEX) -
          //         gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX)) > 1) {
          //   gripper_reference(id_hand_controlelr - 0x80) =
          //       (trigger - gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX)) /
          //       (gripper_reference_min_max[id_hand_controlelr - 0x80](MAX_INDEX) -
          //        gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX));
          // } else {
          //   gripper_reference(id_hand_controlelr - 0x80) = 0.5;
          // }

          gripper_reference(id_hand_controlelr - 0x80) =
              (trigger - gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX)) /
              (gripper_reference_min_max[id_hand_controlelr - 0x80](MAX_INDEX) -
               gripper_reference_min_max[id_hand_controlelr - 0x80](MIN_INDEX));
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
    // if (cnt < 50) {
    //   SendVibration(portHandler, packetHandler, 0x80, 0);
    //   SendVibration(portHandler, packetHandler, 0x81, 0);
    // } else {
    //   // SendVibration(portHandler, packetHandler, 0x80, 10);
    //   // SendVibration(portHandler, packetHandler, 0x81, 10);
    // }

    if (1) {
      // std::cout << "cnt : " << cnt << std::endl;
      // std::cout << "button_info : " << button_info.transpose() << std::endl;
      std::cout << "trigger_info : " << trigger_info.transpose() << std::endl;

      // std::cout << "id : " << 0 << std::endl;
      // std::cout << "gripper_reference_min_max[id_hand_controlelr](MIN_INDEX) : "
      //           << gripper_reference_min_max[0](MIN_INDEX) << std::endl;
      // std::cout << "gripper_reference_min_max[id_hand_controlelr](MAX_INDEX) : "
      //           << gripper_reference_min_max[0](MAX_INDEX) << std::endl;
      // std::cout << "gripper_reference(id_hand_controlelr - 0x80) : " << gripper_reference(0) << std::endl;
      // std::cout << "Duration: "
      //           << (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
      //                                                                           start)
      //                      .count() /
      //                  1.e6
      //           << " ms" << std::endl;
    }

    // Sleep for a while to avoid busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void control_loop_hand(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                       std::vector<int> activeIDs) {

  std::vector<Eigen::Matrix<double, 2, 1>> q_min_max_vector;
  q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());
  q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());

  while (1) {
    static int cnt = 0;
    int is_init = true;

    if (activeIDs.size() != 2) {
      std::cout << "The number of Dynamixels for hand gripper does not match the configuration\n";
      return;
    }

    // total moving angle 540 deg

    for (auto const& id : activeIDs) {
      while (1) {

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

      while (1) {

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
        }
        SendCurrent(portHandler, packetHandler, id, 0.5);
      } else {

        std::optional<double> q = ReadEncoder(portHandler, packetHandler, id);
        if (q.has_value()) {
          q_min_max_vector[id](cnt % 2) = q.value();
        }
        SendCurrent(portHandler, packetHandler, id, -0.5);
      }

      if ((double)(abs(q_min_max_vector[id](MAX_INDEX) - q_min_max_vector[id](MIN_INDEX))) * 180 / 3.141592 <
          540 * 0.9) {
        is_init = false;
      }
    }

    if (is_init) {
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

  while (1) {

    for (auto const& id : activeIDs) {
      std::optional<int> operation_mode = ReadOperationMode(portHandler, packetHandler, id);
      if (operation_mode.has_value()) {
        if (operation_mode.value() != CURRENT_BASED_POSITION_CONTROL_MODE) {
          TorqueEnable(portHandler, packetHandler, id, 0);
          SendOperationMode(portHandler, packetHandler, id, CURRENT_BASED_POSITION_CONTROL_MODE);
          TorqueEnable(portHandler, packetHandler, id, 1);
        } else {
          //control loop

          double goal_position = 0;

          {
            std::lock_guard<std::mutex> lg(gripper_mtx);

            //positive current make robot to grip
            // when robot grip full, it is maximum position
            //negative current make robot to release
            // when robot release full, it is minimum position

            // if (gripper_reference(id) < 1100) {
            //   goal_position = q_min_max_vector[id](MIN_INDEX);
            // } else {
            //   goal_position = q_min_max_vector[id](1);
            // }

            static Eigen::Matrix<double, 2, 1> temp_gripper_reference = Eigen::Matrix<double, 2, 1>::Constant(0.5);

            temp_gripper_reference(id) = temp_gripper_reference(id) * 0.9 + gripper_reference(id) * 0.1;

            if (temp_gripper_reference(id) > 1) {
              temp_gripper_reference(id) = 1;
            }
            if (temp_gripper_reference(id) < 0) {
              temp_gripper_reference(id) = 0;
            }

            gripper_reference(id) = (double)((int)(temp_gripper_reference(id) * 50)) / 50.;

            goal_position = gripper_reference(id) * q_min_max_vector[id](MAX_INDEX) +
                            (1. - gripper_reference(id)) * q_min_max_vector[id](MIN_INDEX);
          }

          // goal_position = (q_min_max_vector[id](1) + q_min_max_vector[id](0)) / 2.;  //rad
          SendGoalPosition(portHandler, packetHandler, id, (int)(goal_position * 4096. / 3.141592 / 2.));
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  {
    std::string filepath = "/sys/bus/usb-serial/devices/ttyUSB0/latency_timer";

    std::ofstream file(filepath);
    if (file.is_open()) {
      file << "1";
      file.close();
      std::cout << "Command executed successully" << std::endl;
    } else {
      std::cout << "Error: Unable to open latency_timer file" << std::endl;
    }
  }

  {
    std::string filepath = "/sys/bus/usb-serial/devices/ttyUSB1/latency_timer";

    std::ofstream file(filepath);
    if (file.is_open()) {
      file << "1";
      file.close();
      std::cout << "Command executed successully" << std::endl;
    } else {
      std::cout << "Error: Unable to open latency_timer file" << std::endl;
    }
  }

  // Initialize PortHandler instance
  dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (!portHandler->openPort()) {
    std::cerr << "Failed to open the port!" << std::endl;
    return 1;
  }

  // Set port baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    std::cerr << "Failed to change the baudrate!" << std::endl;
    return 1;
  }

  std::vector<int> activeIDs;

  // Ping IDs from 0 to 13
  for (int id = 0; id <= 13; ++id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->ping(portHandler, id, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      activeIDs.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }

  // Ping IDs from 0x80 to 0x81 (0x80 right / 0x81 left)
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

  for (int id : activeIDs) {
    // Write current control mode value
    if (id < 0x80) {
      int dxl_comm_result = packetHandler->write1ByteTxOnly(portHandler, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
      if (dxl_comm_result != COMM_SUCCESS) {
        std::cerr << "Failed to write current control mode value: " << packetHandler->getTxRxResult(dxl_comm_result)
                  << std::endl;
        return 1;
      } else {
        std::cout << "Switched to current control mode successfully!" << std::endl;
      }

      // Enable Torque
      TorqueEnable(portHandler, packetHandler, id, 1);
    }
  }

  // Start the event loop in a separate thread
  std::thread eventLoopThread(control_loop, portHandler, packetHandler, activeIDs);

  // Initialize PortHandler instance
  dynamixel::PortHandler* portHandler_hand = dynamixel::PortHandler::getPortHandler(DEVICENAME_HAND);

  // Initialize PacketHandler instance
  dynamixel::PacketHandler* packetHandler_hand = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (!portHandler_hand->openPort()) {
    std::cerr << "Failed to open the port!" << std::endl;
    return 1;
  }

  // Set port baudrate
  if (!portHandler_hand->setBaudRate(BAUDRATE)) {
    std::cerr << "Failed to change the baudrate!" << std::endl;
    return 1;
  }

  std::vector<int> activeIDs_hand;

  // Ping IDs from 0 to 13
  for (int id = 0; id < 2; ++id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_hand->ping(portHandler_hand, id, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      std::cout << "Dynamixel ID " << id << " is active." << std::endl;
      activeIDs_hand.push_back(id);
    } else {
      std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
    }
  }

  for (int id : activeIDs_hand) {
    // Write current control mode value
    int dxl_comm_result =
        packetHandler_hand->write1ByteTxOnly(portHandler_hand, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
    if (dxl_comm_result != COMM_SUCCESS) {
      std::cerr << "Failed to write current control mode value: " << packetHandler_hand->getTxRxResult(dxl_comm_result)
                << std::endl;
      return 1;
    } else {
      std::cout << "Switched to current control mode successfully!" << std::endl;
    }

    // Enable Torque
    TorqueEnable(portHandler_hand, packetHandler_hand, id, 1);
  }

  // Start the event loop in a separate thread
  std::thread eventLoopThread_hand(control_loop_hand, portHandler_hand, packetHandler_hand, activeIDs_hand);

  // Wait for the event loop to finish (it won't in this example)
  eventLoopThread.join();

  // Wait for the event loop to finish (it won't in this example)
  eventLoopThread_hand.join();

  // Close port
  portHandler->closePort();
  
  // Close port
  portHandler_hand->closePort();

  return 0;
}
