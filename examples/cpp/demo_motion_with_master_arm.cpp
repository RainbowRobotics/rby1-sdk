#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>
#include "dynamixel_sdk.h"

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"
#include "rby1-sdk/upc/device.h"

using namespace rb;
using namespace std::chrono_literals;

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

bool redandancy_mode = false;
double m_sf = 0.4;

double init_cnt = 0.;
bool ma_master_mode = false;

using namespace rb::dyn;
using namespace rb;

#define MIN_INDEX 0
#define MAX_INDEX 1

bool is_first_init = false;
std::mutex mtx_q_joint_ma_info;
Eigen::Vector<double, 24> q_redandancy_fix = Eigen::Vector<double, 24>::Zero();
Eigen::Matrix<double, 14, 1> q_joint_ma = Eigen::Matrix<double, 14, 1>::Zero();

bool ma_info_verbose = false;

std::vector<double> torque_constant = {1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043,
                                       1.6591, 1.6591, 1.6591, 1.3043, 1.3043, 1.3043, 1.3043};

const std::string kAll = ".*";

// Network* network;

// #include <QEventLoop>
// #include <QThread>
// #include <QUdpSocket>

// class UdpThread : public QThread {
//   Q_OBJECT
//  public:
//   void run() override {
//     QEventLoop loop;
//     loop.exec();
//   }
// };

Eigen::Vector<double, 24> q_joint_rby1_24x1 = Eigen::Vector<double, 24>::Zero();

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

std::optional<std::vector<std::pair<int, double>>> BulkReadGoalPosition(dynamixel::PortHandler* portHandler,
                                                                        dynamixel::PacketHandler* packetHandler,
                                                                        std::vector<int> ids) {

  std::vector<std::pair<int, double>> position_vector;
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  for (auto const& id : ids) {
    if (id < 0x80) {
      groupBulkRead.addParam(id, ADDR_GOAL_POSITION, 4);
    }
  }

  groupBulkRead.txRxPacket();

  for (auto const& id : ids) {
    if (id < 0x80) {
      if (groupBulkRead.isAvailable(id, ADDR_GOAL_POSITION, 4)) {
        int position = groupBulkRead.getData(id, ADDR_GOAL_POSITION, 4);
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

void BulkWriteGoalPosition(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                           std::vector<std::pair<int, double>> id_and_q_vector) {

  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  for (auto const& id_and_q : id_and_q_vector) {
    if (id_and_q.first < 0x80) {
      int goal_position = (int)(id_and_q.second * 4096. / 2. / 3.141592);
      uint8_t param[4];
      param[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
      param[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
      param[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
      param[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
      groupBulkWrite.addParam(id_and_q.first, ADDR_GOAL_POSITION, 4, param);
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

void control_loop_for_robot(std::shared_ptr<rb::Robot<y1_model::A>> robot) {

  Eigen::Vector<double, 6> q_joint_waist;
  Eigen::Vector<double, 7> q_joint_right_arm;
  Eigen::Vector<double, 7> q_joint_left_arm;
  q_joint_waist.setZero();
  q_joint_right_arm.setZero();
  q_joint_left_arm.setZero();

  // double minimum_time = 4.;
  // double minimum_time = 2.;
  double minimum_time = 1.5;

  //go to init pos
  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(3).SetPosition(
                            Eigen::Vector<double, 6>{0, 30, -60, 30, 0, 0} * D2R))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(3).SetPosition(
                            Eigen::Vector<double, 7>{45, -15, 0, -135, 0, 45, 0} * D2R))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(3).SetPosition(
                            Eigen::Vector<double, 7>{45, 15, 0, -135, 0, 45, 0} * D2R)))))
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Error: Failed to conduct demo motion." << std::endl;
  }

  is_first_init = true;

  if (0) {
    std::cout << "joint position command example 1\n";
    q_joint_waist.setZero();
    q_joint_right_arm.setZero();
    q_joint_left_arm.setZero();

    q_joint_right_arm(1) = -90 * D2R;
    q_joint_left_arm(1) = 90 * D2R;

    // go to ready position
    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder()
                    .SetTorsoCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_waist))
                    .SetRightArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_right_arm))
                    .SetLeftArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time).SetPosition(q_joint_left_arm)))))
            ->Get();
    std::this_thread::sleep_for(1s);

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;

      return;
    }
  }

  if (1) {
    std::cout << "joint position command example 2\n";

    q_joint_waist << 0, 30, -60, 30, 0, 0;
    q_joint_waist *= D2R;

    q_joint_right_arm << -45, -30, 0, -90, 0, 45, 0;
    q_joint_right_arm *= D2R;

    q_joint_left_arm << -45, 30, 0, -90, 0, 45, 0;
    q_joint_left_arm *= D2R;

    Eigen::Vector<double, 20> q;
    q.block(0, 0, 6, 1) = q_joint_waist;
    q.block(6, 0, 7, 1) = q_joint_right_arm;
    q.block(6 + 7, 0, 7, 1) = q_joint_left_arm;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      JointPositionCommandBuilder().SetPosition(q).SetMinimumTime(minimum_time))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  double angular_velocity_limit = 3.141592 * 1.5;
  double linear_velocity_limit = 1.5;
  double acceleration_limit = 1.0;
  double stop_orientation_tracking_error = 1e-5;
  double stop_position_tracking_error = 1e-5;

  Eigen::Matrix<double, 4, 4> T_torso, T_right, T_left;
  T_torso.setIdentity();
  T_right.setIdentity();
  T_left.setIdentity();

  if (1) {
    std::cout << "Cartesian command example 1\n";

    T_torso.block(0, 0, 3, 3).setIdentity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      BodyComponentBasedCommandBuilder()
                          .SetTorsoCommand(CartesianCommandBuilder()
                                               .AddTarget("base", "link_torso_5", T_torso, linear_velocity_limit,
                                                          angular_velocity_limit, acceleration_limit / 2)
                                               .SetMinimumTime(minimum_time * 1)
                                               .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                               .SetStopPositionTrackingError(stop_position_tracking_error))
                          .SetRightArmCommand(CartesianCommandBuilder()
                                                  .AddTarget("base", "ee_right", T_right, linear_velocity_limit,
                                                             angular_velocity_limit, acceleration_limit / 2)
                                                  .SetMinimumTime(minimum_time * 3)
                                                  .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                                  .SetStopPositionTrackingError(stop_position_tracking_error))
                          .SetLeftArmCommand(CartesianCommandBuilder()
                                                 .AddTarget("base", "ee_left", T_left, linear_velocity_limit,
                                                            angular_velocity_limit, acceleration_limit / 2)
                                                 .SetMinimumTime(minimum_time * 3)
                                                 .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                                 .SetStopPositionTrackingError(stop_position_tracking_error)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "Cartesian command example 2\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.4, 1.2;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.4, 1.2;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      CartesianCommandBuilder()
                          .AddTarget("base", "link_torso_5", T_torso, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("base", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("base", "ee_left", T_left, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .SetStopPositionTrackingError(stop_orientation_tracking_error)
                          .SetStopOrientationTrackingError(stop_position_tracking_error)
                          .SetMinimumTime(minimum_time))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "Cartesian command example 3\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.2;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.4, 0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.4, 0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      CartesianCommandBuilder()
                          .AddTarget("base", "link_torso_5", T_torso, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("link_torso_5", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .AddTarget("link_torso_5", "ee_left", T_left, linear_velocity_limit, angular_velocity_limit,
                                     acceleration_limit)
                          .SetStopPositionTrackingError(stop_orientation_tracking_error)
                          .SetStopOrientationTrackingError(stop_position_tracking_error)
                          .SetMinimumTime(minimum_time))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "impedance control command example 1\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.2;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.45, -0.4, -0.1;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.45, 0.4, -0.1;

    ImpedanceControlCommandBuilder torso_command;
    torso_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time))
        .SetReferenceLinkName("base")
        .SetLinkName("link_torso_5")
        .SetTranslationWeight({1000, 1000, 1000})
        .SetRotationWeight({100, 100, 100})
        .SetTransformation(T_torso);

    ImpedanceControlCommandBuilder right_arm_command;
    right_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time))
        .SetReferenceLinkName("link_torso_5")
        .SetLinkName("ee_right")
        .SetTranslationWeight({1000, 1000, 1000})
        .SetRotationWeight({50, 50, 50})
        .SetTransformation(T_right);

    ImpedanceControlCommandBuilder left_arm_command;
    left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time))
        .SetReferenceLinkName("link_torso_5")
        .SetLinkName("ee_left")
        .SetTranslationWeight({1000, 1000, 1000})
        .SetRotationWeight({50, 50, 50})
        .SetTransformation(T_left);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                                        .SetTorsoCommand(torso_command)
                                                                        .SetRightArmCommand(right_arm_command)
                                                                        .SetLeftArmCommand(left_arm_command))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }
  }

  if (1) {
    std::cout << "relative command example 1\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotY(3.141592 / 6.);
    T_torso.block(0, 3, 3, 1) << 0.1, 0, 1.1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.4, 0.9;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.4, 0.9;

    CartesianCommandBuilder right_arm_command;
    right_arm_command.SetMinimumTime(minimum_time)
        .SetStopOrientationTrackingError(stop_orientation_tracking_error)
        .SetStopPositionTrackingError(stop_position_tracking_error)
        .AddTarget("base", "ee_right", T_right, linear_velocity_limit, angular_velocity_limit, acceleration_limit);

    Eigen::Matrix<double, 4, 4> T_diff;
    T_diff.setIdentity();
    T_diff.block(0, 3, 3, 1) << 0, 0.8, 0;

    ImpedanceControlCommandBuilder left_arm_command;
    left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time * 2.0))
        .SetReferenceLinkName("ee_right")
        .SetLinkName("ee_left")
        .SetTranslationWeight({500, 500, 500})
        .SetRotationWeight({50, 50, 50})
        .SetTransformation(T_diff);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                                        .SetRightArmCommand(right_arm_command)
                                                                        .SetLeftArmCommand(left_arm_command))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }
  }

  if (1) {
    std::cout << "joint position command example 3\n";

    q_joint_waist << 0, 30, -60, 30, 0, 0;
    q_joint_waist *= D2R;

    q_joint_right_arm << -45, -30, 0, -90, 0, 45, 0;
    q_joint_right_arm *= D2R;

    q_joint_left_arm << -45, 30, 0, -90, 0, 45, 0;
    q_joint_left_arm *= D2R;

    Eigen::Vector<double, 20> q;
    q.block(0, 0, 6, 1) = q_joint_waist;
    q.block(6, 0, 7, 1) = q_joint_right_arm;
    q.block(6 + 7, 0, 7, 1) = q_joint_left_arm;

    JointPositionCommandBuilder joint_position_command;
    joint_position_command.SetPosition(q).SetMinimumTime(minimum_time);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(joint_position_command)))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  double velocity_tracking_gain = 0.01;
  double stop_cost = 1e-2;
  double weight = 0.001;
  double min_delta_cost = 1e-4;
  int patience = 10;

  if (1) {
    std::cout << "optimal control example 1\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::Identity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1.0;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.2, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.2, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      OptimalControlCommandBuilder()
                          .AddCartesianTarget("base", "link_torso_5", T_torso, weight, weight)
                          .AddCartesianTarget("base", "ee_right", T_right, weight, weight)
                          .AddCartesianTarget("base", "ee_left", T_left, weight, weight)
                          .AddJointPositionTarget("right_arm_2", 3.141592 / 2., weight)
                          .AddJointPositionTarget("left_arm_2", -3.141592 / 2., weight)
                          .SetVelocityLimitScaling(0.05)
                          .SetStopCost(stop_cost)
                          .SetMinDeltaCost(min_delta_cost)
                          .SetPatience(patience))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "optimal control example 2\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::Identity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1.0;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.2, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.2, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      OptimalControlCommandBuilder()
                          .AddCartesianTarget("base", "link_torso_5", T_torso, weight, weight)
                          .AddCartesianTarget("base", "ee_right", T_right, weight, weight)
                          .AddCartesianTarget("base", "ee_left", T_left, weight, weight)
                          .AddJointPositionTarget("right_arm_2", 0., weight)
                          .AddJointPositionTarget("left_arm_2", -0., weight)
                          .SetStopCost(stop_cost)
                          .SetMinDeltaCost(min_delta_cost)
                          .SetPatience(patience))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "optimal control example 3\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotZ(3.141592 / 4.);
    T_torso.block(0, 3, 3, 1) << 0, 0, 1.0;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.4, -0.3, 1.2;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.4, 0.3, 1.2;

    Eigen::Vector<double, 3> COM;
    COM << 0., 0., 0.5;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      OptimalControlCommandBuilder()
                          .SetCenterOfMassTarget("base", COM, weight)
                          .AddCartesianTarget("base", "ee_left", T_left, weight, weight)
                          .AddCartesianTarget("base", "ee_right", T_right, weight, weight)
                          .AddJointPositionTarget("torso_4", 0., weight)
                          .AddJointPositionTarget("torso_2", -3.141592 / 2., weight)
                          .AddJointPositionTarget("right_arm_2", 3.141592 / 4., weight)
                          .AddJointPositionTarget("left_arm_2", -3.141592 / 4., weight)
                          .SetStopCost(stop_cost)
                          .SetMinDeltaCost(min_delta_cost)
                          .SetPatience(patience))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "mixed command example 1\n";

    T_torso.block(0, 0, 3, 3).setIdentity();
    T_torso.block(0, 3, 3, 1) << 0, 0, 1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 4.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.0;

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      BodyComponentBasedCommandBuilder()
                          .SetTorsoCommand(
                              OptimalControlCommandBuilder()
                                  .SetCenterOfMassTarget("base", Eigen::Vector<double, 3>{0, 0, 0.4}, weight * 1e-1)
                                  .AddCartesianTarget("base", "link_torso_5", T_torso, 0, weight)
                                  .AddJointPositionTarget("torso_2", -3.141592 / 2., weight)
                                  .AddJointPositionTarget("torso_0", 0., weight)
                                  .SetStopCost(stop_cost * 1e1)
                                  .SetMinDeltaCost(min_delta_cost)
                                  .SetPatience(patience))
                          .SetRightArmCommand(
                              JointPositionCommandBuilder()
                                  .SetPosition(Eigen::Vector<double, 7>{0, -3.141592 / 4., 0, -3.1415 / 2., 0, 0, 0})
                                  .SetVelocityLimit(Eigen::Vector<double, 7>::Constant(3.141592))
                                  .SetAccelerationLimit(Eigen::Vector<double, 7>::Constant(1.0))
                                  .SetMinimumTime(minimum_time))
                          .SetLeftArmCommand(CartesianCommandBuilder()
                                                 .AddTarget("base", "ee_left", T_left, linear_velocity_limit,
                                                            angular_velocity_limit, acceleration_limit)
                                                 .SetMinimumTime(minimum_time)
                                                 .SetStopOrientationTrackingError(stop_orientation_tracking_error)
                                                 .SetStopPositionTrackingError(stop_position_tracking_error)))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "mixed command example 2\n";

    T_torso.block(0, 0, 3, 3) = math::SO3::RotZ(3.141592 / 4.);
    T_torso.block(0, 3, 3, 1) << 0, 0, 1;

    T_right.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_right.block(0, 3, 3, 1) << 0.5, -0.3, 1.0;

    T_left.block(0, 0, 3, 3) = math::SO3::RotY(-3.141592 / 2.);
    T_left.block(0, 3, 3, 1) << 0.5, 0.3, 1.0;

    OptimalControlCommandBuilder torso_command;
    JointPositionCommandBuilder right_arm_command;
    GravityCompensationCommandBuilder left_arm_command;

    torso_command.SetCenterOfMassTarget("base", Eigen::Vector<double, 3>{0, 0, 0.4}, weight * 1e-1)
        .AddCartesianTarget("base", "link_torso_5", T_torso, 0, weight)
        .AddJointPositionTarget("torso_2", -3.141592 / 2., weight)
        .AddJointPositionTarget("torso_0", 0., weight)
        .SetStopCost(stop_cost)
        .SetMinDeltaCost(min_delta_cost / 10)
        .SetPatience(patience * 10);

    right_arm_command.SetPosition(Eigen::Vector<double, 7>{0, -3.141592 / 4., 0, -3.1415 / 2., 0, 0, 0})
        .SetVelocityLimit(Eigen::Vector<double, 7>::Constant(3.141592))
        .SetAccelerationLimit(Eigen::Vector<double, 7>::Constant(1.0))
        .SetMinimumTime(minimum_time);

    left_arm_command.SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(minimum_time)).SetOn(true);

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(
                      ComponentBasedCommandBuilder().SetBodyCommand(BodyComponentBasedCommandBuilder()
                                                                        .SetTorsoCommand(torso_command)
                                                                        .SetRightArmCommand(right_arm_command)
                                                                        .SetLeftArmCommand(left_arm_command))))
                  ->Get();

    std::cout << "!!" << std::endl;

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (0) {
    std::cout << "go to home pose 1\n";
    q_joint_waist.setZero();
    q_joint_right_arm.setZero();
    q_joint_left_arm.setZero();

    q_joint_right_arm(1) = -135 * D2R;
    q_joint_left_arm(1) = 135 * D2R;

    // go to ready position
    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder()
                    .SetTorsoCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time * 2).SetPosition(q_joint_waist))
                    .SetRightArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time * 2).SetPosition(q_joint_right_arm))
                    .SetLeftArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(minimum_time * 2).SetPosition(q_joint_left_arm)))))
            ->Get();
    std::this_thread::sleep_for(1s);

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;

      return;
    }
  }

  if (1) {
    std::cout << "go to home pose 2\n";

    auto rv = robot
                  ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                      JointPositionCommandBuilder()
                          .SetPosition(Eigen::Vector<double, 20>::Constant(0.0))
                          .SetMinimumTime(minimum_time * 2.0))))
                  ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to conduct demo motion." << std::endl;
      return;
    }

    std::this_thread::sleep_for(1s);
  }

  if (1) {
    std::cout << "power off" << std::endl;
    robot->DisableControlManager();
    std::this_thread::sleep_for(1s);
    robot->PowerOff("12v");
    std::this_thread::sleep_for(1s);
  }

  std::exit(0);
}

void control_loop_for_master_arm(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                                 std::vector<int> activeIDs) {

  auto robot = std::make_shared<rb::dyn::Robot<14>>(LoadRobotFromURDF(PATH "/master_arm.urdf", "Base"));
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>(
      {"Base", "Link_0R", "Link_1R", "Link_2R", "Link_3R", "Link_4R", "Link_5R", "Link_6R", "Link_0L", "Link_1L",
       "Link_2L", "Link_3L", "Link_4L", "Link_5L", "Link_6L"},
      {"J0_Shoulder_Pitch_R", "J1_Shoulder_Roll_R", "J2_Shoulder_Yaw_R", "J3_Elbow_R", "J4_Wrist_Yaw1_R",
       "J5_Wrist_Pitch_R", "J6_Wrist_Yaw2_R", "J7_Shoulder_Pitch_L", "J8_Shoulder_Roll_L", "J9_Shoulder_Yaw_L",
       "J10_Elbow_L", "J11_Wrist_Yaw1_L", "J12_Wrist_Pitch_L", "J13_Wrist_Yaw2_L"});

  state->SetGravity({0, 0, 0, 0, 0, -9.81});

  Eigen::Matrix<double, 14, 1> q_joint, tau_joint, q_joint_target;
  Eigen::Matrix<int, 14, 1> operation_mode, torque_enable;
  std::vector<std::optional<std::pair<int, std::pair<int, int>>>> button_status_vector;
  q_joint.setZero();
  tau_joint.setZero();
  q_joint_target.setZero();

  Eigen::Matrix<double, 14, 1> temp_eigen;
  Eigen::Matrix<double, 2, 1> button_info, trigger_info;
  temp_eigen.setZero();
  button_info.setZero();
  trigger_info.setZero();

  Eigen::Vector<double, 14> q_joint_default;
  q_joint_default << 45, -30, 0, -135, -30, 90, 0, 45, 30, 0, -135, 30, 90, 0;
  q_joint_default = q_joint_default * 3.141592 / 180.;

  while (true) {
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
    std::vector<std::pair<int, double>> id_and_q_vector;
    std::vector<int> id_torque_onoff_vector;
    std::vector<std::pair<int, double>> id_send_torque_vector;
    // std::cout<<"is_first_init: "<<is_first_init<<std::endl;
    if (!is_first_init) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    if (1) {
      Eigen::Vector<double, 14> temp_q;

      {
        std::lock_guard<std::mutex> lg(mtx_q_joint_ma_info);

        init_cnt += 0.005;

        if (init_cnt > 1) {
          init_cnt = 1.;
        }

        temp_q = q_joint_rby1_24x1.block(2 + 6, 0, 14, 1) * init_cnt + q_joint_ma * (1. - init_cnt);
      }
      // right arm
      for (int id = 0; id < 7; id++) {
        // position control
        if (operation_mode(id) != CURRENT_BASED_POSITION_CONTROL_MODE) {
          id_and_mode_vector.push_back(std::make_pair(id, CURRENT_BASED_POSITION_CONTROL_MODE));
          id_torque_onoff_vector.push_back(id);
          id_and_q_vector.push_back(std::make_pair(id, temp_q(id)));
        } else {
          id_and_q_vector.push_back(std::make_pair(id, temp_q(id)));
        }
      }

      //left arm
      for (int id = 7; id < 7 + 7; id++) {
        // position control
        if (operation_mode(id) != CURRENT_BASED_POSITION_CONTROL_MODE) {
          id_and_mode_vector.push_back(std::make_pair(id, CURRENT_BASED_POSITION_CONTROL_MODE));
          id_torque_onoff_vector.push_back(id);
          id_and_q_vector.push_back(std::make_pair(id, temp_q(id)));
        } else {
          id_and_q_vector.push_back(std::make_pair(id, temp_q(id)));
        }
      }

      BulkWriteTorqueEnable(portHandler, packetHandler, id_torque_onoff_vector, 0);
      BulkWriteOperationMode(portHandler, packetHandler, id_and_mode_vector);
      BulkWriteTorqueEnable(portHandler, packetHandler, id_torque_onoff_vector, 1);

      BulkWriteGoalPosition(portHandler, packetHandler, id_and_q_vector);
    }

    static int cnt = 0;
    if (ma_info_verbose) {
      if (cnt++ % 5 == 0) {
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

int main(int argc, char** argv) {

  // QApplication app(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address> [servo]" << std::endl;
    return 1;
  }

  // UdpThread thread;
  // thread.start();
  // network = new Network();

  std::string address{argv[1]};
  std::string servo =
      "^(right_wheel|left_wheel|torso_0|torso_1|torso_2|torso_3|torso_4|torso_5|right_arm_0|right_arm_1|right_arm_2|"
      "right_arm_3|right_arm_4|right_arm_5|right_arm_6|left_arm_0|left_arm_1|left_arm_2|left_arm_3|left_arm_4|left_arm_"
      "5|left_arm_6)$";  // 기본값

  if (argc >= 3) {
    servo = argv[2];
  }

  try {
    upc::InitializeDevice(upc::kMasterArmDeviceName);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  auto robot = rb::Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Starting state update..." << std::endl;

  robot->SetParameter("joint_position_command.cutoff_frequency", "5.0");

  robot->StartStateUpdate(
      [&](const auto& state) {
        if (ma_info_verbose) {
          std::cout << "State Update Received:" << std::endl;
          std::cout << "  Timestamp: " << state.timestamp.tv_sec << ".";
          std::cout << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
          std::cout << "  wasit [deg]     : " << state.position.block(2, 0, 6, 1).transpose() * R2D << std::endl;
          std::cout << "  right arm [deg] : " << state.position.block(2 + 6, 0, 7, 1).transpose() * R2D << std::endl;
          std::cout << "  left arm [deg]  : " << state.position.block(2 + 6 + 7, 0, 7, 1).transpose() * R2D
                    << std::endl;
        }

        q_joint_rby1_24x1.block(2, 0, 20, 1) = state.position.block(2, 0, 20, 1);
      },
      100 /* Hz */);

  std::cout << "!!\n";

  std::this_thread::sleep_for(1s);

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn(kAll)) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(kAll)) {
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

  const char* devicename_master_arm = "/dev/rby1_master_arm";

  dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(devicename_master_arm);
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

  if (activeIDs.size() != 14) {
    std::cerr << "Unable to ping all devices for master arm" << std::endl;
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

  std::thread master_arm_handler(control_loop_for_master_arm, portHandler, packetHandler, activeIDs);
  std::this_thread::sleep_for(1s);
  std::thread robot_op(control_loop_for_robot, robot);

  master_arm_handler.join();
  robot_op.join();

  return 0;
}
