#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__s
#endif

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include "dynamixel_sdk.h"

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"

using namespace rb;
using namespace std::chrono_literals;

const std::string kAll = ".*";

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 2000000

#define ADDR_TORQUE_ENABLE 64
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_CURRENT 102
#define ADDR_OPERATING_MODE 11

#define CURRENT_CONTROL_MODE 0

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

#define D2R 0.017453288888888
#define R2D 57.29579143313326

using namespace rb::dyn;
using namespace rb;

#define MIN_INDEX 0
#define MAX_INDEX 1

bool isInitFinish = false;

void TorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, int onoff) {
  packetHandler->write1ByteTxOnly(portHandler, id, ADDR_TORQUE_ENABLE, onoff);
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

void SendOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id,
                       int operation_mode) {
  packetHandler->write1ByteTxOnly(portHandler, id, ADDR_OPERATING_MODE, operation_mode);
  std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void SendCurrent(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, double current) {
  //current unit is [A]
  int32_t current_value = (int)(current / 2.69 * 1000.);
  packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, current_value);
}

void control_loop_for_gripper(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler,
                              std::vector<int> activeIDs) {

  std::vector<Eigen::Matrix<double, 2, 1>> q_min_max_vector;
  q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());
  q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());
  // q_min_max_vector.push_back((Eigen::Matrix<double, 2, 1>() << 100, -100).finished());
  // q_min_max_vector.push_back((Eigen::Matrix<double, 2, 1>() << 100, -100).finished());
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

      std::cout << " id: " << id << std::endl;
      std::cout << " q_min_max_vector[id](0): " << q_min_max_vector[id](0) * 180 / 3.141592 << std::endl;
      std::cout << " q_min_max_vector[id](1): " << q_min_max_vector[id](1) * 180 / 3.141592 << std::endl;
      std::cout << " is_init: " << is_init << std::endl;
    }

    if (is_init) {
      for (auto const& id : activeIDs) {
        if (q_min_max_vector[id](MIN_INDEX) > q_min_max_vector[id](MAX_INDEX)) {
          double temp = q_min_max_vector[id](MIN_INDEX);
          q_min_max_vector[id](MIN_INDEX) = q_min_max_vector[id](MAX_INDEX);
          q_min_max_vector[id](MAX_INDEX) = temp;
        }

        SendCurrent(portHandler, packetHandler, id, 0);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      isInitFinish = true;

      break;
    }

    cnt++;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  std::cout << "OK\n";
}

int main(int argc, char** argv) {
  try {
    // Latency timer setting
    upc::InitializeDevice(upc::kGripperDeviceName);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};

  auto robot = rb::Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

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

  if (robot->IsPowerOn("48v")) {
    robot->SetToolFlangeOutputVoltage("right", 12);
    robot->SetToolFlangeOutputVoltage("left", 12);
    std::cout << "Attempting to 12V power on for gripper." << std::endl;

    std::this_thread::sleep_for(1s);
  }

  const char* devicename_gripper = "/dev/rby1_gripper";

  dynamixel::PortHandler* portHandler_gripper = dynamixel::PortHandler::getPortHandler(devicename_gripper);
  dynamixel::PacketHandler* packetHandler_gripper = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_gripper->openPort()) {
    std::cerr << "Failed to open the port!" << std::endl;
    return 1;
  }

  if (!portHandler_gripper->setBaudRate(BAUDRATE)) {
    std::cerr << "Failed to change the baudrate!" << std::endl;
    return 1;
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
    return 1;
  }

  for (int id : activeIDs_gripper) {
    int dxl_comm_result =
        packetHandler_gripper->write1ByteTxOnly(portHandler_gripper, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
    if (dxl_comm_result != COMM_SUCCESS) {
      std::cerr << "Failed to write current control mode value: "
                << packetHandler_gripper->getTxRxResult(dxl_comm_result) << std::endl;
      return 1;
    }

    TorqueEnable(portHandler_gripper, packetHandler_gripper, id, 1);
  }

  std::thread gripper_handler(control_loop_for_gripper, portHandler_gripper, packetHandler_gripper, activeIDs_gripper);

  while (1) {
    std::this_thread::sleep_for(5ms);
    if (isInitFinish)
      break;
  }

  gripper_handler.join();
  portHandler_gripper->closePort();

  return 0;
}
