#include "rby1-sdk/base/dynamixel_bus.h"
#include "dynamixel_sdk.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;

namespace rb {

class DynamixelBusImpl {
 public:
  explicit DynamixelBusImpl(const std::string& dev_name) {
    port_handler_ = dynamixel::PortHandler::getPortHandler(dev_name.c_str());
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(DynamixelBus::kProtocolVersion);
  }

  ~DynamixelBusImpl() {
    port_handler_->closePort();

    delete port_handler_;
    delete packet_handler_;
  }

  void SetTorqueConstant(const std::vector<double>& torque_constant) { torque_constant_ = torque_constant; }

  bool OpenPort() { return port_handler_->openPort(); }

  bool SetBaudRate(int baudrate) { return port_handler_->setBaudRate(baudrate); }

  bool Ping(int id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->ping(port_handler_, id, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      return true;
    } else {
      return false;
    }
  }

  std::optional<std::pair<int /* id */, DynamixelBus::ButtonState>> ReadButtonStatus(int id) {
    int32_t position = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, DynamixelBus::kAddrPresentButtonState,
                                                         (uint32_t*)&position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      int button = (position >> 8) & 0xff;
      int trigger = ((position >> 16) & 0xff) | (((position >> 24) & 0xff) << 8);
      DynamixelBus::ButtonState button_state{};
      button_state.button = button;
      button_state.trigger = trigger;
      return std::make_pair(id, button_state);
    } else {
      return {};
    }
  }

  void SendTorqueEnable(int id, int onoff) {
    packet_handler_->write1ByteTxOnly(port_handler_, id, DynamixelBus::kAddrTorqueEnable, onoff);
    std::this_thread::sleep_for(50us);
  }

  std::optional<int> ReadTorqueEnable(int id) {
    int8_t onoff = -1;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, DynamixelBus::kAddrTorqueEnable,
                                                         (uint8_t*)&onoff, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      return onoff;
    } else {
      return {};
    }
  }

  std::optional<double> ReadEncoder(int id) {
    int32_t position = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, DynamixelBus::kAddrPresentPosition,
                                                         (uint32_t*)&position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      return (double)position / 4096. * 2. * 3.141592;  // (rad)
    } else {
      return {};
    }
  }

  void SendGoalPosition(int id, int goal_position) {
    packet_handler_->write4ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalPosition, goal_position);
    std::this_thread::sleep_for(50us);
  }

  std::optional<int> ReadOperationMode(int id) {
    int8_t operation_mode = -1;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, DynamixelBus::kAddrOperatingMode,
                                                         (uint8_t*)&operation_mode, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      return operation_mode;
    } else {
      return {};
    }
  }

  bool SendOperationMode(int id, int operation_mode) {
    int dxl_comm_result =
        packet_handler_->write1ByteTxOnly(port_handler_, id, DynamixelBus::kAddrOperatingMode, operation_mode);
    std::this_thread::sleep_for(50us);
    return dxl_comm_result == COMM_SUCCESS;
  }

  void SendTorque(int id, double joint_torque) {
    if (id >= torque_constant_.size())
      return;

    int32_t torque_value = (int32_t)(joint_torque / torque_constant_[id] * 1000. / 2.69);
    packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalCurrent, torque_value);
  }

  void SendCurrent(int id, double current /* (Amp) */) {
    int32_t current_value = (int)(current / 2.69 * 1000.);
    packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalCurrent, current_value);
  }

  std::optional<std::vector<std::pair<int, int>>> BulkRead(const std::vector<int>& ids, int addr, int len) {
    std::vector<std::pair<int, int>> rv;
    dynamixel::GroupBulkRead groupBulkRead(port_handler_, packet_handler_);

    for (auto const& id : ids) {
      if (id < 0x80) {
        groupBulkRead.addParam(id, addr, len);
      }
    }

    groupBulkRead.txRxPacket();

    for (auto const& id : ids) {
      if (id < 0x80) {
        if (groupBulkRead.isAvailable(id, addr, len)) {
          int data = groupBulkRead.getData(id, addr, len);
          rv.push_back({id, data});
        }
      }
    }

    if (rv.empty()) {
      return {};
    } else {
      return rv;
    }
  }

  std::optional<std::vector<std::pair<int /* id */, double /* enc (rad) */>>> BulkReadEncoder(
      const std::vector<int>& ids) {
    std::vector<std::pair<int, double>> v;

    const auto& rv = BulkRead(ids, DynamixelBus::kAddrPresentPosition, 4);
    if (rv.has_value()) {
      for (const auto& r : rv.value()) {
        v.push_back({r.first, (double)r.second / 4096. * 2. * 3.141592});
      }
      return v;
    } else {
      return {};
    }
  }

  std::optional<std::vector<std::pair<int, int>>> BulkReadOperationMode(const std::vector<int>& ids) {
    std::vector<std::pair<int, int>> operation_mode_vector;
    dynamixel::GroupBulkRead groupBulkRead(port_handler_, packet_handler_);

    for (auto const& id : ids) {
      if (id < 0x80) {
        groupBulkRead.addParam(id, DynamixelBus::kAddrOperatingMode, 1);
      }
    }

    groupBulkRead.txRxPacket();

    for (auto const& id : ids) {
      if (id < 0x80) {
        if (groupBulkRead.isAvailable(id, DynamixelBus::kAddrOperatingMode, 1)) {
          int operation_mode = groupBulkRead.getData(id, DynamixelBus::kAddrOperatingMode, 1);
          operation_mode_vector.push_back(std::make_pair(id, operation_mode));
        }
      }
    }

    if (operation_mode_vector.empty()) {
      return {};
    } else {
      return operation_mode_vector;
    }
  }

  std::optional<std::vector<std::pair<int, int>>> BulkReadTorqueEnable(const std::vector<int>& ids) {
    std::vector<std::pair<int, int>> torque_enable_vector;
    dynamixel::GroupBulkRead groupBulkRead(port_handler_, packet_handler_);

    for (auto const& id : ids) {
      if (id < 0x80) {
        groupBulkRead.addParam(id, DynamixelBus::kAddrTorqueEnable, 1);
      }
    }

    groupBulkRead.txRxPacket();

    for (auto const& id : ids) {
      if (id < 0x80) {
        if (groupBulkRead.isAvailable(id, DynamixelBus::kAddrTorqueEnable, 1)) {
          int operation_mode = groupBulkRead.getData(id, DynamixelBus::kAddrTorqueEnable, 1);
          torque_enable_vector.push_back(std::make_pair(id, operation_mode));
        }
      }
    }

    if (torque_enable_vector.empty()) {
      return {};
    } else {
      return torque_enable_vector;
    }
  }

  std::optional<std::vector<std::pair<int, DynamixelBus::MotorState>>> BulkReadMotorState(const std::vector<int>& ids) {
    std::vector<std::pair<int, DynamixelBus::MotorState>> ms;
    std::unordered_map<int, int> idx;
    for (const auto& id : ids) {
      ms.push_back({id, DynamixelBus::MotorState()});
      idx[id] = ms.size() - 1;
    }

    {
      const auto& rv = BulkRead(ids, DynamixelBus::kAddrTorqueEnable, 1);
      if (rv.has_value()) {
        for (const auto& r : rv.value()) {
          ms[idx[r.first]].second.torque_enable = (r.second == 1);
        }
      } else {
        return {};
      }
    }

    {
      const auto& rv = BulkRead(ids, DynamixelBus::kAddrPresentCurrent, 2);
      if (rv.has_value()) {
        for (const auto& r : rv.value()) {
          ms[idx[r.first]].second.current = (double)r.second / 1000. * 2.69;
        }
      } else {
        return {};
      }
    }

    {
      const auto& rv = BulkRead(ids, DynamixelBus::kAddrPresentVelocity, 4);
      if (rv.has_value()) {
        for (const auto& r : rv.value()) {
          ms[idx[r.first]].second.velocity = (double)r.second * 0.229 * 2 * 3.141592 / 60;
        }
      } else {
        return {};
      }
    }

    {
      const auto& rv = BulkRead(ids, DynamixelBus::kAddrPresentPosition, 4);
      if (rv.has_value()) {
        for (const auto& r : rv.value()) {
          ms[idx[r.first]].second.position = (double)r.second / 4096. * 2. * 3.141592;
        }
      } else {
        return {};
      }
    }

    return ms;
  }

  void BulkWriteTorqueEnable(const std::vector<std::pair<int, int>>& id_and_enable_vector) {
    if (id_and_enable_vector.empty())
      return;

    dynamixel::GroupBulkWrite groupBulkWrite(port_handler_, packet_handler_);

    uint8_t param[1];

    for (auto const& id_and_enable : id_and_enable_vector) {
      if (id_and_enable.first < 0x80) {
        param[0] = id_and_enable.second;
        groupBulkWrite.addParam(id_and_enable.first, DynamixelBus::kAddrTorqueEnable, 1, param);
      }
    }

    groupBulkWrite.txPacket();
    std::this_thread::sleep_for(50us);
  }

  void BulkWriteTorqueEnable(const std::vector<int>& ids, int enable) {
    if (ids.empty())
      return;

    dynamixel::GroupBulkWrite groupBulkWrite(port_handler_, packet_handler_);

    uint8_t param[1];

    for (auto const& id : ids) {
      if (id < 0x80) {
        param[0] = enable;
        groupBulkWrite.addParam(id, DynamixelBus::kAddrTorqueEnable, 1, param);
      }
    }

    groupBulkWrite.txPacket();
    std::this_thread::sleep_for(50us);
  }

  void BulkWriteOperationMode(const std::vector<std::pair<int, int>>& id_and_mode_vector) {
    if (id_and_mode_vector.empty())
      return;

    dynamixel::GroupBulkWrite groupBulkWrite(port_handler_, packet_handler_);

    uint8_t param[1];

    for (auto const& id_and_mode : id_and_mode_vector) {
      if (id_and_mode.first < 0x80) {
        param[0] = id_and_mode.second;
        groupBulkWrite.addParam(id_and_mode.first, DynamixelBus::kAddrOperatingMode, 1, param);
      }
    }

    groupBulkWrite.txPacket();
    std::this_thread::sleep_for(50us);
  }

  void BulkWriteSendPosition(const std::vector<std::pair<int, double>>& id_and_position_vector) {
    if (id_and_position_vector.empty())
      return;

    dynamixel::GroupBulkWrite groupBulkWrite(port_handler_, packet_handler_);

    for (auto const& id_and_position : id_and_position_vector) {
      if (id_and_position.first < 0x80) {
        int goal_position = (int)(id_and_position.second * 4096. / 2. / 3.141592);
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
        groupBulkWrite.addParam(id_and_position.first, DynamixelBus::kAddrGoalPosition, 4, param);
      }
    }

    groupBulkWrite.txPacket();
    std::this_thread::sleep_for(50us);
  }

  void BulkWriteSendTorque(const std::vector<std::pair<int, double>>& id_and_torque_vector) {
    if (id_and_torque_vector.empty())
      return;

    dynamixel::GroupBulkWrite groupBulkWrite(port_handler_, packet_handler_);

    int16_t param[1];

    for (auto const& id_and_torque : id_and_torque_vector) {
      if (id_and_torque.first < 0x80) {
        param[0] = (int16_t)(id_and_torque.second / torque_constant_[id_and_torque.first] * 1000. / 2.69);
        groupBulkWrite.addParam(id_and_torque.first, DynamixelBus::kAddrGoalCurrent, 2,
                                reinterpret_cast<uint8_t*>(&param));
      }
    }

    groupBulkWrite.txPacket();
    std::this_thread::sleep_for(50us);
  }

  void SendVibration(int id, int level) {
    if (id > 0x80) {
      packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalVibrationLevel, level);
      std::this_thread::sleep_for(50us);
    }
  }

 private:
  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;

  std::vector<double> torque_constant_;
};

DynamixelBus::DynamixelBus(const std::string& dev_name) : impl_(std::make_unique<DynamixelBusImpl>(dev_name)) {}

DynamixelBus::~DynamixelBus() = default;

void DynamixelBus::SetTorqueConstant(const std::vector<double>& torque_constant) {
  impl_->SetTorqueConstant(torque_constant);
}

bool DynamixelBus::OpenPort() {
  return impl_->OpenPort();
}

bool DynamixelBus::SetBaudRate(int baudrate) {
  return impl_->SetBaudRate(baudrate);
}

bool DynamixelBus::Ping(int id) {
  return impl_->Ping(id);
}

std::optional<std::pair<int, DynamixelBus::ButtonState>> DynamixelBus::ReadButtonStatus(int id) {
  return impl_->ReadButtonStatus(id);
}

void DynamixelBus::SendTorqueEnable(int id, int onoff) {
  impl_->SendTorqueEnable(id, onoff);
}

std::optional<int> DynamixelBus::ReadTorqueEnable(int id) {
  return impl_->ReadTorqueEnable(id);
}

std::optional<double> DynamixelBus::ReadEncoder(int id) {
  return impl_->ReadEncoder(id);
}

void DynamixelBus::SendGoalPosition(int id, int goal_position) {
  impl_->SendGoalPosition(id, goal_position);
}

std::optional<int> DynamixelBus::ReadOperationMode(int id) {
  return impl_->ReadOperationMode(id);
}

bool DynamixelBus::SendOperationMode(int id, int operation_mode) {
  return impl_->SendOperationMode(id, operation_mode);
}

void DynamixelBus::SendTorque(int id, double joint_torque) {
  impl_->SendTorque(id, joint_torque);
}

void DynamixelBus::SendCurrent(int id, double current) {
  impl_->SendCurrent(id, current);
}

std::optional<std::vector<std::pair<int /* id */, double /* enc (rad) */>>> DynamixelBus::BulkReadEncoder(
    const std::vector<int>& ids) {
  return impl_->BulkReadEncoder(ids);
}

std::optional<std::vector<std::pair<int, int>>> DynamixelBus::BulkReadOperationMode(const std::vector<int>& ids) {
  return impl_->BulkReadOperationMode(ids);
}

std::optional<std::vector<std::pair<int, int>>> DynamixelBus::BulkReadTorqueEnable(const std::vector<int>& ids) {
  return impl_->BulkReadTorqueEnable(ids);
}

std::optional<std::vector<std::pair<int, DynamixelBus::MotorState>>> DynamixelBus::BulkReadMotorState(
    const std::vector<int>& ids) {
  return impl_->BulkReadMotorState(ids);
}

void DynamixelBus::BulkWriteTorqueEnable(const std::vector<std::pair<int, int>>& id_and_enable_vector) {
  impl_->BulkWriteTorqueEnable(id_and_enable_vector);
}

void DynamixelBus::BulkWriteTorqueEnable(const std::vector<int>& ids, int enable) {
  impl_->BulkWriteTorqueEnable(ids, enable);
}

void DynamixelBus::BulkWriteOperationMode(const std::vector<std::pair<int, int>>& id_and_mode_vector) {
  impl_->BulkWriteOperationMode(id_and_mode_vector);
}

void DynamixelBus::BulkWriteSendPosition(const std::vector<std::pair<int, double>>& id_and_position_vector) {
  impl_->BulkWriteSendPosition(id_and_position_vector);
}

void DynamixelBus::BulkWriteSendTorque(const std::vector<std::pair<int, double>>& id_and_torque_vector) {
  impl_->BulkWriteSendTorque(id_and_torque_vector);
}

void DynamixelBus::SendVibration(int id, int level) {
  impl_->SendVibration(id, level);
}
}  // namespace rb