#include "rby1-sdk/base/dynamixel_bus.h"

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif
#include "dynamixel_sdk.h"

#include <chrono>
#include <iostream>
#include <mutex>
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
    port_handler_->clearPort();

    delete port_handler_;
  }

  void SetTorqueConstant(const std::vector<double>& torque_constant) { torque_constant_ = torque_constant; }

  bool OpenPort() { return port_handler_->openPort(); }

  bool SetBaudRate(int baudrate) { return port_handler_->setBaudRate(baudrate); }

  bool Ping(int id) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->ping(port_handler_, id, &dxl_error);
    std::this_thread::sleep_for(100us);
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
    std::this_thread::sleep_for(100us);
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

  void SetPositionGain(int id, std::optional<uint16_t> p_gain = std::nullopt,
                       std::optional<uint16_t> i_gain = std::nullopt, std::optional<uint16_t> d_gain = std::nullopt) {
    if (!p_gain.has_value() && !i_gain.has_value() && !d_gain.has_value()) {
      return;
    }

    if (p_gain.has_value()) {
      packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrPositionPGain, p_gain.value());
      std::this_thread::sleep_for(100us);
    }
    if (i_gain.has_value()) {
      packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrPositionIGain, i_gain.value());
      std::this_thread::sleep_for(100us);
    }
    if (d_gain.has_value()) {
      packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrPositionDGain, d_gain.value());
      std::this_thread::sleep_for(100us);
    }
  }

  void SetPositionGain(int id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
    uint8_t params[6];

    params[0] = DXL_LOBYTE(d_gain);
    params[1] = DXL_HIBYTE(d_gain);
    params[2] = DXL_LOBYTE(i_gain);
    params[3] = DXL_HIBYTE(i_gain);
    params[4] = DXL_LOBYTE(p_gain);
    params[5] = DXL_HIBYTE(p_gain);

    packet_handler_->writeTxOnly(port_handler_, id, DynamixelBus::kAddrPositionDGain, 6, (uint8_t*)&params[0]);
    std::this_thread::sleep_for(100us);
  }

  std::tuple<std::optional<uint16_t>, std::optional<uint16_t>, std::optional<uint16_t>> GetPositionGain(int id) {
    std::optional<uint16_t> p_gain;
    std::optional<uint16_t> i_gain;
    std::optional<uint16_t> d_gain;

    uint8_t values[3 * 2];
    uint8_t dxl_error;

    int result =
        packet_handler_->readTxRx(port_handler_, id, DynamixelBus::kAddrPositionDGain, 6, &values[0], &dxl_error);
    std::this_thread::sleep_for(100us);
    if (result == COMM_SUCCESS) {
      p_gain = DXL_MAKEWORD(values[4], values[5]);
      i_gain = DXL_MAKEWORD(values[2], values[3]);
      d_gain = DXL_MAKEWORD(values[0], values[1]);
    }

    return std::make_tuple(p_gain, i_gain, d_gain);
  }

  std::optional<int> ReadTorqueEnable(int id) {
    int8_t onoff = -1;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, DynamixelBus::kAddrTorqueEnable,
                                                         (uint8_t*)&onoff, &dxl_error);
    std::this_thread::sleep_for(100us);
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
    std::this_thread::sleep_for(100us);
    if (dxl_comm_result == COMM_SUCCESS) {
      return (double)position / 4096. * 2. * 3.141592;  // (rad)
    } else {
      return {};
    }
  }

  void SendGoalPosition(int id, int goal_position) {
    packet_handler_->write4ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalPosition, goal_position);
    std::this_thread::sleep_for(100us);
  }

  std::optional<int> ReadOperatingMode(int id, bool use_cache) {
    if (use_cache) {
      if (operating_modes_.find(id) != operating_modes_.end()) {
        return operating_modes_[id];
      }
    }

    int8_t mode = -1;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, DynamixelBus::kAddrOperatingMode,
                                                         (uint8_t*)&mode, &dxl_error);
    std::this_thread::sleep_for(100us);
    if (dxl_comm_result == COMM_SUCCESS) {
      operating_modes_[id] = mode;
      return mode;
    } else {
      return {};
    }
  }

  bool SendOperatingMode(int id, int mode) {
    int dxl_comm_result = packet_handler_->write1ByteTxOnly(port_handler_, id, DynamixelBus::kAddrOperatingMode, mode);
    std::this_thread::sleep_for(100us);
    if (dxl_comm_result == COMM_SUCCESS) {
      operating_modes_[id] = mode;
      return true;
    }
    return false;
  }

  void SendTorque(int id, double joint_torque) {
    if (id >= torque_constant_.size())
      return;

    auto torque_value = (int32_t)(joint_torque / torque_constant_[id] * 1000. / 2.69);
    packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalCurrent, torque_value);
    std::this_thread::sleep_for(100us);
  }

  void SendCurrent(int id, double current /* (Amp) */) {
    auto current_value = (int16_t)(current / 2.69 * 1000.);
    packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalCurrent, current_value);
    std::this_thread::sleep_for(100us);
  }

  std::optional<int> ReadTemperature(int id) {
    uint8_t dxl_temperature = 0;  // Read as uint8_t
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, DynamixelBus::kAddrCurrentTemperature,
                                                         (uint8_t*)&dxl_temperature, &dxl_error);
    std::this_thread::sleep_for(100us);
    if (dxl_comm_result == COMM_SUCCESS) {
      return static_cast<int>(dxl_temperature);
    } else {
      return std::nullopt;
    }
  }

  std::optional<std::vector<std::pair<int, int>>> GroupFastSyncRead(const std::vector<int>& ids, int addr, int len) {
    std::vector<std::pair<int, int>> rv;
    dynamixel::GroupFastSyncRead group_fast_sync_read(port_handler_, packet_handler_, addr, len);

    for (auto const& id : ids) {
      if (id < 0x80) {
        group_fast_sync_read.addParam(id);
      }
    }

    group_fast_sync_read.txRxPacket();

    for (auto const& id : ids) {
      if (id < 0x80) {
        if (group_fast_sync_read.isAvailable(id, addr, len)) {
          auto data = (int)group_fast_sync_read.getData(id, addr, len);
          rv.emplace_back(id, data);
        }
      }
    }

    std::this_thread::sleep_for(100us);

    if (rv.empty()) {
      return {};
    } else {
      return rv;
    }
  }

  std::optional<std::vector<std::pair<int /* id */, double /* enc (rad) */>>> GroupFastSyncReadEncoder(
      const std::vector<int>& ids) {
    std::vector<std::pair<int, double>> v;

    const auto& rv = GroupFastSyncRead(ids, DynamixelBus::kAddrPresentPosition, 4);
    std::this_thread::sleep_for(100us);
    if (rv.has_value()) {
      for (const auto& r : rv.value()) {
        v.emplace_back(r.first, (double)r.second / 4096. * 2. * 3.141592);
      }
      return v;
    } else {
      return {};
    }
  }

  std::optional<std::vector<std::pair<int, int>>> GroupFastSyncReadOperatingMode(const std::vector<int>& ids,
                                                                                 bool use_cache) {
    std::vector<std::pair<int, int>> operating_mode_vector;
    dynamixel::GroupFastSyncRead group_fast_sync_read(port_handler_, packet_handler_, DynamixelBus::kAddrOperatingMode,
                                                      1);
    for (auto const& id : ids) {
      if (id < 0x80) {
        if (use_cache && operating_modes_.find(id) != operating_modes_.end()) {
          operating_mode_vector.emplace_back(id, operating_modes_[id]);
          continue;
        }
        group_fast_sync_read.addParam(id);
      }
    }

    group_fast_sync_read.txRxPacket();

    for (auto const& id : ids) {
      if (id < 0x80) {
        if (group_fast_sync_read.isAvailable(id, DynamixelBus::kAddrOperatingMode, 1)) {
          auto mode = (int)group_fast_sync_read.getData(id, DynamixelBus::kAddrOperatingMode, 1);
          operating_modes_[id] = mode;
          operating_mode_vector.emplace_back(id, mode);
        }
      }
    }

    std::this_thread::sleep_for(100us);

    if (operating_mode_vector.empty()) {
      return {};
    } else {
      return operating_mode_vector;
    }
  }

  std::optional<std::vector<std::pair<int, int>>> GroupFastSyncReadTorqueEnable(const std::vector<int>& ids) {
    std::vector<std::pair<int, int>> torque_enable_vector;
    dynamixel::GroupFastSyncRead group_fast_sync_read(port_handler_, packet_handler_, DynamixelBus::kAddrTorqueEnable,
                                                      1);

    for (auto const& id : ids) {
      if (id < 0x80) {
        group_fast_sync_read.addParam(id);
      }
    }

    group_fast_sync_read.txRxPacket();

    for (auto const& id : ids) {
      if (id < 0x80) {
        if (group_fast_sync_read.isAvailable(id, DynamixelBus::kAddrTorqueEnable, 1)) {
          auto operation_mode = (int)group_fast_sync_read.getData(id, DynamixelBus::kAddrTorqueEnable, 1);
          torque_enable_vector.emplace_back(id, operation_mode);
        }
      }
    }

    std::this_thread::sleep_for(100us);

    if (torque_enable_vector.empty()) {
      return {};
    } else {
      return torque_enable_vector;
    }
  }

  std::optional<std::vector<std::pair<int, DynamixelBus::MotorState>>> GetMotorStates(const std::vector<int>& ids) {
    std::vector<std::pair<int, DynamixelBus::MotorState>> ms;
    std::unordered_map<int, int> idx;
    for (const auto& id : ids) {
      ms.emplace_back(id, DynamixelBus::MotorState());
      idx[id] = (int)ms.size() - 1;
    }

    {
      const auto& rv = GroupFastSyncRead(ids, DynamixelBus::kAddrTorqueEnable, 1);
      if (rv.has_value()) {
        const auto& torques = rv.value();
        if (torques.size() != ids.size()) {
          return {};
        }
        for (const auto& r : torques) {
          ms[idx[r.first]].second.torque_enable = (r.second == 1);
        }
      } else {
        return {};
      }
    }

    // DynamixelBus::kAddrPresentCurrent, DynamixelBus::kAddrPresentVelocity, DynamixelBus::kAddrPresentPosition, ..., DynamixelBus::kAddrCurrentTemperature
    {
      std::vector<std::pair<int, int>> rv;
      dynamixel::GroupFastSyncRead group_fast_sync_read(port_handler_, packet_handler_,
                                                        DynamixelBus::kAddrPresentCurrent, 2 + 4 + 4 + (4 * 2 + 2) + 1);

      for (auto const& id : ids) {
        group_fast_sync_read.addParam(id);
      }

      group_fast_sync_read.txRxPacket();

      for (auto const& id : ids) {
        auto& m = ms[idx[id]].second;

        if (group_fast_sync_read.isAvailable(id, DynamixelBus::kAddrPresentCurrent, 2)) {
          auto data = (int16_t)group_fast_sync_read.getData(id, DynamixelBus::kAddrPresentCurrent, 2);
          m.current = (double)data / 1000. * 2.69;
        } else {
          return {};
        }

        if (group_fast_sync_read.isAvailable(id, DynamixelBus::kAddrPresentVelocity, 4)) {
          auto data = (int)group_fast_sync_read.getData(id, DynamixelBus::kAddrPresentVelocity, 4);
          m.velocity = (double)data * 0.229 * 2 * 3.141592 / 60;
        } else {
          return {};
        }

        if (group_fast_sync_read.isAvailable(id, DynamixelBus::kAddrPresentPosition, 4)) {
          auto data = (int)group_fast_sync_read.getData(id, DynamixelBus::kAddrPresentPosition, 4);
          m.position = (double)data / 4096. * 2. * 3.141592;
        } else {
          return {};
        }

        if (group_fast_sync_read.isAvailable(id, DynamixelBus::kAddrCurrentTemperature, 1)) {
          auto data = (int)group_fast_sync_read.getData(id, DynamixelBus::kAddrCurrentTemperature, 1);
          m.temperature = data;
        }
      }

      std::this_thread::sleep_for(100us);
    }

    return ms;
  }

  void GroupSyncWriteTorqueEnable(const std::vector<std::pair<int, int>>& id_and_enable_vector) {
    if (id_and_enable_vector.empty())
      return;

    dynamixel::GroupSyncWrite group_sync_write(port_handler_, packet_handler_, DynamixelBus::kAddrTorqueEnable, 1);

    uint8_t param[1];

    for (auto const& id_and_enable : id_and_enable_vector) {
      if (id_and_enable.first < 0x80) {
        param[0] = id_and_enable.second;
        group_sync_write.addParam(id_and_enable.first, param);
      }
    }

    group_sync_write.txPacket();

    std::this_thread::sleep_for(100us);
  }

  void GroupSyncWriteTorqueEnable(const std::vector<int>& ids, int enable) {
    if (ids.empty())
      return;

    dynamixel::GroupSyncWrite group_sync_write(port_handler_, packet_handler_, DynamixelBus::kAddrTorqueEnable, 1);

    uint8_t param[1];

    for (auto const& id : ids) {
      if (id < 0x80) {
        param[0] = enable;
        group_sync_write.addParam(id, param);
      }
    }

    group_sync_write.txPacket();

    std::this_thread::sleep_for(100us);
  }

  void GroupSyncWriteOperatingMode(const std::vector<std::pair<int, int>>& id_and_mode_vector) {
    if (id_and_mode_vector.empty()) {
      return;
    }

    dynamixel::GroupSyncWrite group_sync_write(port_handler_, packet_handler_, DynamixelBus::kAddrOperatingMode, 1);

    uint8_t param[1];

    for (auto const& id_and_mode : id_and_mode_vector) {
      if (id_and_mode.first < 0x80) {
        param[0] = id_and_mode.second;
        group_sync_write.addParam(id_and_mode.first, param);
      }
    }

    int result = group_sync_write.txPacket();

    std::this_thread::sleep_for(100us);

    if (result == COMM_SUCCESS) {
      for (auto const& id_and_mode : id_and_mode_vector) {
        if (id_and_mode.first < 0x80) {
          operating_modes_[id_and_mode.first] = id_and_mode.second;
        }
      }
    }
  }

  void GroupSyncWriteSendPosition(const std::vector<std::pair<int, double>>& id_and_position_vector) {
    if (id_and_position_vector.empty())
      return;

    dynamixel::GroupSyncWrite group_sync_write(port_handler_, packet_handler_, DynamixelBus::kAddrGoalPosition, 4);

    for (auto const& id_and_position : id_and_position_vector) {
      if (id_and_position.first < 0x80) {
        int goal_position = (int)(id_and_position.second * 4096. / 2. / 3.141592);
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
        param[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
        param[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
        param[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));
        group_sync_write.addParam(id_and_position.first, param);
      }
    }

    group_sync_write.txPacket();

    std::this_thread::sleep_for(100us);
  }

  void GroupSyncWriteSendTorque(const std::vector<std::pair<int, double>>& id_and_torque_vector) {
    if (id_and_torque_vector.empty())
      return;

    dynamixel::GroupSyncWrite group_sync_write(port_handler_, packet_handler_, DynamixelBus::kAddrGoalCurrent, 2);

    int16_t param[1];

    for (auto const& id_and_torque : id_and_torque_vector) {
      if (id_and_torque.first < 0x80) {
        param[0] = (int16_t)(id_and_torque.second / torque_constant_[id_and_torque.first] * 1000. / 2.69);
        group_sync_write.addParam(id_and_torque.first, reinterpret_cast<uint8_t*>(&param));
      }
    }

    group_sync_write.txPacket();

    std::this_thread::sleep_for(100us);
  }

  void SendVibration(int id, int level) {
    if (id > 0x80) {
      packet_handler_->write2ByteTxOnly(port_handler_, id, DynamixelBus::kAddrGoalVibrationLevel, level);

      std::this_thread::sleep_for(100us);
    }
  }

 private:
  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;

  std::unordered_map<int, int> operating_modes_;
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

void DynamixelBus::SetPositionPGain(int id, uint16_t p_gain) {
  impl_->SetPositionGain(id, p_gain, std::nullopt, std::nullopt);
}

void DynamixelBus::SetPositionIGain(int id, uint16_t i_gain) {
  impl_->SetPositionGain(id, std::nullopt, i_gain, std::nullopt);
}

void DynamixelBus::SetPositionDGain(int id, uint16_t d_gain) {
  impl_->SetPositionGain(id, std::nullopt, std::nullopt, d_gain);
}

void DynamixelBus::SetPositionPIDGain(int id, std::optional<uint16_t> p_gain, std::optional<uint16_t> i_gain,
                                      std::optional<uint16_t> d_gain) {
  impl_->SetPositionGain(id, p_gain, i_gain, d_gain);
}

void DynamixelBus::SetPositionPIDGain(int id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
  impl_->SetPositionGain(id, p_gain, i_gain, d_gain);
}

void DynamixelBus::SetPositionPIDGain(int id, const DynamixelBus::PIDGain& pid_gain) {
  impl_->SetPositionGain(id, pid_gain.p_gain, pid_gain.i_gain, pid_gain.d_gain);
}

std::optional<uint16_t> DynamixelBus::GetPositionPGain(int id) {
  auto [p_gain, i_gain, d_gain] = impl_->GetPositionGain(id);
  return p_gain;
}

std::optional<uint16_t> DynamixelBus::GetPositionIGain(int id) {
  auto [p_gain, i_gain, d_gain] = impl_->GetPositionGain(id);
  return i_gain;
}

std::optional<uint16_t> DynamixelBus::GetPositionDGain(int id) {
  auto [p_gain, i_gain, d_gain] = impl_->GetPositionGain(id);
  return d_gain;
}

std::optional<DynamixelBus::PIDGain> DynamixelBus::GetPositionPIDGain(int id) {
  auto [p_gain, i_gain, d_gain] = impl_->GetPositionGain(id);
  if (!p_gain.has_value() || !i_gain.has_value() || !d_gain.has_value()) {
    return std::nullopt;
  }
  DynamixelBus::PIDGain pid;
  pid.p_gain = p_gain.value();
  pid.i_gain = i_gain.value();
  pid.d_gain = d_gain.value();
  return pid;
}

// std::tuple<std::optional<uint16_t>, std::optional<uint16_t>, std::optional<uint16_t>> DynamixelBus::GetPositionGain(
//     int id) {
//   return impl_->GetPositionGain(id);
// }

std::optional<int> DynamixelBus::ReadTorqueEnable(int id) {
  return impl_->ReadTorqueEnable(id);
}

std::optional<double> DynamixelBus::ReadEncoder(int id) {
  return impl_->ReadEncoder(id);
}

void DynamixelBus::SendGoalPosition(int id, int goal_position) {
  impl_->SendGoalPosition(id, goal_position);
}

std::optional<int> DynamixelBus::ReadOperatingMode(int id, bool use_cache) {
  return impl_->ReadOperatingMode(id, use_cache);
}

bool DynamixelBus::SendOperatingMode(int id, int operation_mode) {
  return impl_->SendOperatingMode(id, operation_mode);
}

void DynamixelBus::SendTorque(int id, double joint_torque) {
  impl_->SendTorque(id, joint_torque);
}

void DynamixelBus::SendCurrent(int id, double current) {
  impl_->SendCurrent(id, current);
}

std::optional<int> DynamixelBus::ReadTemperature(int id) {
  return impl_->ReadTemperature(id);
}

std::optional<std::vector<std::pair<int, int>>> DynamixelBus::GroupFastSyncRead(const std::vector<int>& ids, int addr,
                                                                                int len) {
  return impl_->GroupFastSyncRead(ids, addr, len);
}

std::optional<std::vector<std::pair<int /* id */, double /* enc (rad) */>>> DynamixelBus::GroupFastSyncReadEncoder(
    const std::vector<int>& ids) {
  return impl_->GroupFastSyncReadEncoder(ids);
}

std::optional<std::vector<std::pair<int, int>>> DynamixelBus::GroupFastSyncReadOperatingMode(
    const std::vector<int>& ids, bool use_cache) {
  return impl_->GroupFastSyncReadOperatingMode(ids, use_cache);
}

std::optional<std::vector<std::pair<int, int>>> DynamixelBus::GroupFastSyncReadTorqueEnable(
    const std::vector<int>& ids) {
  return impl_->GroupFastSyncReadTorqueEnable(ids);
}

std::optional<std::vector<std::pair<int, DynamixelBus::MotorState>>> DynamixelBus::GetMotorStates(
    const std::vector<int>& ids) {
  return impl_->GetMotorStates(ids);
}

void DynamixelBus::GroupSyncWriteTorqueEnable(const std::vector<std::pair<int, int>>& id_and_enable_vector) {
  impl_->GroupSyncWriteTorqueEnable(id_and_enable_vector);
}

void DynamixelBus::GroupSyncWriteTorqueEnable(const std::vector<int>& ids, int enable) {
  impl_->GroupSyncWriteTorqueEnable(ids, enable);
}

void DynamixelBus::GroupSyncWriteOperatingMode(const std::vector<std::pair<int, int>>& id_and_mode_vector) {
  impl_->GroupSyncWriteOperatingMode(id_and_mode_vector);
}

void DynamixelBus::GroupSyncWriteSendPosition(const std::vector<std::pair<int, double>>& id_and_position_vector) {
  impl_->GroupSyncWriteSendPosition(id_and_position_vector);
}

void DynamixelBus::GroupSyncWriteSendTorque(const std::vector<std::pair<int, double>>& id_and_torque_vector) {
  impl_->GroupSyncWriteSendTorque(id_and_torque_vector);
}

void DynamixelBus::SendVibration(int id, int level) {
  impl_->SendVibration(id, level);
}
}  // namespace rb