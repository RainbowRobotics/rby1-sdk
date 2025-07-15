#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rby1-sdk/export.h"

namespace rb {

class DynamixelBusImpl;

class RBY1_SDK_API DynamixelBus {
 public:
  static constexpr float kProtocolVersion = 2.0;

  static constexpr int kDefaultBaudrate = 2000000;

  static constexpr uint16_t kAddrTorqueEnable = 64;
  static constexpr uint16_t kAddrPresentCurrent = 126;
  static constexpr uint16_t kAddrPresentVelocity = 128;
  static constexpr uint16_t kAddrPresentPosition = 132;
  static constexpr uint16_t kAddrGoalCurrent = 102;
  static constexpr uint16_t kAddrGoalPosition = 116;
  static constexpr uint16_t kAddrOperatingMode = 11;
  static constexpr uint16_t kAddrPresentButtonState = 132;
  static constexpr uint16_t kAddrGoalVibrationLevel = 102;

  static constexpr uint16_t kAddrPositionPGain = 84;
  static constexpr uint16_t kAddrPositionIGain = 82;
  static constexpr uint16_t kAddrPositionDGain = 80;

  static constexpr int kTorqueEnable = 1;
  static constexpr int kTorqueDisable = 0;

  static constexpr int kCurrentControlMode = 0;
  static constexpr int kCurrentBasedPositionControlMode = 5;

  static constexpr int kAddrCurrentTemperature = 146;

  struct ButtonState {  // RB Gripper
    int button;
    int trigger;
  };

  struct MotorState {
    bool torque_enable;

    double position;
    double velocity;
    double current;
    double torque;

    int temperature;
  };

  struct PIDGain {
    uint16_t p_gain;
    uint16_t i_gain;
    uint16_t d_gain;
  };

  explicit DynamixelBus(const std::string& dev_name);

  ~DynamixelBus();

  void SetTorqueConstant(const std::vector<double>& torque_constant);

  bool OpenPort();

  bool SetBaudRate(int baudrate);

  bool Ping(int id);

  std::optional<std::pair<int /* id */, ButtonState>> ReadButtonStatus(int id);

  void SendTorqueEnable(int id, int onoff);

  void SetPositionPGain(int id, uint16_t p_gain);

  void SetPositionIGain(int id, uint16_t i_gain);

  void SetPositionDGain(int id, uint16_t d_gain);

  void SetPositionPIDGain(int id, std::optional<uint16_t> p_gain, std::optional<uint16_t> i_gain,
                          std::optional<uint16_t> d_gain);

  void SetPositionPIDGain(int id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);

  void SetPositionPIDGain(int id, const DynamixelBus::PIDGain& pid_gain);

  std::optional<uint16_t> GetPositionPGain(int id);

  std::optional<uint16_t> GetPositionIGain(int id);

  std::optional<uint16_t> GetPositionDGain(int id);

  std::optional<DynamixelBus::PIDGain> GetPositionPIDGain(int id);

  std::optional<int> ReadTorqueEnable(int id);

  std::optional<double> ReadEncoder(int id);

  void SendGoalPosition(int id, int goal_position);

  std::optional<int> ReadOperatingMode(int id, bool use_cache = false);

  bool SendOperatingMode(int id, int mode);

  void SendTorque(int id, double joint_torque);

  void SendCurrent(int id, double current);

  std::optional<int> ReadTemperature(int id);

  std::optional<std::vector<std::pair<int, int>>> GroupFastSyncRead(const std::vector<int>& ids, int addr, int len);

  std::optional<std::vector<std::pair<int /* id */, double /* enc (rad) */>>> GroupFastSyncReadEncoder(
      const std::vector<int>& ids);

  std::optional<std::vector<std::pair<int, int>>> GroupFastSyncReadOperatingMode(const std::vector<int>& ids,
                                                                                 bool use_cache = false);

  std::optional<std::vector<std::pair<int, int>>> GroupFastSyncReadTorqueEnable(const std::vector<int>& ids);

  //
  std::optional<std::vector<std::pair<int, MotorState>>> GetMotorStates(const std::vector<int>& ids);

  void GroupSyncWriteTorqueEnable(const std::vector<std::pair<int, int>>& id_and_eanble_vector);

  void GroupSyncWriteTorqueEnable(const std::vector<int>& ids, int enable);

  void GroupSyncWriteOperatingMode(const std::vector<std::pair<int, int>>& id_and_mode_vector);

  void GroupSyncWriteSendPosition(const std::vector<std::pair<int, double>>& id_and_position_vector);

  void GroupSyncWriteSendTorque(const std::vector<std::pair<int, double>>& id_and_torque_vector);

  void SendVibration(int id, int level);

 private:
  std::unique_ptr<DynamixelBusImpl> impl_;
};

}  // namespace rb