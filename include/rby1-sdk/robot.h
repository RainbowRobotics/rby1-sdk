#pragma once

#include "rby1-sdk/version.h"

#include <condition_variable>
#include <functional>
#include <string>

#include "control_manager_state.h"
#include "dynamics/robot.h"
#include "export.h"
#include "log.h"
#include "net/types.h"
#include "robot_command_builder.h"
#include "robot_command_feedback.h"
#include "robot_info.h"
#include "robot_state.h"

namespace rb {

template <typename T>
class Robot;

template <typename T>
class RobotImpl;

template <typename T>
class RobotCommandHandler;

template <typename T>
class RobotCommandHandlerImpl;

template <typename T>
class RobotCommandStreamHandler;

template <typename T>
class RobotCommandStreamHandlerImpl;

template <typename T>
struct ControlInput;

template <typename T>
struct ControlState;

struct PIDGain;

struct Color;

struct SerialDevice;

class SerialStream;

class SerialStreamImpl;

}  // namespace rb

namespace rb {

template <typename T>
class RBY1_SDK_API Robot : public std::enable_shared_from_this<Robot<T>> {
 public:
  using ModelType = T;

  static std::shared_ptr<Robot<T>> Create(std::string address);

  ~Robot();

  std::string GetAddress();

  /**
   * Connects to the robot.
   *
   * @param max_retries Maximum number of retries to connect.
   * @param timeout_ms Timeout in milliseconds for each connection attempt.
   * @param signal_check Optional function to check for a signal (e.g., Ctrl+C).
   *                     If provided, the connection will be aborted if this function returns false.
   * @return True if the connection was successful, false otherwise.
   */
  bool Connect(int max_retries = 5, int timeout_ms = 1000, std::function<bool()> signal_check = nullptr);

  void Disconnect();

  [[nodiscard]] bool IsConnected() const;

  RobotInfo GetRobotInfo() const;

  double GetTimeScale() const;

  double SetTimeScale(double time_scale) const;

  bool PowerOn(const std::string& dev_name) const;

  bool PowerOff(const std::string& dev_name) const;

  bool IsPowerOn(const std::string& dev_name) const;

  bool ServoOn(const std::string& dev_name) const;

  bool IsServoOn(const std::string& dev_name) const;

  bool ServoOff(const std::string& dev_name) const;

  bool SetPositionPGain(const std::string& dev_name, uint16_t p_gain) const;

  bool SetPositionIGain(const std::string& dev_name, uint16_t i_gain) const;

  bool SetPositionDGain(const std::string& dev_name, uint16_t d_gain) const;

  bool SetPositionPIDGain(const std::string& dev_name, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) const;

  bool SetPositionPIDGain(const std::string& dev_name, const rb::PIDGain& pid_gain) const;

  std::vector<rb::PIDGain> GetTorsoPositionPIDGains() const;

  std::vector<rb::PIDGain> GetRightArmPositionPIDGains() const;

  std::vector<rb::PIDGain> GetLeftArmPositionPIDGains() const;

  std::vector<rb::PIDGain> GetHeadPositionPIDGains() const;

  rb::PIDGain GetPositionPIDGain(const std::string& dev_name) const;

  bool BreakEngage(const std::string& dev_name) const;

  bool BreakRelease(const std::string& dev_name) const;

  bool HomeOffsetReset(const std::string& dev_name) const;

  bool SetPresetPosition(const std::string& joint_name) const;

  bool EnableControlManager(bool unlimited_mode_enabled = false) const;

  bool DisableControlManager() const;

  bool ResetFaultControlManager() const;

  bool CancelControl() const;

  bool SetToolFlangeOutputVoltage(const std::string& name, int voltage) const;

  bool SetToolFlangeDigitalOutput(const std::string& name, unsigned int channel, bool state) const;

  bool SetToolFlangeDigitalOutputDual(const std::string& name, bool state_0, bool state_1) const;

  void StartStateUpdate(const std::function<void(const RobotState<T>&)>& cb, double rate);

  void StartStateUpdate(const std::function<void(const RobotState<T>&, const ControlManagerState&)>& cb, double rate);

  void StopStateUpdate();

  void StartLogStream(const std::function<void(const std::vector<Log>&)>& cb, double rate);

  void StopLogStream();

  RobotState<T> GetState() const;

  std::vector<Log> GetLastLog(unsigned int count) const;

  std::vector<std::string> GetFaultLogList() const;

  ControlManagerState GetControlManagerState() const;

  std::unique_ptr<RobotCommandHandler<T>> SendCommand(const RobotCommandBuilder& builder, int priority = 1);

  std::unique_ptr<RobotCommandStreamHandler<T>> CreateCommandStream(int priority = 1);

  bool Control(std::function<ControlInput<T>(const ControlState<T>&)> control, int port = 0, int priority = 1);

  bool ResetOdometry(double angle, const Eigen::Vector<double, 2>& position);

  std::vector<std::pair<std::string, int>> GetParameterList() const;

  bool SetParameter(const std::string& name, const std::string& value, bool write_db = true);

  std::string GetParameter(const std::string& name) const;

  [[deprecated("Use FactoryReset() instead.")]]
  bool ResetParameterToDefault(const std::string& name) const;

  [[deprecated("Use FactoryResetAllParameters() instead.")]]
  void ResetAllParametersToDefault() const;

  bool FactoryResetParameter(const std::string& name) const;

  void FactoryResetAllParameters() const;

  bool ResetParameter(const std::string& name) const;

  void ResetAllParameters() const;

  std::string GetRobotModel() const;

  bool ImportRobotModel(const std::string& name, const std::string& model) const;

  bool SyncTime();

  bool HasEstablishedTimeSync();

  bool StartTimeSync(long period_sec = 10 /* sec */);

  bool StopTimeSync();

  std::shared_ptr<dyn::Robot<T::kRobotDOF>> GetDynamics(const std::string& urdf_model = "");

  bool SetLEDColor(const Color& color, double duration = 1 /* sec */, double transition_time = 0 /* sec */,
                   bool blinking = false, double blinking_freq = 1 /* Hz */);

  std::tuple<struct timespec, std::string, std::string> GetSystemTime() const;

  bool SetSystemTime(struct timespec utc_time, std::optional<std::string> time_zone = std::nullopt) const;

  bool SetBatteryLevel(double level) const;

  bool SetBatteryConfig(double cutoff_voltage, double fully_charged_voltage, const std::array<double, 4>& coefficients);

  bool ResetBatteryConfig() const;

  bool WaitForControlReady(long timeout_ms) const;

  bool ResetNetworkSetting() const;

  std::vector<WifiNetwork> ScanWifi() const;

  bool ConnectWifi(const std::string& ssid, const std::string& password = "", bool use_dhcp = true,
                   const std::string& ip_address = "", const std::string& gateway = "",
                   const std::vector<std::string>& dns = {}) const;

  bool DisconnectWifi() const;

  std::optional<WifiStatus> GetWifiStatus() const;

  std::vector<SerialDevice> GetSerialDeviceList() const;

  std::unique_ptr<SerialStream> OpenSerialStream(const std::string& device_path, int buadrate, int bytesize,
                                                 char parity, int stopbits) const;

  bool DownloadFile(const std::string& path, std::ostream& output) const;

  bool DownloadFileToCallback(const std::string& path, std::function<void(const char*, size_t)> on_chunk) const;

 private:
  explicit Robot(std::string address);

 private:
  std::shared_ptr<RobotImpl<T>> impl_;
};

template <typename T>
class RBY1_SDK_API RobotCommandHandler {
 public:
  ~RobotCommandHandler();

  bool IsDone() const;

  void Wait();

  bool WaitFor(int timeout_ms);

  void Cancel();

  RobotCommandFeedback Get();  // TODO

  bool GetStatus() const;

 private:
  explicit RobotCommandHandler(std::unique_ptr<RobotCommandHandlerImpl<T>> impl);

  std::unique_ptr<RobotCommandHandlerImpl<T>> impl_;

  friend class RobotImpl<T>;
};

template <typename T>
class RBY1_SDK_API RobotCommandStreamHandler {
 public:
  ~RobotCommandStreamHandler();

  bool IsDone() const;

  void Wait();

  bool WaitFor(int timeout_ms);

  RobotCommandFeedback SendCommand(const RobotCommandBuilder& builder, int timeout_ms = 1000);

  RobotCommandFeedback RequestFeedback(int timeout_ms = 1000);

  void Cancel();

 private:
  explicit RobotCommandStreamHandler(std::unique_ptr<RobotCommandStreamHandlerImpl<T>> impl);

  std::unique_ptr<RobotCommandStreamHandlerImpl<T>> impl_;

  friend class RobotImpl<T>;
};

template <typename T>
struct RBY1_SDK_API ControlInput {
  Eigen::Vector<bool, T::kRobotDOF> mode{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  Eigen::Vector<double, T::kRobotDOF> target{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<unsigned int, T::kRobotDOF> feedback_gain{Eigen::Vector<unsigned int, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> feedforward_torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  bool finish{false};
};

template <typename T>
struct RBY1_SDK_API ControlState {
  double t{0.};
  Eigen::Vector<bool, T::kRobotDOF> is_ready{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  Eigen::Vector<double, T::kRobotDOF> position{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> velocity{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> current{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};
};

struct RBY1_SDK_API PIDGain {
  uint16_t p_gain;
  uint16_t i_gain;
  uint16_t d_gain;
};

struct RBY1_SDK_API Color {
  Color() : r(0), g(0), b(0) {}

  Color(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}

  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
};

struct RBY1_SDK_API SerialDevice {
  std::string path;
  std::string description;
};

class RBY1_SDK_API SerialStream {
 public:
  ~SerialStream();

  bool Connect(bool verbose) const;

  void Disconnect() const;

  void Wait() const;

  bool WaitFor(int timeout_ms) const;

  bool IsOpened() const;

  bool IsCancelled() const;

  bool IsDone() const;

  void SetReadCallback(const std::function<void(const std::string&)>& cb);

  bool Write(const std::string& data);

  bool Write(const char* data);

  bool Write(const char* data, int n);

  bool WriteByte(char ch);

 private:
  explicit SerialStream(std::unique_ptr<SerialStreamImpl> impl);

  std::unique_ptr<SerialStreamImpl> impl_;

  template <typename T>
  friend class RobotImpl;
};

}  // namespace rb