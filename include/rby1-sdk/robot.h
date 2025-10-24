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

/**
 * @brief Robot control interface.
 * 
 * Provides high-level control interface for robot operations including
 * connection management, power control, and command execution.
 * 
 * @tparam T Robot model type
 */
template <typename T>
class RBY1_SDK_API Robot : public std::enable_shared_from_this<Robot<T>> {
 public:
  using ModelType = T;

  /**
   * @brief Create a robot instance.
   * 
   * @param address Robot network address, e.g. "192.168.1.100:50051".
   * @return std::shared_ptr<Robot<T>> Robot instance.
   */
  static std::shared_ptr<Robot<T>> Create(std::string address);

  ~Robot();

  /**
   * @brief Get the robot's network address.
   * 
   * @return std::string The robot's address.
   */
  std::string GetAddress();

  /**
   * @brief Attempts to establish a connection with the robot.
   *
   * @param max_retries Maximum number of retries to connect (default: 5).
   * @param timeout_ms Timeout in milliseconds for each connection attempt (default: 1000).
   * @param signal_check Optional function to check for a signal (e.g., Ctrl+C).
   *                     If provided, the connection will be aborted if this function returns false.
   * @return True if the connection was successful, false otherwise.
   */
  bool Connect(int max_retries = 5, int timeout_ms = 1000, std::function<bool()> signal_check = nullptr);

  /**
   * @brief Disconnect from the robot.
   */
  void Disconnect();

  /**
   * @brief Checks whether the robot is currently connected.
   * 
   * @return True if the robot is connected, false otherwise.
   */
  [[nodiscard]] bool IsConnected() const;

  /**
   * @brief Retrieves static information about the robot.
   * 
   * Gets structured metadata including joint details, device names, robot model information,
   * SDK version, and joint configuration.
   * 
   * @return RobotInfo Robot information structure.
   */
  RobotInfo GetRobotInfo() const;

  /**
   * @brief Get the current time scale factor.
   * 
   * @return double Current time scale factor.
   */
  double GetTimeScale() const;

  /**
   * @brief Set the time scale for motion execution.
   * 
   * @param time_scale Time scale value (0.0 to 1.0). 0.0 stops motion, 1.0 is full speed.
   * @return double The set time scale factor.
   */
  double SetTimeScale(double time_scale) const;

  /**
   * @brief Power on a device.
   * 
   * @param dev_name Device name to power on. Supports regex patterns (e.g., ".*" for all devices).
   * @return True if successful, false otherwise.
   */
  bool PowerOn(const std::string& dev_name) const;

  /**
   * @brief Power off a device.
   * 
   * @param dev_name Device name to power off. Supports regex patterns (e.g., ".*" for all devices).
   * @return True if successful, false otherwise.
   */
  bool PowerOff(const std::string& dev_name) const;

  /**
   * @brief Check if a device is powered on.
   * 
   * @param dev_name Device name to check. Supports regex patterns.
   * @return True if device is powered on, false otherwise.
   */
  bool IsPowerOn(const std::string& dev_name) const;

  /**
   * @brief Enable servo control for a device.
   * 
   * @param dev_name Device name to enable servo control. Supports regex patterns (e.g., ".*" for all devices).
   * @return True if successful, false otherwise.
   */
  bool ServoOn(const std::string& dev_name) const;

  /**
   * @brief Check if servo control is enabled for a device.
   * 
   * @param dev_name Device name to check. Supports regex patterns.
   * @return True if servo control is enabled, false otherwise.
   */
  bool IsServoOn(const std::string& dev_name) const;

  /**
   * @brief Disable servo control for a device.
   * 
   * @param dev_name Device name to disable servo control. Supports regex patterns.
   * @return True if successful, false otherwise.
   */
  bool ServoOff(const std::string& dev_name) const;

  /**
   * @brief Set position proportional gain for a device.
   * 
   * @param dev_name Device name. Supports regex patterns.
   * @param p_gain Proportional gain value.
   * @return True if successful, false otherwise.
   */
  bool SetPositionPGain(const std::string& dev_name, uint16_t p_gain) const;

  /**
   * @brief Set position integral gain for a device.
   * 
   * @param dev_name Device name. Supports regex patterns.
   * @param i_gain Integral gain value.
   * @return True if successful, false otherwise.
   */
  bool SetPositionIGain(const std::string& dev_name, uint16_t i_gain) const;

  /**
   * @brief Set position derivative gain for a device.
   * 
   * @param dev_name Device name. Supports regex patterns.
   * @param d_gain Derivative gain value.
   * @return True if successful, false otherwise.
   */
  bool SetPositionDGain(const std::string& dev_name, uint16_t d_gain) const;

  /**
   * @brief Set position PID gains for a device.
   * 
   * @param dev_name Device name. Supports regex patterns.
   * @param p_gain Proportional gain value.
   * @param i_gain Integral gain value.
   * @param d_gain Derivative gain value.
   * @return True if successful, false otherwise.
   */
  bool SetPositionPIDGain(const std::string& dev_name, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) const;

  /**
   * @brief Set position PID gains for a device.
   * 
   * @param dev_name Device name. Supports regex patterns.
   * @param pid_gain PID gain structure.
   * @return True if successful, false otherwise.
   */
  bool SetPositionPIDGain(const std::string& dev_name, const rb::PIDGain& pid_gain) const;

  /**
   * @brief Get torso position PID gains.
   * 
   * @return std::vector<rb::PIDGain> Vector of PID gains for torso joints.
   */
  std::vector<rb::PIDGain> GetTorsoPositionPIDGains() const;

  /**
   * @brief Get right arm position PID gains.
   * 
   * @return std::vector<rb::PIDGain> Vector of PID gains for right arm joints.
   */
  std::vector<rb::PIDGain> GetRightArmPositionPIDGains() const;

  /**
   * @brief Get left arm position PID gains.
   * 
   * @return std::vector<rb::PIDGain> Vector of PID gains for left arm joints.
   */
  std::vector<rb::PIDGain> GetLeftArmPositionPIDGains() const;

  /**
   * @brief Get head position PID gains.
   * 
   * @return std::vector<rb::PIDGain> Vector of PID gains for head joints.
   */
  std::vector<rb::PIDGain> GetHeadPositionPIDGains() const;

  /**
   * @brief Get position PID gain for a specific device.
   * 
   * @param dev_name Device name.
   * @return rb::PIDGain PID gain for the specified device.
   */
  rb::PIDGain GetPositionPIDGain(const std::string& dev_name) const;

  /**
   * @brief Engage the brake for a device.
   * 
   * @param dev_name Device name (joint name) to engage brake. Supports regex patterns.
   * @return True if successful, false otherwise.
   */
  bool BreakEngage(const std::string& dev_name) const;

  /**
   * @brief Release the brake for a device.
   * 
   * @param dev_name Device name (joint name) to release brake. Supports regex patterns.
   * @return True if successful, false otherwise.
   */
  bool BreakRelease(const std::string& dev_name) const;

  /**
   * @brief Reset home offset for a device.
   * 
   * @param dev_name Device name to reset home offset. Supports regex patterns.
   * @return True if successful, false otherwise.
   */
  bool HomeOffsetReset(const std::string& dev_name) const;

  /**
   * @brief Set preset position for a joint (only available for PVL-based motors).
   * 
   * @param joint_name Joint name to set preset position.
   * @return True if successful, false otherwise.
   */
  bool SetPresetPosition(const std::string& joint_name) const;

  /**
   * @brief Enable the control manager.
   * 
   * Must be called before sending commands to the robot.
   * 
   * @param unlimited_mode_enabled Whether to enable unlimited mode (default: false).
   * @return True if successful, false otherwise.
   */
  bool EnableControlManager(bool unlimited_mode_enabled = false) const;

  /**
   * @brief Disable the control manager.
   * 
   * @return True if successful, false otherwise.
   */
  bool DisableControlManager() const;

  /**
   * @brief Reset fault in the control manager.
   * 
   * @return True if successful, false otherwise.
   */
  bool ResetFaultControlManager() const;

  /**
   * @brief Cancel current control operation.
   * 
   * @return True if successful, false otherwise.
   */
  bool CancelControl() const;

  /**
   * @brief Set tool flange output voltage.
   * 
   * @param name Tool flange name ("right" or "left").
   * @param voltage Output voltage [V]. Common voltages: 12V, 24V. Use 0V to turn off output.
   * @return True if successful, false otherwise.
   */
  bool SetToolFlangeOutputVoltage(const std::string& name, int voltage) const;

  /**
   * @brief Set the digital output state of a specific channel on the tool flange.
   * 
   * @param name Arm identifier ("right" or "left").
   * @param channel Digital output channel index.
   * @param state Desired state of the digital output (true = ON, false = OFF).
   * @return True if the command was successfully sent, false otherwise.
   */
  bool SetToolFlangeDigitalOutput(const std::string& name, unsigned int channel, bool state) const;

  /**
   * @brief Set the digital output states of two channels on the tool flange simultaneously.
   * 
   * @param name Arm identifier ("right" or "left").
   * @param state_0 Desired state for digital output channel 0 (true = ON, false = OFF).
   * @param state_1 Desired state for digital output channel 1 (true = ON, false = OFF).
   * @return True if the command was successfully sent, false otherwise.
   */
  bool SetToolFlangeDigitalOutputDual(const std::string& name, bool state_0, bool state_1) const;

  /**
   * @brief Start state update callback.
   * 
   * @param cb Callback function for robot state updates.
   * @param rate Update rate [Hz].
   */
  void StartStateUpdate(const std::function<void(const RobotState<T>&)>& cb, double rate);

  /**
   * @brief Start state update callback with control manager state.
   * 
   * @param cb Callback function for robot state and control manager state updates.
   * @param rate Update rate [Hz].
   */
  void StartStateUpdate(const std::function<void(const RobotState<T>&, const ControlManagerState&)>& cb, double rate);

  /**
   * @brief Stop state update callback.
   */
  void StopStateUpdate();

  /**
   * @brief Start log stream callback.
   * 
   * @param cb Callback function for log updates.
   * @param rate Update rate [Hz].
   */
  void StartLogStream(const std::function<void(const std::vector<Log>&)>& cb, double rate);

  /**
   * @brief Stop log stream callback.
   */
  void StopLogStream();

  /**
   * @brief Get current robot state.
   * 
   * @return RobotState<T> Current robot state.
   */
  RobotState<T> GetState() const;

  /**
   * @brief Get last log entries.
   * 
   * @param count Number of log entries to retrieve.
   * @return std::vector<Log> List of log entries.
   */
  std::vector<Log> GetLastLog(unsigned int count) const;

  /**
   * @brief Get fault log list.
   * 
   * @return std::vector<std::string> List of fault log entries.
   */
  std::vector<std::string> GetFaultLogList() const;

  /**
   * @brief Get control manager state.
   * 
   * @return ControlManagerState Current control manager state.
   */
  ControlManagerState GetControlManagerState() const;

  /**
   * @brief Send a command to the robot.
   * 
   * @param builder Command builder to send.
   * @param priority Command priority (default: 1).
   * @return std::unique_ptr<RobotCommandHandler<T>> Command handler for monitoring execution.
   */
  std::unique_ptr<RobotCommandHandler<T>> SendCommand(const RobotCommandBuilder& builder, int priority = 1);

  /**
   * @brief Create a command stream for continuous command sending.
   * 
   * @param priority Command priority (default: 1).
   * @return std::unique_ptr<RobotCommandStreamHandler<T>> Command stream handler.
   */
  std::unique_ptr<RobotCommandStreamHandler<T>> CreateCommandStream(int priority = 1);

  /**
   * @brief Start a blocking real-time control loop using a custom control callback.
   * 
   * This function runs a UDP-based real-time loop that repeatedly calls the user-provided
   * control callback. The loop exits when the callback returns a ControlInput with finish=true,
   * or when an error/abort occurs.
   * 
   * @param control A callable that accepts a ControlState and returns a ControlInput.
   * @param port UDP port to bind for the RT server. Use 0 to let the OS choose an available port (default: 0).
   * @param priority Command priority sent with the RT control request (default: 1).
   * @return True if the loop terminated because the callback set finish=true in ControlInput,
   *         false if the loop ended due to an error or abort.
   */
  bool Control(std::function<ControlInput<T>(const ControlState<T>&)> control, int port = 0, int priority = 1);

  /**
   * @brief Reset odometry to specified values.
   * 
   * @param angle New angle [rad].
   * @param position New position [m]. Position is [x, y] in meters.
   * @return True if successful, false otherwise.
   */
  bool ResetOdometry(double angle, const Eigen::Vector<double, 2>& position);

  /**
   * @brief Get list of available parameters.
   * 
   * @return std::vector<std::pair<std::string, int>> List of parameter names and their types.
   */
  std::vector<std::pair<std::string, int>> GetParameterList() const;

  /**
   * @brief Set a robot parameter.
   * 
   * @param name Parameter name.
   * @param value Parameter value.
   * @param write_db Whether to write to database (default: true).
   * @return True if successful, false if matching parameter not found.
   */
  bool SetParameter(const std::string& name, const std::string& value, bool write_db = true);

  /**
   * @brief Get a robot parameter value.
   * 
   * @param name Parameter name.
   * @return std::string Parameter value.
   */
  std::string GetParameter(const std::string& name) const;

  [[deprecated("Use FactoryResetParameter() instead.")]]
  bool ResetParameterToDefault(const std::string& name) const;

  [[deprecated("Use FactoryResetAllParameters() instead.")]]
  void ResetAllParametersToDefault() const;

  /**
   * @brief Factory reset a parameter to its default value.
   * 
   * @param name Parameter name.
   * @return True if successful, false otherwise.
   */
  bool FactoryResetParameter(const std::string& name) const;

  /**
   * @brief Factory reset all parameters to their default values.
   */
  void FactoryResetAllParameters() const;

  /**
   * @brief Reset a parameter to its default value.
   * 
   * @param name Parameter name.
   * @return True if successful, false otherwise.
   */
  bool ResetParameter(const std::string& name) const;

  /**
   * @brief Reset all parameters to their default values.
   */
  void ResetAllParameters() const;

  /**
   * @brief Get robot model information.
   * 
   * @return std::string Robot model string.
   */
  std::string GetRobotModel() const;

  /**
   * @brief Import robot model.
   * 
   * @param name Model name.
   * @param model Model data.
   * @return True if successful, false otherwise.
   */
  bool ImportRobotModel(const std::string& name, const std::string& model) const;

  /**
   * @brief Synchronize time with the robot.
   * 
   * @return True if successful, false otherwise.
   */
  bool SyncTime();

  /**
   * @brief Check if time synchronization has been established.
   * 
   * @return True if time sync is established, false otherwise.
   */
  bool HasEstablishedTimeSync();

  /**
   * @brief Start time synchronization.
   * 
   * @param period_sec Synchronization period [s] (default: 10).
   * @return True if successful, false otherwise.
   */
  bool StartTimeSync(long period_sec = 10 /* sec */);

  /**
   * @brief Stop time synchronization.
   * 
   * @return True if successful, false otherwise.
   */
  bool StopTimeSync();

  /**
   * @brief Get robot dynamics model.
   * 
   * @param urdf_model URDF model path (default: empty string to use robot's model).
   * @return std::shared_ptr<dyn::Robot<T::kRobotDOF>> Robot dynamics model.
   */
  std::shared_ptr<dyn::Robot<T::kRobotDOF>> GetDynamics(const std::string& urdf_model = "");

  /**
   * @brief Set LED color.
   * 
   * @param color LED color.
   * @param duration Duration [s] (default: 1).
   * @param transition_time Transition time [s] (default: 0).
   * @param blinking Whether to blink (default: false).
   * @param blinking_freq Blinking frequency [Hz] (default: 1).
   * @return True if successful, false otherwise.
   */
  bool SetLEDColor(const Color& color, double duration = 1 /* sec */, double transition_time = 0 /* sec */,
                   bool blinking = false, double blinking_freq = 1 /* Hz */);

  /**
   * @brief Get robot system time.
   * 
   * @return std::tuple<struct timespec, std::string, std::string> Robot system time, timezone string, and local time string.
   */
  std::tuple<struct timespec, std::string, std::string> GetSystemTime() const;

  /**
   * @brief Set robot system time.
   * 
   * @param utc_time New system time in UTC.
   * @param time_zone Time zone (optional).
   * @return True if successful, false otherwise.
   */
  bool SetSystemTime(struct timespec utc_time, std::optional<std::string> time_zone = std::nullopt) const;

  /**
   * @brief Set battery level.
   * 
   * @param level Battery level percentage (0.0 to 100.0).
   * @return True if successful, false otherwise.
   */
  bool SetBatteryLevel(double level) const;

  /**
   * @brief Set battery configuration.
   * 
   * @param cutoff_voltage Cutoff voltage [V].
   * @param fully_charged_voltage Fully charged voltage [V].
   * @param coefficients Battery coefficients array.
   * @return True if successful, false otherwise.
   */
  bool SetBatteryConfig(double cutoff_voltage, double fully_charged_voltage, const std::array<double, 4>& coefficients);

  /**
   * @brief Reset battery configuration to default.
   * 
   * @return True if successful, false otherwise.
   */
  bool ResetBatteryConfig() const;

  /**
   * @brief Wait until the robot is ready to accept control commands.
   * 
   * Call after servo_on() and before sending control commands.
   * 
   * @param timeout_ms Timeout [ms].
   * @return True if the robot is ready to accept control, false if timeout expired.
   */
  bool WaitForControlReady(long timeout_ms) const;

  /**
   * @brief Reset network settings to default.
   * 
   * @return True if successful, false otherwise.
   */
  bool ResetNetworkSetting() const;

  /**
   * @brief Scan for available WiFi networks.
   * 
   * @return std::vector<WifiNetwork> List of available WiFi networks.
   */
  std::vector<WifiNetwork> ScanWifi() const;

  /**
   * @brief Connect to WiFi network.
   * 
   * @param ssid WiFi network name.
   * @param password WiFi password (default: empty string).
   * @param use_dhcp Use DHCP for automatic IP configuration (default: true).
   * @param ip_address Static IP address (default: empty string).
   * @param gateway Gateway address (default: empty string).
   * @param dns DNS servers (default: empty vector).
   * @return True if successful, false otherwise.
   */
  bool ConnectWifi(const std::string& ssid, const std::string& password = "", bool use_dhcp = true,
                   const std::string& ip_address = "", const std::string& gateway = "",
                   const std::vector<std::string>& dns = {}) const;

  /**
   * @brief Disconnect from WiFi network.
   * 
   * @return True if successful, false otherwise.
   */
  bool DisconnectWifi() const;

  /**
   * @brief Get WiFi connection status.
   * 
   * @return std::optional<WifiStatus> Current WiFi status.
   */
  std::optional<WifiStatus> GetWifiStatus() const;

  /**
   * @brief Get list of available serial devices on the robot.
   * 
   * @return std::vector<SerialDevice> List of available serial devices.
   */
  std::vector<SerialDevice> GetSerialDeviceList() const;

  /**
   * @brief Open a serial stream.
   * 
   * @param device_path Serial device path.
   * @param buadrate Baud rate.
   * @param bytesize Byte size.
   * @param parity Parity.
   * @param stopbits Stop bits.
   * @return std::unique_ptr<SerialStream> Serial stream instance.
   */
  std::unique_ptr<SerialStream> OpenSerialStream(const std::string& device_path, int buadrate, int bytesize,
                                                 char parity, int stopbits) const;

  /**
   * @brief Download a file from the robot.
   * 
   * @param path File path on the robot.
   * @param output Output stream to write the file content.
   * @return True if successful, false otherwise.
   */
  bool DownloadFile(const std::string& path, std::ostream& output) const;

  /**
   * @brief Download a file from the robot using a callback function.
   * 
   * @param path File path on the robot.
   * @param on_chunk Callback function called for each chunk of data.
   * @return True if successful, false otherwise.
   */
  bool DownloadFileToCallback(const std::string& path, std::function<void(const char*, size_t)> on_chunk) const;

 private:
  explicit Robot(std::string address);

 private:
  std::shared_ptr<RobotImpl<T>> impl_;
};

/**
 * @brief Robot command handler.
 * 
 * Handles robot command execution and provides status monitoring.
 * 
 * @tparam T Robot model type
 */
template <typename T>
class RBY1_SDK_API RobotCommandHandler {
 public:
  ~RobotCommandHandler();

  /**
   * @brief Check if the command execution is complete.
   * 
   * @return True if command is complete, false otherwise.
   */
  bool IsDone() const;

  /**
   * @brief Wait for the command execution to complete.
   * 
   * This method blocks until the command is done or cancelled.
   */
  void Wait();

  /**
   * @brief Wait for the command execution with timeout.
   * 
   * @param timeout_ms Timeout [ms].
   * @return True if command completed, false if timeout.
   */
  bool WaitFor(int timeout_ms);

  /**
   * @brief Cancel the command execution.
   */
  void Cancel();

  /**
   * @brief Wait for the command execution and get feedback.
   * 
   * Waits for command completion and returns detailed controller feedback including
   * position errors, control status, and final execution results.
   * 
   * @return RobotCommandFeedback Final controller state and execution results.
   */
  RobotCommandFeedback Get();

  /**
   * @brief Get gRPC status.
   * 
   * @return bool Status information.
   */
  bool GetStatus() const;

 private:
  explicit RobotCommandHandler(std::unique_ptr<RobotCommandHandlerImpl<T>> impl);

  std::unique_ptr<RobotCommandHandlerImpl<T>> impl_;

  friend class RobotImpl<T>;
};

/**
 * @brief Robot command stream handler for continuous command sending.
 * 
 * Provides a streaming interface for sending multiple commands to the robot
 * in sequence while maintaining a persistent connection. This is more efficient
 * than creating individual command handlers for each command.
 * 
 * @tparam T Robot model type
 */
template <typename T>
class RBY1_SDK_API RobotCommandStreamHandler {
 public:
  ~RobotCommandStreamHandler();

  /**
   * @brief Check if the command stream is complete.
   * 
   * @return True if the stream is done/closed, false otherwise.
   */
  bool IsDone() const;

  /**
   * @brief Wait for the command stream to complete.
   * 
   * This method blocks until the stream is closed or cancelled.
   */
  void Wait();

  /**
   * @brief Wait for the command stream to complete with timeout.
   * 
   * @param timeout_ms Timeout in milliseconds.
   * @return True if stream completed, false if timeout occurred.
   */

  bool WaitFor(int timeout_ms);

  /**
   * @brief Send a command through the stream.
   * 
   * Sends a command via the established stream connection and waits for feedback.
   * The command is executed asynchronously by the robot, and this method returns
   * current controller state information (e.g., position error, control status).
   * 
   * @param builder Command builder containing the command to send.
   * @param timeout_ms Timeout for receiving controller feedback in milliseconds (default: 1000).
   *                   This is the time to wait for controller state information,
   *                   not the command execution timeout.
   * @return RobotCommandFeedback Current controller state information including
   *         position errors, control status, and execution progress.
   */
  RobotCommandFeedback SendCommand(const RobotCommandBuilder& builder, int timeout_ms = 1000);

  /**
   * @brief Request feedback from the current command stream.
   * 
   * Requests current controller state information from the ongoing command execution.
   * The feedback includes control errors, target vs actual positions, and execution status.
   * 
   * @param timeout_ms Timeout for receiving controller feedback in milliseconds (default: 1000).
   * @return RobotCommandFeedback Current controller state including position errors,
   *         control status, and execution progress.
   */
  RobotCommandFeedback RequestFeedback(int timeout_ms = 1000);

  void Cancel();

 private:
  explicit RobotCommandStreamHandler(std::unique_ptr<RobotCommandStreamHandlerImpl<T>> impl);

  std::unique_ptr<RobotCommandStreamHandlerImpl<T>> impl_;

  friend class RobotImpl<T>;
};

/**
 * @brief Robot control input parameters.
 * 
 * Represents control input parameters for robot real-time control including mode, target, and gains.
 * 
 * @tparam T Robot model type
 */
template <typename T>
struct RBY1_SDK_API ControlInput {
  /// Control mode for each joint (boolean array).
  Eigen::Vector<bool, T::kRobotDOF> mode{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  /// Target positions for each joint [rad].
  Eigen::Vector<double, T::kRobotDOF> target{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  /// Feedback gains for each joint.
  Eigen::Vector<unsigned int, T::kRobotDOF> feedback_gain{Eigen::Vector<unsigned int, T::kRobotDOF>::Zero()};
  /// Feedforward torque for each joint [Nm].
  Eigen::Vector<double, T::kRobotDOF> feedforward_torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  /// Whether to finish the current control operation.
  bool finish{false};
};

/**
 * @brief Robot control state information.
 * 
 * Represents the current state of robot real-time control including position, velocity, and torque.
 * 
 * @tparam T Robot model type
 */
template <typename T>
struct RBY1_SDK_API ControlState {
  /// Current time [s].
  double t{0.};
  /// Whether the joint is ready for control.
  Eigen::Vector<bool, T::kRobotDOF> is_ready{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  /// Current joint positions [rad].
  Eigen::Vector<double, T::kRobotDOF> position{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  /// Current joint velocities [rad/s].
  Eigen::Vector<double, T::kRobotDOF> velocity{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  /// Current joint currents [A].
  Eigen::Vector<double, T::kRobotDOF> current{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  /// Current joint torques [Nm].
  Eigen::Vector<double, T::kRobotDOF> torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};
};

/**
 * @brief PID gain configuration.
 * 
 * Represents proportional, integral, and derivative gains for PID control.
 */
struct RBY1_SDK_API PIDGain {
  /// Proportional gain value.
  uint16_t p_gain;
  /// Integral gain value.
  uint16_t i_gain;
  /// Derivative gain value.
  uint16_t d_gain;
};

/**
 * @brief RGB color representation.
 * 
 * Represents a color using red, green, and blue components.
 */
struct RBY1_SDK_API Color {
  /**
   * @brief Construct a Color instance with default values (0, 0, 0).
   */
  Color() : r(0), g(0), b(0) {}

  /**
   * @brief Construct a Color instance with specified RGB values.
   * 
   * @param _r Red component [0, 255].
   * @param _g Green component [0, 255].
   * @param _b Blue component [0, 255].
   */
  Color(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}

  /// Red component [0, 255].
  uint8_t r{0};
  /// Green component [0, 255].
  uint8_t g{0};
  /// Blue component [0, 255].
  uint8_t b{0};
};

/**
 * @brief Serial device information.
 * 
 * Represents information about a serial device.
 */
struct RBY1_SDK_API SerialDevice {
  /// Device path (e.g., "/dev/ttyUSB0").
  std::string path;
  /// Device description.
  std::string description;
};

/**
 * @brief Serial communication stream interface.
 * 
 * Provides bidirectional serial communication with devices connected to the robot.
 * Supports asynchronous read/write operations with callback-based data handling.
 */
class RBY1_SDK_API SerialStream {
 public:
  ~SerialStream();

  /**
   * @brief Connect to the serial device.
   * 
   * Establishes a connection to the serial device with the configured parameters.
   * 
   * @param verbose Enable verbose logging during connection (default: false).
   * @return True if connection was successful, false otherwise.
   */
  bool Connect(bool verbose) const;

  /**
   * @brief Disconnect from the serial device.
   * 
   * Closes the serial connection and releases associated resources.
   */
  void Disconnect() const;

  /**
   * @brief Wait for the serial stream to complete.
   * 
   * Blocks until the serial stream is closed or an error occurs.
   */
  void Wait() const;

  /**
   * @brief Wait for the serial stream to complete with timeout.
   * 
   * @param timeout_ms Timeout in milliseconds.
   * @return True if stream completed normally, false if timeout occurred.
   */
  bool WaitFor(int timeout_ms) const;

  /**
   * @brief Check if the serial port is opened.
   * 
   * @return True if the serial port is currently open, false otherwise.
   */
  bool IsOpened() const;

  /**
   * @brief Check if the serial stream has been cancelled.
   * 
   * @return True if the stream was cancelled, false otherwise.
   */
  bool IsCancelled() const;

  /**
   * @brief Check if the serial stream is done.
   * 
   * @return True if the stream has completed or been closed, false otherwise.
   */
  bool IsDone() const;

  /**
   * @brief Set callback function for incoming data.
   * 
   * Registers a callback function that will be called whenever data is received
   * from the serial device.
   * 
   * @param cb Callback function that receives incoming data as a string.
   */
  void SetReadCallback(const std::function<void(const std::string&)>& cb);

  /**
   * @brief Write string data to the serial port.
   * 
   * @param data String data to write.
   * @return True if write was successful, false otherwise.
   */
  bool Write(const std::string& data);

  /**
   * @brief Write null-terminated string data to the serial port.
   * 
   * @param data Null-terminated string data to write.
   * @return True if write was successful, false otherwise.
   */
  bool Write(const char* data);

  /**
   * @brief Write binary data to the serial port.
   * 
   * @param data Pointer to data buffer.
   * @param n Number of bytes to write.
   * @return True if write was successful, false otherwise.
   */
  bool Write(const char* data, int n);

  /**
   * @brief Write a single byte to the serial port.
   * 
   * @param ch Byte to write.
   * @return True if write was successful, false otherwise.
   */
  bool WriteByte(char ch);

 private:
  explicit SerialStream(std::unique_ptr<SerialStreamImpl> impl);

  std::unique_ptr<SerialStreamImpl> impl_;

  template <typename T>
  friend class RobotImpl;
};

}  // namespace rb