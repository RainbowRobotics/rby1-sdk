#pragma once

#include <condition_variable>
#include <functional>
#include <string>

#include "control_manager_state.h"
#include "log.h"
#include "robot_command_builder.h"
#include "robot_command_feedback.h"
#include "robot_info.h"
#include "robot_state.h"

#include "dynamics/robot.h"

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

}  // namespace rb

namespace rb {

template <typename T>
class Robot : public std::enable_shared_from_this<Robot<T>> {
 public:
  static std::shared_ptr<Robot<T>> Create(std::string address);

  ~Robot();

  std::string GetAddress();

  bool Connect(int max_retries = 5, int timeout_ms = 1000);

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

  bool EnableControlManager(bool unlimited_mode_enabled = false) const;

  bool DisableControlManager() const;

  bool ResetFaultControlManager() const;

  bool SetToolFlangeOutputVoltage(const std::string& name, int voltage) const;

  void StartStateUpdate(const std::function<void(const RobotState<T>&)>& cb, double rate);

  void StopStateUpdate();

  void StartLogStream(const std::function<void(const std::vector<Log>&)>& cb, double rate);

  void StopLogStream();

  RobotState<T> GetState() const;

  std::vector<Log> GetLastLog(unsigned int count) const;

  ControlManagerState GetControlManagerState() const;

  std::unique_ptr<RobotCommandHandler<T>> SendCommand(const RobotCommandBuilder& builder, int priority = 1);

  std::unique_ptr<RobotCommandStreamHandler<T>> CreateCommandStream(int priority = 1);

  bool Control(std::function<ControlInput<T>(const ControlState<T>&)> control, int port = 0, int priority = 1);

  bool ResetOdometry(double angle, const Eigen::Vector<double, 2>& position);

  std::vector<std::pair<std::string, int>> GetParameterList() const;

  bool SetParameter(const std::string& name, const std::string& value);

  std::string GetParameter(const std::string& name) const;

  bool ResetParameterToDefault(const std::string& name) const;

  void ResetAllParametersToDefault() const;

  std::string GetRobotModel() const;

  bool ImportRobotModel(const std::string& name, const std::string& model) const;

  bool SyncTime();

  bool HasEstablishedTimeSync();

  bool StartTimeSync(long period_sec = 10 /* sec */);

  bool StopTimeSync();

  std::shared_ptr<dyn::Robot<T::kRobotDOF>> GetDynamics(const std::string& urdf_model = "");

 private:
  explicit Robot(std::string address);

 private:
  std::shared_ptr<RobotImpl<T>> impl_;
};

template <typename T>
class RobotCommandHandler {
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
class RobotCommandStreamHandler {
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
struct ControlInput {
  Eigen::Vector<bool, T::kRobotDOF> mode{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  Eigen::Vector<double, T::kRobotDOF> target{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<unsigned int, T::kRobotDOF> feedback_gain{Eigen::Vector<unsigned int, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> feedforward_torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  bool finish{false};
};

template <typename T>
struct ControlState {
  double t{0.};
  Eigen::Vector<bool, T::kRobotDOF> is_ready{Eigen::Vector<bool, T::kRobotDOF>::Constant(false)};
  Eigen::Vector<double, T::kRobotDOF> position{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> velocity{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> current{Eigen::Vector<double, T::kRobotDOF>::Zero()};
  Eigen::Vector<double, T::kRobotDOF> torque{Eigen::Vector<double, T::kRobotDOF>::Zero()};
};

}  // namespace rb