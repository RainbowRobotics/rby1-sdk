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

  RobotInfo GetRobotInfo() const;  // NOLINT

  //  bool SetTimeScale(double time_scale) const;  // NOLINT // TODO

  //  double GetTimeScale() const; // NOLINT // TODO

  bool PowerOn(const std::string& dev_name) const;  // NOLINT

  bool PowerOff(const std::string& dev_name) const;  // NOLINT

  bool IsPowerOn(const std::string& dev_name) const;  //NOLINT

  bool ServoOn(const std::string& dev_name) const;  // NOLINT

  bool IsServoOn(const std::string& dev_name) const;  // NOLINT

  bool EnableControlManager() const;  // NOLINT

  bool DisableControlManager() const;  // NOLINT

  bool ResetFaultControlManager() const;  // NOLINT

  bool SetToolFlangeOutputVoltage(const std::string& name, int voltage) const;  // NOLINT

  void StartStateUpdate(const std::function<void(const RobotState<T>&)>& cb, double rate);

  void StopStateUpdate();

  void StartLogStream(const std::function<void(const std::vector<Log>&)>& cb, double rate);

  void StopLogStream();

  RobotState<T> GetState() const;  // NOLINT

  std::vector<Log> GetLastLog(unsigned int count) const;  // NOLINT

  ControlManagerState GetControlManagerState() const;  // NOLINT

  std::unique_ptr<RobotCommandHandler<T>> SendCommand(const RobotCommandBuilder& builder, int priority = 1);

  std::unique_ptr<RobotCommandStreamHandler<T>> CreateCommandStream(int priority = 1);

  //  void Control(std::function<ControlInput(const RobotState<T>&)> control);

  bool ResetOdometry(double angle, const Eigen::Vector<double, 2>& position);

  bool SetParameter(const std::string& name, const std::string& value);

  std::string GetParameter(const std::string& name) const;  // NOLINT

 private:
  explicit Robot(std::string address);

 private:
  std::shared_ptr<RobotImpl<T>> impl_;
};

template <typename T>
class RobotCommandHandler {
 public:
  ~RobotCommandHandler();

  bool IsDone() const;  // NOLINT

  void Wait();

  bool WaitFor(int timeout_ms);

  void Cancel();

  RobotCommandFeedback Get();  // TODO

  bool GetStatus() const;  // NOLINT

 private:
  explicit RobotCommandHandler(std::unique_ptr<RobotCommandHandlerImpl<T>> impl);

  std::unique_ptr<RobotCommandHandlerImpl<T>> impl_;

  friend class RobotImpl<T>;
};

template <typename T>
class RobotCommandStreamHandler {
 public:
  ~RobotCommandStreamHandler();

  bool IsDone() const;  // NOLINT

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

}  // namespace rb