#pragma once

#include <memory>
#include <string>
#include <vector>

namespace rb {

class RobotCommandFeedbackParserImpl;

class Feedback {
 public:
  bool valid() const { return valid_; }

 private:
  bool valid_;

  friend class RobotCommandFeedbackParserImpl;
};

class CommandHeaderFeedback : public Feedback {
 public:
  bool finished() const { return finished_; }

 private:
  bool finished_;

  friend class RobotCommandFeedbackParserImpl;
};

class CommandFeedback : public Feedback {
 public:
  const CommandHeaderFeedback& command_header() const { return command_header_; }

 protected:
  CommandHeaderFeedback command_header_{};

  friend class RobotCommandFeedbackParserImpl;
};

class StopCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RealtimeControlCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class SE2VelocityCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class JogCommandFeedback : public CommandFeedback {
 public:
  const std::string& target_joint_name() const { return target_joint_name_; }

 protected:
  std::string target_joint_name_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class JointVelocityCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class JointPositionCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class CartesianCommandFeedback : public CommandFeedback {
 public:
  struct TrackingError {
    double position_error;
    double rotation_error;
  };

  const std::vector<TrackingError>& tracking_errors() const { return tracking_errors_; }

 protected:
  std::vector<TrackingError> tracking_errors_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class GravityCompensationCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class ImpedanceControlCommandFeedback : public CommandFeedback {
 public:
  struct TrackingError {
    double position_error;
    double rotation_error;
  };

  const TrackingError& tracking_error() const { return tracking_error_; }

 protected:
  TrackingError tracking_error_{};

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class OptimalControlCommandFeedback : public CommandFeedback {
 public:
  double total_cost() const { return total_cost_; }

  const std::vector<double>& cartesian_costs() const { return cartesian_costs_; }

  double center_of_mass_cost() const { return center_of_mass_cost_; }

  const std::vector<double>& joint_position_costs() const { return joint_position_costs_; }

 protected:
  double total_cost_;
  std::vector<double> cartesian_costs_;
  double center_of_mass_cost_;
  std::vector<double> joint_position_costs_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class WholeBodyCommandFeedback : public CommandFeedback {
 public:
  const StopCommandFeedback& stop_command() const { return stop_command_; }
  const RealtimeControlCommandFeedback& realtime_control_command() const { return realtime_control_command_; }

 protected:
  StopCommandFeedback stop_command_;
  RealtimeControlCommandFeedback realtime_control_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class ArmCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }

  const GravityCompensationCommandFeedback& gravity_compensation_command() const {
    return gravity_compensation_command_;
  }

  const CartesianCommandFeedback& cartesian_command() const { return cartesian_command_; }

  const ImpedanceControlCommandFeedback& impedance_control_command() const { return impedance_control_command_; }

 protected:
  JointPositionCommandFeedback joint_position_command_;
  GravityCompensationCommandFeedback gravity_compensation_command_;
  CartesianCommandFeedback cartesian_command_;
  ImpedanceControlCommandFeedback impedance_control_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class TorsoCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }

  const GravityCompensationCommandFeedback& gravity_compensation_command() const {
    return gravity_compensation_command_;
  }

  const CartesianCommandFeedback& cartesian_command() const { return cartesian_command_; }

  const ImpedanceControlCommandFeedback& impedance_control_command() const { return impedance_control_command_; }

  const OptimalControlCommandFeedback& optimal_control_command() const { return optimal_control_command_; }

 protected:
  JointPositionCommandFeedback joint_position_command_;
  GravityCompensationCommandFeedback gravity_compensation_command_;
  CartesianCommandFeedback cartesian_command_;
  ImpedanceControlCommandFeedback impedance_control_command_;
  OptimalControlCommandFeedback optimal_control_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class BodyComponentBasedCommandFeedback : public CommandFeedback {
 public:
  const ArmCommandFeedback& right_arm_command() const { return right_arm_command_; }  // NOLINT

  const ArmCommandFeedback& left_arm_command() const { return left_arm_command_; }  // NOLINT

  const TorsoCommandFeedback& torso_command() const { return torso_command_; }  // NOLINT

 protected:
  ArmCommandFeedback right_arm_command_;
  ArmCommandFeedback left_arm_command_;
  TorsoCommandFeedback torso_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class HeadCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }  // NOLINT

 protected:
  JointPositionCommandFeedback joint_position_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class BodyCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }  // NOLINT

  const OptimalControlCommandFeedback& optimal_control_command() const { return optimal_control_command_; }  // NOLINT

  const GravityCompensationCommandFeedback& gravity_compensation_command() const {  // NOLINT
    return gravity_compensation_command_;
  }

  const CartesianCommandFeedback& cartesian_command() const { return cartesian_command_; }  // NOLINT

  const BodyComponentBasedCommandFeedback& body_component_based_command() const {  // NOLINT
    return body_component_based_command_;
  }

 protected:
  JointPositionCommandFeedback joint_position_command_;
  OptimalControlCommandFeedback optimal_control_command_;
  GravityCompensationCommandFeedback gravity_compensation_command_;
  CartesianCommandFeedback cartesian_command_;
  BodyComponentBasedCommandFeedback body_component_based_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class MobilityCommandFeedback : public CommandFeedback {
 public:
  const JointVelocityCommandFeedback& joint_velocity_command() const { return joint_velocity_command_; }  // NOLINT

  const SE2VelocityCommandFeedback& se2_velocity_command() const { return se2_velocity_command_; }  // NOLINT

 protected:
  JointVelocityCommandFeedback joint_velocity_command_;
  SE2VelocityCommandFeedback se2_velocity_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class ComponentBasedCommandFeedback : public CommandFeedback {
 public:
  const HeadCommandFeedback& head_command() const { return head_command_; }  // NOLINT

  const BodyCommandFeedback& body_command() const { return body_command_; }  // NOLINT

  const MobilityCommandFeedback& mobility_command() const { return mobility_command_; }  // NOLINT

 protected:
  HeadCommandFeedback head_command_;
  BodyCommandFeedback body_command_;
  MobilityCommandFeedback mobility_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RobotCommandFeedback : public CommandFeedback {
 public:
  enum class Status : int { kIdle = 0, kInitializing, kRunning, kFinished };

  enum class FinishCode : int {
    kUnknown = 0,
    kOk,
    kCanceled,
    kPreempted,
    kInitializedFailed,
    kControlManagerIdle,
    kControlManagerFault,
    kUnexpectedState
  };

  const WholeBodyCommandFeedback& whole_body_command() const { return whole_body_command_; }  // NOLINT

  const ComponentBasedCommandFeedback& component_based_command() const { return component_based_command_; }  // NOLINT

  const JogCommandFeedback& jog_command() const { return jog_command_; }  // NOLINT

  const Status& status() const { return status_; }  // NOLINT

  const FinishCode& finish_code() const { return finish_code_; }  // NOLINT

 protected:
  WholeBodyCommandFeedback whole_body_command_;
  ComponentBasedCommandFeedback component_based_command_;
  JogCommandFeedback jog_command_;

  Status status_{};
  FinishCode finish_code_{};

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RobotCommandFeedbackParser {
 public:
  RobotCommandFeedbackParser();

  ~RobotCommandFeedbackParser();

  void Parse(RobotCommandFeedback& self, void* msg);

 private:
  std::unique_ptr<RobotCommandFeedbackParserImpl> impl_;
};

}  // namespace rb