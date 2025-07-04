#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "export.h"

namespace rb {

class RobotCommandFeedbackParserImpl;

class RBY1_SDK_API Feedback {
 public:
  bool valid() const { return valid_; }

 private:
  bool valid_{false};

  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API CommandHeaderFeedback : public Feedback {
 public:
  bool finished() const { return finished_; }

 private:
  bool finished_;

  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API CommandFeedback : public Feedback {
 public:
  const CommandHeaderFeedback& command_header() const { return command_header_; }

 protected:
  CommandHeaderFeedback command_header_{};

  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API StopCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API RealtimeControlCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API SE2VelocityCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API JogCommandFeedback : public CommandFeedback {
 public:
  const std::string& target_joint_name() const { return target_joint_name_; }

 protected:
  std::string target_joint_name_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API JointVelocityCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API JointPositionCommandFeedback : public CommandFeedback {
 public:
  double time_based_progress() const { return time_based_progress_; }

  double position_based_progress() const { return position_based_progress_; }

 protected:
  double time_based_progress_{};
  double position_based_progress_{};

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API JointGroupPositionCommandFeedback : public CommandFeedback {
 public:
  const std::vector<unsigned int>& joint_indices() const { return joint_indices_; }

  double time_based_progress() const { return time_based_progress_; }

  double position_based_progress() const { return position_based_progress_; }

 protected:
  std::vector<unsigned int> joint_indices_;
  double time_based_progress_;
  double position_based_progress_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API CartesianCommandFeedback : public CommandFeedback {
 public:
  struct RBY1_SDK_API TrackingError {
    double position_error;
    double orientation_error;
  };

  const std::vector<TrackingError>& se3_pose_tracking_errors() const { return se3_pose_tracking_errors_; }

  const std::vector<double>& joint_position_tracking_errors() const { return joint_position_tracking_errors_; }

  double remain_time() const { return remain_time_; }

  double manipulability() const { return manipulability_; }

 protected:
  std::vector<TrackingError> se3_pose_tracking_errors_;
  std::vector<double> joint_position_tracking_errors_;
  double remain_time_;
  double manipulability_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API CartesianImpedanceControlCommandFeedback : public CommandFeedback {
 public:
  Eigen::VectorXd set_position() const { return set_position_; }

  double remain_time() const { return remain_time_; }

  double manipulability() const { return manipulability_; }

 private:
  Eigen::VectorXd set_position_;
  double remain_time_;
  double manipulability_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API GravityCompensationCommandFeedback : public CommandFeedback {
 public:
 protected:
 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API ImpedanceControlCommandFeedback : public CommandFeedback {
 public:
  struct RBY1_SDK_API TrackingError {
    double position_error;
    double rotation_error;
  };

  const TrackingError& tracking_error() const { return tracking_error_; }

 protected:
  TrackingError tracking_error_{};

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API JointImpedanceControlCommandFeedback : public CommandFeedback {
 public:
  const std::vector<double>& set_position() const { return set_position_; }

  const std::vector<double>& error() const { return error_; }

 protected:
  std::vector<double> set_position_;
  std::vector<double> error_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API OptimalControlCommandFeedback : public CommandFeedback {
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

class RBY1_SDK_API WholeBodyCommandFeedback : public CommandFeedback {
 public:
  const StopCommandFeedback& stop_command() const { return stop_command_; }

  const RealtimeControlCommandFeedback& realtime_control_command() const { return realtime_control_command_; }

 protected:
  StopCommandFeedback stop_command_;
  RealtimeControlCommandFeedback realtime_control_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API ArmCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }

  const GravityCompensationCommandFeedback& gravity_compensation_command() const {
    return gravity_compensation_command_;
  }

  const CartesianCommandFeedback& cartesian_command() const { return cartesian_command_; }

  const ImpedanceControlCommandFeedback& impedance_control_command() const { return impedance_control_command_; }

  const CartesianImpedanceControlCommandFeedback& cartesian_impedance_control_command() const {
    return cartesian_impedance_control_command_;
  }

  const JointImpedanceControlCommandFeedback& joint_impedance_control_command() const {
    return joint_impedance_control_command_;
  }

 protected:
  JointPositionCommandFeedback joint_position_command_;
  GravityCompensationCommandFeedback gravity_compensation_command_;
  CartesianCommandFeedback cartesian_command_;
  ImpedanceControlCommandFeedback impedance_control_command_;
  CartesianImpedanceControlCommandFeedback cartesian_impedance_control_command_;
  JointImpedanceControlCommandFeedback joint_impedance_control_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API TorsoCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }

  const GravityCompensationCommandFeedback& gravity_compensation_command() const {
    return gravity_compensation_command_;
  }

  const CartesianCommandFeedback& cartesian_command() const { return cartesian_command_; }

  const ImpedanceControlCommandFeedback& impedance_control_command() const { return impedance_control_command_; }

  const OptimalControlCommandFeedback& optimal_control_command() const { return optimal_control_command_; }

  const CartesianImpedanceControlCommandFeedback& cartesian_impedance_control_command() const {
    return cartesian_impedance_control_command_;
  }

  const JointImpedanceControlCommandFeedback& joint_impedance_control_command() const {
    return joint_impedance_control_command_;
  }

  const JointGroupPositionCommandFeedback& joint_group_position_command() const {
    return joint_group_position_command_;
  }

 protected:
  JointPositionCommandFeedback joint_position_command_;
  GravityCompensationCommandFeedback gravity_compensation_command_;
  CartesianCommandFeedback cartesian_command_;
  ImpedanceControlCommandFeedback impedance_control_command_;
  OptimalControlCommandFeedback optimal_control_command_;
  CartesianImpedanceControlCommandFeedback cartesian_impedance_control_command_;
  JointImpedanceControlCommandFeedback joint_impedance_control_command_;
  JointGroupPositionCommandFeedback joint_group_position_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API BodyComponentBasedCommandFeedback : public CommandFeedback {
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

class RBY1_SDK_API HeadCommandFeedback : public CommandFeedback {
 public:
  const JointPositionCommandFeedback& joint_position_command() const { return joint_position_command_; }  // NOLINT

 protected:
  JointPositionCommandFeedback joint_position_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API BodyCommandFeedback : public CommandFeedback {
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

  const CartesianImpedanceControlCommandFeedback& cartesian_impedance_control_command() const {
    return cartesian_impedance_control_command_;
  }

  const JointImpedanceControlCommandFeedback& joint_impedance_control_command() const {
    return joint_impedance_control_command_;
  }

 protected:
  JointPositionCommandFeedback joint_position_command_;
  OptimalControlCommandFeedback optimal_control_command_;
  GravityCompensationCommandFeedback gravity_compensation_command_;
  CartesianCommandFeedback cartesian_command_;
  BodyComponentBasedCommandFeedback body_component_based_command_;
  CartesianImpedanceControlCommandFeedback cartesian_impedance_control_command_;
  JointImpedanceControlCommandFeedback joint_impedance_control_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API MobilityCommandFeedback : public CommandFeedback {
 public:
  const JointVelocityCommandFeedback& joint_velocity_command() const { return joint_velocity_command_; }  // NOLINT

  const SE2VelocityCommandFeedback& se2_velocity_command() const { return se2_velocity_command_; }  // NOLINT

 protected:
  JointVelocityCommandFeedback joint_velocity_command_;
  SE2VelocityCommandFeedback se2_velocity_command_;

 private:
  friend class RobotCommandFeedbackParserImpl;
};

class RBY1_SDK_API ComponentBasedCommandFeedback : public CommandFeedback {
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

class RBY1_SDK_API RobotCommandFeedback : public CommandFeedback {
 public:
  enum class Status : int { kIdle = 0, kInitializing, kRunning, kFinished };

  enum class FinishCode : int {
    kUnknown = 0,
    kOk,
    kCanceled,
    kPreempted,
    kInitializationFailed,
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

class RBY1_SDK_API RobotCommandFeedbackParser {
 public:
  RobotCommandFeedbackParser();

  ~RobotCommandFeedbackParser();

  void Parse(RobotCommandFeedback& self, void* msg);

 private:
  std::unique_ptr<RobotCommandFeedbackParserImpl> impl_;
};

}  // namespace rb