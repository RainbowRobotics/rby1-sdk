#pragma once

#include <memory>
#include <optional>

#include "Eigen/Core"
#include "dynamics/inertial.h"
#include "export.h"
#include "math/liegroup.h"

namespace rb {

class CommandHeaderBuilderImpl;
class JointPositionCommandBuilderImpl;
class JointGroupPositionCommandBuilderImpl;
class JointImpedanceControlCommandBuilderImpl;
class OptimalControlCommandBuilderImpl;
class GravityCompensationCommandBuilderImpl;
class CartesianCommandBuilderImpl;
class CartesianImpedanceControlCommandBuilderImpl;
class ImpedanceControlCommandBuilderImpl;
class JointVelocityCommandBuilderImpl;
class JogCommandBuilderImpl;
class SE2VelocityCommandBuilderImpl;
class StopCommandBuilderImpl;
class RealTimeControlCommandBuilderImpl;
class ArmCommandBuilderImpl;
class TorsoCommandBuilderImpl;
class BodyComponentBasedCommandBuilderImpl;
class BodyCommandBuilderImpl;
class MobilityCommandBuilderImpl;
class HeadCommandBuilderImpl;
class ComponentBasedCommandBuilderImpl;
class WholeBodyCommandBuilderImpl;
class RobotCommandBuilderImpl;

class RBY1_SDK_API CommandHeaderBuilder {
 public:
  CommandHeaderBuilder();

  ~CommandHeaderBuilder();

  CommandHeaderBuilder& SetControlHoldTime(double control_hold_time);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<CommandHeaderBuilderImpl> impl_;

  friend class GravityCompensationCommandBuilderImpl;
  friend class JointPositionCommandBuilderImpl;
  friend class JointGroupPositionCommandBuilderImpl;
  friend class JointImpedanceControlCommandBuilderImpl;
  friend class CartesianCommandBuilderImpl;
  friend class CartesianImpedanceControlCommandBuilderImpl;
  friend class OptimalControlCommandBuilderImpl;
  friend class ImpedanceControlCommandBuilderImpl;
  friend class JointVelocityCommandBuilderImpl;
  friend class StopCommandBuilderImpl;
  friend class SE2VelocityCommandBuilderImpl;
  friend class JogCommandBuilderImpl;
};

class RBY1_SDK_API JointPositionCommandBuilder {
 public:
  JointPositionCommandBuilder();

  ~JointPositionCommandBuilder();

  JointPositionCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  JointPositionCommandBuilder& SetMinimumTime(double minimum_time);

  JointPositionCommandBuilder& SetPosition(const Eigen::VectorXd& position);

  JointPositionCommandBuilder& SetVelocityLimit(const Eigen::VectorXd& velocity_limit);

  JointPositionCommandBuilder& SetAccelerationLimit(const Eigen::VectorXd& acceleration_limit);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<JointPositionCommandBuilderImpl> impl_;

  friend class ArmCommandBuilderImpl;
  friend class TorsoCommandBuilderImpl;
  friend class BodyCommandBuilderImpl;
  friend class HeadCommandBuilderImpl;
};

class RBY1_SDK_API JointGroupPositionCommandBuilder {
 public:
  JointGroupPositionCommandBuilder();

  ~JointGroupPositionCommandBuilder();

  JointGroupPositionCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  JointGroupPositionCommandBuilder& SetJointNames(const std::vector<std::string>& joint_names);

  JointGroupPositionCommandBuilder& SetMinimumTime(double minimum_time);

  JointGroupPositionCommandBuilder& SetPosition(const Eigen::VectorXd& position);

  JointGroupPositionCommandBuilder& SetVelocityLimit(const Eigen::VectorXd& velocity_limit);

  JointGroupPositionCommandBuilder& SetAccelerationLimit(const Eigen::VectorXd& acceleration_limit);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<JointGroupPositionCommandBuilderImpl> impl_;

  friend class TorsoCommandBuilderImpl;
};

class RBY1_SDK_API JointImpedanceControlCommandBuilder {
 public:
  JointImpedanceControlCommandBuilder();

  ~JointImpedanceControlCommandBuilder();

  JointImpedanceControlCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  JointImpedanceControlCommandBuilder& SetMinimumTime(double minimum_time);

  JointImpedanceControlCommandBuilder& SetPosition(const Eigen::VectorXd& position);

  JointImpedanceControlCommandBuilder& SetVelocityLimit(const Eigen::VectorXd& velocity_limit);

  JointImpedanceControlCommandBuilder& SetAccelerationLimit(const Eigen::VectorXd& acceleration_limit);

  JointImpedanceControlCommandBuilder& SetStiffness(const Eigen::VectorXd& stiffness);

  JointImpedanceControlCommandBuilder& SetTorqueLimit(const Eigen::VectorXd& torque_limit);

  JointImpedanceControlCommandBuilder& SetDampingRatio(double damping_ratio);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<JointImpedanceControlCommandBuilderImpl> impl_;

  friend class ArmCommandBuilderImpl;
  friend class TorsoCommandBuilderImpl;
  friend class BodyCommandBuilderImpl;
  friend class HeadCommandBuilderImpl;
};

class RBY1_SDK_API OptimalControlCommandBuilder {
 public:
  OptimalControlCommandBuilder();

  ~OptimalControlCommandBuilder();

  OptimalControlCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  OptimalControlCommandBuilder& AddCartesianTarget(const std::string& ref_link_name,  //
                                                   const std::string& link_name,      //
                                                   const math::SE3::MatrixType& T,    //
                                                   double translation_weight,         //
                                                   double rotation_weight             //
  );

  OptimalControlCommandBuilder& SetCenterOfMassTarget(const std::string& ref_link_name,  //
                                                      const Eigen::Vector3d& pose,       //
                                                      double weight                      //
  );

  OptimalControlCommandBuilder& AddJointPositionTarget(const std::string& joint_name,  //
                                                       double target_position,         //
                                                       double weight                   //
  );

  OptimalControlCommandBuilder& SetErrorScaling(double error_scaling);

  OptimalControlCommandBuilder& SetVelocityLimitScaling(double velocity_limit_scaling);

  OptimalControlCommandBuilder& SetAccelerationLimitScaling(double acceleration_limit_scaling);

  OptimalControlCommandBuilder& SetStopCost(double stop_cost);

  OptimalControlCommandBuilder& SetMinDeltaCost(double min_delta_cost);

  OptimalControlCommandBuilder& SetPatience(int patience);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<OptimalControlCommandBuilderImpl> impl_;

  friend class TorsoCommandBuilderImpl;
  friend class BodyCommandBuilderImpl;
};

class RBY1_SDK_API ImpedanceControlCommandBuilder {
 public:
  ImpedanceControlCommandBuilder();

  ~ImpedanceControlCommandBuilder();

  ImpedanceControlCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  ImpedanceControlCommandBuilder& SetReferenceLinkName(const std::string& name);

  ImpedanceControlCommandBuilder& SetLinkName(const std::string& name);

  ImpedanceControlCommandBuilder& SetTransformation(const math::SE3::MatrixType& T);

  ImpedanceControlCommandBuilder& SetTranslationWeight(const Eigen::Vector3d& weight);

  ImpedanceControlCommandBuilder& SetRotationWeight(const Eigen::Vector3d& weight);

  ImpedanceControlCommandBuilder& SetDampingRatio(double damping_ratio);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<ImpedanceControlCommandBuilderImpl> impl_;

  friend class ArmCommandBuilderImpl;
  friend class TorsoCommandBuilderImpl;
};

class RBY1_SDK_API JointVelocityCommandBuilder {
 public:
  JointVelocityCommandBuilder();

  ~JointVelocityCommandBuilder();

  JointVelocityCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  JointVelocityCommandBuilder& SetMinimumTime(double minimum_time);

  JointVelocityCommandBuilder& SetVelocity(const Eigen::VectorXd& velocity);

  JointVelocityCommandBuilder& SetAccelerationLimit(const Eigen::VectorXd& acceleration_limit);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<JointVelocityCommandBuilderImpl> impl_;

  friend class MobilityCommandBuilderImpl;
};

class RBY1_SDK_API JogCommandBuilder {
 public:
  class RBY1_SDK_API AbsolutePosition {
   public:
    explicit AbsolutePosition(double value) : value_(value) {}

    double value() const { return value_; }

   private:
    double value_{};
  };

  class RBY1_SDK_API RelativePosition {
   public:
    explicit RelativePosition(double value) : value_(value) {}

    double value() const { return value_; }

   private:
    double value_{};
  };

  class RBY1_SDK_API OneStep {
   public:
    explicit OneStep(bool direction) : value_(direction) {}

    bool value() const { return value_; }

   private:
    bool value_{};
  };

  JogCommandBuilder();

  ~JogCommandBuilder();

  JogCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  JogCommandBuilder& SetJointName(const std::string& name);

  JogCommandBuilder& SetVelocityLimit(double value);

  JogCommandBuilder& SetAccelerationLimit(double value);

  JogCommandBuilder& SetCommand(AbsolutePosition absolute_position);

  JogCommandBuilder& SetCommand(RelativePosition relative_position);

  JogCommandBuilder& SetCommand(OneStep one_step);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<JogCommandBuilderImpl> impl_;

  friend class RobotCommandBuilderImpl;
};

class RBY1_SDK_API SE2VelocityCommandBuilder {
 public:
  SE2VelocityCommandBuilder();

  ~SE2VelocityCommandBuilder();

  SE2VelocityCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  SE2VelocityCommandBuilder& SetMinimumTime(double minimum_time);

  SE2VelocityCommandBuilder& SetVelocity(const Eigen::Vector2d& linear, double angular);

  SE2VelocityCommandBuilder& SetAccelerationLimit(const Eigen::Vector2d& linear, double angular);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<SE2VelocityCommandBuilderImpl> impl_;

  friend class MobilityCommandBuilderImpl;
};

class RBY1_SDK_API StopCommandBuilder {
 public:
  StopCommandBuilder();

  ~StopCommandBuilder();

  StopCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<StopCommandBuilderImpl> impl_;

  friend class WholeBodyCommandBuilderImpl;
};

class RBY1_SDK_API RealTimeControlCommandBuilder {
 public:
  RealTimeControlCommandBuilder();

  ~RealTimeControlCommandBuilder();

  RealTimeControlCommandBuilder& SetPort(int port);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<RealTimeControlCommandBuilderImpl> impl_;

  friend class WholeBodyCommandBuilderImpl;
};

class RBY1_SDK_API CartesianCommandBuilder {
 public:
  CartesianCommandBuilder();

  ~CartesianCommandBuilder();

  CartesianCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  CartesianCommandBuilder& SetMinimumTime(double minium_time);

  CartesianCommandBuilder& AddTarget(const std::string& ref_link_name,       //
                                     const std::string& link_name,           //
                                     const math::SE3::MatrixType& T,         // Eigen::Matrix<double, 4, 4>
                                     double linear_velocity_limit,           // (m/s)
                                     double angular_velocity_limit,          // (rad/s)
                                     double acceleration_limit_scaling = 1.  // (0, 1]
  );

  CartesianCommandBuilder& AddJointPositionTarget(const std::string& joint_name,                           //
                                                  double target_position,                                  //
                                                  std::optional<double> velocity_limit = std::nullopt,     //
                                                  std::optional<double> acceleration_limit = std::nullopt  //
  );

  CartesianCommandBuilder& SetStopPositionTrackingError(double stop_position_tracking_error);

  CartesianCommandBuilder& SetStopOrientationTrackingError(double stop_orientation_tracking_error);

  CartesianCommandBuilder& SetStopJointPositionTrackingError(double stop_joint_position_tracking_error);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<CartesianCommandBuilderImpl> impl_;

  friend class ArmCommandBuilderImpl;
  friend class TorsoCommandBuilderImpl;
  friend class BodyCommandBuilderImpl;
};

class RBY1_SDK_API CartesianImpedanceControlCommandBuilder {
 public:
  CartesianImpedanceControlCommandBuilder();

  ~CartesianImpedanceControlCommandBuilder();

  CartesianImpedanceControlCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  CartesianImpedanceControlCommandBuilder& SetMinimumTime(double minium_time);

  CartesianImpedanceControlCommandBuilder& AddTarget(
      const std::string& ref_link_name,                                //
      const std::string& link_name,                                    //
      const math::SE3::MatrixType& T,                                  // Eigen::Matrix<double, 4, 4>
      std::optional<double> linear_velocity_limit = std::nullopt,      // (m/s)
      std::optional<double> angular_velocity_limit = std::nullopt,     // (rad/s)
      std::optional<double> linear_acceleration_limit = std::nullopt,  //
      std::optional<double> angular_acceleration_limit = std::nullopt  //
  );

  CartesianImpedanceControlCommandBuilder& AddJointPositionTarget(
      const std::string& joint_name,                           //
      double target_position,                                  //
      std::optional<double> velocity_limit = std::nullopt,     //
      std::optional<double> acceleration_limit = std::nullopt  //
  );

  CartesianImpedanceControlCommandBuilder& SetStopPositionTrackingError(double stop_position_tracking_error);

  CartesianImpedanceControlCommandBuilder& SetStopOrientationTrackingError(double stop_orientation_tracking_error);

  CartesianImpedanceControlCommandBuilder& SetStopJointPositionTrackingError(double stop_joint_position_tracking_error);

  CartesianImpedanceControlCommandBuilder& SetJointStiffness(const Eigen::VectorXd& stiffness);

  CartesianImpedanceControlCommandBuilder& SetJointDampingRatio(double damping_ratio);

  CartesianImpedanceControlCommandBuilder& SetJointTorqueLimit(const Eigen::VectorXd& torque_limit);

  CartesianImpedanceControlCommandBuilder& AddJointLimit(const std::string& joint_name, double lower, double upper);

  CartesianImpedanceControlCommandBuilder& SetResetReference(bool reset_reference);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<CartesianImpedanceControlCommandBuilderImpl> impl_;

  friend class ArmCommandBuilderImpl;
  friend class TorsoCommandBuilderImpl;
  friend class BodyCommandBuilderImpl;
};

class RBY1_SDK_API GravityCompensationCommandBuilder {
 public:
  GravityCompensationCommandBuilder();

  ~GravityCompensationCommandBuilder();

  GravityCompensationCommandBuilder& SetCommandHeader(const CommandHeaderBuilder& builder);

  GravityCompensationCommandBuilder& SetOn(bool on);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<GravityCompensationCommandBuilderImpl> impl_;

  friend class BodyCommandBuilderImpl;
  friend class ArmCommandBuilderImpl;
  friend class TorsoCommandBuilderImpl;
};

class RBY1_SDK_API ArmCommandBuilder {
 public:
  ArmCommandBuilder();

  ArmCommandBuilder(const JointPositionCommandBuilder& builder);

  ArmCommandBuilder(const GravityCompensationCommandBuilder& builder);

  ArmCommandBuilder(const CartesianCommandBuilder& builder);

  ArmCommandBuilder(const ImpedanceControlCommandBuilder& builder);

  ArmCommandBuilder(const JointImpedanceControlCommandBuilder& builder);

  ArmCommandBuilder(const CartesianImpedanceControlCommandBuilder& builder);

  ~ArmCommandBuilder();

  ArmCommandBuilder& SetCommand(const JointPositionCommandBuilder& builder);

  ArmCommandBuilder& SetCommand(const GravityCompensationCommandBuilder& builder);

  ArmCommandBuilder& SetCommand(const CartesianCommandBuilder& builder);

  ArmCommandBuilder& SetCommand(const ImpedanceControlCommandBuilder& builder);

  ArmCommandBuilder& SetCommand(const JointImpedanceControlCommandBuilder& builder);

  ArmCommandBuilder& SetCommand(const CartesianImpedanceControlCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<ArmCommandBuilderImpl> impl_;

  friend class BodyComponentBasedCommandBuilderImpl;
};

class RBY1_SDK_API TorsoCommandBuilder {
 public:
  TorsoCommandBuilder();

  TorsoCommandBuilder(const JointPositionCommandBuilder& builder);

  TorsoCommandBuilder(const GravityCompensationCommandBuilder& builder);

  TorsoCommandBuilder(const CartesianCommandBuilder& builder);

  TorsoCommandBuilder(const ImpedanceControlCommandBuilder& builder);

  TorsoCommandBuilder(const OptimalControlCommandBuilder& builder);

  TorsoCommandBuilder(const JointImpedanceControlCommandBuilder& builder);

  TorsoCommandBuilder(const CartesianImpedanceControlCommandBuilder& builder);

  TorsoCommandBuilder(const JointGroupPositionCommandBuilder& builder);

  ~TorsoCommandBuilder();

  TorsoCommandBuilder& SetCommand(const JointPositionCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const GravityCompensationCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const CartesianCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const ImpedanceControlCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const OptimalControlCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const JointImpedanceControlCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const CartesianImpedanceControlCommandBuilder& builder);

  TorsoCommandBuilder& SetCommand(const JointGroupPositionCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<TorsoCommandBuilderImpl> impl_;

  friend class BodyComponentBasedCommandBuilderImpl;
};

class RBY1_SDK_API BodyComponentBasedCommandBuilder {
 public:
  BodyComponentBasedCommandBuilder();

  ~BodyComponentBasedCommandBuilder();

  BodyComponentBasedCommandBuilder& SetRightArmCommand(const ArmCommandBuilder& builder);

  BodyComponentBasedCommandBuilder& SetLeftArmCommand(const ArmCommandBuilder& builder);

  BodyComponentBasedCommandBuilder& SetTorsoCommand(const TorsoCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<BodyComponentBasedCommandBuilderImpl> impl_;

  friend class BodyCommandBuilderImpl;
};

class RBY1_SDK_API BodyCommandBuilder {
 public:
  BodyCommandBuilder();

  BodyCommandBuilder(const JointPositionCommandBuilder& builder);

  BodyCommandBuilder(const OptimalControlCommandBuilder& builder);

  BodyCommandBuilder(const GravityCompensationCommandBuilder& builder);

  BodyCommandBuilder(const CartesianCommandBuilder& builder);

  BodyCommandBuilder(const BodyComponentBasedCommandBuilder& builder);

  BodyCommandBuilder(const JointImpedanceControlCommandBuilder& builder);

  BodyCommandBuilder(const CartesianImpedanceControlCommandBuilder& builder);

  ~BodyCommandBuilder();

  BodyCommandBuilder& SetCommand(const JointPositionCommandBuilder& builder);

  BodyCommandBuilder& SetCommand(const OptimalControlCommandBuilder& builder);

  BodyCommandBuilder& SetCommand(const GravityCompensationCommandBuilder& builder);

  BodyCommandBuilder& SetCommand(const CartesianCommandBuilder& builder);

  BodyCommandBuilder& SetCommand(const BodyComponentBasedCommandBuilder& builder);

  BodyCommandBuilder& SetCommand(const JointImpedanceControlCommandBuilder& builder);

  BodyCommandBuilder& SetCommand(const CartesianImpedanceControlCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<BodyCommandBuilderImpl> impl_;

  friend class ComponentBasedCommandBuilderImpl;
};

class RBY1_SDK_API MobilityCommandBuilder {
 public:
  MobilityCommandBuilder();

  MobilityCommandBuilder(const JointVelocityCommandBuilder& builder);

  MobilityCommandBuilder(const SE2VelocityCommandBuilder& builder);

  ~MobilityCommandBuilder();

  MobilityCommandBuilder& SetCommand(const JointVelocityCommandBuilder& builder);

  MobilityCommandBuilder& SetCommand(const SE2VelocityCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<MobilityCommandBuilderImpl> impl_;

  friend class ComponentBasedCommandBuilderImpl;
};

class RBY1_SDK_API HeadCommandBuilder {
 public:
  HeadCommandBuilder();

  HeadCommandBuilder(const JointPositionCommandBuilder& builder);

  ~HeadCommandBuilder();

  HeadCommandBuilder& SetCommand(const JointPositionCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<HeadCommandBuilderImpl> impl_;

  friend class ComponentBasedCommandBuilderImpl;
};

class RBY1_SDK_API ComponentBasedCommandBuilder {
 public:
  ComponentBasedCommandBuilder();

  ~ComponentBasedCommandBuilder();

  ComponentBasedCommandBuilder& SetMobilityCommand(const MobilityCommandBuilder& builder);

  ComponentBasedCommandBuilder& SetBodyCommand(const BodyCommandBuilder& builder);

  ComponentBasedCommandBuilder& SetHeadCommand(const HeadCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<ComponentBasedCommandBuilderImpl> impl_;

  friend class RobotCommandBuilderImpl;
};

class RBY1_SDK_API WholeBodyCommandBuilder {
 public:
  WholeBodyCommandBuilder();

  WholeBodyCommandBuilder(const StopCommandBuilder& builder);

  WholeBodyCommandBuilder(const RealTimeControlCommandBuilder& builder);

  ~WholeBodyCommandBuilder();

  WholeBodyCommandBuilder& SetCommand(const StopCommandBuilder& builder);

  WholeBodyCommandBuilder& SetCommand(const RealTimeControlCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<WholeBodyCommandBuilderImpl> impl_;

  friend class RobotCommandBuilderImpl;
};

class RBY1_SDK_API RobotCommandBuilder {
 public:
  RobotCommandBuilder();

  RobotCommandBuilder(const WholeBodyCommandBuilder& builder);

  RobotCommandBuilder(const ComponentBasedCommandBuilder& builder);

  RobotCommandBuilder(const JogCommandBuilder& builder);

  ~RobotCommandBuilder();

  RobotCommandBuilder& SetCommand(const WholeBodyCommandBuilder& builder);

  RobotCommandBuilder& SetCommand(const ComponentBasedCommandBuilder& builder);

  RobotCommandBuilder& SetCommand(const JogCommandBuilder& builder);

 private:
  [[nodiscard]] void* Build() const;

 private:
  std::unique_ptr<RobotCommandBuilderImpl> impl_;

  template <typename T>
  friend class RobotImpl;

  template <typename T>
  friend class RobotCommandStreamHandlerImpl;
};

}  // namespace rb