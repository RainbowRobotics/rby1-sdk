#include "rby1-sdk/robot_command_builder.h"

#include "rb/api/robot_command.pb.h"

namespace rb {

api::SE3Pose ToProto(const math::SE3::MatrixType& T) {
  api::SE3Pose pose;

  auto* position = pose.mutable_position();
  position->set_x(T(0, 3));
  position->set_y(T(1, 3));
  position->set_z(T(2, 3));

  const auto& r = math::SO3::ToEulerAngle(math::SE3::GetRotation(T), math::EulerAngleType::ZYX);
  auto* euler = pose.mutable_euler();
  euler->set_z(r(0));
  euler->set_y(r(1));
  euler->set_x(r(2));

  return pose;
}

class CommandHeaderBuilderImpl {
 public:
  CommandHeaderBuilderImpl() : req_(std::make_unique<api::CommandHeader::Request>()) {}

  ~CommandHeaderBuilderImpl() = default;

  void SetControlHoldTime(double control_hold_time) {
    auto seconds = (int64_t)(std::floor(control_hold_time));
    auto nanos = (int32_t)((control_hold_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_control_hold_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  api::CommandHeader::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::CommandHeader::Request> req_;
};

class JointPositionCommandBuilderImpl {
 public:
  JointPositionCommandBuilderImpl() : req_(std::make_unique<api::JointPositionCommand::Request>()) {}

  ~JointPositionCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetPosition(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_position();
    for (int i = 0; i < v.size(); i++) {
      req_->add_position(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetVelocityLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_velocity_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_velocity_limit(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetAccelerationLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_acceleration_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_acceleration_limit(v(i));
    }
  }

  api::JointPositionCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::JointPositionCommand::Request> req_;
};

class JointGroupPositionCommandBuilderImpl {
 public:
  JointGroupPositionCommandBuilderImpl() : req_(std::make_unique<api::JointGroupPositionCommand::Request>()) {}

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetJointNames(const std::vector<std::string>& joint_names) {
    req_->clear_joint_names();
    for (const auto& name : joint_names) {
      req_->add_joint_names(name);
    }
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetPosition(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_position();
    for (int i = 0; i < v.size(); i++) {
      req_->add_position(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetVelocityLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_velocity_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_velocity_limit(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetAccelerationLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_acceleration_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_acceleration_limit(v(i));
    }
  }

  api::JointGroupPositionCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::JointGroupPositionCommand::Request> req_;
};

class JointImpedanceControlCommandBuilderImpl {
 public:
  JointImpedanceControlCommandBuilderImpl() : req_(std::make_unique<api::JointImpedanceControlCommand::Request>()) {}

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetPosition(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_position();
    for (int i = 0; i < v.size(); i++) {
      req_->add_position(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetVelocityLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_velocity_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_velocity_limit(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetAccelerationLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_acceleration_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_acceleration_limit(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetStiffness(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_stiffness();
    for (int i = 0; i < v.size(); i++) {
      req_->add_stiffness(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetTorqueLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_torque_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_torque_limit(v(i));
    }
  }

  void SetDampingRatio(double damping_ratio) { req_->mutable_damping_ratio()->set_value(damping_ratio); }

  api::JointImpedanceControlCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::JointImpedanceControlCommand::Request> req_;
};

class OptimalControlCommandBuilderImpl {
 public:
  OptimalControlCommandBuilderImpl() : req_(std::make_unique<api::OptimalControlCommand::Request>()) {
    req_->mutable_velocity_tracking_gain()->set_value(0);
  }

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void AddCartesianTarget(const std::string& ref_link_name, const std::string& link_name,
                          const math::SE3::MatrixType& T, double translation_weight, double rotation_weight) {
    auto& target = *req_->add_cartesian_costs();
    target.set_ref_link_name(ref_link_name);
    target.set_link_name(link_name);
    *target.mutable_t() = ToProto(T);
    target.set_translation_weight(translation_weight);
    target.set_rotation_weight(rotation_weight);
  }

  void SetCenterOfMassTarget(const std::string& ref_link_name, const Eigen::Vector3d& pose, double weight) {
    auto& target = *req_->mutable_center_of_mass_cost();
    target.set_ref_link_name(ref_link_name);
    auto& p = *target.mutable_pose();
    p.set_x(pose(0));
    p.set_y(pose(1));
    p.set_z(pose(2));
    target.set_weight(weight);
  }

  void AddJointPositionTarget(const std::string& joint_name, double target_position, double weight) {
    auto& target = *req_->add_joint_position_costs();
    target.set_joint_name(joint_name);
    target.set_target_position(target_position);
    target.set_weight(weight);
  }

  void SetErrorScaling(double error_scaling) { req_->mutable_error_scaling()->set_value(error_scaling); }

  void SetVelocityLimitScaling(double scaling) { req_->mutable_velocity_limit_scaling()->set_value(scaling); }

  void SetAccelerationLimitScaling(double scaling) { req_->mutable_acceleration_limit_scaling()->set_value(scaling); }

  void SetStopCost(double stop_cost) { req_->mutable_stop_cost()->set_value(stop_cost); }

  void SetMinDeltaCost(double min_delta_cost) { req_->mutable_min_delta_cost()->set_value(min_delta_cost); }

  void SetPatience(int patience) { req_->mutable_patience()->set_value(patience); }

  api::OptimalControlCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::OptimalControlCommand::Request> req_;
};

class ImpedanceControlCommandBuilderImpl {
 public:
  ImpedanceControlCommandBuilderImpl() : req_(std::make_unique<api::ImpedanceControlCommand::Request>()) {}

  ~ImpedanceControlCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetReferenceLinkName(const std::string& name) { req_->set_ref_link_name(name); }

  void SetLinkName(const std::string& name) { req_->set_link_name(name); }

  void SetTransformation(const math::SE3::MatrixType& T) { *req_->mutable_t() = ToProto(T); }

  void SetTranslationWeight(const Eigen::Vector3d& weight) {
    auto& w = *req_->mutable_translation_weight();
    w.set_x(weight(0));
    w.set_y(weight(1));
    w.set_z(weight(2));
  }

  void SetRotationWeight(const Eigen::Vector3d& weight) {
    auto& w = *req_->mutable_rotation_weight();
    w.set_x(weight(0));
    w.set_y(weight(1));
    w.set_z(weight(2));
  }

  void SetDampingRatio(double damping_ratio) { req_->mutable_damping_ratio()->set_value(damping_ratio); }

  api::ImpedanceControlCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::ImpedanceControlCommand::Request> req_;
};

class JointVelocityCommandBuilderImpl {
 public:
  JointVelocityCommandBuilderImpl() : req_(std::make_unique<api::JointVelocityCommand::Request>()) {}

  ~JointVelocityCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  void SetVelocity(const Eigen::VectorXd& velocity) {
    req_->clear_velocity();
    for (int i = 0; i < velocity.size(); i++) {
      req_->add_velocity(velocity(i));
    }
  }

  void SetAccelerationLimit(const Eigen::VectorXd& acceleration_limit) {
    req_->clear_acceleration_limit();
    for (int i = 0; i < acceleration_limit.size(); i++) {
      req_->add_acceleration_limit(acceleration_limit(i));
    }
  }

  api::JointVelocityCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::JointVelocityCommand::Request> req_;
};

class JogCommandBuilderImpl {
 public:
  JogCommandBuilderImpl() : req_(std::make_unique<api::JogCommand::Request>()) {}

  ~JogCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetJointName(const std::string& name) { req_->set_joint_name(name); }

  void SetVelocityLimit(double value) { req_->mutable_velocity_limit()->set_value(value); }

  void SetAccelerationLimit(double value) { req_->mutable_acceleration_limit()->set_value(value); }

  void SetCommand(JogCommandBuilder::AbsolutePosition absolute_position) {
    req_->set_absolute_position(absolute_position.value());
  }

  void SetCommand(JogCommandBuilder::RelativePosition relative_position) {
    req_->set_relative_position(relative_position.value());
  }

  void SetCommand(JogCommandBuilder::OneStep one_step) { req_->set_one_step(one_step.value()); }

  api::JogCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::JogCommand::Request> req_;
};

class SE2VelocityCommandBuilderImpl {
 public:
  SE2VelocityCommandBuilderImpl() : req_(std::make_unique<api::SE2VelocityCommand::Request>()) {}

  ~SE2VelocityCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  void SetVelocity(const Eigen::Vector2d& linear, double angular) {
    auto& velocity = *req_->mutable_velocity();
    auto& l = *velocity.mutable_linear();
    l.set_x(linear(0));
    l.set_y(linear(1));
    velocity.set_angular(angular);
  }

  void SetAccelerationLimit(const Eigen::Vector2d& linear, double angular) {
    auto& acceleration_limit = *req_->mutable_acceleration_limit();
    auto& l = *acceleration_limit.mutable_linear();
    l.set_x(linear(0));
    l.set_y(linear(1));
    acceleration_limit.set_angular(angular);
  }

  api::SE2VelocityCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::SE2VelocityCommand::Request> req_;
};

class StopCommandBuilderImpl {
 public:
  StopCommandBuilderImpl() : req_(std::make_unique<api::StopCommand::Request>()) {}

  ~StopCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  api::StopCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::StopCommand::Request> req_;
};

class RealTimeControlCommandBuilderImpl {
 public:
  RealTimeControlCommandBuilderImpl() : req_(std::make_unique<api::RealTimeControlCommand::Request>()) {}

  ~RealTimeControlCommandBuilderImpl() = default;

  void SetPort(int port) { req_->set_port(port); }

  api::RealTimeControlCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::RealTimeControlCommand::Request> req_;
};

class CartesianCommandBuilderImpl {
 public:
  CartesianCommandBuilderImpl() : req_(std::make_unique<api::CartesianCommand::Request>()) {}

  ~CartesianCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  void AddTarget(const std::string& ref_link_name, const std::string& link_name, const math::SE3::MatrixType& T,
                 double linear_velocity_limit, double angular_velocity_limit, double acceleration_limit_scaling) {
    auto& target = *req_->add_targets();
    target.set_ref_link_name(ref_link_name);
    target.set_link_name(link_name);
    *target.mutable_t() = ToProto(T);
    target.mutable_linear_velocity_limit()->set_value(linear_velocity_limit);
    target.mutable_angular_velocity_limit()->set_value(angular_velocity_limit);
    target.mutable_acceleration_limit_scaling()->set_value(acceleration_limit_scaling);
  }

  void AddJointPositionTarget(const std::string& joint_name, double target_position,
                              std::optional<double> velocity_limit, std::optional<double> acceleration_limit) {
    auto& target = *req_->add_joint_position_targets();
    target.set_joint_name(joint_name);
    target.set_target_position(target_position);
    if (velocity_limit.has_value()) {
      target.mutable_velocity_limit()->set_value(velocity_limit.value());
    }
    if (acceleration_limit.has_value()) {
      target.mutable_acceleration_limit()->set_value(acceleration_limit.value());
    }
  }

  void SetStopPositionTrackingError(double stop_position_tracking_error) {
    req_->mutable_stop_position_tracking_error()->set_value(stop_position_tracking_error);
  }

  void SetStopOrientationTrackingError(double stop_orientation_tracking_error) {
    req_->mutable_stop_orientation_tracking_error()->set_value(stop_orientation_tracking_error);
  }

  void SetStopJointPositionTrackingError(double stop_joint_position_tracking_error) {
    req_->mutable_stop_joint_position_tracking_error()->set_value(stop_joint_position_tracking_error);
  }

  api::CartesianCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::CartesianCommand::Request> req_;
};

class CartesianImpedanceControlCommandBuilderImpl {
 public:
  CartesianImpedanceControlCommandBuilderImpl()
      : req_(std::make_unique<api::CartesianImpedanceControlCommand::Request>()) {}

  ~CartesianImpedanceControlCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetMinimumTime(double minimum_time) {
    auto seconds = (int64_t)(std::floor(minimum_time));
    auto nanos = (int32_t)((minimum_time - (double)seconds) * 1.e9);
    auto& t = *req_->mutable_minimum_time();
    t.set_seconds(seconds);
    t.set_nanos(nanos);
  }

  void AddTarget(const std::string& ref_link_name,                 //
                 const std::string& link_name,                     //
                 const math::SE3::MatrixType& T,                   //
                 std::optional<double> linear_velocity_limit,      //
                 std::optional<double> angular_velocity_limit,     //
                 std::optional<double> linear_acceleration_limit,  //
                 std::optional<double> angular_acceleration_limit  //
  ) {
    auto& target = *req_->add_targets();
    target.set_ref_link_name(ref_link_name);
    target.set_link_name(link_name);
    *target.mutable_t() = ToProto(T);
    if (linear_velocity_limit.has_value()) {
      target.mutable_linear_velocity_limit()->set_value(linear_velocity_limit.value());
    }
    if (angular_velocity_limit.has_value()) {
      target.mutable_angular_velocity_limit()->set_value(angular_velocity_limit.value());
    }
    if (linear_acceleration_limit.has_value()) {
      target.mutable_linear_acceleration_limit()->set_value(linear_acceleration_limit.value());
    }
    if (angular_acceleration_limit.has_value()) {
      target.mutable_angular_acceleration_limit()->set_value(angular_acceleration_limit.value());
    }
  }

  void AddJointPositionTarget(const std::string& joint_name, double target_position,
                              std::optional<double> velocity_limit, std::optional<double> acceleration_limit) {
    auto& target = *req_->add_joint_position_targets();
    target.set_joint_name(joint_name);
    target.set_target_position(target_position);
    if (velocity_limit.has_value()) {
      target.mutable_velocity_limit()->set_value(velocity_limit.value());
    }
    if (acceleration_limit.has_value()) {
      target.mutable_acceleration_limit()->set_value(acceleration_limit.value());
    }
  }

  void SetStopPositionTrackingError(double stop_position_tracking_error) {
    req_->mutable_stop_position_tracking_error()->set_value(stop_position_tracking_error);
  }

  void SetStopOrientationTrackingError(double stop_orientation_tracking_error) {
    req_->mutable_stop_orientation_tracking_error()->set_value(stop_orientation_tracking_error);
  }

  void SetStopJointPositionTrackingError(double stop_joint_position_tracking_error) {
    req_->mutable_stop_joint_position_tracking_error()->set_value(stop_joint_position_tracking_error);
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetJointStiffness(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_joint_stiffness();
    for (int i = 0; i < v.size(); i++) {
      req_->add_joint_stiffness(v(i));
    }
  }

  template <typename Derived, typename = std::enable_if_t<Derived::IsVectorAtCompileTime, void>>
  void SetJointTorqueLimit(const Eigen::MatrixBase<Derived>& v) {
    req_->clear_joint_torque_limit();
    for (int i = 0; i < v.size(); i++) {
      req_->add_joint_torque_limit(v(i));
    }
  }

  void SetJointDampingRatio(double damping_ratio) { req_->mutable_joint_damping_ratio()->set_value(damping_ratio); }

  void AddJointLimit(const std::string& joint_name, double lower, double upper) {
    auto& joint_limit = *req_->add_joint_limits();
    joint_limit.set_joint_name(joint_name);
    joint_limit.set_lower(lower);
    joint_limit.set_upper(upper);
  }

  void SetResetReference(bool reset_reference) { req_->mutable_reset_reference()->set_value(reset_reference); }

  api::CartesianImpedanceControlCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::CartesianImpedanceControlCommand::Request> req_;
};

class GravityCompensationCommandBuilderImpl {
 public:
  GravityCompensationCommandBuilderImpl() : req_(std::make_unique<api::GravityCompensationCommand::Request>()) {}

  ~GravityCompensationCommandBuilderImpl() = default;

  void SetCommandHeader(const CommandHeaderBuilder& builder) {
    req_->set_allocated_command_header(static_cast<api::CommandHeader::Request*>(builder.Build()));
  }

  void SetOn(bool on) { req_->set_on(on); }

  api::GravityCompensationCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::GravityCompensationCommand::Request> req_;
};

class ArmCommandBuilderImpl {
 public:
  ArmCommandBuilderImpl() : req_(std::make_unique<api::ArmCommand::Request>()) {}

  ~ArmCommandBuilderImpl() = default;

  void SetCommand(const JointPositionCommandBuilder& builder) {
    req_->set_allocated_joint_position_command(static_cast<api::JointPositionCommand::Request*>(builder.Build()));
  }

  void SetCommand(const GravityCompensationCommandBuilder& builder) {
    req_->set_allocated_gravity_compensation_command(
        static_cast<api::GravityCompensationCommand::Request*>(builder.Build()));
  }

  void SetCommand(const CartesianCommandBuilder& builder) {
    req_->set_allocated_cartesian_command(static_cast<api::CartesianCommand::Request*>(builder.Build()));
  }

  void SetCommand(const ImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_impedance_control_command(static_cast<api::ImpedanceControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const JointImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_joint_impedance_control_command(
        static_cast<api::JointImpedanceControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const CartesianImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_cartesian_impedance_control_command(
        static_cast<api::CartesianImpedanceControlCommand::Request*>(builder.Build()));
  }

  api::ArmCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::ArmCommand::Request> req_;
};

class TorsoCommandBuilderImpl {
 public:
  TorsoCommandBuilderImpl() : req_(std::make_unique<api::TorsoCommand::Request>()) {}

  ~TorsoCommandBuilderImpl() = default;

  void SetCommand(const JointPositionCommandBuilder& builder) {
    req_->set_allocated_joint_position_command(static_cast<api::JointPositionCommand::Request*>(builder.Build()));
  }

  void SetCommand(const GravityCompensationCommandBuilder& builder) {
    req_->set_allocated_gravity_compensation_command(
        static_cast<api::GravityCompensationCommand::Request*>(builder.Build()));
  }

  void SetCommand(const CartesianCommandBuilder& builder) {
    req_->set_allocated_cartesian_command(static_cast<api::CartesianCommand::Request*>(builder.Build()));
  }

  void SetCommand(const ImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_impedance_control_command(static_cast<api::ImpedanceControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const OptimalControlCommandBuilder& builder) {
    req_->set_allocated_optimal_control_command(static_cast<api::OptimalControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const JointImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_joint_impedance_control_command(
        static_cast<api::JointImpedanceControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const CartesianImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_cartesian_impedance_control_command(
        static_cast<api::CartesianImpedanceControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const JointGroupPositionCommandBuilder& builder) {
    req_->set_allocated_joint_group_position_command(
        static_cast<api::JointGroupPositionCommand::Request*>(builder.Build()));
  }

  api::TorsoCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::TorsoCommand::Request> req_;
};

class BodyComponentBasedCommandBuilderImpl {
 public:
  BodyComponentBasedCommandBuilderImpl() : req_(std::make_unique<api::BodyComponentBasedCommand::Request>()) {}

  ~BodyComponentBasedCommandBuilderImpl() = default;

  void SetRightArmCommand(const ArmCommandBuilder& builder) {
    req_->set_allocated_right_arm_command(static_cast<api::ArmCommand::Request*>(builder.Build()));
  }

  void SetLeftArmCommand(const ArmCommandBuilder& builder) {
    req_->set_allocated_left_arm_command(static_cast<api::ArmCommand::Request*>(builder.Build()));
  }

  void SetTorsoCommand(const TorsoCommandBuilder& builder) {
    req_->set_allocated_torso_command(static_cast<api::TorsoCommand::Request*>(builder.Build()));
  }

  api::BodyComponentBasedCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::BodyComponentBasedCommand::Request> req_;
};

class BodyCommandBuilderImpl {
 public:
  BodyCommandBuilderImpl() : req_(std::make_unique<api::BodyCommand::Request>()) {}

  ~BodyCommandBuilderImpl() = default;

  void SetCommand(const JointPositionCommandBuilder& builder) {
    req_->set_allocated_joint_position_command(static_cast<api::JointPositionCommand::Request*>(builder.Build()));
  }

  void SetCommand(const OptimalControlCommandBuilder& builder) {
    req_->set_allocated_optimal_control_command(static_cast<api::OptimalControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const GravityCompensationCommandBuilder& builder) {
    req_->set_allocated_gravity_compensation_command(
        static_cast<api::GravityCompensationCommand::Request*>(builder.Build()));
  }

  void SetCommand(const CartesianCommandBuilder& builder) {
    req_->set_allocated_cartesian_command(static_cast<api::CartesianCommand::Request*>(builder.Build()));
  }

  void SetCommand(const BodyComponentBasedCommandBuilder& builder) {
    req_->set_allocated_body_component_based_command(
        static_cast<api::BodyComponentBasedCommand::Request*>(builder.Build()));
  }

  void SetCommand(const JointImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_joint_impedance_control_command(
        static_cast<api::JointImpedanceControlCommand::Request*>(builder.Build()));
  }

  void SetCommand(const CartesianImpedanceControlCommandBuilder& builder) {
    req_->set_allocated_cartesian_impedance_control_command(
        static_cast<api::CartesianImpedanceControlCommand::Request*>(builder.Build()));
  }

  api::BodyCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::BodyCommand::Request> req_;
};

class MobilityCommandBuilderImpl {
 public:
  MobilityCommandBuilderImpl() : req_(std::make_unique<api::MobilityCommand::Request>()) {}

  ~MobilityCommandBuilderImpl() = default;

  void SetCommand(const JointVelocityCommandBuilder& builder) {
    req_->set_allocated_joint_velocity_command(static_cast<api::JointVelocityCommand::Request*>(builder.Build()));
  }

  void SetCommand(const SE2VelocityCommandBuilder& builder) {
    req_->set_allocated_se2_velocity_command(static_cast<api::SE2VelocityCommand::Request*>(builder.Build()));
  }

  api::MobilityCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::MobilityCommand::Request> req_;
};

class HeadCommandBuilderImpl {
 public:
  HeadCommandBuilderImpl() : req_(std::make_unique<api::HeadCommand::Request>()) {}

  ~HeadCommandBuilderImpl() = default;

  void SetCommand(const JointPositionCommandBuilder& builder) {
    req_->set_allocated_joint_position_command(static_cast<api::JointPositionCommand::Request*>(builder.Build()));
  }

  api::HeadCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::HeadCommand::Request> req_;
};

class ComponentBasedCommandBuilderImpl {
 public:
  ComponentBasedCommandBuilderImpl() : req_(std::make_unique<api::ComponentBasedCommand::Request>()) {}

  ~ComponentBasedCommandBuilderImpl() = default;

  void SetMobilityCommand(const MobilityCommandBuilder& builder) {
    req_->set_allocated_mobility_command(static_cast<api::MobilityCommand::Request*>(builder.Build()));
  }

  void SetBodyCommand(const BodyCommandBuilder& builder) {
    req_->set_allocated_body_command(static_cast<api::BodyCommand::Request*>(builder.Build()));
  }

  void SetHeadCommand(const HeadCommandBuilder& builder) {
    req_->set_allocated_head_command(static_cast<api::HeadCommand::Request*>(builder.Build()));
  }

  api::ComponentBasedCommand::Request* Build() { return req_.release(); }

 public:
  std::unique_ptr<api::ComponentBasedCommand::Request> req_;
};

class WholeBodyCommandBuilderImpl {
 public:
  WholeBodyCommandBuilderImpl() : req_(std::make_unique<api::WholeBodyCommand::Request>()) {}

  ~WholeBodyCommandBuilderImpl() = default;

  void SetCommand(const StopCommandBuilder& builder) {
    req_->set_allocated_stop_command(static_cast<api::StopCommand::Request*>(builder.Build()));
  }

  void SetCommand(const RealTimeControlCommandBuilder& builder) {
    req_->set_allocated_real_time_control_command(static_cast<api::RealTimeControlCommand::Request*>(builder.Build()));
  }

  api::WholeBodyCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::WholeBodyCommand::Request> req_;
};

class RobotCommandBuilderImpl {
 public:
  RobotCommandBuilderImpl() : req_(std::make_unique<api::RobotCommand::Request>()) {}

  ~RobotCommandBuilderImpl() = default;

  void SetCommand(const WholeBodyCommandBuilder& builder) {
    req_->set_allocated_whole_body_command(static_cast<api::WholeBodyCommand::Request*>(builder.Build()));
  }

  void SetCommand(const ComponentBasedCommandBuilder& builder) {
    req_->set_allocated_component_based_command(static_cast<api::ComponentBasedCommand::Request*>(builder.Build()));
  }

  void SetCommand(const JogCommandBuilder& builder) {
    req_->set_allocated_jog_command(static_cast<api::JogCommand::Request*>(builder.Build()));
  }

  api::RobotCommand::Request* Build() { return req_.release(); }

 private:
  std::unique_ptr<api::RobotCommand::Request> req_;
};

/***************************************************************************************************
 *
 ***************************************************************************************************/

CommandHeaderBuilder::CommandHeaderBuilder() {
  impl_ = std::make_unique<CommandHeaderBuilderImpl>();
}

CommandHeaderBuilder::~CommandHeaderBuilder() = default;

CommandHeaderBuilder& CommandHeaderBuilder::SetControlHoldTime(double control_hold_time) {
  impl_->SetControlHoldTime(control_hold_time);
  return *this;
}

void* CommandHeaderBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

JointPositionCommandBuilder::JointPositionCommandBuilder() {
  impl_ = std::make_unique<JointPositionCommandBuilderImpl>();
}

JointPositionCommandBuilder::~JointPositionCommandBuilder() = default;

JointPositionCommandBuilder& JointPositionCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

JointPositionCommandBuilder& JointPositionCommandBuilder::SetMinimumTime(double minimum_time) {
  impl_->SetMinimumTime(minimum_time);
  return *this;
}

JointPositionCommandBuilder& JointPositionCommandBuilder::SetPosition(const Eigen::VectorXd& position) {
  impl_->SetPosition(position);
  return *this;
}

JointPositionCommandBuilder& JointPositionCommandBuilder::SetVelocityLimit(const Eigen::VectorXd& velocity_limit) {
  impl_->SetVelocityLimit(velocity_limit);
  return *this;
}

JointPositionCommandBuilder& JointPositionCommandBuilder::SetAccelerationLimit(
    const Eigen::VectorXd& acceleration_limit) {
  impl_->SetAccelerationLimit(acceleration_limit);
  return *this;
}

void* JointPositionCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

JointGroupPositionCommandBuilder::JointGroupPositionCommandBuilder() {
  impl_ = std::make_unique<JointGroupPositionCommandBuilderImpl>();
}

JointGroupPositionCommandBuilder::~JointGroupPositionCommandBuilder() = default;

JointGroupPositionCommandBuilder& JointGroupPositionCommandBuilder::SetCommandHeader(
    const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

JointGroupPositionCommandBuilder& JointGroupPositionCommandBuilder::SetJointNames(
    const std::vector<std::string>& joint_names) {
  impl_->SetJointNames(joint_names);
  return *this;
}

JointGroupPositionCommandBuilder& JointGroupPositionCommandBuilder::SetMinimumTime(double minimum_time) {
  impl_->SetMinimumTime(minimum_time);
  return *this;
}

JointGroupPositionCommandBuilder& JointGroupPositionCommandBuilder::SetPosition(const Eigen::VectorXd& position) {
  impl_->SetPosition(position);
  return *this;
}

JointGroupPositionCommandBuilder& JointGroupPositionCommandBuilder::SetVelocityLimit(
    const Eigen::VectorXd& velocity_limit) {
  impl_->SetVelocityLimit(velocity_limit);
  return *this;
}

JointGroupPositionCommandBuilder& JointGroupPositionCommandBuilder::SetAccelerationLimit(
    const Eigen::VectorXd& acceleration_limit) {
  impl_->SetAccelerationLimit(acceleration_limit);
  return *this;
}

void* JointGroupPositionCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

JointImpedanceControlCommandBuilder::JointImpedanceControlCommandBuilder() {
  impl_ = std::make_unique<JointImpedanceControlCommandBuilderImpl>();
}

JointImpedanceControlCommandBuilder::~JointImpedanceControlCommandBuilder() = default;

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetCommandHeader(
    const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetMinimumTime(double minimum_time) {
  impl_->SetMinimumTime(minimum_time);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetPosition(const Eigen::VectorXd& position) {
  impl_->SetPosition(position);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetVelocityLimit(
    const Eigen::VectorXd& velocity_limit) {
  impl_->SetVelocityLimit(velocity_limit);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetAccelerationLimit(
    const Eigen::VectorXd& acceleration_limit) {
  impl_->SetAccelerationLimit(acceleration_limit);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetStiffness(
    const Eigen::VectorXd& stiffness) {
  impl_->SetStiffness(stiffness);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetTorqueLimit(
    const Eigen::VectorXd& torque_limit) {
  impl_->SetTorqueLimit(torque_limit);
  return *this;
}

JointImpedanceControlCommandBuilder& JointImpedanceControlCommandBuilder::SetDampingRatio(double damping_ratio) {
  impl_->SetDampingRatio(damping_ratio);
  return *this;
}

void* JointImpedanceControlCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

OptimalControlCommandBuilder::OptimalControlCommandBuilder()
    : impl_(std::make_unique<OptimalControlCommandBuilderImpl>()) {}

OptimalControlCommandBuilder::~OptimalControlCommandBuilder() = default;

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::AddCartesianTarget(const std::string& ref_link_name,
                                                                               const std::string& link_name,
                                                                               const math::SE3::MatrixType& T,
                                                                               double translation_weight,
                                                                               double rotation_weight) {
  impl_->AddCartesianTarget(ref_link_name, link_name, T, translation_weight, rotation_weight);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetCenterOfMassTarget(const std::string& ref_link_name,
                                                                                  const Eigen::Vector3d& pose,
                                                                                  double weight) {
  impl_->SetCenterOfMassTarget(ref_link_name, pose, weight);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::AddJointPositionTarget(const std::string& joint_name,
                                                                                   double target_position,
                                                                                   double weight) {
  impl_->AddJointPositionTarget(joint_name, target_position, weight);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetErrorScaling(double error_scaling) {
  impl_->SetErrorScaling(error_scaling);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetVelocityLimitScaling(double velocity_limit_scaling) {
  impl_->SetVelocityLimitScaling(velocity_limit_scaling);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetAccelerationLimitScaling(
    double acceleration_limit_scaling) {
  impl_->SetAccelerationLimitScaling(acceleration_limit_scaling);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetStopCost(double stop_cost) {
  impl_->SetStopCost(stop_cost);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetMinDeltaCost(double min_delta_cost) {
  impl_->SetMinDeltaCost(min_delta_cost);
  return *this;
}

OptimalControlCommandBuilder& OptimalControlCommandBuilder::SetPatience(int patience) {
  impl_->SetPatience(patience);
  return *this;
}

void* OptimalControlCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

ImpedanceControlCommandBuilder::ImpedanceControlCommandBuilder()
    : impl_(std::make_unique<ImpedanceControlCommandBuilderImpl>()) {}

ImpedanceControlCommandBuilder::~ImpedanceControlCommandBuilder() = default;

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetReferenceLinkName(const std::string& name) {
  impl_->SetReferenceLinkName(name);
  return *this;
}

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetLinkName(const std::string& name) {
  impl_->SetLinkName(name);
  return *this;
}

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetTransformation(const math::SE3::MatrixType& T) {
  impl_->SetTransformation(T);
  return *this;
}

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetTranslationWeight(const Eigen::Vector3d& weight) {
  impl_->SetTranslationWeight(weight);
  return *this;
}

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetRotationWeight(const Eigen::Vector3d& weight) {
  impl_->SetRotationWeight(weight);
  return *this;
}

ImpedanceControlCommandBuilder& ImpedanceControlCommandBuilder::SetDampingRatio(double damping_ratio) {
  impl_->SetDampingRatio(damping_ratio);
  return *this;
}

void* ImpedanceControlCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

JointVelocityCommandBuilder::JointVelocityCommandBuilder()
    : impl_(std::make_unique<JointVelocityCommandBuilderImpl>()) {}

JointVelocityCommandBuilder::~JointVelocityCommandBuilder() = default;

JointVelocityCommandBuilder& JointVelocityCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

JointVelocityCommandBuilder& JointVelocityCommandBuilder::SetMinimumTime(double minimum_time) {
  impl_->SetMinimumTime(minimum_time);
  return *this;
}

JointVelocityCommandBuilder& JointVelocityCommandBuilder::SetVelocity(const Eigen::VectorXd& velocity) {
  impl_->SetVelocity(velocity);
  return *this;
}

JointVelocityCommandBuilder& JointVelocityCommandBuilder::SetAccelerationLimit(
    const Eigen::VectorXd& acceleration_limit) {
  impl_->SetAccelerationLimit(acceleration_limit);
  return *this;
}

void* JointVelocityCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

JogCommandBuilder::JogCommandBuilder() : impl_(std::make_unique<JogCommandBuilderImpl>()) {}

JogCommandBuilder::~JogCommandBuilder() = default;

JogCommandBuilder& JogCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

JogCommandBuilder& JogCommandBuilder::SetJointName(const std::string& name) {
  impl_->SetJointName(name);
  return *this;
}

JogCommandBuilder& JogCommandBuilder::SetVelocityLimit(double value) {
  impl_->SetVelocityLimit(value);
  return *this;
}

JogCommandBuilder& JogCommandBuilder::SetAccelerationLimit(double value) {
  impl_->SetAccelerationLimit(value);
  return *this;
}

JogCommandBuilder& JogCommandBuilder::SetCommand(JogCommandBuilder::AbsolutePosition absolute_position) {
  impl_->SetCommand(absolute_position);
  return *this;
}

JogCommandBuilder& JogCommandBuilder::SetCommand(JogCommandBuilder::RelativePosition relative_position) {
  impl_->SetCommand(relative_position);
  return *this;
}

JogCommandBuilder& JogCommandBuilder::SetCommand(JogCommandBuilder::OneStep one_step) {
  impl_->SetCommand(one_step);
  return *this;
}

void* JogCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

SE2VelocityCommandBuilder::SE2VelocityCommandBuilder() : impl_(std::make_unique<SE2VelocityCommandBuilderImpl>()) {}

SE2VelocityCommandBuilder::~SE2VelocityCommandBuilder() = default;

SE2VelocityCommandBuilder& SE2VelocityCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

SE2VelocityCommandBuilder& SE2VelocityCommandBuilder::SetMinimumTime(double minimum_time) {
  impl_->SetMinimumTime(minimum_time);
  return *this;
}

SE2VelocityCommandBuilder& SE2VelocityCommandBuilder::SetVelocity(const Eigen::Vector2d& linear, double angular) {
  impl_->SetVelocity(linear, angular);
  return *this;
}

SE2VelocityCommandBuilder& SE2VelocityCommandBuilder::SetAccelerationLimit(const Eigen::Vector2d& linear,
                                                                           double angular) {
  impl_->SetAccelerationLimit(linear, angular);
  return *this;
}

void* SE2VelocityCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

StopCommandBuilder::StopCommandBuilder() : impl_(std::make_unique<StopCommandBuilderImpl>()) {}

StopCommandBuilder::~StopCommandBuilder() = default;

StopCommandBuilder& StopCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

void* StopCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

RealTimeControlCommandBuilder::RealTimeControlCommandBuilder()
    : impl_(std::make_unique<RealTimeControlCommandBuilderImpl>()) {}

RealTimeControlCommandBuilder::~RealTimeControlCommandBuilder() = default;

RealTimeControlCommandBuilder& RealTimeControlCommandBuilder::SetPort(int port) {
  impl_->SetPort(port);
  return *this;
}

void* RealTimeControlCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

CartesianCommandBuilder::CartesianCommandBuilder() {
  impl_ = std::make_unique<CartesianCommandBuilderImpl>();
}

CartesianCommandBuilder::~CartesianCommandBuilder() = default;

CartesianCommandBuilder& CartesianCommandBuilder::SetCommandHeader(const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

CartesianCommandBuilder& CartesianCommandBuilder::SetMinimumTime(double minimum_time) {
  impl_->SetMinimumTime(minimum_time);
  return *this;
}

CartesianCommandBuilder& CartesianCommandBuilder::AddTarget(const std::string& ref_link_name,
                                                            const std::string& link_name,
                                                            const math::SE3::MatrixType& T,
                                                            double linear_velocity_limit, double angular_velocity_limit,
                                                            double acceleration_limit_scaling) {
  impl_->AddTarget(ref_link_name, link_name, T, linear_velocity_limit, angular_velocity_limit,
                   acceleration_limit_scaling);
  return *this;
}

CartesianCommandBuilder& CartesianCommandBuilder::AddJointPositionTarget(const std::string& joint_name,
                                                                         double target_position,
                                                                         std::optional<double> velocity_limit,
                                                                         std::optional<double> acceleration_limit) {
  impl_->AddJointPositionTarget(joint_name, target_position, velocity_limit, acceleration_limit);
  return *this;
}

CartesianCommandBuilder& CartesianCommandBuilder::SetStopPositionTrackingError(double stop_position_tracking_error) {
  impl_->SetStopPositionTrackingError(stop_position_tracking_error);
  return *this;
}

CartesianCommandBuilder& CartesianCommandBuilder::SetStopOrientationTrackingError(
    double stop_orientation_tracking_error) {
  impl_->SetStopOrientationTrackingError(stop_orientation_tracking_error);
  return *this;
}

CartesianCommandBuilder& CartesianCommandBuilder::SetStopJointPositionTrackingError(
    double stop_joint_position_tracking_error) {
  impl_->SetStopJointPositionTrackingError(stop_joint_position_tracking_error);
  return *this;
}

void* CartesianCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

CartesianImpedanceControlCommandBuilder::CartesianImpedanceControlCommandBuilder() {
  impl_ = std::make_unique<CartesianImpedanceControlCommandBuilderImpl>();
}

CartesianImpedanceControlCommandBuilder::~CartesianImpedanceControlCommandBuilder() = default;

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetCommandHeader(
    const rb::CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetMinimumTime(double minium_time) {
  impl_->SetMinimumTime(minium_time);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::AddTarget(
    const std::string& ref_link_name,                 //
    const std::string& link_name,                     //
    const math::SE3::MatrixType& T,                   //
    std::optional<double> linear_velocity_limit,      //
    std::optional<double> angular_velocity_limit,     //
    std::optional<double> linear_acceleration_limit,  //
    std::optional<double> angular_acceleration_limit  //
) {
  impl_->AddTarget(ref_link_name, link_name, T, linear_velocity_limit, angular_velocity_limit,
                   linear_acceleration_limit, angular_acceleration_limit);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::AddJointPositionTarget(
    const std::string& joint_name, double target_position, std::optional<double> velocity_limit,
    std::optional<double> acceleration_limit) {
  impl_->AddJointPositionTarget(joint_name, target_position, velocity_limit, acceleration_limit);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetStopPositionTrackingError(
    double stop_position_tracking_error) {
  impl_->SetStopPositionTrackingError(stop_position_tracking_error);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetStopOrientationTrackingError(
    double stop_orientation_tracking_error) {
  impl_->SetStopOrientationTrackingError(stop_orientation_tracking_error);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetStopJointPositionTrackingError(
    double stop_joint_position_tracking_error) {
  impl_->SetStopJointPositionTrackingError(stop_joint_position_tracking_error);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetJointStiffness(
    const Eigen::VectorXd& stiffness) {
  impl_->SetJointStiffness(stiffness);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetJointTorqueLimit(
    const Eigen::VectorXd& torque_limit) {
  impl_->SetJointTorqueLimit(torque_limit);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetJointDampingRatio(
    double damping_ratio) {
  impl_->SetJointDampingRatio(damping_ratio);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::AddJointLimit(
    const std::string& joint_name, double lower, double upper) {
  impl_->AddJointLimit(joint_name, lower, upper);
  return *this;
}

CartesianImpedanceControlCommandBuilder& CartesianImpedanceControlCommandBuilder::SetResetReference(
    bool reset_reference) {
  impl_->SetResetReference(reset_reference);
  return *this;
}

void* CartesianImpedanceControlCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

GravityCompensationCommandBuilder::GravityCompensationCommandBuilder() {
  impl_ = std::make_unique<GravityCompensationCommandBuilderImpl>();
}

GravityCompensationCommandBuilder::~GravityCompensationCommandBuilder() = default;

GravityCompensationCommandBuilder& GravityCompensationCommandBuilder::SetCommandHeader(
    const CommandHeaderBuilder& builder) {
  impl_->SetCommandHeader(builder);
  return *this;
}

GravityCompensationCommandBuilder& GravityCompensationCommandBuilder::SetOn(bool on) {
  impl_->SetOn(on);
  return *this;
}

void* GravityCompensationCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

ArmCommandBuilder::ArmCommandBuilder() {
  impl_ = std::make_unique<ArmCommandBuilderImpl>();
}

ArmCommandBuilder::ArmCommandBuilder(const JointPositionCommandBuilder& builder) : ArmCommandBuilder() {
  SetCommand(builder);
}

ArmCommandBuilder::ArmCommandBuilder(const GravityCompensationCommandBuilder& builder) : ArmCommandBuilder() {
  SetCommand(builder);
}

ArmCommandBuilder::ArmCommandBuilder(const CartesianCommandBuilder& builder) : ArmCommandBuilder() {
  SetCommand(builder);
}

ArmCommandBuilder::ArmCommandBuilder(const ImpedanceControlCommandBuilder& builder) : ArmCommandBuilder() {
  SetCommand(builder);
}

ArmCommandBuilder::ArmCommandBuilder(const JointImpedanceControlCommandBuilder& builder) : ArmCommandBuilder() {
  SetCommand(builder);
}

ArmCommandBuilder::ArmCommandBuilder(const CartesianImpedanceControlCommandBuilder& builder) : ArmCommandBuilder() {
  SetCommand(builder);
}

ArmCommandBuilder::~ArmCommandBuilder() = default;

ArmCommandBuilder& ArmCommandBuilder::SetCommand(const JointPositionCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

ArmCommandBuilder& ArmCommandBuilder::SetCommand(const GravityCompensationCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

ArmCommandBuilder& ArmCommandBuilder::SetCommand(const CartesianCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

ArmCommandBuilder& ArmCommandBuilder::SetCommand(const ImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

ArmCommandBuilder& ArmCommandBuilder::SetCommand(const JointImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

ArmCommandBuilder& ArmCommandBuilder::SetCommand(const CartesianImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* ArmCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

TorsoCommandBuilder::TorsoCommandBuilder() : impl_(std::make_unique<TorsoCommandBuilderImpl>()) {}

TorsoCommandBuilder::TorsoCommandBuilder(const JointPositionCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const GravityCompensationCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const CartesianCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const ImpedanceControlCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const OptimalControlCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const JointImpedanceControlCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const CartesianImpedanceControlCommandBuilder& builder)
    : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::TorsoCommandBuilder(const JointGroupPositionCommandBuilder& builder) : TorsoCommandBuilder() {
  SetCommand(builder);
}

TorsoCommandBuilder::~TorsoCommandBuilder() = default;

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const JointPositionCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const GravityCompensationCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const CartesianCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const ImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const OptimalControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const JointImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const CartesianImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

TorsoCommandBuilder& TorsoCommandBuilder::SetCommand(const JointGroupPositionCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* TorsoCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

BodyComponentBasedCommandBuilder::BodyComponentBasedCommandBuilder() {
  impl_ = std::make_unique<BodyComponentBasedCommandBuilderImpl>();
}

BodyComponentBasedCommandBuilder::~BodyComponentBasedCommandBuilder() = default;

BodyComponentBasedCommandBuilder& BodyComponentBasedCommandBuilder::SetRightArmCommand(
    const ArmCommandBuilder& builder) {
  impl_->SetRightArmCommand(builder);
  return *this;
}

BodyComponentBasedCommandBuilder& BodyComponentBasedCommandBuilder::SetLeftArmCommand(
    const ArmCommandBuilder& builder) {
  impl_->SetLeftArmCommand(builder);
  return *this;
}

BodyComponentBasedCommandBuilder& BodyComponentBasedCommandBuilder::SetTorsoCommand(
    const TorsoCommandBuilder& builder) {
  impl_->SetTorsoCommand(builder);
  return *this;
}

void* BodyComponentBasedCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

BodyCommandBuilder::BodyCommandBuilder() {
  impl_ = std::make_unique<BodyCommandBuilderImpl>();
}

BodyCommandBuilder::BodyCommandBuilder(const JointPositionCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::BodyCommandBuilder(const OptimalControlCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::BodyCommandBuilder(const GravityCompensationCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::BodyCommandBuilder(const CartesianCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::BodyCommandBuilder(const BodyComponentBasedCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::BodyCommandBuilder(const JointImpedanceControlCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::BodyCommandBuilder(const CartesianImpedanceControlCommandBuilder& builder) : BodyCommandBuilder() {
  SetCommand(builder);
}

BodyCommandBuilder::~BodyCommandBuilder() = default;

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const JointPositionCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const OptimalControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const GravityCompensationCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const CartesianCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const BodyComponentBasedCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const JointImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

BodyCommandBuilder& BodyCommandBuilder::SetCommand(const CartesianImpedanceControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* BodyCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

MobilityCommandBuilder::MobilityCommandBuilder() : impl_(std::make_unique<MobilityCommandBuilderImpl>()) {}

MobilityCommandBuilder::MobilityCommandBuilder(const JointVelocityCommandBuilder& builder) : MobilityCommandBuilder() {
  SetCommand(builder);
}

MobilityCommandBuilder::MobilityCommandBuilder(const SE2VelocityCommandBuilder& builder) : MobilityCommandBuilder() {
  SetCommand(builder);
}

MobilityCommandBuilder::~MobilityCommandBuilder() = default;

MobilityCommandBuilder& MobilityCommandBuilder::SetCommand(const JointVelocityCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

MobilityCommandBuilder& MobilityCommandBuilder::SetCommand(const SE2VelocityCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* MobilityCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

HeadCommandBuilder::HeadCommandBuilder() : impl_(std::make_unique<HeadCommandBuilderImpl>()) {}

HeadCommandBuilder::HeadCommandBuilder(const JointPositionCommandBuilder& builder) : HeadCommandBuilder() {
  SetCommand(builder);
}

HeadCommandBuilder::~HeadCommandBuilder() = default;

HeadCommandBuilder& HeadCommandBuilder::SetCommand(const JointPositionCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* HeadCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

ComponentBasedCommandBuilder::ComponentBasedCommandBuilder() {
  impl_ = std::make_unique<ComponentBasedCommandBuilderImpl>();
}

ComponentBasedCommandBuilder::~ComponentBasedCommandBuilder() = default;

ComponentBasedCommandBuilder& ComponentBasedCommandBuilder::SetMobilityCommand(const MobilityCommandBuilder& builder) {
  impl_->SetMobilityCommand(builder);
  return *this;
}

ComponentBasedCommandBuilder& ComponentBasedCommandBuilder::SetBodyCommand(const BodyCommandBuilder& builder) {
  impl_->SetBodyCommand(builder);
  return *this;
}

ComponentBasedCommandBuilder& ComponentBasedCommandBuilder::SetHeadCommand(const HeadCommandBuilder& builder) {
  impl_->SetHeadCommand(builder);
  return *this;
}

void* ComponentBasedCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

WholeBodyCommandBuilder::WholeBodyCommandBuilder() : impl_(std::make_unique<WholeBodyCommandBuilderImpl>()) {}

WholeBodyCommandBuilder::WholeBodyCommandBuilder(const StopCommandBuilder& builder) : WholeBodyCommandBuilder() {
  SetCommand(builder);
}

WholeBodyCommandBuilder::WholeBodyCommandBuilder(const RealTimeControlCommandBuilder& builder)
    : WholeBodyCommandBuilder() {
  SetCommand(builder);
}

WholeBodyCommandBuilder::~WholeBodyCommandBuilder() = default;

WholeBodyCommandBuilder& WholeBodyCommandBuilder::SetCommand(const StopCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

WholeBodyCommandBuilder& WholeBodyCommandBuilder::SetCommand(const RealTimeControlCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* WholeBodyCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

RobotCommandBuilder::RobotCommandBuilder() {
  impl_ = std::make_unique<RobotCommandBuilderImpl>();
}

RobotCommandBuilder::RobotCommandBuilder(const WholeBodyCommandBuilder& builder) : RobotCommandBuilder() {
  SetCommand(builder);
}

RobotCommandBuilder::RobotCommandBuilder(const ComponentBasedCommandBuilder& builder) : RobotCommandBuilder() {
  SetCommand(builder);
}

RobotCommandBuilder::RobotCommandBuilder(const JogCommandBuilder& builder) : RobotCommandBuilder() {
  SetCommand(builder);
}

RobotCommandBuilder::~RobotCommandBuilder() = default;

RobotCommandBuilder& RobotCommandBuilder::SetCommand(const WholeBodyCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

RobotCommandBuilder& RobotCommandBuilder::SetCommand(const ComponentBasedCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

RobotCommandBuilder& RobotCommandBuilder::SetCommand(const JogCommandBuilder& builder) {
  impl_->SetCommand(builder);
  return *this;
}

void* RobotCommandBuilder::Build() const {
  return static_cast<void*>(impl_->Build());
}

}  // namespace rb