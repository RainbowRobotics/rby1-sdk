#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <optional>

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/export.h"
#include "rby1-sdk/math/qp_solver.h"
#include "rby1-sdk/math/se3.h"

namespace rb {

template <int DOF>
class OptimalControl {
 public:
  enum class ExitCode : int { kNoError = 0, kInequalityConstraintViolation, kQPSolverError };

  struct LinkTarget {
    int ref_link_index{};
    int link_index{};
    math::SE3::MatrixType T{math::SE3::Identity()};
    double weight_position{0.};
    double weight_orientation{0.};
  };

  struct COMTarget {
    int ref_link_index{};
    Eigen::Vector3d com{Eigen::Vector3d::Zero()};
    double weight{0.};
  };

  struct JointAngleTarget {
    explicit JointAngleTarget(int dof) {
      q = Eigen::Vector<double, DOF>::Zero(dof);
      weight = Eigen::Vector<double, DOF>::Zero(dof);
    }

    JointAngleTarget() {
      if constexpr (DOF > 0) {
        q.setZero();
        weight.setZero();
      } else {
        static_assert(DOF > 0, "JointAngleTarget() is only available for fixed-size DOF");
      }
    }

    Eigen::Vector<double, DOF> q;
    Eigen::Vector<double, DOF> weight;
  };

  struct Input {
    std::optional<std::vector<LinkTarget>> link_targets;
    std::optional<COMTarget> com_target;
    std::optional<JointAngleTarget> q_target;
  };

  explicit OptimalControl(std::shared_ptr<dyn::Robot<DOF>> robot,      //
                          const std::vector<unsigned int>& joint_idx,  //
                          bool nullspace_mapping = true,               //
                          bool soft_position_boundary = false          //
                          )
      : robot_(std::move(robot)),
        joint_idx_(joint_idx),
        dof_(robot_->GetDOF()),
        n_joints_((int)joint_idx.size()),
        nullspace_mapping_(nullspace_mapping),
        soft_position_boundary_(soft_position_boundary) {
    n_vars_ = soft_position_boundary_ ? 3 * n_joints_ : n_joints_;
    n_consts_ = soft_position_boundary_ ? 5 * n_joints_ : n_joints_;
    qp_solver_.Setup(n_vars_, n_consts_);

    std::vector<bool> is_selected(dof_, false);
    for (const auto& i : joint_idx_) {
      if (i < dof_) {
        is_selected[i] = true;
      }
    }
    unselected_joint_idx_.reserve(dof_ - n_joints_);
    for (int i = 0; i < dof_; i++) {
      if (!is_selected[i]) {
        unselected_joint_idx_.push_back(i);
      }
    }
  }

  std::optional<Eigen::VectorXd> Solve(Input in,                                                //
                                       std::shared_ptr<dyn::State<DOF>> state,                  //
                                       double dt,                                               //
                                       double error_scaling,                                    //
                                       const Eigen::Vector<double, DOF>& position_lower_limit,  //
                                       const Eigen::Vector<double, DOF>& position_upper_limit,  //
                                       const Eigen::Vector<double, DOF>& velocity_limit,        //
                                       const Eigen::Vector<double, DOF>& acceleration_limit,    //
                                       bool need_forward_kinematics = false                     //
  ) {
    exit_code_ = ExitCode::kNoError;
    exit_code_msg_ = "";

    double err_sum = 0;
    double max_cond = 1.0;

    dt = std::max(dt, 1.e-6);

    if (need_forward_kinematics) {
      robot_->ComputeForwardKinematics(state);
    }

    qp_solver_.InitFunction();

    if (in.link_targets.has_value()) {
      AddLinkTargetCosts(in.link_targets.value(), state, dt, error_scaling, err_sum, max_cond);
    }

    if (in.com_target.has_value()) {
      AddCOMTargetCost(in.com_target.value(), state, dt, error_scaling, err_sum, max_cond);
    }

    if (in.q_target.has_value()) {
      AddJointTargetCost(in.q_target.value(), state, dt, error_scaling, err_sum);
    }

    if (nullspace_mapping_) {
      AddNullspaceCost(1);
    }

    if (soft_position_boundary_) {
      const double weight = 1.;
      const double safety_margin = 5 * math::kDeg2Rad;

      Eigen::VectorXd q = state->GetQ();

      Eigen::MatrixXd upper_penalty = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
      Eigen::MatrixXd lower_penalty = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
      for (int i = 0; i < n_joints_; i++) {
        if (q(joint_idx_[i]) > position_upper_limit(joint_idx_[i]) - safety_margin) {
          const double alpha = std::clamp(
              (safety_margin - (position_upper_limit(joint_idx_[i]) - q(joint_idx_[i]))) / safety_margin, 1.e-6, 1.0);
          upper_penalty(i, n_joints_ + i) = weight * alpha;
        } else {
          upper_penalty(i, n_joints_ + i) = 1.e-6;  // DUMMY FOR PATTERN LOCK
        }
        if (q(joint_idx_[i]) < position_lower_limit(joint_idx_[i]) + safety_margin) {
          const double alpha = std::clamp(
              (safety_margin - (q(joint_idx_[i]) - position_lower_limit(joint_idx_[i]))) / safety_margin, 1.e-6, 1.0);
          lower_penalty(i, 2 * n_joints_ + i) = weight * alpha;
        } else {
          lower_penalty(i, 2 * n_joints_ + i) = 1.e-6;  // DUMMY FOR PATTERN LOCK
        }
      }
      qp_solver_.AddCostFunction(upper_penalty, Eigen::VectorXd::Zero(n_joints_));
      qp_solver_.AddCostFunction(lower_penalty, Eigen::VectorXd::Zero(n_joints_));
    }

    qp_solver_.SetCostFunction(qp_solver_.GetACost(), qp_solver_.GetBCost());

    if (!SetInequalityConstraints(state->GetQ()(joint_idx_),         //
                                  state->GetQdot()(joint_idx_),      //
                                  position_lower_limit(joint_idx_),  //
                                  position_upper_limit(joint_idx_),  //
                                  velocity_limit(joint_idx_),        //
                                  acceleration_limit(joint_idx_),    //
                                  dt                                 //
                                  )) {
      return {};
    }

    Eigen::VectorXd primal_variable = Eigen::VectorXd::Zero(n_vars_);
    primal_variable.head(n_joints_) = state->GetQdot()(joint_idx_);
    qp_solver_.SetPrimalVariable(primal_variable);

    try {
      Eigen::VectorXd solution = state->GetQdot();
      const auto& result = qp_solver_.Solve();
      solution(joint_idx_) = result.head(n_joints_);
      err_ = std::sqrt(err_sum);
      manipulability_ = max_cond;
      return solution;
    } catch (math::QPSolverException& e) {
      std::stringstream ss;
      ss << "[OptimalControl::Solve] OSQP Error: " << e.what();
      exit_code_ = ExitCode::kQPSolverError;
      exit_code_msg_ = ss.str();
      return {};
    }
  }

  std::optional<Eigen::VectorXd> Solve(Input in,                                              //
                                       std::shared_ptr<dyn::State<DOF>> state,                //
                                       double dt,                                             //
                                       double error_scaling,                                  //
                                       const Eigen::Vector<double, DOF>& velocity_limit,      //
                                       const Eigen::Vector<double, DOF>& acceleration_limit,  //
                                       bool need_forward_kinematics = false                   //
  ) {
    return Solve(in, state, dt, error_scaling, robot_->GetLimitQLower(state), robot_->GetLimitQUpper(state),
                 velocity_limit, acceleration_limit, need_forward_kinematics);
  }

  ExitCode GetExitCode() const { return exit_code_; }

  std::string GetExitCodeMessage() const { return exit_code_msg_; }

  double GetError() const { return err_; }

  double GetManipulability() const { return manipulability_; }

 protected:
  template <int N>
  double CalculateConditionNumber(const Eigen::Matrix<double, N, N>& A) {
    constexpr double kEpsilon = 1e-6;

    Eigen::JacobiSVD<Eigen::Matrix<double, N, N>> svd(A);
    const auto& singular_values = svd.singularValues();

    double max_sv = singular_values.maxCoeff();
    double min_sv = singular_values.minCoeff();

    if (std::abs(min_sv) <= kEpsilon) {
      return std::numeric_limits<double>::infinity();
    }
    return max_sv / min_sv;
  }

  Eigen::MatrixXd ComputeNullspaceProjection(const Eigen::MatrixXd& J) {
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd pinv = J.transpose() * JJt.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());
    return I - pinv * J;
  }

  void AddLinkTargetCosts(const std::vector<LinkTarget>& link_targets,    //
                          const std::shared_ptr<dyn::State<DOF>>& state,  //
                          double dt,                                      //
                          double error_scaling,                           //
                          double& err_sum,                                //
                          double& max_cond                                //
  ) {
    using namespace math;

    Eigen::Vector<double, DOF> qdot = state->GetQdot();
    qdot(unselected_joint_idx_).setZero();

    for (const auto& link_target : link_targets) {
      Eigen::Matrix4d T_cur = robot_->ComputeTransformation(state, link_target.ref_link_index, link_target.link_index);
      Eigen::Matrix4d T_err = T_cur.inverse() * link_target.T;

      Eigen::Matrix<double, 6, DOF> J_full =
          robot_->ComputeBodyJacobian(state, link_target.ref_link_index, link_target.link_index);
      J_full(Eigen::all, unselected_joint_idx_).setZero();

      Eigen::Vector<double, 6> w;
      w.head<3>().fill(link_target.weight_orientation < 1e-6 ? 0. : link_target.weight_orientation);
      w.tail<3>().fill(link_target.weight_position < 1e-6 ? 0. : link_target.weight_position);

      Eigen::Matrix<double, 6, 6> JJt = J_full(Eigen::all, joint_idx_) * J_full(Eigen::all, joint_idx_).transpose();
      max_cond = std::max(max_cond, CalculateConditionNumber(JJt));

      Eigen::Vector<double, 6> err = w.asDiagonal() * (SE3::Log(T_err) / dt * error_scaling - J_full * qdot / 2);
      Eigen::Matrix<double, 6, DOF> J_weighted = w.asDiagonal() * J_full / 2;
      err_sum += err.squaredNorm();

      Eigen::MatrixXd A{Eigen::MatrixXd::Zero(6, n_vars_)};
      A.block(0, 0, 6, n_joints_) = J_weighted(Eigen::all, joint_idx_);

      qp_solver_.AddCostFunction(A, err);
    }
  }

  void AddCOMTargetCost(const COMTarget& target,                        //
                        const std::shared_ptr<dyn::State<DOF>>& state,  //
                        double dt,                                      //
                        double error_scaling,                           //
                        double& err_sum,                                //
                        double& max_cond                                //
  ) {
    Eigen::Vector<double, DOF> qdot = state->GetQdot();
    qdot(unselected_joint_idx_).setZero();

    Eigen::Vector3d com = robot_->ComputeCenterOfMass(state, target.ref_link_index);
    Eigen::Matrix<double, 3, DOF> J_com = robot_->ComputeCenterOfMassJacobian(state, target.ref_link_index);
    J_com(Eigen::all, unselected_joint_idx_).setZero();

    Eigen::Vector<double, 3> w;
    w.fill(target.weight < 1e-6 ? 0. : target.weight);

    Eigen::Matrix3d JJt = J_com * J_com.transpose();
    max_cond = std::max(max_cond, CalculateConditionNumber(JJt));

    Eigen::Vector3d err = w.asDiagonal() * ((target.com - com) / dt * error_scaling - J_com * qdot / 2);
    Eigen::Matrix<double, 3, DOF> J_com_weighted = w.asDiagonal() * J_com / 2;
    err_sum += err.squaredNorm();

    Eigen::MatrixXd A{Eigen::MatrixXd::Zero(3, n_vars_)};
    A.block(0, 0, 3, n_joints_) = J_com_weighted(Eigen::all, joint_idx_);

    qp_solver_.AddCostFunction(A, err);
  }

  void AddJointTargetCost(const JointAngleTarget& target,                 //
                          const std::shared_ptr<dyn::State<DOF>>& state,  //
                          double dt,                                      //
                          double error_scaling,                           //
                          double& err_sum                                 //
  ) {
    Eigen::VectorXd q = state->GetQ()(joint_idx_);
    Eigen::VectorXd qdot = state->GetQdot()(joint_idx_);

    Eigen::MatrixXd J_full = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(n_joints_, n_joints_);
    for (int i = 0; i < n_joints_; i++) {
      if (target.weight[joint_idx_[i]] > 1e-6) {
        w(i, i) = target.weight[joint_idx_[i]];
      }
    }

    Eigen::VectorXd err = w * ((target.q(joint_idx_) - q) / dt * error_scaling - qdot / 2);
    Eigen::MatrixXd J_weighted = w * J_full / 2;
    err_sum += err.squaredNorm();

    Eigen::MatrixXd A{Eigen::MatrixXd::Zero(n_joints_, n_vars_)};
    A.block(0, 0, n_joints_, n_joints_) = J_weighted;

    qp_solver_.AddCostFunction(A, err);
  }

  void AddNullspaceCost(double weight = 1e-3) {
    Eigen::MatrixXd P = ComputeNullspaceProjection(qp_solver_.GetACost().leftCols(n_joints_));
    Eigen::MatrixXd A{Eigen::MatrixXd::Zero(n_joints_, n_vars_)};
    A.block(0, 0, n_joints_, n_joints_) = P;

    qp_solver_.AddCostFunction(A * weight, Eigen::VectorXd::Zero(n_joints_));
  }

  bool SetInequalityConstraints(const Eigen::VectorXd& q,                   //
                                const Eigen::VectorXd& qdot,                //
                                const Eigen::VectorXd& q_lb,                //
                                const Eigen::VectorXd& q_ub,                //
                                const Eigen::VectorXd& velocity_limit,      //
                                const Eigen::VectorXd& acceleration_limit,  //
                                double dt                                   //
  ) {
    Eigen::MatrixXd A_const = Eigen::MatrixXd::Zero(n_consts_, n_vars_);
    A_const.block(0, 0, n_joints_, n_joints_).setIdentity();
    if (soft_position_boundary_) {
      A_const.block(n_joints_, 0, n_joints_, n_joints_) = Eigen::VectorXd::Constant(n_joints_, -1).asDiagonal();
      A_const.block(n_joints_, n_joints_, n_joints_, n_joints_).setIdentity();
      A_const.block(2 * n_joints_, n_joints_, n_joints_, n_joints_).setIdentity();
      A_const.block(3 * n_joints_, 0, n_joints_, n_joints_).setIdentity();
      A_const.block(3 * n_joints_, 2 * n_joints_, n_joints_, n_joints_).setIdentity();
      A_const.block(4 * n_joints_, 2 * n_joints_, n_joints_, n_joints_).setIdentity();
    }

    Eigen::VectorXd qdot_lb = -velocity_limit;
    Eigen::VectorXd qdot_ub = velocity_limit;
    qdot_lb = qdot_lb.cwiseMax(qdot - acceleration_limit * dt);
    qdot_ub = qdot_ub.cwiseMin(qdot + acceleration_limit * dt);
    qdot_lb = qdot_lb.cwiseMax(2 * (q_lb - q) / dt - qdot);
    qdot_ub = qdot_ub.cwiseMin(2 * (q_ub - q) / dt - qdot);

    if ((qdot_lb.array() <= qdot_ub.array()).all()) {
      Eigen::VectorXd lb = Eigen::VectorXd::Zero(n_consts_);
      Eigen::VectorXd ub = Eigen::VectorXd::Zero(n_consts_);

      lb.head(n_joints_) = qdot_lb;
      ub.head(n_joints_) = qdot_ub;
      lb.tail(n_consts_ - n_joints_).setZero();  // Slack variables
      ub.tail(n_consts_ - n_joints_).setConstant(1e6);

      qp_solver_.SetConstraintsFunction(A_const, lb, ub);

      return true;
    }

    Eigen::IOFormat eigen_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
    std::stringstream ss;
    ss << "Inequality constraint violation - qdot_lb: [" << qdot_lb.format(eigen_fmt) << "], qdot_ub: ["
       << qdot_ub.format(eigen_fmt) << "]";
    exit_code_ = ExitCode::kInequalityConstraintViolation;
    exit_code_msg_ = ss.str();
    return false;
  }

 private:
  ExitCode exit_code_{ExitCode::kNoError};
  std::string exit_code_msg_{};

  double err_{};
  double manipulability_{};

  math::QPSolver qp_solver_;

  std::shared_ptr<dyn::Robot<DOF>> robot_;
  int dof_{};
  int n_joints_{};
  int n_vars_{};
  int n_consts_{};
  bool nullspace_mapping_{false};
  bool soft_position_boundary_{false};

  std::vector<unsigned int> joint_idx_;
  std::vector<unsigned int> unselected_joint_idx_;
};

}  // namespace rb