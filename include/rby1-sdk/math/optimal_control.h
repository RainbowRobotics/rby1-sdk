#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <optional>

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/math/qp_solver.h"
#include "rby1-sdk/math/se3.h"

namespace rb {

template <int DOF>
class OptimalControl {
 public:
  struct LinkTarget {
    int ref_link_index{};
    int link_index{};
    math::SE3::MatrixType T{math::SE3::Identity()};
    double weight_position{0.};
    double weight_orientation{0.};
  };

  struct COMTarget {
    int ref_link_index{};
    Eigen::Vector3d com;
    double weight{0.};
  };

  struct JointAngleTarget {
    explicit JointAngleTarget(int n_joints) {
      q.resize(n_joints);
      weight.resize(n_joints);

      q.setZero();
      weight.setZero();
    }

    JointAngleTarget() {
      static_assert(DOF > 0);

      q.setZero();
      weight.setZero();
    }

    Eigen::Vector<double, DOF> q;
    Eigen::Vector<double, DOF> weight;
  };

  struct Input {
    std::optional<std::vector<LinkTarget>> link_targets;
    std::optional<COMTarget> com_target;
    std::optional<JointAngleTarget> q_target;
  };

  explicit OptimalControl(std::shared_ptr<dyn::Robot<DOF>> robot, std::vector<unsigned int> joint_idx,
                          const Eigen::Vector<double, DOF>& velocity_limit)
      : robot_(std::move(robot)), joint_idx_(std::move(joint_idx)), velocity_limit_(velocity_limit) {
    n_joints_ = robot_->GetDOF();
    qp_solver_.Setup(n_joints_, n_joints_);

    std::vector<bool> check;
    check.resize(n_joints_, false);
    for (const auto& i : joint_idx_) {
      check[i] = true;
    }
    for (int i = 0; i < n_joints_; i++) {
      if (!check[i]) {
        rev_joint_idx_.push_back(i);
      }
    }
  }

  template <int N>
  double CalculateConditionVariable(const Eigen::Matrix<double, N, N>& A) {
    Eigen::JacobiSVD<Eigen::Matrix<double, N, N>> svd(A);
    const auto& singular_values = svd.singularValues();
    double max_sv = singular_values(0);
    double min_sv = singular_values(singular_values.size() - 1);
    if (std::abs(min_sv) <= 1e-6) {
      return std::numeric_limits<double>::infinity();
    }
    return max_sv / min_sv;
  }

  std::optional<Eigen::VectorXd> Solve(Input in, std::shared_ptr<dyn::State<DOF>> state, double dt,
                                       std::optional<Eigen::Vector<double, DOF>> velocity_limit = std::nullopt,
                                       std::optional<Eigen::Vector<double, DOF>> acceleration_limit = std::nullopt,
                                       bool need_forward_kinematics = false) {
    Eigen::Vector<double, DOF> q = state->GetQ();
    Eigen::Vector<double, DOF> qdot = state->GetQdot();

    using namespace math;

    if (dt <= 1.e-6) {
      dt = 1.e-6;
    }

    if (need_forward_kinematics) {
      robot_->ComputeForwardKinematics(state);
    }

    qp_solver_.InitFunction();

    double max_cond = 1.0;

    double err_sum = 0;

    if (in.link_targets.has_value()) {
      for (const auto& link_target : in.link_targets.value()) {
        Eigen::Matrix<double, 4, 4> T_cur =
            robot_->ComputeTransformation(state, link_target.ref_link_index, link_target.link_index);
        Eigen::Matrix<double, 4, 4> T_target = link_target.T;
        Eigen::Matrix<double, 4, 4> T_err = T_cur.inverse() * T_target;
        Eigen::Matrix<double, 6, DOF> J =
            robot_->ComputeBodyJacobian(state, link_target.ref_link_index, link_target.link_index);
        J(Eigen::all, rev_joint_idx_).setZero();

        Eigen::Matrix<double, 6, 6> A = J * J.transpose();
        double cond = CalculateConditionVariable(A);
        max_cond = std::max(max_cond, cond);

        Eigen::Matrix<double, 6, 1> err = 2 * SE3::Log(T_err) / dt - J * qdot;

        if (link_target.weight_orientation < 1e-6) {
          err.topRows(3).setZero();
        }

        if (link_target.weight_position < 1e-6) {
          err.bottomRows(3).setZero();
        }

        err.topRows(3) = err.topRows(3) * link_target.weight_orientation;
        err.bottomRows(3) = err.bottomRows(3) * link_target.weight_position;
        err_sum += err.squaredNorm();

        qp_solver_.AddCostFunction(J, err);
      }
    }

    if (in.com_target.has_value()) {
      Eigen::Matrix<double, 3, 1> com;
      Eigen::Matrix<double, 3, DOF> J_com;
      J_com.resize(3, n_joints_);
      com.setZero();
      J_com.setZero();

      com = robot_->ComputeCenterOfMass(state, in.com_target.value().ref_link_index);
      J_com = robot_->ComputeCenterOfMassJacobian(state, in.com_target.value().ref_link_index);
      J_com(Eigen::all, rev_joint_idx_).setZero();

      Eigen::Matrix3d A = J_com * J_com.transpose();
      double cond = CalculateConditionVariable(A);
      max_cond = std::max(max_cond, cond);

      Eigen::Matrix<double, 3, 1> com_target = in.com_target.value().com;
      Eigen::Matrix<double, 3, 1> err = 2 * (com_target - com) / dt - J_com * qdot;

      err = err * in.com_target.value().weight;
      err_sum += err.squaredNorm();

      qp_solver_.AddCostFunction(J_com, err);
    }

    if (in.q_target.has_value()) {
      Eigen::Matrix<double, DOF, 1> err;
      err = 2 * (in.q_target.value().q - q) / dt - qdot;

      Eigen::Matrix<double, DOF, DOF> J;
      J.resize(n_joints_, n_joints_);
      J.setZero();
      for (int i = 0; i < n_joints_; i++) {
        if (in.q_target.value().weight[i] > 1e-6) {
          J(i, i) = 1;
        }
      }
      J(Eigen::all, rev_joint_idx_).setZero();

      err = err.cwiseProduct(in.q_target.value().weight);
      err_sum += err.squaredNorm();

      qp_solver_.AddCostFunction(J, err);
    }

    Eigen::Matrix<double, DOF, DOF> penalty_term_A;
    Eigen::Vector<double, DOF> penalty_term_b;
    penalty_term_A.resize(n_joints_, n_joints_);
    penalty_term_b.resize(n_joints_);
    penalty_term_A.setZero();
    penalty_term_A.diagonal()(joint_idx_).fill(1e-2);
    penalty_term_b.setZero();

    qp_solver_.AddCostFunction(penalty_term_A, penalty_term_b);
    qp_solver_.SetCostFunction(qp_solver_.GetACost(), qp_solver_.GetBCost());

    Eigen::Matrix<double, DOF, DOF> A_const;
    A_const.resize(n_joints_, n_joints_);
    A_const.setIdentity();

    Eigen::Vector<double, DOF> q_lb, q_ub, qdot_lb, qdot_ub;
    qdot_lb.resize(n_joints_);
    qdot_ub.resize(n_joints_);
    q_lb = robot_->GetLimitQLower(state);
    q_ub = robot_->GetLimitQUpper(state);
    if (!velocity_limit.has_value()) {
      velocity_limit = velocity_limit_;
    }
    if (!acceleration_limit.has_value()) {
      acceleration_limit = robot_->GetLimitQddotUpper(state);
    }
    qdot_lb = -velocity_limit.value();
    qdot_ub = velocity_limit.value();
    qdot_lb = qdot_lb.cwiseMax(qdot - acceleration_limit.value() * dt);
    qdot_ub = qdot_ub.cwiseMin(qdot + acceleration_limit.value() * dt);
    qdot_lb = qdot_lb.cwiseMax(2 * (q_lb - q) / dt - qdot);
    qdot_ub = qdot_ub.cwiseMin(2 * (q_ub - q) / dt - qdot);

    qdot_lb(rev_joint_idx_) = qdot(rev_joint_idx_);
    qdot_ub(rev_joint_idx_) = qdot(rev_joint_idx_);

    if ((qdot_lb.array() <= qdot_ub.array()).all())
      ;
    else {
      return {};
    }

    qp_solver_.SetConstraintsFunction(A_const, qdot_lb, qdot_ub);
    //    qp_solver_.SetPrimalVariable(Eigen::Vector<double, DOF>::Zero(n_joints_));
    if ((qdot_lb.array() <= qdot.array()).all() && (qdot.array() <= qdot_ub.array()).all()) {
      qp_solver_.SetPrimalVariable(qdot);
    } else {
      qp_solver_.SetPrimalVariable(qdot_lb);
    }

    auto ret = qp_solver_.Solve();

    Eigen::MatrixXd err;
    if (ret.has_value()) {
      err_ = std::sqrt(err_sum);
    }
    manipulability_ = max_cond;

    if (ret.has_value()) {
      return ret;
    }
    return {};
  }

  [[nodiscard]] double GetError() const { return err_; }

  [[nodiscard]] double GetManipulability() const { return manipulability_; }

 private:
  double err_{};
  double manipulability_{};
  math::QPSolver qp_solver_;

  std::shared_ptr<dyn::Robot<DOF>> robot_;
  int n_joints_;
  Eigen::Vector<double, DOF> velocity_limit_;

  std::vector<unsigned int> joint_idx_;
  std::vector<unsigned int> rev_joint_idx_;
};

}  // namespace rb