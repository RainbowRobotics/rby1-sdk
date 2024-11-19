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
                          const Eigen::VectorXd& velocity_limit = Eigen::Vector<double, 0>())
      : robot_(std::move(robot)), joint_idx_(std::move(joint_idx)) {
    n_joints_ = robot_->GetDOF();
    qp_solver_.Setup(n_joints_, n_joints_ * 2);

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

    velocity_limit_ = velocity_limit;
  }

  std::optional<Eigen::VectorXd> Solve(Input in, std::shared_ptr<dyn::State<DOF>> state, double dt,
                                       double velocity_limit_scaling = 1.0, bool need_forward_kinematics = false) {
    using namespace math;

    if (dt <= 1.e-6) {
      dt = 1.e-6;
    }

    if (need_forward_kinematics) {
      robot_->ComputeForwardKinematics(state);
    }

    qp_solver_.InitFunction();

    double err_sum = 0;

    if (in.link_targets.has_value()) {
      for (const auto& link_target : in.link_targets.value()) {
        Eigen::Matrix<double, 4, 4> T_cur =
            robot_->ComputeTransformation(state, link_target.ref_link_index, link_target.link_index);
        Eigen::Matrix<double, 4, 4> T_target = link_target.T;
        Eigen::Matrix<double, 4, 4> T_err = T_cur.inverse() * T_target;

        Eigen::Matrix<double, 6, 1> err = SE3::Log(T_err) / dt;

        if (link_target.weight_orientation < 1e-6) {
          err.topRows(3).setZero();
        }

        if (link_target.weight_position < 1e-6) {
          err.bottomRows(3).setZero();
        }

        err_sum += (err * dt).squaredNorm();

        Eigen::Matrix<double, 6, DOF> J =
            robot_->ComputeBodyJacobian(state, link_target.ref_link_index, link_target.link_index);
        J(Eigen::all, rev_joint_idx_).setZero();

        err.topRows(3) = err.topRows(3) * link_target.weight_orientation;
        err.bottomRows(3) = err.bottomRows(3) * link_target.weight_position;

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

      Eigen::Matrix<double, 3, 1> com_target = in.com_target.value().com;
      Eigen::Matrix<double, 3, 1> err = (com_target - com) / dt;
      err_sum += (err * dt).squaredNorm();

      err = err * in.com_target.value().weight;
      J_com(Eigen::all, rev_joint_idx_).setZero();

      qp_solver_.AddCostFunction(J_com, err);
    }

    if (in.q_target.has_value()) {
      Eigen::Matrix<double, DOF, 1> err;
      err = (in.q_target.value().q - state->GetQ()) / dt;

      Eigen::Matrix<double, DOF, DOF> J;
      J.resize(n_joints_, n_joints_);
      J.setZero();
      for (int i = 0; i < n_joints_; i++) {
        if (in.q_target.value().weight[i] > 1e-6) {
          J(i, i) = 1;
        }
      }
      J(Eigen::all, rev_joint_idx_).setZero();

      err_sum += (J * err * dt).squaredNorm();

      err = err.cwiseProduct(in.q_target.value().weight);
      qp_solver_.AddCostFunction(J, err);
    }

    qp_solver_.AddCostFunction(Eigen::Matrix<double, DOF, DOF>::Identity(n_joints_, n_joints_) * 1e-5,
                               Eigen::Vector<double, DOF>::Zero());
    qp_solver_.SetCostFunction(qp_solver_.GetACost(), qp_solver_.GetBCost());

    Eigen::Matrix<double, (DOF * 2 < 0 ? -1 : DOF * 2), DOF> A_const;
    Eigen::Vector<double, (DOF * 2 < 0 ? -1 : DOF * 2)> lb, ub;
    A_const.resize(n_joints_ * 2, n_joints_);
    lb.resize(2 * n_joints_);
    ub.resize(2 * n_joints_);
    A_const.setZero();
    lb.setZero();
    ub.setZero();

    if constexpr (DOF < 0) {
      A_const.topRows(n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
      A_const.bottomRows(n_joints_) = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
    } else {
      A_const.template topRows<DOF>() = Eigen::Matrix<double, DOF, DOF>::Identity();
      A_const.template bottomRows<DOF>() = Eigen::Matrix<double, DOF, DOF>::Identity();
    }

    Eigen::Vector<double, DOF> q_lb, q_ub, qdot_lb, qdot_ub;
    q_lb = robot_->GetLimitQLower(state);
    q_ub = robot_->GetLimitQUpper(state);
    if (velocity_limit_.size() == n_joints_) {
      qdot_lb = -velocity_limit_ * velocity_limit_scaling;
      qdot_ub = velocity_limit_ * velocity_limit_scaling;
    } else {
      qdot_lb = robot_->GetLimitQdotLower(state) * velocity_limit_scaling;
      qdot_ub = robot_->GetLimitQdotUpper(state) * velocity_limit_scaling;
    }

    if constexpr (DOF < 0) {
      lb.topRows(n_joints_) = qdot_lb;
      ub.topRows(n_joints_) = qdot_ub;
      lb.bottomRows(n_joints_) = (q_lb - state->GetQ()) / dt;
      ub.bottomRows(n_joints_) = (q_ub - state->GetQ()) / dt;
    } else {
      lb.template topRows<DOF>() = qdot_lb;
      ub.template topRows<DOF>() = qdot_ub;
      lb.template bottomRows<DOF>() = (q_lb - state->GetQ()) / dt;
      ub.template bottomRows<DOF>() = (q_ub - state->GetQ()) / dt;
    }

    qp_solver_.SetConstraintsFunction(A_const, lb, ub);
    qp_solver_.SetPrimalVariable(Eigen::Vector<double, DOF>::Zero(n_joints_));

    auto ret = qp_solver_.Solve();

    Eigen::MatrixXd err;
    if (ret.has_value()) {
      err_ = std::sqrt(err_sum);
    }

    if (ret.has_value()) {
      return ret;
    }
    return {};
  }

  [[nodiscard]] double GetError() const { return err_; }  // NOLINT

 private:
  double err_{};
  math::QPSolver qp_solver_;

  std::shared_ptr<dyn::Robot<DOF>> robot_;
  int n_joints_;

  std::vector<unsigned int> joint_idx_;
  std::vector<unsigned int> rev_joint_idx_;

  Eigen::VectorXd velocity_limit_;
};

}  // namespace rb