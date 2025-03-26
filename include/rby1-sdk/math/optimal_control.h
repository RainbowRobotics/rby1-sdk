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
  enum class ExitCode : int {
    kNoError = 0,

    kInequalityConstraintViolation,
    kQPSolverError
  };

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

  explicit OptimalControl(std::shared_ptr<dyn::Robot<DOF>> robot, std::vector<unsigned int> joint_idx)
      : robot_(std::move(robot)), joint_idx_(std::move(joint_idx)) {
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

  std::optional<Eigen::VectorXd> Solve(Input in, std::shared_ptr<dyn::State<DOF>> state, double dt,
                                       double error_scaling, Eigen::Vector<double, DOF> velocity_limit,
                                       Eigen::Vector<double, DOF> acceleration_limit,
                                       bool need_forward_kinematics = false) {
    exit_code_ = ExitCode::kNoError;
    exit_code_msg_ = "";

    double err_sum = 0;
    double max_cond = 1.0;
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

    if (in.link_targets.has_value()) {
      for (const auto& link_target : in.link_targets.value()) {
        Eigen::Matrix4d T_cur =
            robot_->ComputeTransformation(state, link_target.ref_link_index, link_target.link_index);
        Eigen::Matrix4d T_target = link_target.T;
        Eigen::Matrix4d T_err = T_cur.inverse() * T_target;
        Eigen::Matrix<double, 6, DOF> J =
            robot_->ComputeBodyJacobian(state, link_target.ref_link_index, link_target.link_index);
        J(Eigen::all, rev_joint_idx_).setZero();
        Eigen::Vector<double, 6> w;
        w.head<3>().fill(link_target.weight_orientation < 1e-6 ? 0. : link_target.weight_orientation);
        w.tail<3>().fill(link_target.weight_position < 1e-6 ? 0. : link_target.weight_position);

        {
          Eigen::Matrix<double, 6, 6> A = J * J.transpose();
          double cond = CalculateConditionVariable(A);
          max_cond = std::max(max_cond, cond);
        }

        Eigen::Matrix<double, 6, 1> err = w.asDiagonal() * (SE3::Log(T_err) / dt * error_scaling - J * qdot / 2);
        J = w.asDiagonal() * J / 2;

        err_sum += err.squaredNorm();

        qp_solver_.AddCostFunction(J, err);
      }
    }

    if (in.com_target.has_value()) {
      const auto& target = in.com_target.value();

      Eigen::Vector3d com;
      Eigen::Matrix<double, 3, DOF> J_com;
      J_com.resize(3, n_joints_);
      com.setZero();
      J_com.setZero();

      com = robot_->ComputeCenterOfMass(state, target.ref_link_index);
      J_com = robot_->ComputeCenterOfMassJacobian(state, target.ref_link_index);
      J_com(Eigen::all, rev_joint_idx_).setZero();
      Eigen::Vector<double, 3> w;
      w.fill(target.weight < 1e-6 ? 0. : target.weight);

      {
        Eigen::Matrix3d A = J_com * J_com.transpose();
        double cond = CalculateConditionVariable(A);
        max_cond = std::max(max_cond, cond);
      }

      Eigen::Vector3d err = w.asDiagonal() * ((target.com - com) / dt * error_scaling - J_com * qdot / 2);
      J_com = w.asDiagonal() * J_com / 2;

      err_sum += err.squaredNorm();

      qp_solver_.AddCostFunction(J_com, err);
    }

    if (in.q_target.has_value()) {
      const auto& target = in.q_target.value();

      Eigen::Matrix<double, DOF, DOF> J;
      J.resize(n_joints_, n_joints_);
      J.setIdentity();
      J(Eigen::all, rev_joint_idx_).setZero();

      Eigen::Matrix<double, DOF, DOF> w;
      w.resize(n_joints_, n_joints_);
      w.setZero();
      for (int i = 0; i < n_joints_; i++) {
        if (target.weight[i] > 1e-6) {
          w(i, i) = target.weight[i];
        }
      }

      Eigen::Matrix<double, DOF, 1> err;
      err = w * ((in.q_target.value().q - q) / dt * error_scaling - qdot / 2);
      J = w * J / 2;

      err_sum += err.squaredNorm();

      qp_solver_.AddCostFunction(J, err);
    }

    Eigen::MatrixXd J = qp_solver_.GetACost();
    Eigen::Matrix<double, DOF, -1> J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Matrix<double, DOF, DOF> P = Eigen::Matrix<double, DOF, DOF>::Identity(n_joints_, n_joints_) - J_pinv * J;

    qp_solver_.AddCostFunction(P * 1e-3, Eigen::Vector<double, DOF>::Zero(n_joints_));
    qp_solver_.SetCostFunction(qp_solver_.GetACost(), qp_solver_.GetBCost());

    Eigen::Matrix<double, DOF, DOF> A_const;
    A_const.resize(n_joints_, n_joints_);
    A_const.setIdentity();

    Eigen::Vector<double, DOF> q_lb, q_ub, qdot_lb, qdot_ub;
    qdot_lb.resize(n_joints_);
    qdot_ub.resize(n_joints_);
    q_lb = robot_->GetLimitQLower(state);
    q_ub = robot_->GetLimitQUpper(state);
    qdot_lb = -velocity_limit;
    qdot_ub = velocity_limit;
    qdot_lb = qdot_lb.cwiseMax(qdot - acceleration_limit * dt);
    qdot_ub = qdot_ub.cwiseMin(qdot + acceleration_limit * dt);
    qdot_lb = qdot_lb.cwiseMax(2 * (q_lb - q) / dt - qdot);
    qdot_ub = qdot_ub.cwiseMin(2 * (q_ub - q) / dt - qdot);
    qdot_lb(rev_joint_idx_) = qdot(rev_joint_idx_);
    qdot_ub(rev_joint_idx_) = qdot(rev_joint_idx_);

    if ((qdot_lb.array() <= qdot_ub.array()).all())
      ;
    else {
      // acceleration limit 를 제외 하고 다시 시도
      qdot_lb = -velocity_limit;
      qdot_ub = velocity_limit;
      qdot_lb = qdot_lb.cwiseMax(2 * (q_lb - q) / dt - qdot);
      qdot_ub = qdot_ub.cwiseMin(2 * (q_ub - q) / dt - qdot);
      qdot_lb(rev_joint_idx_) = qdot(rev_joint_idx_);
      qdot_ub(rev_joint_idx_) = qdot(rev_joint_idx_);

      if ((qdot_lb.array() <= qdot_ub.array()).all())
        ;
      else {
        Eigen::IOFormat eigen_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");
        std::stringstream ss;
        ss << "Inequality constraint violation - qdot_lb: [" << qdot_lb.format(eigen_fmt) << "], qdot_ub: ["
           << qdot_ub.format(eigen_fmt) << "]";
        exit_code_ = ExitCode::kInequalityConstraintViolation;
        exit_code_msg_ = ss.str();
        return {};
      }
    }

    qp_solver_.SetConstraintsFunction(A_const, qdot_lb, qdot_ub);
    if ((qdot_lb.array() <= qdot.array()).all() && (qdot.array() <= qdot_ub.array()).all()) {
      qp_solver_.SetPrimalVariable(qdot);
    } else {
      qp_solver_.SetPrimalVariable(qdot_lb);
    }

    Eigen::VectorXd solution;
    try {
      solution = qp_solver_.Solve();
    } catch (math::QPSolverException& e) {
      std::stringstream ss;
      ss << "QPSolver error - " << e.what();
      exit_code_ = ExitCode::kQPSolverError;
      exit_code_msg_ = ss.str();
      return {};
    }

    err_ = std::sqrt(err_sum);
    manipulability_ = max_cond;
    return solution;
  }

  ExitCode GetExitCode() const { return exit_code_; }

  std::string GetExitCodeMessage() const { return exit_code_msg_; }

  double GetError() const { return err_; }

  double GetManipulability() const { return manipulability_; }

 protected:
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

 private:
  ExitCode exit_code_{ExitCode::kNoError};
  std::string exit_code_msg_{};

  double err_{};
  double manipulability_{};
  math::QPSolver qp_solver_;

  std::shared_ptr<dyn::Robot<DOF>> robot_;
  int n_joints_;

  std::vector<unsigned int> joint_idx_;
  std::vector<unsigned int> rev_joint_idx_;
};

}  // namespace rb