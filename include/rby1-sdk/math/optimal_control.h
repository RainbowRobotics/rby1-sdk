#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
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

  struct NullspaceJointTarget {
    explicit NullspaceJointTarget(int dof) {
      q = Eigen::Vector<double, DOF>::Zero(dof);
      weight = Eigen::Vector<double, DOF>::Zero(dof);
    }

    NullspaceJointTarget() {
      if constexpr (DOF > 0) {
        q.setZero();
        weight.setZero();
      } else {
        static_assert(DOF > 0, "NullspaceJointTarget() is only available for fixed-size DOF");
      }
    }

    Eigen::Vector<double, DOF> q;
    Eigen::Vector<double, DOF> weight;
    double k_p{0.2};
    double k_d{0.2};
    double cost_weight{1e-3};
  };

  struct Input {
    std::optional<std::vector<LinkTarget>> link_targets;
    std::optional<COMTarget> com_target;
    std::optional<JointAngleTarget> q_target;
    std::optional<NullspaceJointTarget> nullspace_q_target;
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
    // The slack-based soft position boundary is superseded by the sqrt
    // braking envelope. Removing the slacks shrinks
    // the QP threefold (e.g. whole-body 60x100 -> 20x20) and eliminates the
    // 1e-12 dummy-entry eigenvalues that dominate the Hessian conditioning.
    if (std::getenv("RBY1_QP_NO_SLACK") != nullptr) {
      if (soft_position_boundary_ && std::getenv("RBY1_QP_SQRT_BRAKING") == nullptr) {
        std::cerr << "[OptimalControl] RBY1_QP_NO_SLACK without RBY1_QP_SQRT_BRAKING: the one-step "
                     "position stop law alone may abort limit approaches; enable braking"
                  << std::endl;
      }
      soft_position_boundary_ = false;
    }
    n_vars_ = soft_position_boundary_ ? 3 * n_joints_ : n_joints_;
    n_consts_ = soft_position_boundary_ ? 5 * n_joints_ : n_joints_;
    // Runtime distance dampers for arm-vs-structure pairs.
    // Adds up to kMaxDamperRows general inequality rows on q̇ - use with the
    // proxqp/daqp backends (dense; OSQP would re-factorize on the varying
    // sparsity pattern every tick).
    damper_enabled_ = std::getenv("RBY1_ARM_DAMPER") != nullptr;
    if (damper_enabled_) {
      // Soft rows: a·q̇ + s >= lb with s >= 0 and a strong quadratic penalty
      // on s. Always feasible by construction (s absorbs any conflict with
      // the velocity/braking box), and a violation is automatically the
      // minimum one - i.e. maximum-effort braking - so no feasibility ramp,
      // box clamp or drop-and-retry machinery is needed.
      damper_col_offset_ = n_vars_;
      n_vars_ += kMaxDamperRows;
      n_consts_ += 2 * kMaxDamperRows;  // damper rows + s >= 0 rows
    }
    if (std::getenv("RBY1_QP_LEGACY_SETTINGS") != nullptr) {
      qp_solver_.Setup(n_vars_, n_consts_);  // 2ms time limit, solver-default tolerances
    } else {
      qp_solver_.Setup(n_vars_, n_consts_, math::QPSolverSettings{});
    }
    if (std::getenv("RBY1_QP_WARMSTART_PREV") != nullptr) {
      qp_solver_.SetWarmStartMode(math::QPWarmStartMode::kPreviousSolution);
    }
    sqrt_braking_ = std::getenv("RBY1_QP_SQRT_BRAKING") != nullptr;
    // Worst-case stacked-cost rows: kMaxReservedLinkTargets SE3 targets (6 rows
    // each) + COM (3) + joint target (n) + nullspace posture (n) + nullspace
    // regularization (n) + soft-boundary penalties (2n). Exceeding the
    // reservation stays correct but reallocates.
    constexpr int kMaxReservedLinkTargets = 8;
    qp_solver_.ReserveCostRows(6 * kMaxReservedLinkTargets + 3 + 5 * n_joints_ + kMaxDamperRows);

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

    // Pressed = some selected joint sits within margin of its effective
    // position bound (static URDF or per-command). Used by controllers to
    // tell "stalled against a hard limit" from "still converging" (the
    // slack-free formulation has no soft cushion, so a limit-blocked target
    // is a dead press at zero velocity).
    position_bound_pressed_ = false;
    {
      constexpr double kPressedMargin = 1 * math::kDeg2Rad;
      for (const auto& i : joint_idx_) {
        const double qi = state->GetQ()(i);
        if (qi > position_upper_limit(i) - kPressedMargin || qi < position_lower_limit(i) + kPressedMargin) {
          position_bound_pressed_ = true;
          break;
        }
      }
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

    // Snapshot of the cost before adding nullspace cost
    const Eigen::MatrixXd A_cost_primary = qp_solver_.GetACost();
    const Eigen::MatrixXd J_primary = A_cost_primary.leftCols(n_joints_);

    if (in.nullspace_q_target) {
      AddNullspaceCostTowardQTarget_Projected(*in.nullspace_q_target, state, dt, J_primary);
    }

    if (nullspace_mapping_) {
      AddNullspaceCost_Projected(1e-4, J_primary);
    }

    if (soft_position_boundary_) {
      const double weight = 1.;
      const double safety_margin = 5 * math::kDeg2Rad;

      Eigen::VectorXd q = state->GetQ();

      Eigen::MatrixXd upper_penalty = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
      Eigen::MatrixXd lower_penalty = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
      for (int i = 0; i < n_joints_; i++) {
        if (q(joint_idx_[i]) > position_upper_limit(joint_idx_[i]) - safety_margin) {
          const double alpha =
              std::clamp((safety_margin - (position_upper_limit(joint_idx_[i]) - q(joint_idx_[i]))) / safety_margin, 1.e-6, 1.0);
          upper_penalty(i, n_joints_ + i) = weight * alpha;
        } else {
          upper_penalty(i, n_joints_ + i) = 1.e-6;  // DUMMY FOR PATTERN LOCK
        }
        if (q(joint_idx_[i]) < position_lower_limit(joint_idx_[i]) + safety_margin) {
          const double alpha =
              std::clamp((safety_margin - (q(joint_idx_[i]) - position_lower_limit(joint_idx_[i]))) / safety_margin, 1.e-6, 1.0);
          lower_penalty(i, 2 * n_joints_ + i) = weight * alpha;
        } else {
          lower_penalty(i, 2 * n_joints_ + i) = 1.e-6;  // DUMMY FOR PATTERN LOCK
        }
      }
      qp_solver_.AddCostFunction(upper_penalty, Eigen::VectorXd::Zero(n_joints_));
      qp_solver_.AddCostFunction(lower_penalty, Eigen::VectorXd::Zero(n_joints_));
    }

    if (damper_enabled_) {
      if (damper_A_.size() == 0) {
        InitDamper(state);
      }
      if (damper_enabled_) {
        AssembleDamperRows(state);
        // Strong quadratic penalty on the damper slacks: violating a row by
        // v [m/s] costs (kDamperPenalty*v)^2, dwarfing the task terms, so
        // the QP brakes with maximum effort before conceding any approach.
        constexpr double kDamperPenalty = 50.0;
        Eigen::MatrixXd slack_penalty = Eigen::MatrixXd::Zero(kMaxDamperRows, n_vars_);
        for (int k = 0; k < kMaxDamperRows; k++) {
          slack_penalty(k, damper_col_offset_ + k) = kDamperPenalty;
        }
        qp_solver_.AddCostFunction(slack_penalty, Eigen::VectorXd::Zero(kMaxDamperRows));
      }
    }

    qp_solver_.SetCostFunction(qp_solver_.GetACost(), qp_solver_.GetBCost());

    // Braking deceleration for the position-limit envelope. Cartesian paths
    // pass acceleration_limit = 1e6 (accel shaping is upstream), so fall back
    // to the model's qddot limits for a physically meaningful braking rate.
    Eigen::VectorXd brake_accel = robot_->GetLimitQddotUpper(state).cwiseMin(acceleration_limit);


    if (!SetInequalityConstraints(state->GetQ()(joint_idx_),         //
                                  state->GetQdot()(joint_idx_),      //
                                  position_lower_limit(joint_idx_),  //
                                  position_upper_limit(joint_idx_),  //
                                  velocity_limit(joint_idx_),        //
                                  acceleration_limit(joint_idx_),    //
                                  brake_accel(joint_idx_),           //
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
      ss << "[OptimalControl::Solve] QP error: " << e.what();
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

  /** True if some selected joint is pressed against its effective position
   * bound (static URDF or per-command) as of the last Solve. Lets
   * controllers distinguish "stalled against a hard limit" from "still
   * converging" and finish gracefully. */
  bool GetPositionBoundPressed() const { return position_bound_pressed_; }

  /** True if any arm distance-damper row was active in the last Solve
   * (RBY1_ARM_DAMPER): the commanded approach is being held back by a
   * proximity constraint, so a stalled target may be damper-blocked. */
  bool GetDamperActive() const { return damper_enabled_ && n_active_damper_rows_ > 0; }

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

  void AddNullspaceCost_Projected(double weight, const Eigen::MatrixXd& J_primary) {
    Eigen::MatrixXd P = ComputeNullspaceProjection(J_primary);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
    A.block(0, 0, n_joints_, n_joints_) = weight * P;
    qp_solver_.AddCostFunction(A, Eigen::VectorXd::Zero(n_joints_));
  }

  void AddNullspaceCostToward_Projected(const Eigen::Vector<double, DOF>& v_des_full, double weight,
                                        const Eigen::MatrixXd& J_primary) {
    const Eigen::MatrixXd P = ComputeNullspaceProjection(J_primary);
    Eigen::VectorXd v_des = v_des_full(joint_idx_);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
    A.block(0, 0, n_joints_, n_joints_) = weight * P;
    Eigen::VectorXd b = weight * P * v_des;
    qp_solver_.AddCostFunction(A, b);
  }

  void AddNullspaceCostTowardQTarget_Projected(const NullspaceJointTarget& target,
                                               const std::shared_ptr<dyn::State<DOF>>& state, double dt,
                                               const Eigen::MatrixXd& J_primary) {
    const double h = std::max(dt, 1e-6);
    const Eigen::VectorXd q = state->GetQ();
    const Eigen::VectorXd qdot = state->GetQdot();

    Eigen::Vector<double, DOF> v_des;
    if constexpr (DOF > 0) {
      v_des.setZero();
    } else {
      v_des = Eigen::Vector<double, DOF>::Zero(q.size());
    }
    v_des(joint_idx_) = target.k_p * (target.q(joint_idx_) - q(joint_idx_)) / h - target.k_d * qdot(joint_idx_);

    const Eigen::MatrixXd P = ComputeNullspaceProjection(J_primary);

    if ((target.weight.array() > 1e-6).any()) {
      Eigen::VectorXd w = target.weight(joint_idx_).cwiseMax(0.0);
      Eigen::MatrixXd W = w.asDiagonal();
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_joints_, n_vars_);
      A.block(0, 0, n_joints_, n_joints_) = target.cost_weight * (W * P);
      Eigen::VectorXd b = target.cost_weight * (W * P) * v_des(joint_idx_);
      qp_solver_.AddCostFunction(A, b);
    } else {
      AddNullspaceCostToward_Projected(v_des, target.cost_weight, J_primary);
    }
  }

  // Distance-damper assembly: nearest arm-vs-(base|torso) pairs get a row
  //   n̂ᵀ(ṗ1 − ṗ2) = a·q̇ ≥ −ξ·(d − d_safe)/(d_infl − d_safe)
  // which caps the approach speed proportionally to the remaining clearance
  // (feasible by construction while d > d_safe). Rows live in
  // damper_A_/damper_lb_; unused rows stay zero with ±1e6 bounds.
  void AssembleDamperRows(const std::shared_ptr<dyn::State<DOF>>& state) {
    constexpr double kInfluence = 0.20;   // [m] activation distance
    constexpr double kSafe = 0.03;        // [m] hard floor
    constexpr double kXi = 0.5;           // [m/s] max approach speed at d = d_infl
    damper_A_.setZero();
    damper_lb_.setConstant(-1e6);
    damper_ub_.setConstant(1e6);

    robot_->ComputeForwardKinematics(state);
    auto results = robot_->DetectCollisionsOrNearestLinks(state, 16);
    if (link_index_.empty()) {
      const auto names = state->GetLinkNames();
      for (int i = 0; i < (int)names.size(); i++) {
        link_index_[names[i]] = i;
      }
    }

    int row = 0;
    for (const auto& r : results) {
      if (row >= kMaxDamperRows || r.distance >= kInfluence) {
        break;  // results are sorted by distance
      }
      const bool arm1 = r.link1.find("_arm_") != std::string::npos;
      const bool arm2 = r.link2.find("_arm_") != std::string::npos;
      const bool st1 = r.link1 == "base" || r.link1.rfind("link_torso_", 0) == 0;
      const bool st2 = r.link2 == "base" || r.link2.rfind("link_torso_", 0) == 0;
      if (!((arm1 && st2) || (arm2 && st1) || (arm1 && arm2))) {
        continue;
      }
      const double d = std::max(r.distance, 1e-6);
      Eigen::Vector3d n_hat = r.position1 - r.position2;
      if (n_hat.norm() < 1e-9) {
        continue;
      }
      n_hat.normalize();
      const Eigen::Vector3d p = 0.5 * (r.position1 + r.position2);

      // a·q̇ = n̂ᵀ(ṗ on link1 − ṗ on link2), space Jacobian: ṗ = ω_s×p + v_s
      auto add_link = [&](const std::string& link_name, double sign) {
        auto it = link_index_.find(link_name);
        if (it == link_index_.end() || it->second == 0) {
          return;  // base or unknown: zero Jacobian
        }
        const auto J = robot_->ComputeSpaceJacobian(state, 0, it->second);
        for (int c = 0; c < n_joints_; c++) {
          const int j = (int)joint_idx_[c];
          const Eigen::Vector3d w = J.template block<3, 1>(0, j);
          const Eigen::Vector3d v = J.template block<3, 1>(3, j);
          damper_A_(row, c) += sign * n_hat.dot(w.cross(p) + v);
        }
      };
      add_link(r.link1, +1.0);
      add_link(r.link2, -1.0);
      if (damper_A_.row(row).cwiseAbs().maxCoeff() < 1e-9) {
        damper_A_.row(row).setZero();
        continue;  // pair not influenced by our joints
      }
      const double margin = std::max(0.0, d - kSafe) / (kInfluence - kSafe);
      const double lb_target = -kXi * margin;  // ḋ may not fall below this
      double adot_now = 0;
      for (int c = 0; c < n_joints_; c++) {
        adot_now += damper_A_(row, c) * state->GetQdot()((int)joint_idx_[c]);
      }
      // Only constrain pairs that are actually about to violate the cap.
      // Structurally-adjacent pairs (e.g. the shoulder mount) sit at constant
      // small distance with ḋ≈0 and would otherwise saturate the row slots.
      if (adot_now >= lb_target + 0.05) {
        damper_A_.row(row).setZero();
        continue;
      }
      // Soft row: the slack absorbs whatever the velocity/braking box cannot
      // deliver, so the full cap is demanded directly.
      damper_lb_(row) = lb_target;
      if (std::getenv("RBY1_ARM_DAMPER_DEBUG") != nullptr) {
        std::cerr << "[damper] " << r.link1 << "<->" << r.link2 << " d=" << d << " lb_target=" << lb_target
                  << " adot=" << adot_now << "\n";
      }
      row++;
    }
    n_active_damper_rows_ = row;
  }

  // Augment the controller's dynamics model (analysis-style base rails +
  // pelvis-enabled fold pairs) so the damper can see arm-vs-base proximity;
  // the shipped URDF gives the base no collision geometry at all. Geometry
  // provenance identical to tools/torso_limit_analysis - validate on robot.
  void InitDamper(const std::shared_ptr<dyn::State<DOF>>& state) {
    auto base = robot_->GetLink("base");
    if (!base) {
      damper_enabled_ = false;
      return;
    }
    for (const auto& col : base->GetCollisions()) {
      if (col->GetName().rfind("analysis_base_", 0) == 0) {
        ResizeDamperBuffers();
        return;  // already augmented (shared robot, another instance did it)
      }
    }
    unsigned int arm_types = 0;
    const auto link_names = state->GetLinkNames();
    for (const auto& name : link_names) {
      if (name.find("_arm_") == std::string::npos) {
        continue;
      }
      auto link = robot_->GetLink(name);
      if (!link) {
        continue;
      }
      for (const auto& col : link->GetCollisions()) {
        for (const auto& geom : col->GetGeoms()) {
          arm_types |= geom->GetColtype();
        }
      }
    }
    auto add_capsule = [&](const std::string& name, const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy,
                           double radius, double length) {
      auto col = std::make_shared<dyn::Collision>(name);
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.template block<3, 3>(0, 0) = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                                         .toRotationMatrix();
      T.template block<3, 1>(0, 3) = xyz;
      col->SetOrigin(T);
      col->AddGeom(std::make_shared<dyn::GeomCapsule>(length, radius, 1u << 22, arm_types));
      base->AddCollision(col);
    };
    add_capsule("analysis_base_rail_left", {0.0, 0.23, 0.20}, {0, M_PI_2, 0}, 0.09, 0.46);
    add_capsule("analysis_base_rail_right", {0.0, -0.23, 0.20}, {0, M_PI_2, 0}, 0.09, 0.46);
    add_capsule("analysis_base_front", {0.30, 0.0, 0.20}, {M_PI_2, 0, 0}, 0.09, 0.40);
    ResizeDamperBuffers();
  }

  void ResizeDamperBuffers() {
    damper_A_.resize(kMaxDamperRows, n_joints_);
    damper_lb_.resize(kMaxDamperRows);
    damper_ub_.resize(kMaxDamperRows);
  }

  bool SetInequalityConstraints(const Eigen::VectorXd& q,                   //
                                const Eigen::VectorXd& qdot,                //
                                const Eigen::VectorXd& q_lb,                //
                                const Eigen::VectorXd& q_ub,                //
                                const Eigen::VectorXd& velocity_limit,      //
                                const Eigen::VectorXd& acceleration_limit,  //
                                const Eigen::VectorXd& brake_accel,         //
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
    if (damper_enabled_) {
      const int r0 = n_consts_ - 2 * kMaxDamperRows;  // damper rows: a·q̇ + s
      A_const.block(r0, 0, kMaxDamperRows, n_joints_) = damper_A_;
      for (int k = 0; k < kMaxDamperRows; k++) {
        A_const(r0 + k, damper_col_offset_ + k) = 1.0;
        A_const(n_consts_ - kMaxDamperRows + k, damper_col_offset_ + k) = 1.0;  // s >= 0
      }
    }

    Eigen::VectorXd qdot_lb = -velocity_limit;
    Eigen::VectorXd qdot_ub = velocity_limit;
    qdot_lb = qdot_lb.cwiseMax(qdot - acceleration_limit * dt);
    qdot_ub = qdot_ub.cwiseMin(qdot + acceleration_limit * dt);
    if (sqrt_braking_) {
      // Discrete-time viability braking envelope: the next-tick velocity v may
      // not exceed the speed from which braking at a_brake still stops at the
      // bound, accounting for the trapezoidal position update of this tick:
      //   d - (q̇+v)/2*dt >= v^2/(2a)  =>  v <= a*(-dt/2 + sqrt(dt²/4 + 2*d_eff/a))
      // with d_eff = d - q̇*dt/2. Replaces the one-step stop law below, which
      // forbids approach speeds a full braking trajectory can handle and
      // aborts motions instead of decelerating them.
      for (int i = 0; i < n_joints_; i++) {
        const double a = std::max(1e-9, brake_accel(i));
        const double d_ub = std::max(0.0, q_ub(i) - q(i) - 0.5 * dt * qdot(i));
        const double d_lb = std::max(0.0, q(i) - q_lb(i) + 0.5 * dt * qdot(i));
        const double ub_brake = a * (-0.5 * dt + std::sqrt(0.25 * dt * dt + 2.0 * d_ub / a));
        const double lb_brake = -a * (-0.5 * dt + std::sqrt(0.25 * dt * dt + 2.0 * d_lb / a));
        qdot_ub(i) = std::min(qdot_ub(i), ub_brake);
        qdot_lb(i) = std::max(qdot_lb(i), lb_brake);
        if (qdot_lb(i) > qdot_ub(i)) {
          // Position safety wins over the acceleration-rate cap: allow a
          // harder deceleration than acceleration_limit rather than fail.
          qdot_ub(i) = std::min(velocity_limit(i), ub_brake);
          qdot_lb(i) = std::max(-velocity_limit(i), lb_brake);
          if (qdot_lb(i) > qdot_ub(i)) {  // only possible if q_ub < q_lb
            qdot_lb(i) = qdot_ub(i);
          }
        }
      }
    } else {
      qdot_lb = qdot_lb.cwiseMax(2 * (q_lb - q) / dt - qdot);
      qdot_ub = qdot_ub.cwiseMin(2 * (q_ub - q) / dt - qdot);
    }

    if ((qdot_lb.array() <= qdot_ub.array()).all()) {
      Eigen::VectorXd lb = Eigen::VectorXd::Zero(n_consts_);
      Eigen::VectorXd ub = Eigen::VectorXd::Zero(n_consts_);

      lb.head(n_joints_) = qdot_lb;
      ub.head(n_joints_) = qdot_ub;
      lb.tail(n_consts_ - n_joints_).setZero();
      ub.tail(n_consts_ - n_joints_).setConstant(1e6);
      if (damper_enabled_) {
        lb.segment(n_consts_ - 2 * kMaxDamperRows, kMaxDamperRows) = damper_lb_;
        ub.segment(n_consts_ - 2 * kMaxDamperRows, kMaxDamperRows) = damper_ub_;
        lb.tail(kMaxDamperRows).setZero();          // s >= 0
        ub.tail(kMaxDamperRows).setConstant(1e6);
      }

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
  bool sqrt_braking_{false};

  bool position_bound_pressed_{false};

  static constexpr int kMaxDamperRows = 8;
  bool damper_enabled_{false};
  int damper_col_offset_{0};
  int n_active_damper_rows_{0};
  Eigen::MatrixXd damper_A_;
  Eigen::VectorXd damper_lb_, damper_ub_;
  std::map<std::string, int> link_index_;
  bool nullspace_mapping_{false};
  bool soft_position_boundary_{false};

  std::vector<unsigned int> joint_idx_;
  std::vector<unsigned int> unselected_joint_idx_;
};

}  // namespace rb