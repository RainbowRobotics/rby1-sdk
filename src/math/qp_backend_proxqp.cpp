#include <proxsuite/proxqp/dense/dense.hpp>

#include <algorithm>

#include "qp_backend.h"

namespace rb::math {

namespace {

using proxsuite::nullopt;
using proxsuite::proxqp::InitialGuessStatus;
using proxsuite::proxqp::QPSolverOutput;

// Exception codes 200+ are the ProxQP range (OSQP owns <100 and 100+status).
constexpr int kProxqpCodeBase = 200;

int MapStatus(QPSolverOutput status) {
  switch (status) {
    case QPSolverOutput::PROXQP_MAX_ITER_REACHED:
      return kProxqpCodeBase + 1;
    case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
      return kProxqpCodeBase + 2;
    case QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
      return kProxqpCodeBase + 3;
    case QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
      return kProxqpCodeBase + 4;
    default:
      return kProxqpCodeBase + 9;
  }
}

// ProxQP dense backend. Problem form: min 1/2 x'Hx + g'x s.t. l <= Cx <= u
// (no equality block). Notes vs OSQP backend:
//  - no wall-clock time limit; bounded by max_iter (settings.time_limit ignored)
//  - handles semidefinite H natively, so the pattern-lock hack is unnecessary
//  - warm start is explicit via solve(x0, y0, z0)
//  - pure-box problems (A == I, the RBY1_QP_NO_SLACK formulation) are routed
//    through proxsuite's dedicated box-constraint path (n_in = 0), which
//    avoids carrying an identity general-inequality block through the
//    factorization.
class ProxqpBackend final : public QPBackend {
 public:
  void Setup(int n_var, int n_const, const QPSolverSettings& settings, bool /*legacy_tolerances*/) override {
    n_var_ = n_var;
    n_const_ = n_const;
    settings_ = settings;
    qp_.reset();
    first_ = true;
    has_warm_x_ = false;
    has_warm_z_ = false;
  }

  void ApplySettings() {
    // ProxQP reaches 1e-6 at the same cost as 1e-3 on this problem family
    // (measured: identical p50/p99), so never run it looser than 1e-6. A
    // caller asking for even tighter tolerances is honored.
    qp_->settings.eps_abs = std::min(settings_.eps_abs, 1e-6);
    qp_->settings.eps_rel = std::min(settings_.eps_rel, 1e-6);
    qp_->settings.max_iter = settings_.max_iter;
    qp_->settings.initial_guess = InitialGuessStatus::WARM_START;
    qp_->settings.verbose = false;
  }

  void SetProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
                  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) override {
    H_ = H;
    g_ = g;
    C_ = A;
    lb_ = lb;
    ub_ = ub;
  }

  void WarmStart(const Eigen::VectorXd& x, const Eigen::VectorXd* y) override {
    warm_x_ = x;
    has_warm_x_ = true;
    if (y != nullptr) {
      warm_z_ = *y;
      has_warm_z_ = true;
    } else {
      has_warm_z_ = false;
    }
  }

  Eigen::VectorXd Solve(QPSolveStats& stats) override {
    if (first_) {
      // Decide box vs general mode from the first problem's structure (it is
      // fixed per OptimalControl instance).
      box_mode_ = (n_const_ == n_var_) && C_.isIdentity(1e-12);
      if (box_mode_) {
        qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(
            n_var_, 0, 0, /*box_constraints=*/true, proxsuite::proxqp::HessianType::Dense);
        ApplySettings();
        qp_->init(H_, g_, nullopt, nullopt, nullopt, nullopt, nullopt, lb_, ub_);
      } else {
        qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(n_var_, 0, n_const_);
        ApplySettings();
        qp_->init(H_, g_, nullopt, nullopt, C_, lb_, ub_);
      }
      first_ = false;
    } else if (box_mode_) {
      qp_->update(H_, g_, nullopt, nullopt, nullopt, nullopt, nullopt, lb_, ub_);
    } else {
      qp_->update(H_, g_, nullopt, nullopt, C_, lb_, ub_);
    }

    if (has_warm_x_ && has_warm_z_) {
      qp_->solve(warm_x_, nullopt, warm_z_);
    } else if (has_warm_x_) {
      qp_->solve(warm_x_, nullopt, nullopt);
    } else {
      qp_->solve();
    }

    stats.iterations = (int)qp_->results.info.iter;

    const auto status = qp_->results.info.status;
    if (status != QPSolverOutput::PROXQP_SOLVED) {
      throw QPSolverException(MapStatus(status));
    }
    return qp_->results.x;
  }

  Eigen::VectorXd GetDualSolution() override { return qp_->results.z; }

  void Reset() override { first_ = true; }

 private:
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;

  int n_var_{};
  int n_const_{};
  bool first_{true};
  bool box_mode_{false};
  QPSolverSettings settings_;

  Eigen::MatrixXd H_, C_;
  Eigen::VectorXd g_, lb_, ub_;
  Eigen::VectorXd warm_x_, warm_z_;
  bool has_warm_x_{false}, has_warm_z_{false};
};

}  // namespace

std::unique_ptr<QPBackend> MakeProxqpBackend() {
  return std::make_unique<ProxqpBackend>();
}

}  // namespace rb::math
