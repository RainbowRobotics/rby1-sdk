#pragma once

// Internal QP backend interface (not installed). QPSolver owns problem
// assembly (stacked least-squares cost, dense H = A^T A, constraints, dump,
// timing); a backend owns one solver library. Backends must be swappable
// without touching rb::math::QPSolver's public API.

#include <memory>

#include "Eigen/Core"
#include "rby1-sdk/math/qp_solver.h"

namespace rb::math {

// Backend-neutral solve status. Backends map their native codes onto this;
// QPSolverException keeps the legacy OSQP-derived integer codes for backward
// compatibility (core stringifies them only).
enum class QPStatus {
  kSolved,
  kSolvedInaccurate,
  kPrimalInfeasible,
  kDualInfeasible,
  kMaxIterReached,
  kTimeLimitReached,
  kNonConvex,
  kNumericalError,
  kUnsolved,
};

struct QPSolveStats {
  int iterations{-1};
};

class QPBackend {
 public:
  virtual ~QPBackend() = default;

  virtual void Setup(int n_var, int n_const, const QPSolverSettings& settings, bool legacy_tolerances) = 0;

  /** Full problem for this tick: min 1/2 x'Hx + g'x  s.t. lb <= Ax <= ub. */
  virtual void SetProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
                          const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) = 0;

  /** Primal (and optional dual, pass nullptr to skip) warm start. */
  virtual void WarmStart(const Eigen::VectorXd& x, const Eigen::VectorXd* y) = 0;

  /**
   * Solve the problem set by SetProblem.
   * @return solution
   * @throw QPSolverException on failure (backend maps native codes)
   */
  virtual Eigen::VectorXd Solve(QPSolveStats& stats) = 0;

  /** Dual solution of the last successful Solve (empty if unsupported). */
  virtual Eigen::VectorXd GetDualSolution() = 0;

  /** Reset internal state so the next Solve performs a cold setup. */
  virtual void Reset() = 0;
};

std::unique_ptr<QPBackend> MakeOsqpBackend();

#ifdef RBY1_WITH_PROXQP
std::unique_ptr<QPBackend> MakeProxqpBackend();
#endif

#ifdef RBY1_WITH_DAQP
std::unique_ptr<QPBackend> MakeDaqpBackend();
#endif

}  // namespace rb::math
