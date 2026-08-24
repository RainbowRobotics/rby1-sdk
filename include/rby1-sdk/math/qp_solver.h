#pragma once

#include <exception>
#include <memory>
#include <optional>

#include "Eigen/Core"
#include "rby1-sdk/export.h"

namespace rb::math {

class RBY1_SDK_API QPSolverException : public std::exception {
 public:
  static constexpr unsigned int kStatusOffset = 100;

  explicit QPSolverException(int error_code);

  const char* what() const noexcept override;

  int code() const noexcept;

  static std::string GenerateMessage(int code);

 private:
  int error_code_;
  std::string message_;
};

class QPSolverImpl;

/**
 * Solver settings. Defaults are the Stage-2 tuned values; QPSolver::Setup's
 * time_limit overload keeps the legacy behavior (2ms limit, solver-default
 * tolerances) for backward compatibility.
 */
struct RBY1_SDK_API QPSolverSettings {
  double time_limit{1e-3};   // [s] hard cap, well under the 0.9*dt control watchdog
  double eps_abs{1e-3};
  double eps_rel{1e-3};
  int max_iter{4000};
  int check_termination{25};
};

enum class QPWarmStartMode {
  kExternal,           // warm start from SetPrimalVariable() (legacy: measured velocity)
  kPreviousSolution,   // warm start from the previous Solve() primal+dual
};

enum class QPBackendType {
  kOSQP,
  kProxQP,  // available when built with RBY1_WITH_PROXQP
  kDAQP,    // available when built with RBY1_WITH_DAQP
};

class RBY1_SDK_API QPSolver {
 public:
  QPSolver();

  ~QPSolver();

  void Setup(int n_var, int n_const, double time_limit = 2e-3);

  void Setup(int n_var, int n_const, const QPSolverSettings& settings);

  /**
   * Preallocate the stacked least-squares cost storage for up to max_rows
   * rows, so AddCostFunction does not reallocate in the control loop. Rows
   * beyond the reservation fall back to reallocation (correct but slower).
   */
  void ReserveCostRows(int max_rows);

  void SetWarmStartMode(QPWarmStartMode mode);

  /** Drop the cached previous solution (kPreviousSolution mode). */
  void ResetWarmStart();

  /**
   * Select the solver backend. Takes effect at the next Setup() call
   * (or set env RBY1_QP_BACKEND before construction).
   */
  void SetBackend(QPBackendType type);

  void InitFunction();

  void AddCostFunction(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

  void SetCostFunction(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

  void SetConstraintsFunction(const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);

  void SetPrimalVariable(const Eigen::VectorXd& pv);

  void ResetIsFirst();

  /**
   * Solve the QP problem
   * @return Solution
   * @throw QPSolverException
   */
  Eigen::VectorXd Solve();

  Eigen::MatrixXd GetACost() const;  // NOLINT

  Eigen::VectorXd GetBCost() const;  // NOLINT

  Eigen::MatrixXd GetAConst() const;  // NOLINT

  Eigen::VectorXd GetLowerBound() const;  // NOLINT

  Eigen::VectorXd GetUpperBound() const;  // NOLINT

  /**
   * Wall-clock duration of the last Solve() call in seconds.
   * Valid after the first Solve(); diagnostics only.
   */
  double GetLastSolveTime() const;  // NOLINT

  /**
   * Iteration count reported by the underlying solver for the last Solve() call.
   * Valid after the first Solve(); diagnostics only.
   */
  int GetLastIterations() const;  // NOLINT

 private:
  std::unique_ptr<QPSolverImpl> impl_;
};

}  // namespace rb::math