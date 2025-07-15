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

class RBY1_SDK_API QPSolver {
 public:
  QPSolver();

  ~QPSolver();

  void Setup(int n_var, int n_const, double time_limit = 2e-3);

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

 private:
  std::unique_ptr<QPSolverImpl> impl_;
};

}  // namespace rb::math