#include "OsqpEigen/OsqpEigen.h"
#include "qp_backend.h"

namespace rb::math {

namespace {

// OSQP backend. Preserves the pre-refactor QPSolver behavior exactly:
// sparse in-place updates while nnz(H) is unchanged (the "pattern lock"
// contract with OptimalControl's dummy entries), full re-init otherwise,
// legacy OSQP-derived exception codes.
class OsqpBackend final : public QPBackend {
 public:
  void Setup(int n_var, int n_const, const QPSolverSettings& settings, bool legacy_tolerances) override {
    n_var_ = n_var;
    n_const_ = n_const;

    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setTimeLimit(settings.time_limit);
    if (!legacy_tolerances) {
      solver_.settings()->setAbsoluteTolerance(settings.eps_abs);
      solver_.settings()->setRelativeTolerance(settings.eps_rel);
      solver_.settings()->setMaxIteration(settings.max_iter);
      solver_.settings()->setCheckTermination(settings.check_termination);
    }

    solver_.data()->setNumberOfVariables(n_var);
    solver_.data()->setNumberOfConstraints(n_const);

    is_first_ = true;
    n_hessian_element_ = 0;
    has_warm_x_ = false;
    has_warm_y_ = false;
  }

  void SetProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
                  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) override {
    hessian_ = H.sparseView();
    gradient_ = g;
    linearMatrix_ = A.sparseView();
    lowerBound_ = lb;
    upperBound_ = ub;
  }

  void WarmStart(const Eigen::VectorXd& x, const Eigen::VectorXd* y) override {
    warm_x_ = x;
    has_warm_x_ = true;
    if (y != nullptr) {
      warm_y_ = *y;
      has_warm_y_ = true;
    } else {
      has_warm_y_ = false;
    }
  }

  Eigen::VectorXd Solve(QPSolveStats& stats) override {
    bool need_init = false;

    if (!is_first_) {
      if (n_hessian_element_ == hessian_.nonZeros()) {
        if (!solver_.updateHessianMatrix(hessian_)) {
          need_init = true;
        }
        if (!solver_.updateGradient(gradient_)) {
          need_init = true;
        }
        if (!solver_.updateLinearConstraintsMatrix(linearMatrix_)) {
          need_init = true;
        }
        if (!solver_.updateBounds(lowerBound_, upperBound_)) {
          need_init = true;
        }
      } else {
        need_init = true;
      }
    } else {
      need_init = true;
    }

    if (need_init) {
      solver_.data()->clearHessianMatrix();
      solver_.data()->clearLinearConstraintsMatrix();
      solver_.clearSolver();

      n_hessian_element_ = (int)hessian_.nonZeros();

      if (!solver_.data()->setHessianMatrix(hessian_)) {
        throw QPSolverException(-1);
      }
      if (!solver_.data()->setGradient(gradient_)) {
        throw QPSolverException(-1);
      }
      if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix_)) {
        throw QPSolverException(-1);
      }
      if (!solver_.data()->setLowerBound(lowerBound_)) {
        throw QPSolverException(-1);
      }
      if (!solver_.data()->setUpperBound(upperBound_)) {
        throw QPSolverException(-1);
      }

      if (!solver_.initSolver()) {
        throw QPSolverException(-1);
      }

      is_first_ = false;
    }

    if (has_warm_x_) {
      solver_.setPrimalVariable(warm_x_);
      if (has_warm_y_ && warm_y_.size() == n_const_) {
        solver_.setDualVariable(warm_y_);
      }
    }

    const auto result = solver_.solveProblem();
    if (result != OsqpEigen::ErrorExitFlag::NoError) {
      stats.iterations = -1;
      throw QPSolverException(static_cast<int>(result));
    }
#ifdef OSQP_EIGEN_OSQP_IS_V1
    stats.iterations = (int)solver_.solver()->info->iter;
#else
    stats.iterations = (int)solver_.workspace()->info->iter;
#endif
    if (solver_.getStatus() != OsqpEigen::Status::Solved) {
      throw QPSolverException((int)QPSolverException::kStatusOffset + static_cast<int>(solver_.getStatus()));
    }

    return solver_.getSolution();
  }

  Eigen::VectorXd GetDualSolution() override { return solver_.getDualSolution(); }

  void Reset() override { is_first_ = true; }

 private:
  OsqpEigen::Solver solver_;

  int n_var_{};
  int n_const_{};
  int is_first_{true};
  int n_hessian_element_{};

  Eigen::SparseMatrix<double> hessian_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> gradient_;
  Eigen::SparseMatrix<double> linearMatrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> lowerBound_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> upperBound_;

  Eigen::VectorXd warm_x_, warm_y_;
  bool has_warm_x_{false}, has_warm_y_{false};
};

}  // namespace

std::unique_ptr<QPBackend> MakeOsqpBackend() {
  return std::make_unique<OsqpBackend>();
}

}  // namespace rb::math
