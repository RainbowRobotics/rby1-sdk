#include "rby1-sdk/math/qp_solver.h"

#include "OsqpEigen/OsqpEigen.h"

namespace rb::math {

class QPSolverImpl {
 public:
  void SetupImpl(int n_var, int n_const) {
    n_var_ = n_var;
    n_const_ = n_const;

    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);

    solver_.data()->setNumberOfVariables(n_var);
    solver_.data()->setNumberOfConstraints(n_const);

    primal_variable_for_warmstart_.resize(n_var_, 1);
    primal_variable_for_warmstart_.setZero();

    is_first_ = true;

    err_code_ = 0;
    n_hessian_element_ = 0;
  }

  void InitFunctionImpl() {
    A_cost_.resize(0, n_var_);
    b_cost_.resize(0, 1);

    A_const_.resize(0, n_var_);
    lb_.resize(0, 1);
    ub_.resize(0, 1);

    A_cost_.setZero();
    b_cost_.setZero();

    A_const_.setZero();
    lb_.setZero();
    ub_.setZero();
  }

  void AddCostFunctionImpl(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    if (A.rows() != b.rows()) {
      std::cerr << "OSQP Solver Add cost function failed: Size issue Ax = b" << std::endl;
      return;
    }

    if (b.cols() != 1) {
      std::cerr << "OSQP Solver Add cost function failed: Size issue: b (Nx1)" << std::endl;
      return;
    }

    int n_rows_old = (int)A_cost_.rows();
    int n_rows_add = (int)A.rows();

    if (n_rows_old == -1) {
      A_cost_.resize(n_rows_add, Eigen::NoChange);
      b_cost_.resize(n_rows_add, Eigen::NoChange);
      A_cost_.bottomRows(n_rows_add) = A;
      b_cost_.bottomRows(n_rows_add) = b;

    } else {
      A_cost_.conservativeResize(n_rows_old + n_rows_add, Eigen::NoChange);
      b_cost_.conservativeResize(n_rows_old + n_rows_add, Eigen::NoChange);
      A_cost_.bottomRows(n_rows_add) = A;
      b_cost_.bottomRows(n_rows_add) = b;
    }
  }

  void SetCostFunctionImpl(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
    // Ax = b

    hessian_ = (A.transpose() * A).sparseView();
    gradient_ = -b.transpose() * A;
  }

  void SetConstraintsFunctionImpl(const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
    // lb <= Ax <= ub

    linearMatrix_ = A.sparseView();
    lowerBound_ = lb;
    upperBound_ = ub;
  }

  void SetPrimalVariableImpl(const Eigen::VectorXd& pv) { primal_variable_for_warmstart_ = pv; }

  void ResetIsFirstImpl() { is_first_ = true; }

  std::optional<Eigen::VectorXd> SolveImpl() {
    Eigen::MatrixXd ret;

    int err = 0;

    if (!is_first_) {
      if (n_hessian_element_ == hessian_.nonZeros()) {
        if (!solver_.updateHessianMatrix(hessian_)) {
          err = -1;
        }
        if (!solver_.updateGradient(gradient_)) {
          err = -1;
        }
        if (!solver_.updateLinearConstraintsMatrix(linearMatrix_)) {
          err = -1;
        }
        if (!solver_.updateBounds(lowerBound_, upperBound_)) {
          err = -1;
        }
      } else {
        err = -1;
      }
    }

    if ((err == -1) || is_first_) {
      solver_.data()->clearHessianMatrix();
      solver_.data()->clearLinearConstraintsMatrix();
      solver_.clearSolver();
      err = 0;

      n_hessian_element_ = (int)hessian_.nonZeros();

      if (!solver_.data()->setHessianMatrix(hessian_)) {
        err = -1;
      }
      if (!solver_.data()->setGradient(gradient_)) {
        err = -1;
      }
      if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix_)) {
        err = -1;
      }
      if (!solver_.data()->setLowerBound(lowerBound_)) {
        err = -1;
      }
      if (!solver_.data()->setUpperBound(upperBound_)) {
        err = -1;
      }

      if (!solver_.initSolver()) {
        err = -1;
      }

      solver_.setPrimalVariable(primal_variable_for_warmstart_);

      is_first_ = false;
    }

    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
      err = -1;
    }

    if (err == 0) {
      ret = solver_.getSolution();
    } else {
      return {};
    }

    err_code_ = err;

    return ret;
  }

  Eigen::MatrixXd GetACostImpl() const {  // NOLINT
    return A_cost_;
  }

  Eigen::VectorXd GetBCostImpl() const {  // NOLINT
    return b_cost_;
  }

  Eigen::MatrixXd GetAConstImpl() const {  // NOLINT
    return A_const_;
  }

  Eigen::VectorXd GetLowerBoundImpl() const {  // NOLINT
    return lb_;
  }

  Eigen::VectorXd GetUpperBoundImpl() const {  // NOLINT
    return ub_;
  }

 private:
  OsqpEigen::Solver solver_;

  int n_var_{};
  int n_const_{};
  int err_code_{};

  int is_first_{};

  int n_hessian_element_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_cost_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> b_cost_;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_const_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> lb_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> ub_;

  Eigen::SparseMatrix<double> hessian_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> gradient_;

  Eigen::SparseMatrix<double> linearMatrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> lowerBound_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> upperBound_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> primal_variable_for_warmstart_;
};

QPSolver::QPSolver() {
  impl_ = std::make_unique<QPSolverImpl>();
}

QPSolver::~QPSolver() = default;

void QPSolver::Setup(int n_var, int n_const) {
  impl_->SetupImpl(n_var, n_const);
}

void QPSolver::InitFunction() {
  impl_->InitFunctionImpl();
}

void QPSolver::AddCostFunction(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  impl_->AddCostFunctionImpl(A, b);
}

void QPSolver::SetCostFunction(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) {
  impl_->SetCostFunctionImpl(A, b);
}

void QPSolver::SetConstraintsFunction(const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
  impl_->SetConstraintsFunctionImpl(A, lb, ub);
}

void QPSolver::SetPrimalVariable(const Eigen::VectorXd& pv) {
  impl_->SetPrimalVariableImpl(pv);
}

void QPSolver::ResetIsFirst() {
  impl_->ResetIsFirstImpl();
}

std::optional<Eigen::VectorXd> QPSolver::Solve() {
  return impl_->SolveImpl();
}

Eigen::MatrixXd QPSolver::GetACost() const {
  return impl_->GetACostImpl();
}

Eigen::VectorXd QPSolver::GetBCost() const {
  return impl_->GetBCostImpl();
}

Eigen::MatrixXd QPSolver::GetAConst() const {
  return impl_->GetACostImpl();
}

Eigen::VectorXd QPSolver::GetLowerBound() const {
  return impl_->GetLowerBoundImpl();
}

Eigen::VectorXd QPSolver::GetUpperBound() const {
  return impl_->GetUpperBoundImpl();
}

}  // namespace rb::math