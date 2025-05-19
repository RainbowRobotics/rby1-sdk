#include "rby1-sdk/math/qp_solver.h"

#include "OsqpEigen/OsqpEigen.h"

namespace rb::math {

QPSolverException::QPSolverException(int error_code) : error_code_(error_code), message_(GenerateMessage(error_code)) {}

const char* QPSolverException::what() const noexcept {
  return message_.c_str();
}

int QPSolverException::code() const noexcept {
  return error_code_;
}

std::string QPSolverException::GenerateMessage(int code) {
  static const std::unordered_map<int, std::string> error_messages = {
      {OSQP_DATA_VALIDATION_ERROR, "Data validation error"},
      {OSQP_SETTINGS_VALIDATION_ERROR, "Settings validation error"},
#ifdef OSQP_EIGEN_OSQP_IS_V1
      {OSQP_ALGEBRA_LOAD_ERROR, "Linear system solver load error"},
#else
      {OSQP_LINSYS_SOLVER_LOAD_ERROR, "Linear system solver load error"},
#endif
      {OSQP_LINSYS_SOLVER_INIT_ERROR, "Linear system solver initialization error"},
      {OSQP_NONCVX_ERROR, "Non-convex error"},
      {OSQP_MEM_ALLOC_ERROR, "Memory allocation error"},
      {OSQP_WORKSPACE_NOT_INIT_ERROR, "Workspace not initialized"},

      {QPSolverException::kStatusOffset + OSQP_SOLVED, "Solved"},
      {QPSolverException::kStatusOffset + OSQP_SOLVED_INACCURATE, "Solved inaccurate"},
      {QPSolverException::kStatusOffset + OSQP_PRIMAL_INFEASIBLE, "Primal infeasible"},
      {QPSolverException::kStatusOffset + OSQP_PRIMAL_INFEASIBLE_INACCURATE, "Primal infeasible inaccurate"},
      {QPSolverException::kStatusOffset + OSQP_DUAL_INFEASIBLE, "Dual infeasible"},
      {QPSolverException::kStatusOffset + OSQP_DUAL_INFEASIBLE_INACCURATE, "Dual infeasible inaccurate"},
      {QPSolverException::kStatusOffset + OSQP_MAX_ITER_REACHED, "Maximum iterations reached"},
      {QPSolverException::kStatusOffset + OSQP_TIME_LIMIT_REACHED, "Run time limit reached"},
      {QPSolverException::kStatusOffset + OSQP_NON_CVX, "Problem non convex"},
      {QPSolverException::kStatusOffset + OSQP_SIGINT, "Interrupted"},
      {QPSolverException::kStatusOffset + OSQP_UNSOLVED, "Unsolved"},

      {-1, "General solver error or matrix update failure"},
  };

  auto it = error_messages.find(code);
  if (it != error_messages.end()) {
    return it->second;
  }
  return "Unknown solver error (code: " + std::to_string(code) + ")";
}

class QPSolverImpl {
 public:
  void SetupImpl(int n_var, int n_const, double time_limit = 2e-3) {
    n_var_ = n_var;
    n_const_ = n_const;

    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setTimeLimit(time_limit);

    solver_.data()->setNumberOfVariables(n_var);
    solver_.data()->setNumberOfConstraints(n_const);

    primal_variable_for_warmstart_.resize(n_var_, 1);
    primal_variable_for_warmstart_.setZero();

    is_first_ = true;

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
      std::cerr << "OSQP Solver Add cost function failed: Size issue Ax = b (A.rows(): " << A.rows()
                << ", b.rows(): " << b.rows() << ")" << std::endl;
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

  Eigen::VectorXd SolveImpl() {
    Eigen::MatrixXd ret;

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

    solver_.setPrimalVariable(primal_variable_for_warmstart_);

    const auto result = solver_.solveProblem();
    if (result != OsqpEigen::ErrorExitFlag::NoError) {
      throw QPSolverException(static_cast<int>(result));
    }
    if (solver_.getStatus() != OsqpEigen::Status::Solved) {
      throw QPSolverException((int)QPSolverException::kStatusOffset + static_cast<int>(solver_.getStatus()));
    }

    return solver_.getSolution();
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

void QPSolver::Setup(int n_var, int n_const, double time_limit) {
  impl_->SetupImpl(n_var, n_const, time_limit);
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

Eigen::VectorXd QPSolver::Solve() {
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