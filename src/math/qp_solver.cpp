#include "rby1-sdk/math/qp_solver.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include "OsqpEigen/OsqpEigen.h"
#include "qp_backend.h"

namespace rb::math {

namespace {

// Binary QP instance dump for offline solver benchmarking (tools/qp_benchmark).
// Enabled by setting RBY1_QP_DUMP_DIR; intended for simulation runs only —
// writing to disk from the 500Hz control path is not RT-safe.
constexpr char kDumpDirEnv[] = "RBY1_QP_DUMP_DIR";
constexpr char kDumpMaxMbEnv[] = "RBY1_QP_DUMP_MAX_MB";  // per-instance cap, default 256
constexpr char kDumpMagic[8] = {'R', 'B', 'Q', 'P', 'D', 'M', 'P', '1'};
constexpr int32_t kDumpRecordMarker = 0x52454331;  // "REC1"
constexpr size_t kDumpDefaultMaxBytes = 256ull * 1024 * 1024;

std::atomic<int> dump_instance_counter{0};

void WriteRaw(std::ofstream& os, const void* data, size_t size) {
  os.write(static_cast<const char*>(data), (std::streamsize)size);
}

void WriteInt32(std::ofstream& os, int32_t v) {
  WriteRaw(os, &v, sizeof(v));
}

void WriteDouble(std::ofstream& os, double v) {
  WriteRaw(os, &v, sizeof(v));
}

void WriteMatrix(std::ofstream& os, const Eigen::MatrixXd& m) {  // column-major, dims implied by header
  WriteRaw(os, m.data(), sizeof(double) * (size_t)m.size());
}

}  // namespace

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

      // 200-range: ProxQP backend (see qp_backend_proxqp.cpp)
      {201, "ProxQP: maximum iterations reached"},
      {202, "ProxQP: primal infeasible"},
      {203, "ProxQP: dual infeasible"},
      {204, "ProxQP: solved to closest primal feasible"},
      {209, "ProxQP: not run / unknown status"},

      // 300-range: DAQP backend (see qp_backend_daqp.cpp)
      {301, "DAQP: soft-optimal"},
      {302, "DAQP: primal infeasible"},
      {303, "DAQP: cycling detected"},
      {304, "DAQP: unbounded"},
      {305, "DAQP: iteration limit reached"},
      {306, "DAQP: nonconvex problem"},
      {307, "DAQP: initial working set overdetermined"},
      {309, "DAQP: unknown error"},
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
    // Legacy setup: 2ms time limit, solver-default tolerances.
    QPSolverSettings settings;
    settings.time_limit = time_limit;
    SetupImpl(n_var, n_const, settings, /*legacy_tolerances=*/true);
  }

  void SetupImpl(int n_var, int n_const, const QPSolverSettings& settings, bool legacy_tolerances = false) {
    n_var_ = n_var;
    n_const_ = n_const;

    if (const char* backend_env = std::getenv("RBY1_QP_BACKEND"); backend_env != nullptr && backend_env[0] != '\0') {
      if (std::strcmp(backend_env, "osqp") == 0) {
        backend_type_ = QPBackendType::kOSQP;
      } else if (std::strcmp(backend_env, "proxqp") == 0) {
        backend_type_ = QPBackendType::kProxQP;
      } else if (std::strcmp(backend_env, "daqp") == 0) {
        backend_type_ = QPBackendType::kDAQP;
      } else {
        std::cerr << "[QPSolver] unknown RBY1_QP_BACKEND '" << backend_env << "', using osqp" << std::endl;
        backend_type_ = QPBackendType::kOSQP;
      }
    }
    switch (backend_type_) {
      case QPBackendType::kProxQP:
#ifdef RBY1_WITH_PROXQP
        backend_ = MakeProxqpBackend();
        break;
#else
        std::cerr << "[QPSolver] proxqp backend not compiled in (RBY1_WITH_PROXQP=OFF), using osqp" << std::endl;
        backend_ = MakeOsqpBackend();
        break;
#endif
      case QPBackendType::kDAQP:
#ifdef RBY1_WITH_DAQP
        backend_ = MakeDaqpBackend();
        break;
#else
        std::cerr << "[QPSolver] daqp backend not compiled in (RBY1_WITH_DAQP=OFF), using osqp" << std::endl;
        backend_ = MakeOsqpBackend();
        break;
#endif
      case QPBackendType::kOSQP:
      default:
        backend_ = MakeOsqpBackend();
        break;
    }
    backend_->Setup(n_var, n_const, settings, legacy_tolerances);

    has_previous_solution_ = false;

    primal_variable_for_warmstart_.resize(n_var_, 1);
    primal_variable_for_warmstart_.setZero();

    if (const char* dump_dir = std::getenv(kDumpDirEnv); dump_dir != nullptr && dump_dir[0] != '\0') {
      std::ostringstream filename;
      filename << dump_dir << "/qpdump_" << dump_instance_counter.fetch_add(1) << "_" << n_var << "x" << n_const
               << ".qplog";
      dump_stream_ = std::make_unique<std::ofstream>(filename.str(), std::ios::binary | std::ios::trunc);
      if (dump_stream_->good()) {
        WriteRaw(*dump_stream_, kDumpMagic, sizeof(kDumpMagic));
        WriteInt32(*dump_stream_, n_var_);
        WriteInt32(*dump_stream_, n_const_);
        dump_bytes_written_ = sizeof(kDumpMagic) + 8;
        dump_max_bytes_ = kDumpDefaultMaxBytes;
        if (const char* max_mb = std::getenv(kDumpMaxMbEnv); max_mb != nullptr && max_mb[0] != '\0') {
          dump_max_bytes_ = (size_t)std::strtoull(max_mb, nullptr, 10) * 1024 * 1024;
        }
      } else {
        dump_stream_.reset();
      }
    } else {
      dump_stream_.reset();
    }
  }

  void InitFunctionImpl() {
    if (cost_capacity_ > 0) {
      cost_rows_ = 0;
      return;
    }
    A_cost_.resize(0, n_var_);
    b_cost_.resize(0, 1);

    A_cost_.setZero();
    b_cost_.setZero();
  }

  void ReserveCostRowsImpl(int max_rows) {
    cost_capacity_ = max_rows;
    cost_rows_ = 0;
    A_cost_.resize(max_rows, n_var_);
    b_cost_.resize(max_rows, 1);
    A_cost_.setZero();
    b_cost_.setZero();
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

    const int n_rows_add = (int)A.rows();

    if (cost_capacity_ > 0) {
      if (cost_rows_ + n_rows_add > cost_capacity_) {
        // Reservation exceeded: grow storage (slow path, keeps correctness).
        cost_capacity_ = cost_rows_ + n_rows_add;
        A_cost_.conservativeResize(cost_capacity_, Eigen::NoChange);
        b_cost_.conservativeResize(cost_capacity_, Eigen::NoChange);
      }
      A_cost_.middleRows(cost_rows_, n_rows_add) = A;
      b_cost_.segment(cost_rows_, n_rows_add) = b;
      cost_rows_ += n_rows_add;
      return;
    }

    int n_rows_old = (int)A_cost_.rows();

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

    hessian_ = A.transpose() * A;
    gradient_ = -b.transpose() * A;
  }

  void SetConstraintsFunctionImpl(const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
    // lb <= Ax <= ub

    linearMatrix_ = A;
    lowerBound_ = lb;
    upperBound_ = ub;
  }

  void SetBackendImpl(QPBackendType type) { backend_type_ = type; }

  void SetPrimalVariableImpl(const Eigen::VectorXd& pv) { primal_variable_for_warmstart_ = pv; }

  void ResetIsFirstImpl() { backend_->Reset(); }

  Eigen::VectorXd SolveImpl() {
    backend_->SetProblem(hessian_, gradient_, linearMatrix_, lowerBound_, upperBound_);

    if (warm_start_mode_ == QPWarmStartMode::kPreviousSolution && has_previous_solution_) {
      backend_->WarmStart(previous_primal_, previous_dual_.size() == n_const_ ? &previous_dual_ : nullptr);
    } else {
      backend_->WarmStart(primal_variable_for_warmstart_, nullptr);
    }

    QPSolveStats stats;
    const auto t_start = std::chrono::steady_clock::now();
    try {
      Eigen::VectorXd solution = backend_->Solve(stats);
      last_solve_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
      last_iterations_ = stats.iterations;

      if (warm_start_mode_ == QPWarmStartMode::kPreviousSolution) {
        previous_primal_ = solution;
        previous_dual_ = backend_->GetDualSolution();
        has_previous_solution_ = true;
      }
      DumpRecord(/*solved=*/true, 0, solution);
      return solution;
    } catch (const QPSolverException& e) {
      last_solve_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
      last_iterations_ = stats.iterations;
      DumpRecord(/*solved=*/false, e.code(), Eigen::VectorXd::Zero(n_var_));
      throw;
    }
  }

  void SetWarmStartModeImpl(QPWarmStartMode mode) { warm_start_mode_ = mode; }

  void ResetWarmStartImpl() { has_previous_solution_ = false; }

  void DumpRecord(bool solved, int status_code, const Eigen::VectorXd& solution) {
    if (!dump_stream_) {
      return;
    }
    // Cap the per-instance dump size: a command stuck in a solve loop would
    // otherwise fill the disk at tens of MB/s and stall the whole system.
    const size_t record_bytes =
        4 + sizeof(double) * ((size_t)n_var_ * n_var_ + (size_t)n_const_ * n_var_ + 2 * n_const_ + 3 * n_var_) + 12 + 8;
    if (dump_bytes_written_ + record_bytes > dump_max_bytes_) {
      std::cerr << "[QPSolver] dump size cap reached (" << dump_max_bytes_ / (1024 * 1024)
                << " MB); further records dropped" << std::endl;
      dump_stream_.reset();
      return;
    }
    dump_bytes_written_ += record_bytes;
    std::ofstream& os = *dump_stream_;
    WriteInt32(os, kDumpRecordMarker);
    WriteMatrix(os, hessian_);
    WriteMatrix(os, gradient_);
    WriteMatrix(os, linearMatrix_);
    WriteMatrix(os, lowerBound_);
    WriteMatrix(os, upperBound_);
    WriteMatrix(os, primal_variable_for_warmstart_);
    WriteInt32(os, solved ? 1 : 0);
    WriteInt32(os, status_code);
    WriteInt32(os, last_iterations_);
    WriteDouble(os, last_solve_time_);
    WriteMatrix(os, solution);
    os.flush();
  }

  double GetLastSolveTimeImpl() const { return last_solve_time_; }

  int GetLastIterationsImpl() const { return last_iterations_; }

  Eigen::MatrixXd GetACostImpl() const {  // NOLINT
    return cost_capacity_ > 0 ? A_cost_.topRows(cost_rows_) : A_cost_;
  }

  Eigen::VectorXd GetBCostImpl() const {  // NOLINT
    return cost_capacity_ > 0 ? b_cost_.head(cost_rows_) : b_cost_;
  }

  Eigen::MatrixXd GetAConstImpl() const {  // NOLINT
    return linearMatrix_;
  }

  Eigen::VectorXd GetLowerBoundImpl() const {  // NOLINT
    return lowerBound_;
  }

  Eigen::VectorXd GetUpperBoundImpl() const {  // NOLINT
    return upperBound_;
  }

 private:
  std::unique_ptr<QPBackend> backend_;
  QPBackendType backend_type_{QPBackendType::kOSQP};

  int n_var_{};
  int n_const_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_cost_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> b_cost_;

  Eigen::MatrixXd hessian_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> gradient_;

  Eigen::MatrixXd linearMatrix_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> lowerBound_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> upperBound_;

  Eigen::Matrix<double, Eigen::Dynamic, 1> primal_variable_for_warmstart_;

  int cost_capacity_{0};
  int cost_rows_{0};

  QPWarmStartMode warm_start_mode_{QPWarmStartMode::kExternal};
  bool has_previous_solution_{false};
  Eigen::VectorXd previous_primal_;
  Eigen::VectorXd previous_dual_;

  double last_solve_time_{};
  int last_iterations_{-1};

  std::unique_ptr<std::ofstream> dump_stream_;
  size_t dump_bytes_written_{0};
  size_t dump_max_bytes_{kDumpDefaultMaxBytes};
};

QPSolver::QPSolver() {
  impl_ = std::make_unique<QPSolverImpl>();
}

QPSolver::~QPSolver() = default;

void QPSolver::Setup(int n_var, int n_const, double time_limit) {
  impl_->SetupImpl(n_var, n_const, time_limit);
}

void QPSolver::Setup(int n_var, int n_const, const QPSolverSettings& settings) {
  impl_->SetupImpl(n_var, n_const, settings);
}

void QPSolver::ReserveCostRows(int max_rows) {
  impl_->ReserveCostRowsImpl(max_rows);
}

void QPSolver::SetWarmStartMode(QPWarmStartMode mode) {
  impl_->SetWarmStartModeImpl(mode);
}

void QPSolver::SetBackend(QPBackendType type) {
  impl_->SetBackendImpl(type);
}

void QPSolver::ResetWarmStart() {
  impl_->ResetWarmStartImpl();
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
  return impl_->GetAConstImpl();
}

Eigen::VectorXd QPSolver::GetLowerBound() const {
  return impl_->GetLowerBoundImpl();
}

Eigen::VectorXd QPSolver::GetUpperBound() const {
  return impl_->GetUpperBoundImpl();
}

double QPSolver::GetLastSolveTime() const {
  return impl_->GetLastSolveTimeImpl();
}

int QPSolver::GetLastIterations() const {
  return impl_->GetLastIterationsImpl();
}

}  // namespace rb::math