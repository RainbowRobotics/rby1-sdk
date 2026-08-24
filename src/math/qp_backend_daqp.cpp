extern "C" {
#include "api.h"
#include "constants.h"
}

#include <algorithm>
#include <vector>

#include "qp_backend.h"

namespace rb::math {

namespace {

// Exception codes 300+ are the DAQP range.
constexpr int kDaqpCodeBase = 300;

int MapExitFlag(int exitflag) {
  switch (exitflag) {
    case DAQP_EXIT_SOFT_OPTIMAL:
      return kDaqpCodeBase + 1;
    case DAQP_EXIT_INFEASIBLE:
      return kDaqpCodeBase + 2;
    case DAQP_EXIT_CYCLE:
      return kDaqpCodeBase + 3;
    case DAQP_EXIT_UNBOUNDED:
      return kDaqpCodeBase + 4;
    case DAQP_EXIT_ITERLIMIT:
      return kDaqpCodeBase + 5;
    case DAQP_EXIT_NONCONVEX:
      return kDaqpCodeBase + 6;
    case DAQP_EXIT_OVERDETERMINED_INITIAL:
      return kDaqpCodeBase + 7;
    default:
      return kDaqpCodeBase + 9;
  }
}

// DAQP dense active-set backend. Problem form:
//   min 1/2 x'Hx + f'x  s.t.  blower <= [x; Ax] <= bupper  (ms simple bounds)
// We map lb <= Ax <= ub with ms = 0 (all rows general constraints). DAQP's
// proximal-point outer loop tolerates semidefinite H. daqp_quadprog performs
// setup+solve each call (cold); active-set warm start via a persistent
// workspace can be added later if DAQP is selected as default.
class DaqpBackend final : public QPBackend {
 public:
  void Setup(int n_var, int n_const, const QPSolverSettings& settings, bool /*legacy_tolerances*/) override {
    n_var_ = n_var;
    n_const_ = n_const;
    daqp_default_settings(&settings_);
    settings_.iter_limit = settings.max_iter;
    // DAQP primal tolerance; keep its default dual tolerance.
    settings_.primal_tol = settings.eps_abs;
    sense_.assign(n_const_, 0);
    x_.resize(n_var_);
    lam_.resize(n_const_);
  }

  void SetProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A,
                  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) override {
    H_ = H;
    g_ = g;
    // DAQP expects row-major A.
    A_rowmajor_ = A;
    lb_ = lb;
    ub_ = ub;
  }

  void WarmStart(const Eigen::VectorXd& x, const Eigen::VectorXd* /*y*/) override {
    // daqp_quadprog has no primal warm-start input; ignored.
    (void)x;
  }

  Eigen::VectorXd Solve(QPSolveStats& stats) override {
    DAQPProblem qp{};
    qp.n = n_var_;
    qp.m = n_const_;
    qp.ms = 0;
    qp.H = H_.data();
    qp.f = g_.data();
    qp.A = A_rowmajor_.data();
    qp.bupper = ub_.data();
    qp.blower = lb_.data();
    qp.sense = sense_.data();
    qp.break_points = nullptr;
    qp.nh = 0;
    qp.problem_type = 0;

    DAQPResult result;
    result.x = x_.data();
    result.lam = lam_.data();

    std::fill(sense_.begin(), sense_.end(), 0);
    daqp_quadprog(&result, &qp, &settings_);

    stats.iterations = result.iter;

    if (result.exitflag != DAQP_EXIT_OPTIMAL) {
      throw QPSolverException(MapExitFlag(result.exitflag));
    }
    return x_;
  }

  Eigen::VectorXd GetDualSolution() override { return lam_; }

  void Reset() override {}

 private:
  int n_var_{};
  int n_const_{};

  DAQPSettings settings_{};
  std::vector<int> sense_;

  Eigen::MatrixXd H_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rowmajor_;
  Eigen::VectorXd g_, lb_, ub_;
  Eigen::VectorXd x_, lam_;
};

}  // namespace

std::unique_ptr<QPBackend> MakeDaqpBackend() {
  return std::make_unique<DaqpBackend>();
}

}  // namespace rb::math
