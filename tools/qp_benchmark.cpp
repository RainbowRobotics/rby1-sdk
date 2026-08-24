// Offline QP benchmark harness.
//
// Replays QP instance logs dumped by rb::math::QPSolver (env RBY1_QP_DUMP_DIR)
// and reports timing/accuracy statistics. Used to (a) establish the OSQP
// baseline and (b) compare solver backends on identical, controller-generated
// problem sequences.
//
// Usage: qp_benchmark <dump.qplog> [more.qplog ...]
//
// Modes per file:
//   warm  - one solver instance, matrix updates + warm start per record
//           (mimics the 500Hz controller loop, incl. re-init on nnz change)
//   cold  - fresh solver per record, zero warm start
//
// File format (native endian, written by qp_solver.cpp):
//   header: "RBQPDMP1" | int32 n_var | int32 n_const
//   record: int32 marker 0x52454331
//           H (n_var*n_var doubles, col-major) | g (n_var)
//           A (n_const*n_var, col-major) | lb (n_const) | ub (n_const)
//           x_warm (n_var)
//           int32 solved | int32 status | int32 iters | double solve_time
//           x_solution (n_var)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Sparse"
#include "OsqpEigen/OsqpEigen.h"
#include "qp_backend.h"

namespace {

constexpr char kMagic[8] = {'R', 'B', 'Q', 'P', 'D', 'M', 'P', '1'};
constexpr int32_t kRecordMarker = 0x52454331;

// Solver configuration under test. Defaults reproduce QPSolver::Setup as of
// Stage 1 (OSQP defaults, 2ms time limit, warm start from measured qdot).
struct Config {
  double time_limit = 2e-3;
  double eps_abs = 1e-3;   // OSQP default
  double eps_rel = 1e-3;   // OSQP default
  int max_iter = 4000;     // OSQP default
  int check_termination = 25;
  enum class WarmMode { kDump, kPrev, kZero } warm_mode = WarmMode::kDump;
};

struct Record {
  Eigen::MatrixXd H;
  Eigen::VectorXd g;
  Eigen::MatrixXd A;
  Eigen::VectorXd lb, ub;
  Eigen::VectorXd x_warm;
  bool solved{};
  int32_t status{};
  int32_t iters{};
  double solve_time{};
  Eigen::VectorXd x_solution;
};

struct Dump {
  int32_t n_var{};
  int32_t n_const{};
  std::vector<Record> records;
};

bool ReadRaw(std::ifstream& is, void* data, size_t size) {
  return static_cast<bool>(is.read(static_cast<char*>(data), (std::streamsize)size));
}

bool ReadMatrix(std::ifstream& is, Eigen::MatrixXd& m, Eigen::Index rows, Eigen::Index cols) {
  m.resize(rows, cols);
  return ReadRaw(is, m.data(), sizeof(double) * (size_t)m.size());
}

bool ReadVector(std::ifstream& is, Eigen::VectorXd& v, Eigen::Index size) {
  v.resize(size);
  return ReadRaw(is, v.data(), sizeof(double) * (size_t)v.size());
}

bool LoadDump(const std::string& path, Dump& dump) {
  std::ifstream is(path, std::ios::binary);
  if (!is.good()) {
    std::cerr << "cannot open: " << path << "\n";
    return false;
  }
  char magic[8];
  if (!ReadRaw(is, magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(magic)) != 0) {
    std::cerr << "bad magic: " << path << "\n";
    return false;
  }
  if (!ReadRaw(is, &dump.n_var, 4) || !ReadRaw(is, &dump.n_const, 4)) {
    return false;
  }
  while (true) {
    int32_t marker;
    if (!ReadRaw(is, &marker, 4)) {
      break;  // EOF
    }
    if (marker != kRecordMarker) {
      std::cerr << "bad record marker in " << path << " after " << dump.records.size() << " records\n";
      return false;
    }
    Record r;
    bool ok = ReadMatrix(is, r.H, dump.n_var, dump.n_var) && ReadVector(is, r.g, dump.n_var) &&
              ReadMatrix(is, r.A, dump.n_const, dump.n_var) && ReadVector(is, r.lb, dump.n_const) &&
              ReadVector(is, r.ub, dump.n_const) && ReadVector(is, r.x_warm, dump.n_var);
    int32_t solved;
    ok = ok && ReadRaw(is, &solved, 4) && ReadRaw(is, &r.status, 4) && ReadRaw(is, &r.iters, 4) &&
         ReadRaw(is, &r.solve_time, 8) && ReadVector(is, r.x_solution, dump.n_var);
    if (!ok) {
      std::cerr << "truncated record in " << path << " after " << dump.records.size() << " records\n";
      return false;
    }
    r.solved = solved != 0;
    dump.records.push_back(std::move(r));
  }
  return true;
}

struct SolveResult {
  bool ok{};
  double wall_time{};
  int iters{-1};
  Eigen::VectorXd x;
};

struct Stats {
  std::vector<double> times;
  std::vector<double> iters;
  double max_constraint_violation{};
  double max_diff_vs_dump{};
  int failures{};
  int n{};

  static double Percentile(std::vector<double> v, double p) {
    if (v.empty()) {
      return 0;
    }
    std::sort(v.begin(), v.end());
    const auto idx = (size_t)std::min((double)v.size() - 1, std::ceil(p / 100.0 * (double)v.size()) - 1);
    return v[std::max((size_t)0, idx)];
  }

  void Add(const Record& r, const SolveResult& s) {
    n++;
    if (!s.ok) {
      failures++;
      return;
    }
    times.push_back(s.wall_time);
    if (s.iters >= 0) {
      iters.push_back(s.iters);
    }
    const Eigen::VectorXd ax = r.A * s.x;
    const double viol = std::max((r.lb - ax).maxCoeff(), (ax - r.ub).maxCoeff());
    max_constraint_violation = std::max(max_constraint_violation, viol);
    if (r.solved) {
      max_diff_vs_dump = std::max(max_diff_vs_dump, (s.x - r.x_solution).lpNorm<Eigen::Infinity>());
    }
  }

  void Print(const std::string& label) const {
    std::cout << "  [" << label << "] n=" << n << " fail=" << failures                        //
              << " t_p50=" << Percentile(times, 50) * 1e6 << "us"                             //
              << " t_p99=" << Percentile(times, 99) * 1e6 << "us"                             //
              << " t_max=" << (times.empty() ? 0 : *std::max_element(times.begin(), times.end())) * 1e6 << "us"  //
              << " iter_p50=" << Percentile(iters, 50)                                        //
              << " iter_max=" << (iters.empty() ? 0 : *std::max_element(iters.begin(), iters.end()))  //
              << " viol_max=" << max_constraint_violation                                     //
              << " dx_vs_dump=" << max_diff_vs_dump << "\n";
  }
};

class OsqpReplay {
 public:
  OsqpReplay(int n_var, int n_const, const Config& cfg) : n_var_(n_var), n_const_(n_const), cfg_(cfg) {
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setTimeLimit(cfg.time_limit);
    solver_.settings()->setAbsoluteTolerance(cfg.eps_abs);
    solver_.settings()->setRelativeTolerance(cfg.eps_rel);
    solver_.settings()->setMaxIteration(cfg.max_iter);
    solver_.settings()->setCheckTermination(cfg.check_termination);
    solver_.data()->setNumberOfVariables(n_var);
    solver_.data()->setNumberOfConstraints(n_const);
  }

  SolveResult Solve(const Record& r) {
    SolveResult out;
    Eigen::SparseMatrix<double> h = r.H.sparseView();
    Eigen::SparseMatrix<double> a = r.A.sparseView();
    Eigen::VectorXd g = r.g;
    Eigen::VectorXd lb = r.lb;
    Eigen::VectorXd ub = r.ub;

    bool need_init = first_;
    if (!first_ && n_hessian_element_ == h.nonZeros()) {
      if (!solver_.updateHessianMatrix(h) || !solver_.updateGradient(g) ||
          !solver_.updateLinearConstraintsMatrix(a) || !solver_.updateBounds(lb, ub)) {
        need_init = true;
      }
    } else {
      need_init = true;
    }
    if (need_init) {
      solver_.data()->clearHessianMatrix();
      solver_.data()->clearLinearConstraintsMatrix();
      solver_.clearSolver();
      n_hessian_element_ = h.nonZeros();
      if (!solver_.data()->setHessianMatrix(h) || !solver_.data()->setGradient(g) ||
          !solver_.data()->setLinearConstraintsMatrix(a) || !solver_.data()->setLowerBound(lb) ||
          !solver_.data()->setUpperBound(ub) || !solver_.initSolver()) {
        return out;
      }
      first_ = false;
    }
    Eigen::VectorXd x_warm;
    switch (cfg_.warm_mode) {
      case Config::WarmMode::kDump:
        x_warm = r.x_warm;
        break;
      case Config::WarmMode::kPrev:
        x_warm = have_prev_ ? prev_solution_ : r.x_warm;
        break;
      case Config::WarmMode::kZero:
        x_warm = Eigen::VectorXd::Zero(n_var_);
        break;
    }
    solver_.setPrimalVariable(x_warm);
    if (cfg_.warm_mode == Config::WarmMode::kPrev && have_prev_dual_) {
      Eigen::VectorXd dual = prev_dual_;
      solver_.setDualVariable(dual);
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto rv = solver_.solveProblem();
    out.wall_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (rv != OsqpEigen::ErrorExitFlag::NoError || solver_.getStatus() != OsqpEigen::Status::Solved) {
      return out;
    }
#ifdef OSQP_EIGEN_OSQP_IS_V1
    out.iters = (int)solver_.solver()->info->iter;
#else
    out.iters = (int)solver_.workspace()->info->iter;
#endif
    out.x = solver_.getSolution();
    if (cfg_.warm_mode == Config::WarmMode::kPrev) {
      prev_solution_ = out.x;
      have_prev_ = true;
      prev_dual_ = solver_.getDualSolution();
      have_prev_dual_ = prev_dual_.size() == n_const_;
    }
    out.ok = true;
    return out;
  }

 private:
  OsqpEigen::Solver solver_;
  int n_var_, n_const_;
  Config cfg_;
  Eigen::Index n_hessian_element_{-1};
  bool first_{true};
  Eigen::VectorXd prev_solution_, prev_dual_;
  bool have_prev_{false}, have_prev_dual_{false};
};

}  // namespace

// Report statistics of the solutions recorded IN the dump (server-side
// behavior), without re-solving.
void PrintDumpStats(const Dump& dump, const std::string& file) {
  Stats stats;
  for (const auto& r : dump.records) {
    SolveResult s;
    s.ok = r.solved;
    s.wall_time = r.solve_time;
    s.iters = r.iters;
    s.x = r.x_solution;
    stats.Add(r, s);
  }
  std::cout << file << ": n_var=" << dump.n_var << " n_const=" << dump.n_const
            << " records=" << dump.records.size() << "\n";
  stats.Print("dump/recorded");
}

std::unique_ptr<rb::math::QPBackend> MakeBackendByName(const std::string& name) {
  if (name == "osqp") {
    return rb::math::MakeOsqpBackend();
  }
#ifdef RBY1_WITH_PROXQP
  if (name == "proxqp") {
    return rb::math::MakeProxqpBackend();
  }
#endif
#ifdef RBY1_WITH_DAQP
  if (name == "daqp") {
    return rb::math::MakeDaqpBackend();
  }
#endif
  return nullptr;
}

// Generic replay through a QPBackend (the exact code path the controller
// uses). Timing includes SetProblem (sparsification etc. for OSQP) so
// backends are compared fairly.
int ReplayEngine(const std::vector<std::string>& files, const Config& cfg, const std::string& engine, bool run_cold) {
  rb::math::QPSolverSettings settings;
  settings.time_limit = cfg.time_limit;
  settings.eps_abs = cfg.eps_abs;
  settings.eps_rel = cfg.eps_rel;
  settings.max_iter = cfg.max_iter;
  settings.check_termination = cfg.check_termination;

  for (const auto& file : files) {
    Dump dump;
    if (!LoadDump(file, dump)) {
      return 1;
    }
    std::cout << file << ": n_var=" << dump.n_var << " n_const=" << dump.n_const
              << " records=" << dump.records.size() << "\n";
    if (dump.records.empty()) {
      continue;
    }

    auto run = [&](bool warm_sequential) {
      Stats stats;
      std::unique_ptr<rb::math::QPBackend> backend;
      Eigen::VectorXd prev_x, prev_y;
      bool have_prev = false;
      for (const auto& r : dump.records) {
        if (!backend || !warm_sequential) {
          backend = MakeBackendByName(engine);
          backend->Setup(dump.n_var, dump.n_const, settings, false);
          have_prev = false;
        }
        SolveResult s;
        const auto t0 = std::chrono::steady_clock::now();
        try {
          backend->SetProblem(r.H, r.g, r.A, r.lb, r.ub);
          if (cfg.warm_mode == Config::WarmMode::kPrev && have_prev) {
            backend->WarmStart(prev_x, prev_y.size() == dump.n_const ? &prev_y : nullptr);
          } else if (cfg.warm_mode != Config::WarmMode::kZero) {
            Eigen::VectorXd w = r.x_warm;
            backend->WarmStart(w, nullptr);
          } else {
            Eigen::VectorXd w = Eigen::VectorXd::Zero(dump.n_var);
            backend->WarmStart(w, nullptr);
          }
          rb::math::QPSolveStats st;
          s.x = backend->Solve(st);
          s.iters = st.iterations;
          s.ok = true;
          if (cfg.warm_mode == Config::WarmMode::kPrev) {
            prev_x = s.x;
            prev_y = backend->GetDualSolution();
            have_prev = true;
          }
        } catch (const rb::math::QPSolverException&) {
          s.ok = false;
        }
        s.wall_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        stats.Add(r, s);
      }
      stats.Print(engine + (warm_sequential ? "/warm" : "/cold"));
    };

    run(true);
    if (run_cold) {
      run(false);
    }
  }
  return 0;
}

// A/B equivalence check for the Stage-3 refactor: identical recorded problems
// through (a) the legacy inline OSQP sequence (OsqpReplay, a faithful replica
// of the pre-refactor QPSolverImpl::SolveImpl) and (b) the new QPBackend path
// (rb::math::MakeOsqpBackend). Expects bit-identical solutions.
int AbBackends(const std::vector<std::string>& files, const Config& cfg) {
  rb::math::QPSolverSettings settings;
  settings.time_limit = cfg.time_limit;
  settings.eps_abs = cfg.eps_abs;
  settings.eps_rel = cfg.eps_rel;
  settings.max_iter = cfg.max_iter;
  settings.check_termination = cfg.check_termination;

  for (const auto& file : files) {
    Dump dump;
    if (!LoadDump(file, dump)) {
      return 1;
    }
    OsqpReplay inline_engine(dump.n_var, dump.n_const, cfg);
    auto backend = rb::math::MakeOsqpBackend();
    backend->Setup(dump.n_var, dump.n_const, settings, /*legacy_tolerances=*/false);

    double d_max = 0;
    int mismatch = 0, n_ok = 0;
    for (const auto& r : dump.records) {
      const SolveResult a = inline_engine.Solve(r);
      bool b_ok = true;
      Eigen::VectorXd xb;
      try {
        backend->SetProblem(r.H, r.g, r.A, r.lb, r.ub);
        Eigen::VectorXd warm = r.x_warm;
        backend->WarmStart(warm, nullptr);
        rb::math::QPSolveStats stats;
        xb = backend->Solve(stats);
      } catch (const rb::math::QPSolverException&) {
        b_ok = false;
      }
      if (a.ok != b_ok) {
        mismatch++;
        continue;
      }
      if (a.ok) {
        n_ok++;
        d_max = std::max(d_max, (a.x - xb).lpNorm<Eigen::Infinity>());
      }
    }
    std::cout << file << ": records=" << dump.records.size() << " ok=" << n_ok << " status_mismatch=" << mismatch
              << " d_solution_max=" << d_max << (mismatch == 0 && d_max == 0 ? "  [IDENTICAL]" : "") << "\n";
  }
  return 0;
}

// Record-by-record comparison of two dumps (e.g. before/after a refactor on
// the same deterministic sim scenario). Ignores timing; reports max diffs of
// problem data and solutions.
int CompareDumps(const std::string& file_a, const std::string& file_b) {
  Dump a, b;
  if (!LoadDump(file_a, a) || !LoadDump(file_b, b)) {
    return 1;
  }
  if (a.n_var != b.n_var || a.n_const != b.n_const) {
    std::cout << "DIFFERENT problem sizes: " << a.n_var << "x" << a.n_const << " vs " << b.n_var << "x" << b.n_const
              << "\n";
    return 1;
  }
  const size_t n = std::min(a.records.size(), b.records.size());
  double d_problem = 0, d_solution = 0, d_warm = 0;
  int status_mismatch = 0;
  for (size_t i = 0; i < n; i++) {
    const auto& ra = a.records[i];
    const auto& rb = b.records[i];
    d_problem = std::max({d_problem, (ra.H - rb.H).cwiseAbs().maxCoeff(), (ra.g - rb.g).cwiseAbs().maxCoeff(),
                          (ra.A - rb.A).cwiseAbs().maxCoeff(), (ra.lb - rb.lb).cwiseAbs().maxCoeff(),
                          (ra.ub - rb.ub).cwiseAbs().maxCoeff()});
    d_warm = std::max(d_warm, (ra.x_warm - rb.x_warm).lpNorm<Eigen::Infinity>());
    d_solution = std::max(d_solution, (ra.x_solution - rb.x_solution).lpNorm<Eigen::Infinity>());
    if (ra.solved != rb.solved) {
      status_mismatch++;
    }
  }
  std::cout << file_a << " vs " << file_b << ": records " << a.records.size() << "/" << b.records.size()
            << " compared=" << n << "\n  d_problem_max=" << d_problem << " d_warm_max=" << d_warm
            << " d_solution_max=" << d_solution << " status_mismatch=" << status_mismatch << "\n";
  return 0;
}

int main(int argc, char** argv) {
  Config cfg;
  bool run_cold = true;
  bool dump_stats_only = false;
  bool compare_mode = false;
  bool ab_backends_mode = false;
  std::string engine;
  std::string csv_path;
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    auto value = [&](const char* name) -> std::string {
      const auto pos = arg.find('=');
      if (pos == std::string::npos) {
        std::cerr << name << " requires =VALUE\n";
        exit(1);
      }
      return arg.substr(pos + 1);
    };
    if (arg.rfind("--time-limit=", 0) == 0) {
      cfg.time_limit = std::stod(value("--time-limit"));
    } else if (arg.rfind("--eps-abs=", 0) == 0) {
      cfg.eps_abs = std::stod(value("--eps-abs"));
    } else if (arg.rfind("--eps-rel=", 0) == 0) {
      cfg.eps_rel = std::stod(value("--eps-rel"));
    } else if (arg.rfind("--max-iter=", 0) == 0) {
      cfg.max_iter = std::stoi(value("--max-iter"));
    } else if (arg.rfind("--check-termination=", 0) == 0) {
      cfg.check_termination = std::stoi(value("--check-termination"));
    } else if (arg.rfind("--warm-mode=", 0) == 0) {
      const std::string m = value("--warm-mode");
      if (m == "dump") {
        cfg.warm_mode = Config::WarmMode::kDump;
      } else if (m == "prev") {
        cfg.warm_mode = Config::WarmMode::kPrev;
      } else if (m == "zero") {
        cfg.warm_mode = Config::WarmMode::kZero;
      } else {
        std::cerr << "unknown warm mode: " << m << "\n";
        return 1;
      }
    } else if (arg == "--no-cold") {
      run_cold = false;
    } else if (arg.rfind("--csv=", 0) == 0) {
      csv_path = value("--csv");
    } else if (arg == "--dump-stats") {
      dump_stats_only = true;
    } else if (arg == "--compare") {
      compare_mode = true;
    } else if (arg == "--ab-backends") {
      ab_backends_mode = true;
    } else if (arg.rfind("--engine=", 0) == 0) {
      engine = value("--engine");
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "unknown option: " << arg << "\n";
      return 1;
    } else {
      files.push_back(arg);
    }
  }
  if (compare_mode) {
    if (files.size() != 2) {
      std::cerr << "--compare requires exactly two files\n";
      return 1;
    }
    return CompareDumps(files[0], files[1]);
  }
  if (ab_backends_mode) {
    return AbBackends(files, cfg);
  }
  if (!engine.empty()) {
    if (!MakeBackendByName(engine)) {
      std::cerr << "unknown or not-compiled engine: " << engine << "\n";
      return 1;
    }
    std::cout << "config: engine=" << engine << " eps_abs=" << cfg.eps_abs << " eps_rel=" << cfg.eps_rel
              << " max_iter=" << cfg.max_iter << " warm_mode="
              << (cfg.warm_mode == Config::WarmMode::kDump   ? "dump"
                  : cfg.warm_mode == Config::WarmMode::kPrev ? "prev"
                                                             : "zero")
              << "\n";
    return ReplayEngine(files, cfg, engine, run_cold);
  }
  if (files.empty()) {
    std::cerr << "usage: qp_benchmark [--time-limit=S] [--eps-abs=E] [--eps-rel=E] [--max-iter=N]\n"
                 "                    [--check-termination=N] [--warm-mode=dump|prev|zero] [--no-cold]\n"
                 "                    [--dump-stats] [--compare a.qplog b.qplog]\n"
                 "                    <dump.qplog> [more.qplog ...]\n";
    return 1;
  }
  std::cout << "config: time_limit=" << cfg.time_limit << " eps_abs=" << cfg.eps_abs << " eps_rel=" << cfg.eps_rel
            << " max_iter=" << cfg.max_iter << " check_termination=" << cfg.check_termination
            << " warm_mode=" << (cfg.warm_mode == Config::WarmMode::kDump ? "dump"
                                 : cfg.warm_mode == Config::WarmMode::kPrev ? "prev" : "zero")
            << "\n";

  for (const auto& file : files) {
    Dump dump;
    if (!LoadDump(file, dump)) {
      return 1;
    }
    if (dump_stats_only) {
      PrintDumpStats(dump, file);
      continue;
    }
    std::cout << file << ": n_var=" << dump.n_var << " n_const=" << dump.n_const
              << " records=" << dump.records.size() << "\n";
    if (dump.records.empty()) {
      continue;
    }

    {
      OsqpReplay replay(dump.n_var, dump.n_const, cfg);
      Stats stats;
      std::ofstream csv;
      if (!csv_path.empty()) {
        csv.open(csv_path);
        csv << "idx,ok,iters,wall_us\n";
      }
      int idx = 0;
      for (const auto& r : dump.records) {
        const auto s = replay.Solve(r);
        if (csv.is_open()) {
          csv << idx++ << "," << (s.ok ? 1 : 0) << "," << s.iters << "," << s.wall_time * 1e6 << "\n";
        }
        stats.Add(r, s);
      }
      stats.Print("osqp/warm");
    }
    if (run_cold) {
      Stats stats;
      for (const auto& r : dump.records) {
        OsqpReplay replay(dump.n_var, dump.n_const, cfg);
        stats.Add(r, replay.Solve(r));
      }
      stats.Print("osqp/cold");
    }
  }
  return 0;
}
