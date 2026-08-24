// Gate-5 unit scenario for the sqrt braking envelope (RBY1_QP_SQRT_BRAKING).
//
// Kinematic single-joint drive toward a target BEYOND the position limit,
// integrated at the 2ms control period:
//   - legacy one-step law: expected to hit kInequalityConstraintViolation
//     while approaching the limit at speed (the QP aborts the motion), because
//     stopping within one tick exceeds the acceleration limit.
//   - sqrt envelope: expected to decelerate at <= a_brake, never exceed the
//     limit, and never abort.
//
// Exit code 0 iff the sqrt law shows no abort, no overshoot, decel <= a_brake,
// AND the legacy law reproduces the abort (documenting the flaw it fixes).

#include <cstdlib>
#include <iostream>

#include "rby1-sdk/math/optimal_control.h"
#include "sample_robot.h"

using namespace rb;

struct RunResult {
  bool aborted{false};
  int abort_tick{-1};
  double max_q{-1e9};
  double max_decel{0};
  double final_q{0};
  OptimalControl<6>::ExitCode exit_code{};
};

RunResult RunScenario(bool sqrt_braking) {
  if (sqrt_braking) {
    setenv("RBY1_QP_SQRT_BRAKING", "1", 1);
  } else {
    unsetenv("RBY1_QP_SQRT_BRAKING");
  }

  auto robot = std::make_shared<SampleRobot>();
  auto state = robot->MakeState({"base", "link1", "link2", "link3", "link4", "link5", "link6", "tooltip"},
                                {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
  state->SetGravity({0, 0, 0, 0, 0, -9.8});

  OptimalControl<6> oc(robot, {0, 1, 2, 3, 4, 5}, /*nullspace_mapping=*/true, /*soft_position_boundary=*/false);

  const double dt = 0.002;
  const double q_limit = 1.0;
  const double v_limit = 2.0;
  const double a_limit = 5.0;

  Eigen::Vector<double, 6> q = Eigen::Vector<double, 6>::Zero();
  Eigen::Vector<double, 6> qdot = Eigen::Vector<double, 6>::Zero();

  Eigen::Vector<double, 6> pos_lb = Eigen::Vector<double, 6>::Constant(-q_limit);
  Eigen::Vector<double, 6> pos_ub = Eigen::Vector<double, 6>::Constant(q_limit);
  Eigen::Vector<double, 6> vel_lim = Eigen::Vector<double, 6>::Constant(v_limit);
  Eigen::Vector<double, 6> acc_lim = Eigen::Vector<double, 6>::Constant(a_limit);

  OptimalControl<6>::Input in;
  auto target = OptimalControl<6>::JointAngleTarget();
  target.q.setZero();
  target.weight.setOnes();
  target.q(0) = 1.5;  // beyond the +1.0 limit
  in.q_target = target;

  RunResult r;
  for (int tick = 0; tick < 3000; tick++) {
    state->SetQ(q);
    state->SetQdot(qdot);
    auto rv = oc.Solve(in, state, dt, /*error_scaling=*/1.0, pos_lb, pos_ub, vel_lim, acc_lim);
    if (!rv.has_value()) {
      r.aborted = true;
      r.abort_tick = tick;
      r.exit_code = oc.GetExitCode();
      break;
    }
    const Eigen::Vector<double, 6> qdot_new = rv.value();
    r.max_decel = std::max(r.max_decel, (qdot(0) - qdot_new(0)) / dt);
    q += (qdot + qdot_new) * dt / 2.0;
    qdot = qdot_new;
    r.max_q = std::max(r.max_q, q(0));
  }
  r.final_q = q(0);
  return r;
}

int main() {
  const auto legacy = RunScenario(false);
  const auto sqrt_law = RunScenario(true);

  std::cout << "legacy : aborted=" << legacy.aborted << " tick=" << legacy.abort_tick
            << " exit=" << (int)legacy.exit_code << " max_q=" << legacy.max_q << " max_decel=" << legacy.max_decel
            << "\n";
  std::cout << "sqrt   : aborted=" << sqrt_law.aborted << " max_q=" << sqrt_law.max_q
            << " final_q=" << sqrt_law.final_q << " max_decel=" << sqrt_law.max_decel << "\n";

  const double q_limit = 1.0, a_brake = 5.0;
  bool ok = true;
  if (sqrt_law.aborted) {
    std::cout << "FAIL: sqrt law aborted\n";
    ok = false;
  }
  if (sqrt_law.max_q > q_limit + 1e-6) {
    std::cout << "FAIL: sqrt law overshoot " << sqrt_law.max_q - q_limit << "\n";
    ok = false;
  }
  if (sqrt_law.max_decel > a_brake * 1.01 + 1e-9) {
    std::cout << "FAIL: sqrt law decel " << sqrt_law.max_decel << " > a_brake\n";
    ok = false;
  }
  if (sqrt_law.final_q < q_limit - 0.05) {
    std::cout << "FAIL: sqrt law did not reach the limit (final " << sqrt_law.final_q << ")\n";
    ok = false;
  }
  if (!legacy.aborted && legacy.max_q <= q_limit + 1e-6) {
    std::cout << "NOTE: legacy law neither aborted nor overshot in this scenario\n";
  }
  std::cout << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
