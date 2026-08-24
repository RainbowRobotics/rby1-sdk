// Unit scenario for the runtime arm distance damper (RBY1_ARM_DAMPER).
//
// Kinematic whole-body drive into a deep chest bow via a LINK (cartesian)
// target, integrated at the 2ms control period, arms hanging at q = 0:
//   - damper OFF: the hanging forearms sweep toward the (augmented) base
//     rails / pelvis as the torso folds.
//   - damper ON : approach speed is capped by the distance rows; the minimum
//     arm-vs-structure distance must never fall below (d_safe - eps).
//
// Joint-name-free by design: the SDK's MakeState joint-name labels do not
// define the q ordering for URDF-loaded robots (labels vs internal order
// mismatch), so the test avoids name->index mapping entirely - the cost is a
// chest pose target resolved by LINK name, limits come from the robot in its
// own internal ordering, and results are read back through FK.
//
// Exit code 0 iff the ON run respects the floor and never aborts.

#include <cstdlib>
#include <iostream>

#include "rby1-sdk/math/optimal_control.h"

using namespace rb;

struct RunResult {
  double min_arm_dist{1e9};
  double chest_drop{0};  // [m] how far the chest came down (fold depth proxy)
  int aborts{0};
};

RunResult RunScenario(const char* urdf, bool damper) {
  setenv("RBY1_QP_NO_SLACK", "1", 1);
  setenv("RBY1_QP_SQRT_BRAKING", "1", 1);
  if (damper) {
    setenv("RBY1_ARM_DAMPER", "1", 1);
  } else {
    unsetenv("RBY1_ARM_DAMPER");
  }

  auto config = dyn::LoadRobotFromURDF(urdf, "base");
  auto robot = std::make_shared<dyn::Robot<-1>>(config);
  auto state = robot->MakeState(robot->GetLinkNames(), robot->GetJointNames());
  const int dof = robot->GetDOF();

  int chest_link = -1;
  {
    const auto links = state->GetLinkNames();
    for (int i = 0; i < (int)links.size(); i++) {
      if (links[i] == "link_torso_5") {
      chest_link = i;
      }
    }
  }
  if (chest_link < 0) {
    std::cerr << "link_torso_5 not found\n";
    std::exit(2);
  }

  // All joints participate; limits in the robot's own (internal) ordering.
  std::vector<unsigned int> all_joints(dof);
  for (int i = 0; i < dof; i++) {
    all_joints[i] = (unsigned int)i;
  }
  OptimalControl<-1> oc(robot, all_joints, /*nullspace_mapping=*/true, /*soft_position_boundary=*/false);

  const double dt = 0.002;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(dof);
  const Eigen::VectorXd pos_lb = robot->GetLimitQLower(state);
  const Eigen::VectorXd pos_ub = robot->GetLimitQUpper(state);
  const Eigen::VectorXd vel_lim = Eigen::VectorXd::Constant(dof, 2.0);
  const Eigen::VectorXd acc_lim = Eigen::VectorXd::Constant(dof, 5.0);

  // Chest target: deep forward bow - down 0.45m and 0.25m forward of the
  // home chest pose, pitched 100 deg forward. Deliberately extreme so the
  // fold is driven until limits/dampers stop it.
  state->SetQ(q);
  robot->ComputeForwardKinematics(state);
  const Eigen::Matrix4d T_home = robot->ComputeTransformation(state, 0, chest_link);
  Eigen::Matrix4d T_goal = Eigen::Matrix4d::Identity();
  const double pitch = 100.0 * M_PI / 180.0;
  T_goal.block<3, 3>(0, 0) = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  T_goal.block<3, 1>(0, 3) = T_home.block<3, 1>(0, 3) + Eigen::Vector3d(0.25, 0.0, -0.45);

  typename OptimalControl<-1>::Input in;
  in.link_targets = std::vector<typename OptimalControl<-1>::LinkTarget>();
  in.link_targets->push_back({0, chest_link, T_goal, 1.0, 1.0});

  RunResult r;
  for (int tick = 0; tick < 4000; tick++) {
    state->SetQ(q);
    state->SetQdot(qdot);
    auto rv = oc.Solve(in, state, dt, 1.0, pos_lb, pos_ub, vel_lim, acc_lim, /*need_forward_kinematics=*/true);
    if (!rv.has_value()) {
      r.aborts++;
      std::cerr << "abort @tick " << tick << " exit=" << (int)oc.GetExitCode() << " msg=" << oc.GetExitCodeMessage()
                << "\n";
      break;
    }
    const Eigen::VectorXd qdot_new = rv.value();
    q += (qdot + qdot_new) * dt / 2.0;
    qdot = qdot_new;

    state->SetQ(q);
    robot->ComputeForwardKinematics(state);
    auto results = robot->DetectCollisionsOrNearestLinks(state, 20);
    for (const auto& c : results) {
      const bool arm = c.link1.find("_arm_") != std::string::npos || c.link2.find("_arm_") != std::string::npos;
      if (arm && c.distance < r.min_arm_dist) {
        r.min_arm_dist = c.distance;
        if (std::getenv("RBY1_ARM_DAMPER_DEBUG") != nullptr) {
          std::cerr << "[min] tick=" << tick << " " << c.link1 << "<->" << c.link2 << " d=" << c.distance << "\n";
        }
      }
    }
  }
  state->SetQ(q);
  robot->ComputeForwardKinematics(state);
  const Eigen::Matrix4d T_end = robot->ComputeTransformation(state, 0, chest_link);
  r.chest_drop = T_home(2, 3) - T_end(2, 3);
  return r;
}

int main(int argc, char** argv) {
  const char* urdf = argc > 1 ? argv[1] : "models/rby1m/urdf/model_v1.3.urdf";

  const auto off = RunScenario(urdf, false);
  const auto on = RunScenario(urdf, true);

  std::cout << "damper OFF: min_arm_dist=" << off.min_arm_dist << " chest_drop=" << off.chest_drop
            << " aborts=" << off.aborts << "\n";
  std::cout << "damper ON : min_arm_dist=" << on.min_arm_dist << " chest_drop=" << on.chest_drop
            << " aborts=" << on.aborts << "\n";

  bool ok = true;
  if (on.min_arm_dist < 0.02) {  // d_safe(0.03) with tolerance
    std::cout << "FAIL: damper ON still lets arms within " << on.min_arm_dist << " m\n";
    ok = false;
  }
  if (on.aborts > 0) {
    std::cout << "FAIL: damper ON aborted\n";
    ok = false;
  }
  if (on.chest_drop < 0.10) {
    std::cout << "FAIL: fold barely moved (chest_drop=" << on.chest_drop << ") - scenario invalid\n";
    ok = false;
  }
  // The OFF run measures on the UN-augmented model (no base rails): it
  // documents the baseline arm-vs-torso clearance only.
  std::cout << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
