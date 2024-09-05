#include "rby1-sdk/dynamics/robot.h"
#include "sample_robot.h"

using namespace rb::math;
using namespace rb::dyn;

int main() {
  auto robot = std::make_shared<SampleRobot>();
  auto state = robot->MakeState({"base", "link1", "link2", "link3", "link4", "link5", "link6", "tooltip"},
                                {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

  state->SetGravity({0, 0, 0, 0, 0, -9.8});
  state->SetQ(Eigen::Vector<double, 6>{1, 2, 3, 4, 5, 6});
  state->SetQdot(Eigen::Vector<double, 6>{6, 5, 4, 3, 2, 1});
  state->SetQddot(Eigen::Vector<double, 6>{10, 9, 8, 7, 6, 5});

  robot->ComputeForwardKinematics(state);
  for (int i = 0; i < 8; i++) {
    std::cout << i << " :: " << std::endl;
    std::cout << robot->ComputeTransformation(state, 2, i) << std::endl;
  }

  std::cout << "Body Jacobian" << std::endl;
  std::cout << robot->ComputeBodyJacobian(state, 0, 6) << std::endl;

  robot->ComputeDiffForwardKinematics(state);
  std::cout << "Body Velocity" << std::endl;
  std::cout << robot->ComputeBodyVelocity(state, 0, 7).transpose() << std::endl;
  std::cout << (robot->ComputeBodyJacobian(state, 0, 7) * state->GetQdot()).transpose() << std::endl;

  robot->Compute2ndDiffForwardKinematics(state);
  robot->ComputeInverseDynamics(state);
  std::cout << "Inverse Dynamics" << std::endl;
  std::cout << state->GetTau().transpose() << std::endl;

  return 0;
}