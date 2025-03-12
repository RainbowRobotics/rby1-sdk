#include <chrono>
#include <iostream>

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/math/optimal_control.h"

using namespace rb;
using namespace rb::dyn;
using namespace rb::math;

// 0 1 2 3 4 5 -> body
// 6 7 8 9 10 11 12 -> right arm
// 13 14 15 16 17 18 19 -> left arm

enum LinkName {
  BASE = 0,

  AR,
  AP,
  KN,
  HP,
  HR,
  HY,

  RSP,
  RSR,
  RSY,
  REB,
  RWP,
  RWY,
  RH,

  LSP,
  LSR,
  LSY,
  LEB,
  LWP,
  LWY,
  LH,

};

int main() {
  auto robot = std::make_shared<Robot<22>>(LoadRobotFromURDF(PATH "/sample_model.urdf", "base"));
  auto state =
      robot->MakeState<std::vector<std::string>, std::vector<std::string>>({"base",
                        "Link_1_Anckle_Roll",
                        "Link_2_Anckle_Pitch",
                        "Link_3_Knee_Pitch",
                        "Link_4_Hip_Pitch",
                        "Link_5_Hip_Roll",
                        "Link_6_Hip_Yaw",
                        "Link_7_Right_Shoulder_Pitch",
                        "Link_8_Right_Shoulder_Roll",
                        "Link_9_Right_Shoulder_Yaw",
                        "Link_10_Right_Elbow_Pitch",
                        "Link_11_Right_Wrist_Yaw1",
                        "Link_12_Right_Wrist_Pitch",
                        "Link_13_Right_ToolFlange",
                        "Link_14_Left_Shoulder_Pitch",
                        "Link_15_Left_Shoulder_Roll",
                        "Link_16_Left_Shoulder_Yaw",
                        "Link_17_Left_Elbow_Pitch",
                        "Link_18_Left_Wrist_Yaw1",
                        "Link_19_Left_Wrist_Pitch",
                        "Link_20_Left_ToolFlange",
                        "wheel_r",
                        "wheel_l",
                        "ee_right",
                        "ee_finger_r1",
                        "ee_finger_r2",
                        "ee_left",
                        "ee_finger_l1",
                        "ee_finger_l2"},
                       {"torso_1",     "torso_2",     "torso_3",     "torso_4",
                        "torso_5",     "torso_6",     "right_arm_1", "right_arm_2", "right_arm_3", "right_arm_4",
                        "right_arm_5", "right_arm_6", "right_arm_7", "left_arm_1",  "left_arm_2",  "left_arm_3",
                        "left_arm_4",  "left_arm_5",  "left_arm_6",  "left_arm_7"});

  Eigen::Matrix<double, 22, 1> q;
  q.setZero();

  for (int i = 0; i < 6; i++) {
    q(i) = 0.01;
  }

  state->SetQ(q);

  robot->ComputeForwardKinematics(state);

  double control_period = 1. / 500.;
  OptimalControl<22> optimal_control(robot, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19});

  OptimalControl<22>::Input input;

  for (int i = 0; i < 20; i++) {
    q(i) = 0;
  }

  q(1) = -0.1;
  q(2) = 0.2;
  q(3) = -0.1;

  q(6 + 3) = -3.141592 / 2.;
  q(6 + 7 + 3) = -3.141592 / 2.;

  state->SetQ(q);

  robot->ComputeForwardKinematics(state);

  std::vector<OptimalControl<22>::LinkTarget> link_targets;
  OptimalControl<22>::COMTarget com_target;
  OptimalControl<22>::JointAngleTarget q_target(22);

  //make link target reference
  {
    OptimalControl<22>::LinkTarget link_target;
    Eigen::Matrix<double, 4, 4> T;
    T.setIdentity();
    link_target.ref_link_index = LinkName::BASE;
    link_target.link_index = LinkName::RH;
    link_target.weight_position = 200*control_period;
    link_target.weight_orientation = 1*control_period;
    link_target.T = robot->ComputeTransformation(state, LinkName::BASE, LinkName::RH);
    link_targets.push_back(link_target);

    link_target.ref_link_index = LinkName::BASE;
    link_target.link_index = LinkName::LH;
    link_target.weight_position = 200*control_period;
    link_target.weight_orientation = 1*control_period;
    link_target.T = robot->ComputeTransformation(state, LinkName::BASE, LinkName::LH);
    link_targets.push_back(link_target);

    link_target.ref_link_index = LinkName::BASE;
    link_target.link_index = LinkName::HY;
    link_target.weight_position = 0*control_period;
    link_target.weight_orientation = 1*control_period;
    link_target.T = robot->ComputeTransformation(state, 0, LinkName::HY);
    link_targets.push_back(link_target);

    input.link_targets = link_targets;
  }

  //make com reference
  {
    com_target.ref_link_index = LinkName::BASE;
    com_target.com.setZero();
    //    com_target.com(1) = 0.1;
    com_target.com(2) = 0.45;
    com_target.weight = 1*control_period;

    input.com_target = com_target;
  }

  //make q_joint reference
  {
    q_target.q.resize(robot->GetDOF());
    q_target.q.setZero();

    q_target.q(0) = 10 * 3.14159265 / 180;

    q_target.weight.resize(robot->GetDOF());
    q_target.weight.setZero();
    q_target.weight(0) = 10*control_period;

    input.q_target = q_target;
  }

  for (int i = 0; i < 20; i++) {
    q(i) += 0.2;
  }
  state->SetQ(q);
  robot->ComputeForwardKinematics(state);

  {
    std::cout << "--- before optimization ---\n";

    std::cout << "T_RH : \n" << robot->ComputeTransformation(state, LinkName::BASE, LinkName::RH) << std::endl;
    std::cout << "T_LH: \n" << robot->ComputeTransformation(state, LinkName::BASE, LinkName::LH) << std::endl;
    std::cout << "T_HY : \n" << robot->ComputeTransformation(state, LinkName::BASE, LinkName::HY) << std::endl;

    Eigen::Matrix<double, 3, 1> com;
    Eigen::Matrix<double, 3, 22> J_com;

    com = robot->ComputeCenterOfMass(state, 0);

    std::cout << "com [m]: " << com.transpose() << std::endl;
    std::cout << "q [deg] : " << q.transpose() * 180 / 3.141592 << std::endl;
  }

  auto start = std::chrono::steady_clock::now();

  int N = 200000;
  for (int i = 0; i < N; i++) {
    robot->ComputeForwardKinematics(state);
    auto ret = optimal_control.Solve(input, state, control_period, 1, robot->GetLimitQUpper(state), robot->GetLimitQdotUpper(state));
    if (ret.has_value()) {
      //      std::cout << ret.value().transpose() << std::endl;
      state->SetQdot(ret.value());
      state->SetQ(state->GetQ() + ret.value() * control_period);
    } else {
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! can't solve" << std::endl;
      break;
    }
  }

  std::cout << "Duration: "
            << (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start)
                       .count() /
                   1.e6 / N
            << " ms" << std::endl;

  robot->ComputeForwardKinematics(state);

  Eigen::Matrix<double, 4, 4> T_cur, T_target;

  if (1) {

    std::cout << "--- target optimization ---\n";

    std::cout << "T_RH : \n" << link_targets[0].T << std::endl;
    std::cout << "T_LH: \n" << link_targets[1].T << std::endl;
    std::cout << "T_HY : \n" << link_targets[2].T << std::endl;

    std::cout << "com : " << com_target.com.transpose() << std::endl;
    std::cout << "q [deg] : " << q_target.q.transpose() * 180 / 3.141592 << std::endl;

    std::cout << "--- result optimization ---\n";

    T_cur = robot->ComputeTransformation(state, 0, LinkName::RH);
    std::cout << "T_RH \n" << T_cur << std::endl;

    T_cur = robot->ComputeTransformation(state, 0, LinkName::LH);
    std::cout << "T_LH \n" << T_cur << std::endl;

    T_cur = robot->ComputeTransformation(state, 0, LinkName::HY);
    std::cout << "T_HY \n" << T_cur << std::endl;

    Eigen::Matrix<double, 3, 1> com;
    Eigen::Matrix<double, 3, 22> J_com;

    com = robot->ComputeCenterOfMass(state, 0);

    std::cout << "com : " << com.transpose() << std::endl;
    std::cout << "q [deg] : " << state->GetQ().eval().transpose() * 180 / 3.141592 << std::endl;
  }

  // Eigen::MatrixXd temp = robot->ComputeCenterOfWholeMass(state, 0);

  return 0;
}