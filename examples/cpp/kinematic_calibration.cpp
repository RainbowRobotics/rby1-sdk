#include <chrono>
#include <iostream>
#include <random>
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
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>(
      {"base",
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
      {"torso_1",     "torso_2",     "torso_3",     "torso_4",     "torso_5",     "torso_6",     "right_arm_1",
       "right_arm_2", "right_arm_3", "right_arm_4", "right_arm_5", "right_arm_6", "right_arm_7", "left_arm_1",
       "left_arm_2",  "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6",  "left_arm_7"});

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

  // std::cout << "right : \n" << robot->ComputeTransformation(state, LinkName::BASE, LinkName::RH) << std::endl;
  // std::cout << "left : \n" << robot->ComputeTransformation(state, LinkName::BASE, LinkName::LH) << std::endl;

  std::vector<OptimalControl<22>::LinkTarget> link_targets;
  OptimalControl<22>::COMTarget com_target;
  OptimalControl<22>::JointAngleTarget q_target(22);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-1.0, 1.0);

  std::vector<Eigen::Matrix<double, 22, 1>> q_list;

  int n_pos = 9999;

  for (int i = 0; i < n_pos; i++) {

    if (input.link_targets.has_value()) {
      input.link_targets.value().clear();
    }
    link_targets.clear();

    Eigen::Matrix<double, 4, 4> T_target;
    T_target.setIdentity();

    T_target.block(0, 0, 3, 3) << 0, 0, -1, 0, 1, 0, 1, 0, 0;

    T_target(0, 3) = 0.25 + dis(gen) * 0.3;
    T_target(1, 3) = 0 + dis(gen) * 0.3;
    T_target(2, 3) = 1.15 + dis(gen) * 0.3;

    T_target.block(0, 0, 3, 3) = T_target.block(0, 0, 3, 3) * SO3::RotZ(dis(gen) * 3.141592 / 180. * 30.) *
                                 SO3::RotY(dis(gen) * 3.141592 / 180. * 30.) *
                                 SO3::RotX(dis(gen) * 3.141592 / 180. * 30.);

    std::cout << "[" << i << "] : T_target\n" << T_target << std::endl;

    {
      OptimalControl<22>::LinkTarget link_target;
      Eigen::Matrix<double, 4, 4> T;
      T.setIdentity();
      link_target.ref_link_index = LinkName::BASE;
      link_target.link_index = LinkName::RH;
      link_target.weight_position = 1000 * control_period;
      link_target.weight_orientation = 10 * control_period;
      link_target.T.setIdentity();
      link_target.T = T_target;
      link_target.T.block(0, 3, 3, 1) -= link_target.T.block(0, 1, 3, 1) * 0.2;

      link_targets.push_back(link_target);

      link_target.ref_link_index = LinkName::BASE;
      link_target.link_index = LinkName::LH;
      link_target.weight_position = 1000 * control_period;
      link_target.weight_orientation = 10 * control_period;
      link_target.T.setIdentity();
      link_target.T = T_target;
      link_target.T.block(0, 3, 3, 1) += link_target.T.block(0, 1, 3, 1) * 0.2;

      link_targets.push_back(link_target);

      link_target.ref_link_index = LinkName::BASE;
      link_target.link_index = LinkName::HY;
      link_target.weight_position = 0 * control_period;
      link_target.weight_orientation = 10 * control_period;
      link_target.T = Eigen::MatrixXd::Identity(4, 4);
      link_targets.push_back(link_target);

      input.link_targets = link_targets;
    }

    state->SetQ(q);
    robot->ComputeForwardKinematics(state);

    int N = 1000;
    Eigen::MatrixXd q_joint_opt;
    for (int i = 0; i < N; i++) {
      robot->ComputeForwardKinematics(state);
      auto ret = optimal_control.Solve(input, state, control_period, 1, robot->GetLimitQUpper(state), robot->GetLimitQdotUpper(state));

      if (ret.has_value()) {
        auto err = optimal_control.GetError();
        state->SetQ(state->GetQ() + ret.value() * control_period);

        if (err < 1e-5) {
          q_joint_opt = state->GetQ();
          q_list.push_back(q_joint_opt);
          break;
        }
      } else {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! can't solve" << std::endl;
        break;
      }
    }

    if (q_list.size() >= 50) {
      break;
    }

    robot->ComputeForwardKinematics(state);
  }

  std::cout << "number of q_list : " << q_list.size() << std::endl;

  if (q_list.size() == 0) {
    //there is no converged solution
    return 0;
  }

  Eigen::Matrix<double, 22, 1> q_delta;
  q_delta.setZero();
  q_delta.block(6, 0, 14, 1).setConstant(0.1);  // 0.5deg

  Eigen::Matrix<double, 22, 1> q_comp;
  q_comp.setZero();

  Eigen::Matrix<double, 4, 4> T_cur, T_target;

  Eigen::Matrix<double, 4, 4> T_ref;
  T_ref.setIdentity();
  T_ref.block(0, 3, 3, 1) << 0, 0.4, 0;

  std::vector<Eigen::Matrix<double, 6, 1>> err_vector;
  std::vector<Eigen::Matrix<double, 6, 22>> J_vector;

  int n_iter = 100;

  for (int iter; iter < n_iter; iter++) {
    err_vector.clear();
    J_vector.clear();

    for (auto& q_joint : q_list) {

      Eigen::Matrix<double, 22, 1> q_temp = q_joint + q_delta + q_comp;
      state->SetQ(q_temp);
      robot->ComputeForwardKinematics(state);

      Eigen::Matrix<double, 4, 4> T_right_cur = robot->ComputeTransformation(state, 0, LinkName::RH);
      Eigen::Matrix<double, 4, 4> T_left_cur = robot->ComputeTransformation(state, 0, LinkName::LH);
      Eigen::Matrix<double, 4, 4> T_err = T_right_cur.inverse() * T_left_cur;
      Eigen::Matrix<double, 6, 1> err_right_to_left = SE3::Log((T_err.inverse() * T_ref));
      Eigen::Matrix<double, 6, 22> J_right_to_left = robot->ComputeBodyJacobian(state, LinkName::RH, LinkName::LH);

      err_vector.push_back(err_right_to_left);
      J_vector.push_back(J_right_to_left);
    }

    Eigen::MatrixXd J_total;
    Eigen::MatrixXd err_total;
    J_total.resize(6 * J_vector.size(), 22);
    J_total.setZero();
    err_total.resize(6 * err_vector.size(), 1);
    err_total.setZero();

    for (int i = 0; i < J_vector.size(); i++) {
      J_total.block(6 * i, 0, 6, 22) = J_vector[i];
      err_total.block(6 * i, 0, 6, 1) = err_vector[i];
    }

    Eigen::Matrix<double, 22, 1> q_update = J_total.completeOrthogonalDecomposition().solve(err_total);

    q_comp += q_update;
  }

  std::cout << "q_comp: " << -q_comp.block(6, 0, 14, 1).transpose() << std::endl;
  std::cout << "q_delta: " << q_delta.block(6, 0, 14, 1).transpose() << std::endl;
  std::cout << "error: " << (q_delta + q_comp).block(6, 0, 14, 1).transpose() << std::endl;
  std::cout << std::endl;

  return 0;
}
