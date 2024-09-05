#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/math/optimal_control.h"

using namespace rb;
using namespace rb::dyn;
using namespace rb::math;

// 0 1 2 3 4 5 -> body
// 6 7 8 9 10 11 12 -> right arm
// 13 14 15 16 17 18 19 -> left arm

std::vector<Eigen::Matrix<double, 22, 1>> readCSV(const std::string& filename) {

  std::vector<Eigen::Matrix<double, 22, 1>> ret;

  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "can't open file: " << filename << std::endl;
    return ret;
  }

  std::string line;

  while (std::getline(file, line)) {
    Eigen::Matrix<double, 22, 1> q_joint;
    q_joint.setZero();
    std::stringstream ss(line);
    std::string cell;
    std::vector<std::string> row;

    while (std::getline(ss, cell, ',')) {
      row.push_back(cell);
    }

    int idx = 0;
    for (const auto& value : row) {
      if (idx < 22) {
        q_joint(idx++) = std::stod(value);
      }
    }
    ret.push_back(q_joint);
  }

  file.close();

  return ret;
}

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

  std::vector<Eigen::Matrix<double, 22, 1>> temp_q_list;
  std::vector<Eigen::Matrix<double, 22, 1>> q_list;
  temp_q_list = readCSV("output.txt");

  std::cout << "number of q_list : " << temp_q_list.size() << std::endl;

  if (temp_q_list.size() == 0) {
    //there is no converged solution
    return 0;
  }

  for (auto const& q_joint : temp_q_list) {
    Eigen::Matrix<double, 22, 1> q_joint_sorted;
    q_joint_sorted.setZero();
    q_joint_sorted.block(0, 0, 20, 1) = q_joint.block(2, 0, 20, 1);
    q_list.push_back(q_joint_sorted);
  }

  Eigen::Matrix<double, 22, 1> q_comp;
  q_comp.setZero();

  Eigen::Matrix<double, 4, 4> T_cur, T_target;

  Eigen::Matrix<double, 4, 4> T_ref;
  T_ref.setIdentity();
  T_ref.block(0, 3, 3, 1) << 0, 0.44, 0;
  // T_ref.block(0, 0, 3, 3) = SO3::RotZ(-3.141592);

  int n_iter = 100;

  for (int iter = 0; iter < n_iter; iter++) {
    std::vector<Eigen::Matrix<double, 6, 1>> err_vector;
    std::vector<Eigen::Matrix<double, 6, 22>> J_vector;

    double pos_err = 0;
    double ori_err = 0;

    for (auto& q_joint : q_list) {

      Eigen::Matrix<double, 22, 1> q_temp = q_joint + q_comp;
      // q_temp(6+6) += 3.141502/2.;
      // q_temp(6+6+7) -= 3.141502/2.;

      state->SetQ(q_temp);
      robot->ComputeForwardKinematics(state);

      Eigen::Matrix<double, 4, 4> T_right_cur = robot->ComputeTransformation(state, 0, LinkName::RH);
      Eigen::Matrix<double, 4, 4> T_left_cur = robot->ComputeTransformation(state, 0, LinkName::LH);
      Eigen::Matrix<double, 4, 4> T_err = T_right_cur.inverse() * T_left_cur;
      Eigen::Matrix<double, 6, 1> err_right_to_left = SE3::Log((T_err.inverse() * T_ref));
      Eigen::Matrix<double, 6, 22> J_right_to_left = robot->ComputeBodyJacobian(state, LinkName::RH, LinkName::LH);

      // std::cout << "q : " << q_temp.transpose() * 180 / 3.141592 << std::endl;
      // std::cout << "T_right_cur\n" << T_right_cur << std::endl;
      // std::cout << "T_left_cur\n" << T_left_cur << std::endl;
      // std::cout << "T_err\n" << T_err << std::endl;
      // return 0;

      ori_err += err_right_to_left.topRows(3).norm() / q_list.size();
      pos_err += err_right_to_left.bottomRows(3).norm() / q_list.size();
      // pos_err += (T_err.inverse() * T_ref).block(0, 3, 3, 1).norm() / q_list.size();

      err_vector.push_back(err_right_to_left);
      J_vector.push_back(J_right_to_left);
    }

    if (iter == 0 || iter == n_iter - 1) {
      std::cout << "iter : " << iter << std::endl;
      std::cout << "ori_err [deg] : " << ori_err * 180 / 3.141592 << std::endl;
      std::cout << "pos_err [mm]   : " << pos_err * 1000 << std::endl;
    }

    ori_err = 0;
    pos_err = 0;

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

  std::cout << "right q_comp [deg]: " << -q_comp.block(6, 0, 7, 1).transpose() * 180 / 3.141592 << std::endl;
  std::cout << "left q_comp [deg]: " << -q_comp.block(6 + 7, 0, 7, 1).transpose() * 180 / 3.141592 << std::endl;

  return 0;
}
