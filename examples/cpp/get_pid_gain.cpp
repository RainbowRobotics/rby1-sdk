#include <chrono>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  auto robot = Robot<y1_model::A>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }

  std::this_thread::sleep_for(1s);
  if (!robot->IsPowerOn(".*")) {
    robot->PowerOn(".*");
    std::this_thread::sleep_for(1s);
  }

  std::cout << std::endl << " >>> Using Component Name" << std::endl;
  auto gain_list = robot->GetTorsoPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "[torso_" << i << "] p gain: " << gain_list[i].p_gain << ", i gain: " << gain_list[i].i_gain
              << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetRightArmPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "[right_arm_" << i << "] p gain: " << gain_list[i].p_gain << ", i gain: " << gain_list[i].i_gain
              << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetLeftArmPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "[left_arm_" << i << "] p gain: " << gain_list[i].p_gain << ", i gain: " << gain_list[i].i_gain
              << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetHeadPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "[head_" << i << "] p gain: " << gain_list[i].p_gain << ", i gain: " << gain_list[i].i_gain
              << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  std::cout << std::endl << " >>> Using Joint Name" << std::endl;
  auto taregt_joint_name = "torso_0";
  auto gain = robot->GetPositionPIDGain(taregt_joint_name);
  std::cout << "[" << taregt_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  taregt_joint_name = "right_arm_0";
  gain = robot->GetPositionPIDGain(taregt_joint_name);
  std::cout << "[" << taregt_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  taregt_joint_name = "left_arm_0";
  gain = robot->GetPositionPIDGain(taregt_joint_name);
  std::cout << "[" << taregt_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  taregt_joint_name = "head_0";
  gain = robot->GetPositionPIDGain(taregt_joint_name);
  std::cout << "[" << taregt_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  return 0;
}
