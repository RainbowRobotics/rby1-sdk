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

  auto target_joint_name = "right_arm_3";
  std::cout << ">>> Before" << std::endl;
  auto gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  robot->SetPositionPGain(target_joint_name, 100);
  std::cout << ">>> After" << std::endl;
  // Ensure PID Gain update compleation
  std::this_thread::sleep_for(50ms);
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  target_joint_name = "left_arm_3";
  std::cout << ">>> Before" << std::endl;
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  robot->SetPositionPIDGain(target_joint_name, 60, 10, 100);
  std::cout << ">>> After" << std::endl;
  // Ensure PID Gain update compleation
  std::this_thread::sleep_for(50ms);
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  target_joint_name = "head_0";
  std::cout << ">>> Before" << std::endl;
  // Ensure PID Gain update compleation
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  robot->SetPositionPIDGain(target_joint_name, 700, 0, 3500);
  std::cout << ">>> After" << std::endl;
  // Ensure PID Gain update compleation
  std::this_thread::sleep_for(50ms);
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  target_joint_name = "head_1";
  std::cout << ">>> Before" << std::endl;
  // Ensure PID Gain update compleation
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  robot->SetPositionPGain(target_joint_name, 300);
  std::cout << ">>> After" << std::endl;
  // Ensure PID Gain update compleation
  std::this_thread::sleep_for(50ms);
  gain = robot->GetPositionPIDGain(target_joint_name);
  std::cout << "[" << target_joint_name << "] p gain: " << gain.p_gain << ", i gain: " << gain.i_gain
            << ", d gain: " << gain.d_gain << std::endl
            << std::endl;

  return 0;
}
