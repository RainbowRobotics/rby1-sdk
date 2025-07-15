#include <chrono>
#include <iostream>
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

  std::cout << ">>> Before" << std::endl;
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

  // Torso joints
  robot->SetPositionPIDGain("torso_0", 100, 20, 900);
  robot->SetPositionPIDGain("torso_1", 1000, 38, 900);
  robot->SetPositionPIDGain("torso_2", 1000, 38, 900);
  robot->SetPositionPIDGain("torso_3", 220, 40, 400);
  robot->SetPositionPIDGain("torso_4", 50, 20, 400);
  robot->SetPositionPIDGain("torso_5", 220, 40, 400);

  // Right arm joints
  robot->SetPositionPIDGain("right_arm_0", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_1", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_2", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_3", 35, 5, 80);
  robot->SetPositionPIDGain("right_arm_4", 30, 5, 70);
  robot->SetPositionPIDGain("right_arm_5", 30, 5, 70);
  robot->SetPositionPIDGain("right_arm_6", 100, 5, 120);

  // Left arm joints
  robot->SetPositionPIDGain("left_arm_0", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_1", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_2", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_3", 35, 5, 80);
  robot->SetPositionPIDGain("left_arm_4", 30, 5, 70);
  robot->SetPositionPIDGain("left_arm_5", 30, 5, 70);
  robot->SetPositionPIDGain("left_arm_6", 100, 5, 150);

  // Head joints
  robot->SetPositionPIDGain("head_0", 800, 0, 4000);
  robot->SetPositionPIDGain("head_1", 800, 0, 4000);

  // Ensure PID Gain update compleation
  std::this_thread::sleep_for(50ms);

  std::cout << ">>> After" << std::endl;
  gain_list = robot->GetTorsoPositionPIDGains();
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
  return 0;
}
