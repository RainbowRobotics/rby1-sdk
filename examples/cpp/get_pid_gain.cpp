#include <iomanip>
#include <iostream>
#include <thread>
#include <optional>
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
  robot->PowerOn(".*");

  //HEAD TEST
  auto gain_list = robot->GetHeadPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "[head_" << i << "] p gain: " << gain_list[i].p_gain << "i gain: " << gain_list[i].i_gain
              << "d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetRightArmPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "[right_arm_" << i << "] p gain: " << gain_list[i].p_gain << "i gain: " << gain_list[i].i_gain
              << "d gain: " << gain_list[i].d_gain << std::endl;
  }

  auto gain = robot->GetPositionPIDGain("left_arm_0");
  std::cout << "[left_arm_0] p gain: " << gain.p_gain << "i gain: " << gain.i_gain
          << "d gain: " << gain.d_gain << std::endl;

  return 0;
}
