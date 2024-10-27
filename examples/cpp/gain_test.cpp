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
  auto gains = robot->GetHeadPositionGain();
  if (gains.has_value()){
    auto[p_gain, i_gain, d_gain] = gains.value();
    std::cout<<"[head_0] p gain: "<< p_gain[0] << "i gain: "<< i_gain[0] << "d gain: "<< d_gain[0]<<std::endl;
    std::cout<<"[head_1] p gain: "<< p_gain[1] << "i gain: "<< i_gain[1] << "d gain: "<< d_gain[1]<<std::endl;
  }
  
  std::string joint_name = "head_0";
  robot->SetPositionGain(joint_name, 100, 0, 0);

  return 0;
}
