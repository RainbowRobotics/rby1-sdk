#include <chrono>
#include <iostream>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

const std::string kAll = "^((?!wheel|right|left|torso|head).)*";

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};

  auto robot = Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn(kAll)) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn(kAll)) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  robot->StartStateUpdate(
      [](const auto& state) {
        std::cout << "right ft sensor" << std::endl;
        std::cout << "time stamp : "
                  << (state.timestamp.tv_nsec - state.tool_flange_right.time_since_last_update.tv_nsec) << std::endl;
        std::cout << "gyro : " << state.tool_flange_right.gyro.transpose() << std::endl;
        std::cout << "acceleration : " << state.tool_flange_right.acceleration.transpose() << std::endl;
        std::cout << "switch_A : " << state.tool_flange_right.switch_A << std::endl;

        std::cout << "left ft sensor" << std::endl;
        std::cout << "time stamp : "
                  << (state.timestamp.tv_nsec - state.tool_flange_left.time_since_last_update.tv_nsec) << std::endl;
        std::cout << "gyro : " << state.tool_flange_left.gyro.transpose() << std::endl;
        std::cout << "acceleration : " << state.tool_flange_left.acceleration.transpose() << std::endl;
        std::cout << "switch_A : " << state.tool_flange_left.switch_A << std::endl;
      },
      10);

  std::this_thread::sleep_for(10s);

  return 0;
}