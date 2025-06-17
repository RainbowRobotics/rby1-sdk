#include <chrono>
#include <iomanip>
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

  robot->StartStateUpdate(
      [](const auto& state, const auto& control_manager) {
        std::cout << "Timestamp: " << state.timestamp.tv_sec << ".";
        std::cout << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
        std::cout << "  Position: " << state.position.transpose() << std::endl;
        std::cout << "Control Manager State: " << rb::to_string(control_manager.state) << std::endl;
        for (std::size_t i = 0; i < state.joint_states.size(); ++i) {
          auto& js = state.joint_states[i];
          std::cout << "Index: " << i << " Temperature: " << js.temperature << std::endl;
        }
      },
      100);

  std::this_thread::sleep_for(1s);

  robot->PowerOn(".*");

  while (true)
    ;

  return 0;
}