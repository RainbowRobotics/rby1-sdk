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

  robot->ResetOdometry(1.54, {0, 0});

  while (true)
    ;

  return 0;
}