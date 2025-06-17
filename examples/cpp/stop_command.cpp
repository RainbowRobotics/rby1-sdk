#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

#define D2R 0.017453
#define R2D 57.296

const std::string kAll = ".*";

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

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(WholeBodyCommandBuilder().SetCommand(
                                  StopCommandBuilder().SetCommandHeader(CommandHeaderBuilder()))),
                              99)
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    return 1;
  }
  robot->PowerOff(kAll);

  return 0;
}
