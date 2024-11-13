#include "rby1-sdk/upc/master_arm.h"

#include <iostream>

#include <csignal>
#include <cstdlib>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

std::shared_ptr<Robot<y1_model::A>> robot;
auto master_arm = std::make_shared<upc::MasterArm>("/dev/rby1_master_arm");

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  robot = Robot<y1_model::A>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn("12v")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn("12v")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
    std::this_thread::sleep_for(1s);
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  try {
    // Latency timer setting
    upc::InitializeDevice("/dev/rby1_master_arm");
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  auto active_ids = master_arm->Initialize();
  if (active_ids.size() != upc::MasterArm::kDOF + 2) {
    return 1;
  }

  std::cout << ">>> Before" << std::endl;
  auto gain_list = master_arm->GetMasterRightArmPositionPIDGains();

  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "RIGHT [" << i << "]: " << gain_list[i].p_gain << ", " << gain_list[i].i_gain << ", "
              << gain_list[i].d_gain << std::endl;
  }

  gain_list = master_arm->GetMasterLeftArmPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "LEFT [" << i << "]: " << gain_list[i].p_gain << ", " << gain_list[i].i_gain << ", "
              << gain_list[i].d_gain << std::endl;
  }

  master_arm->SetPositionPIDGain(0, 800, 0, 6500);
  master_arm->SetPositionPIDGain(1, 800, 0, 6500);
  master_arm->SetPositionPIDGain(2, 800, 0, 6500);
  master_arm->SetPositionPIDGain(3, 800, 0, 4000);
  master_arm->SetPositionPIDGain(4, 800, 0, 4000);
  master_arm->SetPositionPIDGain(5, 800, 0, 4000);
  master_arm->SetPositionPIDGain(6, 800, 0, 4000);
  master_arm->SetPositionPIDGain(7, 800, 0, 6500);
  master_arm->SetPositionPIDGain(8, 800, 0, 6500);
  master_arm->SetPositionPIDGain(9, 800, 0, 6500);
  master_arm->SetPositionPIDGain(10, 800, 0, 4000);
  master_arm->SetPositionPIDGain(11, 800, 0, 4000);
  master_arm->SetPositionPIDGain(12, 800, 0, 4000);
  master_arm->SetPositionPIDGain(13, 800, 0, 4000);

  std::cout << ">>> After" << std::endl;

  gain_list = master_arm->GetMasterRightArmPositionPIDGains();

  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "RIGHT [" << i << "]: " << gain_list[i].p_gain << ", " << gain_list[i].i_gain << ", "
              << gain_list[i].d_gain << std::endl;
  }

  gain_list = master_arm->GetMasterLeftArmPositionPIDGains();
  for (auto i = 0; i < gain_list.size(); i++) {
    std::cout << "LEFT [" << i << "]: " << gain_list[i].p_gain << ", " << gain_list[i].i_gain << ", "
              << gain_list[i].d_gain << std::endl;
  }

  return 0;
}