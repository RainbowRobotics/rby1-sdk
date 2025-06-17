#include <chrono>
#include <fstream>
#include <iostream>
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

  std::vector<Eigen::Matrix<double, y1_model::A::kRobotDOF, 1>> q_joint_vector;
  char userInput;

  while (true) {
    Eigen::Matrix<double, y1_model::A::kRobotDOF, 1> q_joint;

    std::cout << "Matrix added. Press 'q' to quit and save, or any other key to add another matrix: ";
    std::cin >> userInput;

    if (userInput == 'q' || userInput == 'Q') {
      break;
    } else {
      auto robot_state = robot->GetState();
      q_joint = robot_state.position;
      q_joint_vector.push_back(q_joint);
    }

    // Clear the newline character left in the input buffer
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // 파일에 출력
  std::ofstream file("output.txt");

  if (!file.is_open()) {
    std::cerr << "can't open file!" << std::endl;
    return 1;
  }

  for (const auto& matrix : q_joint_vector) {
    // std::cout << "matrix : " << matrix.transpose() << std::endl;
    // file << "Matrix:\n";
    for (int i = 0; i < matrix.rows(); ++i) {
      file << matrix(i) << ", ";
    }
    file << "\n";
  }

  file.close();
  std::cout << "finish!" << std::endl;

  return 0;
}