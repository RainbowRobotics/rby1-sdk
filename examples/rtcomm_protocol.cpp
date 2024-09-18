#include <Eigen/Core>
#include <iostream>
#include "rby1-sdk/net/real_time_control_protocol.h"

int main() {
  std::array<unsigned char, 1024> packet{};

  constexpr size_t DOF = 22;

  std::cout << "Robot State" << std::endl;
  {
    double t = 10.8;
    Eigen::Vector<bool, DOF> is_ready;
    Eigen::Vector<double, DOF> pos, vel, cur, tq;
    is_ready.setConstant(false);
    is_ready << true, true, false, false, true;
    pos.setRandom();
    vel.setRandom();
    cur.setRandom();
    tq.setRandom();

    int len = rb::BuildRobotStateRTPacket(packet.data(), DOF, t, is_ready.data(), pos.data(), vel.data(), cur.data(),
                                          tq.data());

    std::cout << "len: " << len << std::endl;
    for (int i = 0; i < len; i++) {
      std::cout << std::hex << (int)packet[i] << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec;

    /////
    auto valid = rb::ValidateRTProtocol(packet.data(), len);
    std::cout << "Validation: " << std::boolalpha << valid.first << " " << valid.second << std::endl;

    size_t N;
    double new_t;
    Eigen::Vector<bool, DOF> new_is_ready;
    Eigen::Vector<double, DOF> new_pos, new_vel, new_cur, new_tq;
    rb::ParseRobotStateRTProtocol(packet.data(), &N, &new_t, new_is_ready.data(), new_pos.data(), new_vel.data(),
                                  new_cur.data(), new_tq.data());

    std::cout << "t: " << t << " / " << new_t << std::endl;
    std::cout << "is_ready: " << std::endl
              << is_ready.transpose() << std::endl
              << new_is_ready.transpose() << std::endl;
    std::cout << "position: " << std::endl << pos.transpose() << std::endl << new_pos.transpose() << std::endl;
    std::cout << "velocity: " << std::endl << vel.transpose() << std::endl << new_vel.transpose() << std::endl;
    std::cout << "current: " << std::endl << cur.transpose() << std::endl << new_cur.transpose() << std::endl;
    std::cout << "torque: " << std::endl << tq.transpose() << std::endl << new_tq.transpose() << std::endl;
  }
  std::cout << std::endl;

  /////
  std::cout << "Robot Command" << std::endl;
  {
    Eigen::Vector<bool, DOF> mode;
    Eigen::Vector<double, DOF> target, feedforward_torque;
    Eigen::Vector<unsigned int, DOF> feedback_gain;
    bool finished = false;
    mode.setRandom();
    target.setRandom();
    feedback_gain.setRandom();
    feedforward_torque.setRandom();

    int len = rb::BuildRobotCommandRTPacket(packet.data(), DOF, mode.data(), target.data(), feedback_gain.data(),
                                            feedforward_torque.data(), finished);
    std::cout << "len: " << len << std::endl;
    for (int i = 0; i < len; i++) {
      std::cout << std::hex << (int)packet[i] << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec;

    auto valid = rb::ValidateRTProtocol(packet.data(), len);
    std::cout << "Validation: " << std::boolalpha << valid.first << " " << valid.second << std::endl;

    size_t new_N;
    Eigen::Vector<bool, DOF> new_mode;
    Eigen::Vector<double, DOF> new_target, new_feedforward_torque;
    Eigen::Vector<unsigned int, DOF> new_feedback_gain;
    bool new_finished;
    rb::ParseRobotCommandRTProtocol(packet.data(), &new_N, new_mode.data(), new_target.data(),
                                    new_feedback_gain.data(), new_feedforward_torque.data(), &new_finished);

    std::cout << "N: " << DOF << " / " << new_N << std::endl;
    std::cout << "mode: " << std::endl << mode.transpose() << std::endl << new_mode.transpose() << std::endl;
    std::cout << "target: " << std::endl << target.transpose() << std::endl << new_target.transpose() << std::endl;
    std::cout << "feedback_gain: " << std::endl
              << feedback_gain.transpose() << std::endl
              << new_feedback_gain.transpose() << std::endl;
    std::cout << "feedforward_gain: " << std::endl
              << feedforward_torque.transpose() << std::endl
              << new_feedforward_torque.transpose() << std::endl;
    std::cout << "finished: " << std::boolalpha << finished << " / " << new_finished << std::endl;
  }
}