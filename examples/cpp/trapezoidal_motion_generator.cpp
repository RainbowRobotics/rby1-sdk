#include "rby1-sdk/math/trapezoidal_motion_generator.h"
#include <chrono>
#include <iostream>

int main() {
  rb::TrapezoidalMotionGenerator<-1> tmg;
  rb::TrapezoidalMotionGenerator<-1>::Input input;

  input.current_position = Eigen::Vector2d{5, -8};
  input.current_velocity = Eigen::Vector2d{-2, 0};
  input.target_position = Eigen::Vector2d{0, 4};
  input.velocity_limit = Eigen::Vector2d{1, 3};
  input.acceleration_limit = Eigen::Vector2d{0.5, 2};
  input.minimum_time = 10;

  double step = 0.01;

  // Offline
  tmg.Update(input);
  for (double t = step; t < 10; t += step) {
    const auto& [p, v, _] = tmg(t);
    std::cout << t << " " << p.transpose() << " " << v.transpose() << std::endl;

    if (tmg.IsReached(t))
      break;
  }

  // Oneline
  //  auto start = std::chrono::steady_clock::now();
  //  double t = 0;
  //  int count = 0;
  //  while (true) {
  //    count++;
  //    tmg.Update(input);
  //    const auto& [p, v] = tmg(step);
  //
  //    t += step;
  //    std::cout << t << " " << p.transpose() << " " << v.transpose() << std::endl;
  //    input.current_position = p;
  //    input.current_velocity = v;
  //    input.minimum_time -= step;
  //
  //    if (tmg.IsReached(step))
  //      break;
  //  }
  //  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count() /
  //                   1.e9 / count
  //            << std::endl;

  return 0;
}