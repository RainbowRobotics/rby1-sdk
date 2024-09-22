#include <iostream>

#include "rby1-sdk/math/constants.h"
#include "rby1-sdk/math/velocity_estimator.h"

rb::VelocityEstimator velocity_estimator(0, 0, 0.1, 50, 0.1);

int main() {
  double dt = 0.01;

  for (double t = 0; t < rb::math::kPi * 2; t += dt) {
    double position = std::sin(t);

    velocity_estimator.Predict(dt);
    velocity_estimator.Update(position);

    std::cout << "measured: " << position << ", estimated velocity: " << velocity_estimator.GetVelocity()
              << ", true velocity: " << std::cos(t) << std::endl;
  }

  return 0;
}