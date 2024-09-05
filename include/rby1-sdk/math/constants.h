#pragma once

#include "Eigen/Core"

namespace rb::math {

constexpr double kPi = 3.1415926535897932384626433832795;                ///< \pi
constexpr double kPiHalf = 1.5707963267948966192313216916398;            ///< \frac{\pi}{2}
constexpr double kPiDouble = 6.283185307179586476925286766559;           ///< \pi*2
constexpr double kInvPiDouble = 0.15915494309189533576888376337251;      ///< \frac{1}{\pi*2}
constexpr double kPiDividedBySqrt2 = 2.2214414690791831235079404950303;  ///< \frac{\pi}{\sqrt{2}}
constexpr double kPiSquare = 9.8696044010893586188344909998762;          ///< \pi^2

constexpr double kRad2Deg = 57.295779513082320876798154814105;   ///< \frac{180}{\pi}
constexpr double kDeg2Rad = 0.01745329251994329576923690768489;  /// \frac{\pi}{180}

static const Eigen::Matrix<double, 1, 4> kAffineConstant = {0, 0, 0, 1};

static const double kDoubleEpsilon = std::numeric_limits<double>::epsilon();

/// Calculate sine and cosine simultaneously
inline void fsincos(double theta,   ///< Angle in radians
                    double& sine,   ///< Variable for storing a sine value
                    double& cosine  ///< Variable for storing a sine value
) {
  using std::sin;
  using std::sqrt;

  theta -= (int)(theta * kInvPiDouble) * kPiDouble;
  if (theta < 0)
    theta += kPiDouble;

  sine = sin(theta);
  if (theta < kPiHalf) {
    cosine = sqrt(1 - sine * sine);
    return;
  } else if (theta < kPi + kPiHalf) {
    cosine = -sqrt(1 - sine * sine);
    return;
  }
  cosine = sqrt(1 - sine * sine);
}

}  // namespace rb::math