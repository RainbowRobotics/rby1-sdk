#include "rby1-sdk/math/se2.h"

namespace rb::math {

SE2::MatrixType SE2::Identity() {
  return MatrixType::Identity();
}

SE2::MatrixType SE2::T(double angle, const Eigen::Vector<double, 2>& translation) {
  double s, c;
  fsincos(angle, s, c);

  MatrixType T;
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  T(0, 2) = translation(0);
  T(1, 2) = translation(1);
  T(2, 0) = 0;
  T(2, 1) = 0;
  T(2, 2) = 1;

  return T;
}

SE2::MatrixType SE2::Exp(const se2v::MatrixType& s, double angle) {
  double a = s[0] * angle;
  Eigen::Vector<double, 2> t = s.tail<2>() * angle;
  return T(a, t);
}

se2v::MatrixType SE2::Log(const MatrixType& T) {
  se2v::MatrixType S;
  S(0) = std::atan2(T(1, 0), T(1, 1));
  S.tail<2>() = T.block<2, 1>(0, 2);
  return S;
}

}  // namespace rb::math