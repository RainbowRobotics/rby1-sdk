#pragma once

#include "Eigen/Core"
#include "constants.h"
#include "rby1-sdk/export.h"

namespace rb::math {

class SE2;
class se2v;

class RBY1_SDK_API se2v {
 public:
  using MatrixType = Eigen::Vector3d;

 private:
  se2v() = default;
};

class RBY1_SDK_API SE2 {
 public:
  using MatrixType = Eigen::Matrix3d;

  static MatrixType Identity();

  static MatrixType T(double angle, const Eigen::Vector<double, 2>& translation = {0, 0});

  static MatrixType Exp(const se2v::MatrixType& s, double angle = 1.0);

  static se2v::MatrixType Log(const MatrixType& T);
};

}  // namespace rb::math