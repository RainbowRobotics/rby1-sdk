#include "rby1-sdk/math/so3.h"

namespace rb::math {

SO3::MatrixType SO3::Identity() {
  return MatrixType::Identity();
}

SO3::MatrixType SO3::Inverse(const MatrixType& R) {
  return R.transpose();
}

SO3::MatrixType SO3::Exp(so3v::MatrixType w, double angle) {
  w *= angle;

  using std::cos;
  using std::sin;
  using std::sqrt;

  double sq0 = w(0) * w(0), sq1 = w(1) * w(1), sq2 = w(2) * w(2);
  double theta = sqrt(sq0 + sq1 + sq2);
  double st_t, ct_t;

  if (theta < kDoubleEpsilon) {
    st_t = 1.0 - theta * theta / 6.0;
    ct_t = 0.5 - theta * theta / 24.0;
  } else {
    double s, c;
    fsincos(theta, s, c);
    double itheta = 1.0 / theta;
    st_t = s * itheta;
    itheta *= itheta;
    ct_t = (1.0 - c) * itheta;
  }

  MatrixType m;

  m(0, 0) = 1.0 - ct_t * (sq1 + sq2);
  m(0, 1) = ct_t * w(0) * w(1) - st_t * w(2);
  m(0, 2) = ct_t * w(0) * w(2) + st_t * w(1);
  m(1, 0) = ct_t * w(0) * w(1) + st_t * w(2);
  m(1, 1) = 1.0 - ct_t * (sq0 + sq2);
  m(1, 2) = ct_t * w(1) * w(2) - st_t * w(0);
  m(2, 0) = ct_t * w(0) * w(2) - st_t * w(1);
  m(2, 1) = ct_t * w(1) * w(2) + st_t * w(0);
  m(2, 2) = 1.0 - ct_t * (sq0 + sq1);

  return m;
}

so3v::MatrixType SO3::Log(const MatrixType& R) {
  using std::acos;
  using std::sin;
  using std::sqrt;

  double theta = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1.0), t_st;

  if (theta < kDoubleEpsilon - 1.0) {
    if (R(0, 0) > 1.0 - kDoubleEpsilon)
      return {kPi, 0.0, 0.0};
    else if (R(1, 1) > 1.0 - kDoubleEpsilon)
      return {0.0, kPi, 0.0};
    else if (R(2, 2) > 1.0 - kDoubleEpsilon)
      return {0.0, 0.0, kPi};

    return {(kPiDividedBySqrt2 * sqrt((R(1, 0) * R(1, 0) + R(2, 0) * R(2, 0)) / (1.0 - R(0, 0)))),
            (kPiDividedBySqrt2 * sqrt((R(0, 1) * R(0, 1) + R(2, 1) * R(2, 1)) / (1.0 - R(1, 1)))),
            (kPiDividedBySqrt2 * sqrt((R(0, 2) * R(0, 2) + R(1, 2) * R(1, 2)) / (1.0 - R(2, 2))))};
  }
  theta = acos(theta);
  if (theta < kDoubleEpsilon)
    t_st = 3.0 / (6.0 - theta * theta);
  else
    t_st = theta / (2.0 * sin(theta));
  return {t_st * (R(2, 1) - R(1, 2)), t_st * (R(0, 2) - R(2, 0)), t_st * (R(1, 0) - R(0, 1))};
}

SO3::MatrixType SO3::RotX(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

SO3::MatrixType SO3::RotY(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

SO3::MatrixType SO3::RotZ(double angle) {
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

SO3::MatrixType SO3::FromEulerAngle(const Eigen::Vector3d& angles, EulerAngleType type) {
  switch (type) {
    case EulerAngleType::ZYX:
      return RotZ(angles[0]) * RotY(angles[1]) * RotX(angles[2]);
    case EulerAngleType::ZYZ:
      return RotZ(angles[0]) * RotY(angles[1]) * RotZ(angles[2]);
  }
  assert(false);
  return Identity();
}

Eigen::Vector3d SO3::ToEulerAngle(const SO3::MatrixType& R, EulerAngleType type) {
  using std::atan2;

  switch (type) {
    case EulerAngleType::ZYX:
      return {atan2(R(1, 0), R(0, 0)), atan2(-R(2, 0), sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))),
              atan2(R(2, 1), R(2, 2))};
    case EulerAngleType::ZYZ:
      return {atan2(R(1, 2), R(0, 2)), atan2(sqrt(R(0, 2) * R(0, 2) + R(1, 2) * R(1, 2)), R(2, 2)),
              atan2(R(2, 1), -R(2, 0))};
  }
  assert(false);
  return Eigen::Vector3d::Zero();
}

SO3::MatrixType SO3::FromQuaternion(const Eigen::Quaterniond& q) {
  return q.toRotationMatrix();
}

Eigen::Quaterniond SO3::ToQuaternion(const SO3::MatrixType& R) {
  return Eigen::Quaterniond(R);
}

SO3::MatrixType SO3::FromRPY(const Eigen::Vector3d& angles) {
  double r = angles(0), p = angles(1), y = angles(2);
  return FromEulerAngle({y, p, r}, EulerAngleType::ZYX);
}

Eigen::Vector3d SO3::ToRPY(const MatrixType& R) {
  const auto angles = ToEulerAngle(R, EulerAngleType::ZYX);
  return {angles[2], angles[1], angles[0]};
}

SO3::MatrixType SO3::Projection(const MatrixType& m) {
  Eigen::JacobiSVD<MatrixType> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (m.determinant() > 0) {
    return svd.matrixV() * svd.matrixU().transpose();
  }
  return svd.matrixV() * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * svd.matrixU().transpose();
}

Eigen::Vector3d SO3::GetX(const MatrixType& R) {
  return R.template block<3, 1>(0, 0);
}

Eigen::Vector3d SO3::GetY(const MatrixType& R) {
  return R.template block<3, 1>(0, 1);
}

Eigen::Vector3d SO3::GetZ(const MatrixType& R) {
  return R.template block<3, 1>(0, 2);
}

so3v::MatrixType SO3::Vec(const so3::MatrixType& r) {
  so3v::MatrixType w;
  w(0) = -r(1, 2);
  w(1) = r(0, 2);
  w(2) = -r(0, 1);
  return w;
}

so3::MatrixType SO3::Hat(const so3v::MatrixType& w) {
  so3::MatrixType m;
  m(0, 0) = 0;
  m(0, 1) = -w(2);
  m(0, 2) = w(1);
  m(1, 0) = w(2);
  m(1, 1) = 0;
  m(1, 2) = -w(0);
  m(2, 0) = -w(1);
  m(2, 1) = w(0);
  m(2, 2) = 0;
  return m;
}

}  // namespace rb::math
