#pragma once

#include <Eigen/Core>
#include <optional>

#include "so3.h"
#include "rby1-sdk/export.h"

namespace rb::math {

class SE3;
class se3;
class se3v;

class RBY1_SDK_API se3 {
 public:
  using MatrixType = Eigen::Matrix4d;

 private:
  se3() = default;
};

class RBY1_SDK_API se3v {
 public:
  using MatrixType = Eigen::Vector<double, 6>;

 private:
  se3v() = default;
};

class RBY1_SDK_API SE3 {
 public:
  using MatrixType = Eigen::Matrix4d;

  static MatrixType Identity();

  static MatrixType T(const Eigen::Vector3d& p);

  static MatrixType T(const SO3::MatrixType& R, const Eigen::Vector3d& p = Eigen::Vector3d::Zero());

  static MatrixType Inverse(const MatrixType& T);

  static SO3::MatrixType GetRotation(const MatrixType& T);

  static Eigen::Vector3d GetPosition(const MatrixType& T);

  static MatrixType Exp(const se3v::MatrixType& S, double angle = 1.);

  static MatrixType Exp(so3v::MatrixType w, Eigen::Vector3d v, double angle = 1.);

  static se3v::MatrixType Log(const MatrixType& T);

  static typename Eigen::Matrix<double, 6, 6> Ad(const MatrixType& T);

  static se3v::MatrixType Ad(const MatrixType& T, const se3v::MatrixType& S);

  static Eigen::Matrix<double, 6, 6> InvAd(const MatrixType& T);

  static se3v::MatrixType InvAd(const MatrixType& T, const se3v::MatrixType& S);

  static typename Eigen::Matrix<double, 6, 6> ad(const se3v::MatrixType& S);

  static Eigen::Matrix<double, 6, 6> adTranspose(const se3v::MatrixType& S);

  static se3v::MatrixType ad(const se3v::MatrixType& S1, const se3v::MatrixType& S2);

  static se3v::MatrixType adTranspose(const se3v::MatrixType& S1, const se3v::MatrixType& S2);

  static Eigen::Vector3d Multiply(const MatrixType& T, const Eigen::Vector3d& p);

  template <typename Container, typename = std::enable_if_t<std::is_same_v<typename Container::value_type, MatrixType>>>
  static std::optional<MatrixType> Average(const Container& matrices, double eps, int max_iter = -1);

  static se3v::MatrixType Vec(const se3::MatrixType& s);

  static se3::MatrixType Hat(const se3v::MatrixType& v);

 private:
  SE3() = default;
};

}  // namespace rb::math