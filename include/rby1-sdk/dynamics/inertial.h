#pragma once

#include <Eigen/Core>

#include "rby1-sdk/export.h"
#include "rby1-sdk/math/se3.h"

namespace rb::dyn {

class Inertial;

class RBY1_SDK_API Inertial {
 public:
  using MatrixType = Eigen::Matrix<double, 6, 6>;

  static MatrixType I(double mass);

  static MatrixType I(double mass, double ixx, double iyy, double izz,
                      const math::SE3::MatrixType& T = math::SE3::Identity());

  static MatrixType I(double mass, double ixx, double iyy, double izz,
                      const Eigen::Vector3d& com);

  static MatrixType I(double mass, double ixx, double iyy, double izz, double ixy, double ixz, double iyz,
                      const math::SE3::MatrixType& T = math::SE3::Identity());

  static MatrixType I(double mass, double ixx, double iyy, double izz, double ixy, double ixz, double iyz,
                      const Eigen::Vector3d& com);

  static MatrixType I(double mass, const Eigen::Vector3d& inertia,
                      const math::SE3::MatrixType& T = math::SE3::Identity());

  static MatrixType I(double mass, const Eigen::Matrix<double, 6, 1>& inertia,
                      const math::SE3::MatrixType& T = math::SE3::Identity());

  static MatrixType Transform(const math::SE3::MatrixType& T, const MatrixType& I);

  static Eigen::Vector3d GetCOM(const MatrixType& I);

  static double GetMass(const MatrixType& I);

  // [I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{xz}, I_{yz}]^T
  static Eigen::Vector<double, 6> GetInertia(const MatrixType& I);
};

}  // namespace rb::dyn