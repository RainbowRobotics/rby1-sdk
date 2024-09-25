#include "rby1-sdk/dynamics/inertial.h"

namespace rb::dyn {

Inertial::MatrixType Inertial::I(double mass) {
  return mass * MatrixType::Identity();
}

Inertial::MatrixType Inertial::I(double mass, double Ixx, double Iyy, double Izz, const math::SE3::MatrixType& T) {
  return I(mass, Ixx, Iyy, Izz, 0, 0, 0, T);
}

Inertial::MatrixType Inertial::I(double mass, double Ixx, double Iyy, double Izz, const Eigen::Vector3d& com) {
  return I(mass, Ixx, Iyy, Izz, 0, 0, 0, math::SE3::T(math::SO3::Identity(), com));
}

Inertial::MatrixType Inertial::I(double mass, double ixx, double iyy, double izz, double ixy, double ixz, double iyz,
                                 const math::SE3::MatrixType& T) {
  MatrixType I;
  I(0, 0) = ixx;
  I(0, 1) = ixy;
  I(0, 2) = ixz;
  I(1, 0) = ixy;
  I(1, 1) = iyy;
  I(1, 2) = iyz;
  I(2, 0) = ixz;
  I(2, 1) = iyz;
  I(2, 2) = izz;
  I.block<3, 3>(0, 3).setZero();
  I.block<3, 3>(3, 0).setZero();
  I.block<3, 3>(3, 3) = mass * Eigen::Matrix3d::Identity();
  return Transform(T, I);
}

Inertial::MatrixType Inertial::I(double mass, double ixx, double iyy, double izz, double ixy, double ixz, double iyz,
                                 const Eigen::Vector3d& com) {
  return I(mass, ixx, iyy, izz, ixy, ixz, iyz, math::SE3::T(math::SO3::Identity(), com));
}

Inertial::MatrixType Inertial::I(double mass, const Eigen::Vector3d& inertia, const math::SE3::MatrixType& T) {
  return I(mass, inertia(0), inertia(1), inertia(2), T);
}

Inertial::MatrixType Inertial::I(double mass, const Eigen::Matrix<double, 6, 1>& inertia,
                                 const math::SE3::MatrixType& T) {
  return I(mass, inertia(0), inertia(1), inertia(2), inertia(3), inertia(4), inertia(5), T);
}

Inertial::MatrixType Inertial::Transform(const math::SE3::MatrixType& T, const Inertial::MatrixType& I) {
  math::SE3::MatrixType invT = math::SE3::Inverse(T);
  return math::SE3::Ad(invT).transpose() * I * math::SE3::Ad(invT);
}

Eigen::Vector3d Inertial::GetCOM(const Inertial::MatrixType& I) {
  double mass = GetMass(I);
  return Eigen::Vector3d{-I(1, 5), I(0, 5), -I(0, 4)} / mass;
}

double Inertial::GetMass(const Inertial::MatrixType& I) {
  return I(3, 3);
}

Eigen::Vector<double, 6> Inertial::GetInertia(const Inertial::MatrixType& I) {
  return {I(0, 0), I(1, 1), I(2, 2), I(0, 1), I(0, 2), I(1, 2)};
}

}  // namespace rb::dyn