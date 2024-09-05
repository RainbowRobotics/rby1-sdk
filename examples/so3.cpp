//
// Created by keunjun on 24. 7. 4.
//

#include <iostream>

//#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/math/liegroup.h"

using namespace rb::math;

int main() {
  //  rb::dyn::Robot<1, 2, double> robot;

  SO3::MatrixType R = SO3::Identity();
  //  std::cout << SO3f::FromQuaternion(Eigen::Quaternionf(1 / 1.414, 0, 0, 1 / 1.414)) << std::endl;
  //  std::cout << SO3f::RotX(kPi / 2) << std::endl;
  //  so3vd::MatrixType v{1, 2, 3};
  //  v.normalize();
  //  double angle = 1.5;
  //  const auto& w = SO3d::Exp(v, angle);
  //  std::cout << (v * angle).transpose() << std::endl;
  //  std::cout << SO3d::Log(w).transpose() << std::endl;
  //
  //  double s, c;
  //  rb::math::fsincos(0.2, s, c);
  //  std::cout << s << " " << c << std::endl;
  //
  //  std::cout << R << std::endl;

  //  const auto& S = SE3::hat({1, 2, 3, 4, 5, 6});
  //  std::cout << S << std::endl;
  //  std::cout << SE3::vec(S) << std::endl;

  //  const auto& T = SE3::Exp({1, 2, 3}, {4, 5, 6}, 7.);
  //  std::cout << T << std::endl << std::endl;
  //
  //  const auto& S = SE3::Log(T).transpose();
  //  std::cout << S  << std::endl << std::endl;
  //
  //  std::cout << SE3::Exp(S) << std::endl;

  auto T = SE3::T(SO3::RotX(0.1) * SO3::RotZ(0.4), Eigen::Vector3d{1, 2, 3});

  std::cout << "----------------" << std::endl;
  std::cout << "T = " << std::endl << T << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "T*inv(T) = " << std::endl << T * SE3::Inverse(T) << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "Log(T) = " << std::endl << SE3::Log(T).transpose() << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "Exp(Log(T)) = " << std::endl << SE3::Exp(SE3::Log(T)) << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "ad(Log(T)) = " << std::endl << SE3::ad(SE3::Log(T)) << std::endl;

  std::cout << "----------------" << std::endl;
  Eigen::Vector<double, 6> S;
  S << 1, 2, 3, 4, 5, 6;
  std::cout << "S = " << std::endl << S.transpose() << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "ad(Log(T)) * S = " << std::endl << (SE3::ad(SE3::Log(T)) * S).transpose() << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "ad(Log(T), S) = " << std::endl << SE3::ad(SE3::Log(T), S).transpose() << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "ad(Log(T))^T * S = " << std::endl << (SE3::ad(SE3::Log(T)).transpose() * S).transpose() << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "adTranspose(Log(T)) * S = " << std::endl
            << (SE3::ad(SE3::Log(T)).transpose() * S).transpose() << std::endl;

  std::cout << "----------------" << std::endl;
  std::cout << "adTranspose(Log(T), S) = " << std::endl << SE3::adTranspose(SE3::Log(T), S).transpose() << std::endl;

  return 0;
}