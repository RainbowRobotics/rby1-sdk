#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <utility>

#include "constants.h"
#include "rby1-sdk/export.h"

namespace rb::math {

enum class EulerAngleType { ZYX, ZYZ };

class SO3;
class so3;
class so3v;

class RBY1_SDK_API so3 {
 public:
  using MatrixType = Eigen::Matrix3d;

 private:
  so3() = default;
};

class RBY1_SDK_API so3v {
 public:
  using MatrixType = Eigen::Vector3d;

 private:
  so3v() = default;
};

class RBY1_SDK_API SO3 {
 public:
  using MatrixType = Eigen::Matrix3d;

  static MatrixType Identity();

  static MatrixType Inverse(const MatrixType& R);

  static MatrixType Exp(so3v::MatrixType w, double angle = 1.0);

  static so3v::MatrixType Log(const MatrixType& R);

  static MatrixType RotX(double angle);

  static MatrixType RotY(double angle);

  static MatrixType RotZ(double angle);

  static MatrixType FromEulerAngle(const Eigen::Vector3d& angles, EulerAngleType type);

  static Eigen::Vector3d ToEulerAngle(const MatrixType& R, EulerAngleType type);

  static MatrixType FromQuaternion(const Eigen::Quaterniond& q);

  static Eigen::Quaterniond ToQuaternion(const MatrixType& R);

  /**
   * Calculate rotation from roll, pitch and yqw angles
   * @param angles [0] roll [1] pitch [2] yaw
   * @return
   */
  static MatrixType FromRPY(const Eigen::Vector3d& angles);

  /**
   *
   * @param R
   * @return [0] roll [1] pitch [2] yaw
   */
  static Eigen::Vector3d ToRPY(const MatrixType& R);

  static MatrixType Projection(const MatrixType& m);

  template <typename Container, typename = std::enable_if_t<std::is_same_v<typename Container::value_type, MatrixType>>>
  static std::optional<MatrixType> Average(const Container& matrices, double eps, int max_iter = -1) {
    if (matrices.size() == 0) {
      return Identity();
    }

    SO3::MatrixType avg = *matrices.begin();
    for (int i = 0; max_iter < 0 || i < max_iter; i++) {
      Eigen::Vector3d w = Eigen::Vector3d::Zero();

      for (const auto& m : matrices) {
        w += Log(m * avg.inverse());
      }
      w /= matrices.size();
      avg = SO3::Exp(w) * avg;
      if (w.norm() < eps) {
        return avg;
      }
    }

    return std::nullopt;
  }

  static Eigen::Vector3d GetX(const MatrixType& R);

  static Eigen::Vector3d GetY(const MatrixType& R);

  static Eigen::Vector3d GetZ(const MatrixType& R);

  static so3v::MatrixType Vec(const so3::MatrixType& r);

  static so3::MatrixType Hat(const so3v::MatrixType& w);

 private:
  SO3() = default;
};

}  // namespace rb::math