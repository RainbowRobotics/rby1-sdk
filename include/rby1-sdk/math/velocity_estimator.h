#pragma once

#include <queue>

#include "Eigen/Dense"
#include "rby1-sdk/export.h"

namespace rb {

class RBY1_SDK_API VelocityFilterEstimator {
 public:
  explicit VelocityFilterEstimator(double dt, int avg_count = 5)
      : avg_count_(avg_count), valid_(false), velocity_(0), dt_(dt), sum_velocity_(0) {}

  void Update(double position, bool valid) {
    if (valid && valid_) {
      double v = (position - prev_position_) / dt_;
      velocities_.push(v);
      sum_velocity_ += v;
      while (velocities_.size() > avg_count_) {
        sum_velocity_ -= velocities_.front();
        velocities_.pop();
      }
    } else {
      while (!velocities_.empty()) {
        velocities_.pop();
      }
      sum_velocity_ = 0;
    }
    prev_position_ = position;
    valid_ = valid;
  }

  double GetVelocity() {
    if (velocities_.empty()) {
      return 0.;
    }
    return sum_velocity_ / (double)velocities_.size();
  }

 private:
  unsigned int avg_count_;
  std::queue<double> velocities_;
  double sum_velocity_;

  bool valid_;
  double prev_position_{0.};

  double velocity_;
  double dt_;
};

class VelocityEstimator {
 public:
  VelocityEstimator() {
    x_.setZero();
    P_.setIdentity();
    F_.setZero();
    Q_.setZero();
    H_ << 1, 0;
    R_.setZero();
  }

  VelocityEstimator(double initial_position, double initial_velocity, double position_variance,
                    double velocity_variance, double measurement_variance)
      : VelocityEstimator() {

    SetInitialPosition(initial_position);
    SetInitialVelocity(initial_velocity);
    SetPositionVariance(position_variance);
    SetVelocityVariance(velocity_variance);
    SetMeasurementVariance(measurement_variance);
  }

  void SetInitialPosition(double initial_position) { x_(0) = initial_position; }

  void SetInitialVelocity(double initial_velocity) { x_(1) = initial_velocity; }

  void SetPositionVariance(double position_variance) { Q_(0, 0) = position_variance; }

  void SetVelocityVariance(double velocity_variance) { Q_(1, 1) = velocity_variance; }

  void SetMeasurementVariance(double measurement_variance) { R_(0, 0) = measurement_variance; }

  void Predict(double dt) {
    F_(0, 0) = 1;
    F_(0, 1) = dt;
    F_(1, 0) = 0;
    F_(1, 1) = 1;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
  }

  void Update(double measured_position) {
    Eigen::Vector<double, 1> y;
    y(0) = measured_position - H_ * x_;

    Eigen::Matrix<double, 1, 1> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 2, 1> K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
  }

  double GetVelocity() { return x_(1); }

 private:
  Eigen::Vector2d x_;
  Eigen::Matrix2d P_;
  Eigen::Matrix2d F_;
  Eigen::Matrix2d Q_;
  Eigen::Matrix<double, 1, 2> H_;
  Eigen::Matrix<double, 1, 1> R_;
};

};  // namespace rb