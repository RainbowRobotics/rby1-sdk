#pragma once

#include <Eigen/Core>
#include <cstdlib>
#include <iostream>

#include "rby1-sdk/export.h"

namespace rb {

template <int N>
class TrapezoidalMotionGenerator {
 public:
  struct Input {
    Eigen::Vector<double, N> current_position;
    Eigen::Vector<double, N> current_velocity;

    Eigen::Vector<double, N> target_position;
    Eigen::Vector<double, N> velocity_limit;
    Eigen::Vector<double, N> acceleration_limit;
    double minimum_time;
  };

  struct Output {
    Eigen::Vector<double, N> position;
    Eigen::Vector<double, N> velocity;
    Eigen::Vector<double, N> acceleration;
  };

  struct Coeff {
    double start_t;
    double end_t;
    double init_p;
    double init_v;
    double a;
  };

  explicit TrapezoidalMotionGenerator(unsigned int max_iter = 30) : max_iter_(max_iter) {}

  void Update(const Input& input) {
    n_joints_ = input.target_position.size();

    if (input.acceleration_limit.size() != n_joints_ || input.velocity_limit.size() != n_joints_ ||
        input.current_position.size() != n_joints_ || input.current_velocity.size() != n_joints_) {
      throw std::invalid_argument("Optimal control input argument size inconsistent");
    }

    last_input_ = input;

    Eigen::Vector<double, N> l_v, r_v, m_v;
    l_v.resize(n_joints_);
    r_v.resize(n_joints_);
    m_v.resize(n_joints_);
    l_v.setZero();
    r_v = input.velocity_limit;
    m_v = input.velocity_limit;
    Eigen::Array<Coeff, N, 4> spline_coeffs;
    Eigen::Array<Coeff, N, 4> ans_spline_coeffs;
    spline_coeffs.resize(n_joints_, 4);
    ans_spline_coeffs.resize(n_joints_, 4);

    Eigen::Vector<bool, N> check;
    check.resize(n_joints_);
    check.fill(false);

    double max = input.minimum_time;

    bool first = true;
    for (int it = 0; it < max_iter_; it++) {
      for (int i = 0; i < n_joints_; i++) {
        if (check[i])
          continue;

        double cp = input.current_position[i];
        double cv = input.current_velocity[i];

        double tp = input.target_position[i];
        double v_max = m_v[i];
        double a_max = input.acceleration_limit[i];

        bool dir = tp > cp;

        double t = 0;
        if (dir) {
          if (cv > 0) {
            if (cp + 0.5 * cv * cv / a_max < tp) {
              if (cv < v_max) {
                Coeff coeff;  // TODO
                coeff.start_t = 0;
                coeff.end_t = 0;
                coeff.init_p = cp;
                coeff.init_v = cv;
                coeff.a = -a_max;
                spline_coeffs(i, 0) = coeff;
                t = -cv / a_max;
                cp -= 0.5 * cv * cv / a_max;
                cv = 0;
              } else {
                spline_coeffs(i, 0) = {0, (cv - v_max) / a_max, cp, cv, -a_max};
                t = (cv - v_max) / a_max - v_max / a_max;
                cp += 0.5 * (cv + v_max) * (cv - v_max) / a_max - 0.5 * v_max * v_max / a_max;
                cv = 0;
              }
            } else {
              spline_coeffs(i, 0) = {0, cv / a_max, cp, cv, -a_max};
              t = cv / a_max;
              cp += 0.5 * cv * cv / a_max;
              cv = 0;
              dir = false;
            }
          } else {
            spline_coeffs(i, 0) = {0, -cv / a_max, cp, cv, a_max};
            t = -cv / a_max;
            cp -= 0.5 * cv * cv / a_max;
            cv = 0;
          }
        } else {
          if (cv > 0) {
            spline_coeffs(i, 0) = {0, cv / a_max, cp, cv, -a_max};
            t = cv / a_max;
            cp += 0.5 * cv * cv / a_max;
            cv = 0;
          } else {
            if (cp - 0.5 * cv * cv / a_max > tp) {
              if (cv > -v_max) {
                spline_coeffs(i, 0) = {0, 0, cp, cv, a_max};
                t = cv / a_max;
                cp += 0.5 * cv * cv / a_max;
                cv = 0;
              } else {
                spline_coeffs(i, 0) = {0, (-cv - v_max) / a_max, cp, cv, a_max};
                t = (-cv - v_max) / a_max - v_max / a_max;
                cp -= 0.5 * (cv + v_max) * (cv - v_max) / a_max - 0.5 * v_max * v_max / a_max;
                cv = 0;
              }
            } else {
              spline_coeffs(i, 0) = {0, -cv / a_max, cp, cv, a_max};
              t = -cv / a_max;
              cp -= 0.5 * cv * cv / a_max;
              cv = 0;
              dir = true;
            }
          }
        }

        if (dir) {
          double remain_p = tp - cp;
          if (remain_p > v_max * v_max / a_max) {
            double t2 = (remain_p - v_max * v_max / a_max) / v_max;

            spline_coeffs(i, 1) = {t, t + v_max / a_max, cp, cv, a_max};
            t += v_max / a_max;
            cp += 0.5 * v_max * v_max / a_max;
            cv = v_max;

            spline_coeffs(i, 2) = {t, t + t2, cp, cv, 0};
            t += t2;
            cp += v_max * t2;

            spline_coeffs(i, 3) = {t, t + v_max / a_max, cp, cv, -a_max};
            t += v_max / a_max;
            cp += 0.5 * v_max * v_max / a_max;
            cv = 0;
          } else {
            double t1 = std::sqrt(remain_p / a_max);

            spline_coeffs(i, 1) = {t, t + t1, cp, cv, a_max};
            t += t1;
            cp += 0.5 * a_max * t1 * t1;
            cv += a_max * t1;

            spline_coeffs(i, 2) = {t, t, cp, cv, 0};

            spline_coeffs(i, 3) = {t, t + t1, cp, cv, -a_max};
            t += t1;
            cp += 0.5 * a_max * t1 * t1;
            cv = 0;
          }
        } else {
          double remain_p = tp - cp;
          if (-remain_p > v_max * v_max / a_max) {
            double t2 = (-remain_p - v_max * v_max / a_max) / v_max;

            spline_coeffs(i, 1) = {t, t + v_max / a_max, cp, cv, -a_max};
            t += v_max / a_max;
            cp -= 0.5 * v_max * v_max / a_max;
            cv = -v_max;

            spline_coeffs(i, 2) = {t, t + t2, cp, cv, 0};
            cp -= v_max * t2;
            t += t2;

            spline_coeffs(i, 3) = {t, t + v_max / a_max, cp, cv, a_max};
            t += v_max / a_max;
            cp -= 0.5 * v_max * v_max / a_max;
            cv = 0;
          } else {
            double t1 = std::sqrt(-remain_p / a_max);

            spline_coeffs(i, 1) = {t, t + t1, cp, cv, -a_max};
            t += t1;
            cp -= 0.5 * a_max * t1 * t1;
            cv -= a_max * t1;

            spline_coeffs(i, 2) = {t, t, cp, cv, 0};

            spline_coeffs(i, 3) = {t, t + t1, cp, cv, a_max};
            t += t1;
            cp -= 0.5 * a_max * t1 * t1;
            cv = 0;
          }
        }

        if (first) {
          if (max < spline_coeffs(i, 3).end_t) {
            max = spline_coeffs(i, 3).end_t;
          }
        }
      }
      if (first) {
        total_time_ = max;
        first = false;
      }
      for (int i = 0; i < n_joints_; i++) {
        if (std::fabs(spline_coeffs(i, 3).end_t - total_time_) < 1e-5) {
          ans_spline_coeffs.row(i) = spline_coeffs.row(i);
          check[i] = true;
          continue;
        } else if (total_time_ < spline_coeffs(i, 3).end_t) {
          l_v(i) = m_v(i);
        } else {
          ans_spline_coeffs.row(i) = spline_coeffs.row(i);
          r_v(i) = m_v(i);
        }
        m_v(i) = (l_v(i) + r_v(i)) / 2.;
      }
    }
    spline_coeffs_ = ans_spline_coeffs;
  }

  Input GetLastInput() const { return last_input_; }

  bool IsReached(double t) { return t >= total_time_; }

  [[nodiscard]] double GetTotalTime() const { return total_time_; }

  Output operator()(double t) { return at_time(t); }

  Output at_time(double t) {
    if (n_joints_ == 0) {
      throw std::runtime_error("Not initialized problem");
    }

    Output out;
    out.position.resize(n_joints_);
    out.position.fill(0.);
    out.velocity.resize(n_joints_);
    out.velocity.fill(0.);
    out.acceleration.resize(n_joints_);
    out.acceleration.fill(0.);

    if (IsReached(t)) {
      out.position = last_input_.target_position;
      out.velocity.setZero();
      out.acceleration.setZero();
    } else {
      for (int i = 0; i < n_joints_; i++) {
        double stretch = (spline_coeffs_(i, 3).end_t / total_time_);
        double local_t = t * stretch;
        double p = last_input_.target_position(i);
        double v = 0;
        double a = 0;
        double dt = 0;
        for (int j = 0; j < 4; j++) {
          if (local_t < spline_coeffs_(i, j).end_t) {
            p = spline_coeffs_(i, j).init_p;
            v = spline_coeffs_(i, j).init_v;
            a = spline_coeffs_(i, j).a;
            dt = local_t - spline_coeffs_(i, j).start_t;
            break;
          }
        }
        out.position(i) = p + v * dt + 0.5 * a * dt * dt;
        out.velocity(i) = (v + a * dt) * stretch;
        out.acceleration(i) = a;
      }
    }

    return out;
  }

 private:
  unsigned int max_iter_;

  unsigned int n_joints_{0};
  Input last_input_;
  Eigen::Array<Coeff, N, 4> spline_coeffs_;  // [0] =0 속도로 감속
  // [1] = v_max 속도로 가속
  // [2] =v_max 속도 유지
  // [3] = 0 속도로 감속
  double total_time_{};
};

}  // namespace rb