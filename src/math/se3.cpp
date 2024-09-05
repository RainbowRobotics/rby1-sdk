#include "rby1-sdk/math/se3.h"

namespace rb::math {

SE3::MatrixType SE3::Identity() {
  return MatrixType::Identity();
}

SE3::MatrixType SE3::T(const Eigen::Vector3d& p) {
  MatrixType T = Identity();
  T.block<3, 1>(0, 3) = p;
  return T;
}

SE3::MatrixType SE3::T(const SO3::MatrixType& R, const Eigen::Vector3d& p) {
  MatrixType T;
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  T.block<1, 4>(3, 0) = kAffineConstant;
  return T;
}

SE3::MatrixType SE3::Inverse(const MatrixType& T) {
  MatrixType invT;
  invT.block<3, 3>(0, 0) = SO3::Inverse(GetRotation(T));
  invT.block<3, 1>(0, 3) = -invT.block<3, 3>(0, 0) * GetPosition(T);
  invT.block<1, 4>(3, 0) = kAffineConstant;
  return invT;
}

SO3::MatrixType SE3::GetRotation(const MatrixType& T) {
  return T.block<3, 3>(0, 0);
}

Eigen::Vector3d SE3::GetPosition(const MatrixType& T) {
  return T.block<3, 1>(0, 3);
}

SE3::MatrixType SE3::Exp(const se3v::MatrixType& S, double angle) {
  return Exp(S.head<3>(), S.tail<3>(), angle);
}

SE3::MatrixType SE3::Exp(so3v::MatrixType w, Eigen::Vector3d v, double angle) {
  w *= angle;
  v *= angle;

  using std::sqrt;

  double sq0 = w(0) * w(0), sq1 = w(1) * w(1), sq2 = w(2) * w(2);
  double theta = sqrt(sq0 + sq1 + sq2);
  double st_t, ct_t, vt_t;

  if (theta < kDoubleEpsilon) {
    st_t = 1.0 - theta * theta / 6.0;
    ct_t = 0.5 - theta * theta / 24.0;
    vt_t = (w(0) * v(0) + w(1) * v(1) + w(2) * v(2)) * (1.0 - theta * theta / 20.0) / 6.0;
  } else {
    double s, c;
    fsincos(theta, s, c);
    double itheta = 1.0 / theta;
    st_t = s * itheta;
    itheta *= itheta;
    ct_t = (1.0 - c) * itheta;
    vt_t = (w(0) * v(0) + w(1) * v(1) + w(2) * v(2)) * (1.0 - st_t) * itheta;
  }

  MatrixType T;

  T(0, 0) = 1.0 - ct_t * (sq1 + sq2);
  T(0, 1) = ct_t * w(0) * w(1) - st_t * w(2);
  T(0, 2) = ct_t * w(0) * w(2) + st_t * w(1);
  T(1, 0) = ct_t * w(0) * w(1) + st_t * w(2);
  T(1, 1) = 1.0 - ct_t * (sq0 + sq2);
  T(1, 2) = ct_t * w(1) * w(2) - st_t * w(0);
  T(2, 0) = ct_t * w(0) * w(2) - st_t * w(1);
  T(2, 1) = ct_t * w(1) * w(2) + st_t * w(0);
  T(2, 2) = 1.0 - ct_t * (sq0 + sq1);
  T(0, 3) = st_t * v(0) + vt_t * w(0) + ct_t * (w(1) * v(2) - w(2) * v(1));
  T(1, 3) = st_t * v(1) + vt_t * w(1) + ct_t * (w(2) * v(0) - w(0) * v(2));
  T(2, 3) = st_t * v(2) + vt_t * w(2) + ct_t * (w(0) * v(1) - w(1) * v(0));
  T.block<1, 4>(3, 0) = kAffineConstant;

  return T;
}

se3v::MatrixType SE3::Log(const MatrixType& T) {
  using std::cos;
  using std::sin;

  se3v::MatrixType S;

  double theta = 0.5 * (T(0, 0) + T(1, 1) + T(2, 2) - 1.0), t_st;

  if (theta < kDoubleEpsilon - 1.0) {
    double w[3];
    if (T(0, 0) > 1.0 - kDoubleEpsilon) {
      w[0] = kPi;
      w[1] = w[2] = 0.0;
    } else if (T(1, 1) > 1.0 - kDoubleEpsilon) {
      w[1] = kPi;
      w[0] = w[2] = 0.0;
    } else if (T(2, 2) > 1.0 - kDoubleEpsilon) {
      w[2] = kPi;
      w[0] = w[1] = 0.0;
    } else {
      w[0] = kPiDividedBySqrt2 * sqrt((T(1, 0) * T(1, 0) + T(2, 0) * T(2, 0)) / (1.0 - T(0, 0)));
      w[1] = kPiDividedBySqrt2 * sqrt((T(0, 1) * T(0, 1) + T(2, 1) * T(2, 1)) / (1.0 - T(1, 1)));
      w[2] = kPiDividedBySqrt2 * sqrt((T(0, 2) * T(0, 2) + T(1, 2) * T(1, 2)) / (1.0 - T(2, 2)));
    }
    double w2[] = {w[0] * w[0], w[1] * w[1], w[2] * w[2]}, w3[] = {w2[0] * w[0], w2[1] * w[1], w2[2] * w[2]},
           v[] = {w[1] * w[2], w[2] * w[0], w[0] * w[1]};
    double id = 0.25 * kPiSquare /
                (w2[0] * w2[0] + w2[1] * w2[1] + w2[2] * w2[2] + 2.0 * (w2[0] * w2[1] + w2[1] * w2[2] + w2[2] * w2[0]));
    double p[3] = {T(0, 3) * id, T(1, 3) * id, T(2, 3) * id};

    S(0) = w[0];
    S(1) = w[1];
    S(2) = w[2];
    S(3) = 2.0 * (2.0 * w2[0] * p[0] + (w3[2] + v[1] * w[0] + v[0] * w[1] + 2.0 * v[2]) * p[1] +
                  (2.0 * v[1] - w3[1] - w[0] * v[2] - w[2] * v[0]) * p[2]);
    S(4) = 2.0 * ((2.0 * v[2] - w3[2] - v[0] * w[1] - v[1] * w[0]) * p[0] + 2.0 * w2[1] * p[1] +
                  (w3[0] + w[2] * v[1] + v[2] * w[1] + 2.0 * v[0]) * p[2]);
    S(5) = 2.0 * ((w[0] * v[2] + w[2] * v[0] + 2.0 * v[1] + w3[1]) * p[0] +
                  (2.0 * v[0] - w3[0] - v[2] * w[1] - w[2] * v[1]) * p[1] + 2.0 * w2[2] * p[2]);
  } else {
    if (theta > 1.0) {
      theta = 1.0;
    } else if (theta < -1.0) {
      theta = -1.0;
    }

    theta = acos(theta);
    if (theta < kDoubleEpsilon)
      t_st = 3.0 / (6.0 - theta * theta);
    else
      t_st = theta / (2.0 * sin(theta));
    double stct, st = sin(theta), w[] = {T(2, 1) - T(1, 2), T(0, 2) - T(2, 0), T(1, 0) - T(0, 1)},
                 w2[] = {w[0] * w[0], w[1] * w[1], w[2] * w[2]}, w3[] = {w[1] * w[2], w[2] * w[0], w[0] * w[1]};
    w[0] = t_st * w[0];
    w[1] = t_st * w[1];
    w[2] = t_st * w[2];

    if (theta < kDoubleEpsilon)
      stct = theta / 48.0;
    else
      stct = (2.0 * st - theta * (1.0 + cos(theta))) / (8.0 * st * st * st);

    S(0) = w[0];
    S(1) = w[1];
    S(2) = w[2];
    S(3) = (1.0 - stct * (w2[1] + w2[2])) * T(0, 3) + (0.5 * w[2] + stct * w3[2]) * T(1, 3) +
           (stct * w3[1] - 0.5 * w[1]) * T(2, 3);
    S(4) = (stct * w3[2] - 0.5 * w[2]) * T(0, 3) + (1.0 - stct * (w2[0] + w2[2])) * T(1, 3) +
           (0.5 * w[0] + stct * w3[0]) * T(2, 3);
    S(5) = (0.5 * w[1] + stct * w3[1]) * T(0, 3) + (stct * w3[0] - 0.5 * w[0]) * T(1, 3) +
           (1.0 - stct * (w2[1] + w2[0])) * T(2, 3);
  }

  return S;
}

Eigen::Matrix<double, 6, 6> SE3::Ad(const MatrixType& T) {
  Eigen::Matrix<double, 6, 6> m;

  m(0, 0) = T(0, 0);
  m(0, 1) = T(0, 1);
  m(0, 2) = T(0, 2);
  m(0, 3) = 0;
  m(0, 4) = 0;
  m(0, 5) = 0;

  m(1, 0) = T(1, 0);
  m(1, 1) = T(1, 1);
  m(1, 2) = T(1, 2);
  m(1, 3) = 0;
  m(1, 4) = 0;
  m(1, 5) = 0;

  m(2, 0) = T(2, 0);
  m(2, 1) = T(2, 1);
  m(2, 2) = T(2, 2);
  m(2, 3) = 0;
  m(2, 4) = 0;
  m(2, 5) = 0;

  m(3, 0) = -T(2, 3) * T(1, 0) + T(1, 3) * T(2, 0);
  m(3, 1) = -T(2, 3) * T(1, 1) + T(1, 3) * T(2, 1);
  m(3, 2) = -T(2, 3) * T(1, 2) + T(1, 3) * T(2, 2);
  m(3, 3) = T(0, 0);
  m(3, 4) = T(0, 1);
  m(3, 5) = T(0, 2);

  m(4, 0) = T(2, 3) * T(0, 0) - T(0, 3) * T(2, 0);
  m(4, 1) = T(2, 3) * T(0, 1) - T(0, 3) * T(2, 1);
  m(4, 2) = T(2, 3) * T(0, 2) - T(0, 3) * T(2, 2);
  m(4, 3) = T(1, 0);
  m(4, 4) = T(1, 1);
  m(4, 5) = T(1, 2);

  m(5, 0) = -T(1, 3) * T(0, 0) + T(0, 3) * T(1, 0);
  m(5, 1) = -T(1, 3) * T(0, 1) + T(0, 3) * T(1, 1);
  m(5, 2) = -T(1, 3) * T(0, 2) + T(0, 3) * T(1, 2);
  m(5, 3) = T(2, 0);
  m(5, 4) = T(2, 1);
  m(5, 5) = T(2, 2);

  return m;
}

se3v::MatrixType SE3::Ad(const MatrixType& T, const se3v::MatrixType& S) {
  double Rw1, Rw2, Rw3;
  Rw1 = T(0, 0) * S(0) + T(0, 1) * S(1) + T(0, 2) * S(2);
  Rw2 = T(1, 0) * S(0) + T(1, 1) * S(1) + T(1, 2) * S(2);
  Rw3 = T(2, 0) * S(0) + T(2, 1) * S(1) + T(2, 2) * S(2);
  se3v::MatrixType AdT_S;
  AdT_S(0) = Rw1;
  AdT_S(1) = Rw2;
  AdT_S(2) = Rw3;
  AdT_S(3) = -T(2, 3) * Rw2 + T(1, 3) * Rw3 + T(0, 0) * S(3) + T(0, 1) * S(4) + T(0, 2) * S(5);
  AdT_S(4) = T(2, 3) * Rw1 - T(0, 3) * Rw3 + T(1, 0) * S(3) + T(1, 1) * S(4) + T(1, 2) * S(5);
  AdT_S(5) = -T(1, 3) * Rw1 + T(0, 3) * Rw2 + T(2, 0) * S(3) + T(2, 1) * S(4) + T(2, 2) * S(5);
  return AdT_S;
}

Eigen::Matrix<double, 6, 6> SE3::InvAd(const MatrixType& T) {
  typename Eigen::Matrix<double, 6, 6> m;

  m(0, 0) = T(0, 0);
  m(0, 1) = T(1, 0);
  m(0, 2) = T(2, 0);
  m(0, 3) = 0;
  m(0, 4) = 0;
  m(0, 5) = 0;

  m(1, 0) = T(0, 1);
  m(1, 1) = T(1, 1);
  m(1, 2) = T(2, 1);
  m(1, 3) = 0;
  m(1, 4) = 0;
  m(1, 5) = 0;

  m(2, 0) = T(0, 2);
  m(2, 1) = T(1, 2);
  m(2, 2) = T(2, 2);
  m(2, 3) = 0;
  m(2, 4) = 0;
  m(2, 5) = 0;

  m(3, 0) = -T(1, 0) * T(2, 3) + T(2, 0) * T(1, 3);
  m(3, 1) = T(0, 0) * T(2, 3) - T(2, 0) * T(0, 3);
  m(3, 2) = -T(0, 0) * T(1, 3) + T(1, 0) * T(0, 3);
  m(3, 3) = T(0, 0);
  m(3, 4) = T(1, 0);
  m(3, 5) = T(2, 0);

  m(4, 0) = -T(1, 1) * T(2, 3) + T(2, 1) * T(1, 3);
  m(4, 1) = T(0, 1) * T(2, 3) - T(2, 1) * T(0, 3);
  m(4, 2) = -T(0, 1) * T(1, 3) + T(1, 1) * T(0, 3);
  m(4, 3) = T(0, 1);
  m(4, 4) = T(1, 1);
  m(4, 5) = T(2, 1);

  m(5, 0) = -T(1, 2) * T(2, 3) + T(2, 2) * T(1, 3);
  m(5, 1) = T(0, 2) * T(2, 3) - T(2, 2) * T(0, 3);
  m(5, 2) = -T(0, 2) * T(1, 3) + T(1, 2) * T(0, 3);
  m(5, 3) = T(0, 2);
  m(5, 4) = T(1, 2);
  m(5, 5) = T(2, 2);

  return m;
}

se3v::MatrixType SE3::InvAd(const MatrixType& T, const se3v::MatrixType& S) {
  return Ad(T.inverse(), S);
}

Eigen::Matrix<double, 6, 6> SE3::ad(const se3v::MatrixType& S) {
  Eigen::Matrix<double, 6, 6> m;

  m(0, 0) = 0;
  m(0, 1) = -S(2);
  m(0, 2) = S(1);
  m(0, 3) = 0;
  m(0, 4) = 0;
  m(0, 5) = 0;

  m(1, 0) = S(2);
  m(1, 1) = 0;
  m(1, 2) = -S(0);
  m(1, 3) = 0;
  m(1, 4) = 0;
  m(1, 5) = 0;

  m(2, 0) = -S(1);
  m(2, 1) = S(0);
  m(2, 2) = 0;
  m(2, 3) = 0;
  m(2, 4) = 0;
  m(2, 5) = 0;

  m(3, 0) = 0;
  m(3, 1) = -S(5);
  m(3, 2) = S(4);
  m(3, 3) = 0;
  m(3, 4) = -S(2);
  m(3, 5) = S(1);

  m(4, 0) = S(5);
  m(4, 1) = 0;
  m(4, 2) = -S(3);
  m(4, 3) = S(2);
  m(4, 4) = 0;
  m(4, 5) = -S(0);

  m(5, 0) = -S(4);
  m(5, 1) = S(3);
  m(5, 2) = 0;
  m(5, 3) = -S(1);
  m(5, 4) = S(0);
  m(5, 5) = 0;

  return m;
}

Eigen::Matrix<double, 6, 6> SE3::adTranspose(const se3v::MatrixType& S) {
  Eigen::Matrix<double, 6, 6> m;

  m(0, 0) = 0;
  m(0, 1) = S(2);
  m(0, 2) = -S(1);
  m(0, 3) = 0;
  m(0, 4) = S(5);
  m(0, 5) = -S(4);

  m(1, 0) = -S(2);
  m(1, 1) = 0;
  m(1, 2) = S(0);
  m(1, 3) = -S(5);
  m(1, 4) = 0;
  m(1, 5) = S(3);

  m(2, 0) = S(1);
  m(2, 1) = -S(0);
  m(2, 2) = 0;
  m(2, 3) = S(4);
  m(2, 4) = -S(3);
  m(2, 5) = 0;

  m(3, 0) = 0;
  m(3, 1) = 0;
  m(3, 2) = 0;
  m(3, 3) = 0;
  m(3, 4) = S(2);
  m(3, 5) = -S(1);

  m(4, 0) = 0;
  m(4, 1) = 0;
  m(4, 2) = 0;
  m(4, 3) = -S(2);
  m(4, 4) = 0;
  m(4, 5) = S(0);

  m(5, 0) = 0;
  m(5, 1) = 0;
  m(5, 2) = 0;
  m(5, 3) = S(1);
  m(5, 4) = -S(0);
  m(5, 5) = 0;

  return m;
}

se3v::MatrixType SE3::ad(const se3v::MatrixType& S1, const se3v::MatrixType& S2) {
  se3v::MatrixType S;

  S(0) = -S1(2) * S2(1) + S1(1) * S2(2);
  S(1) = S1(2) * S2(0) - S1(0) * S2(2);
  S(2) = -S1(1) * S2(0) + S1(0) * S2(1);
  S(3) = -S1(5) * S2(1) + S1(4) * S2(2) - S1(2) * S2(4) + S1(1) * S2(5);
  S(4) = S1(5) * S2(0) - S1(3) * S2(2) + S1(2) * S2(3) - S1(0) * S2(5);
  S(5) = -S1(4) * S2(0) + S1(3) * S2(1) - S1(1) * S2(3) + S1(0) * S2(4);

  return S;
}

se3v::MatrixType SE3::adTranspose(const se3v::MatrixType& S1, const se3v::MatrixType& S2) {
  se3v::MatrixType S;

  S(0) = S1(2) * S2(1) - S1(1) * S2(2) + S1(5) * S2(4) - S1(4) * S2(5);
  S(1) = -S1(2) * S2(0) + S1(0) * S2(2) - S1(5) * S2(3) + S1(3) * S2(5);
  S(2) = S1(1) * S2(0) - S1(0) * S2(1) + S1(4) * S2(3) - S1(3) * S2(4);
  S(3) = S1(2) * S2(4) - S1(1) * S2(5);
  S(4) = -S1(2) * S2(3) + S1(0) * S2(5);
  S(5) = S1(1) * S2(3) - S1(0) * S2(4);

  return S;
}

Eigen::Vector3d SE3::Multiply(const MatrixType& T, const Eigen::Vector3d& p) {
  return T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
}

se3v::MatrixType SE3::Vec(const se3::MatrixType& s) {
  se3v::MatrixType v;
  v.head<3>() = SO3::Vec(s.block<3, 3>(0, 0));
  v.tail<3>() = s.block<3, 1>(0, 3);
  return v;
}

se3::MatrixType SE3::Hat(const se3v::MatrixType& v) {
  se3::MatrixType s;
  s.setZero();
  s.block<3, 3>(0, 0) = SO3::Hat(v.head<3>());
  s.block<3, 1>(0, 3) = v.tail<3>();
  return s;
}

}  // namespace rb::math