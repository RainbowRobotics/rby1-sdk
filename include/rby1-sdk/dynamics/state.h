#pragma once

#include <Eigen/Core>

#include "rby1-sdk/export.h"

namespace rb::dyn {

template <int DOF>
class Robot;

template <int DOF>
class State {
 public:
  template <int>
  friend class Robot;

  template <typename T, int N>
  using ContainerType = typename std::conditional_t<(N > 0), std::array<T, (unsigned int)N>, std::vector<T>>;

  unsigned int GetBaseLinkIdx() const {  // NOLINT
    return base_link_user_idx;
  }

  template <typename Derived>
  void SetQ(const Eigen::MatrixBase<Derived>& new_q) {
    Set<Derived::RowsAtCompileTime>(q, new_q.eval());
  }

  Eigen::Vector<double, DOF> GetQ() const { return q(utr_joint_map); }

  template <typename Derived>
  void SetQdot(const Eigen::MatrixBase<Derived>& new_qdot) {
    Set<Derived::RowsAtCompileTime>(qdot, new_qdot.eval());
  }

  Eigen::Vector<double, DOF> GetQdot() const { return qdot(utr_joint_map); }

  template <typename Derived>
  void SetQddot(const Eigen::MatrixBase<Derived>& new_qddot) {
    Set<Derived::RowsAtCompileTime>(qddot, new_qddot.eval());
  }

  Eigen::Vector<double, DOF> GetQddot() const { return qddot(utr_joint_map); }

  template <typename Derived>
  void SetTau(const Eigen::MatrixBase<Derived>& new_tau) {
    Set<Derived::RowsAtCompileTime>(tau, new_tau.eval());
  }

  const Eigen::Vector<double, DOF> GetTau() const { return tau(utr_joint_map); }

  const math::se3v::MatrixType& GetV0() const { return V0; }

  void SetV0(const math::se3v::MatrixType& new_V0) { V0 = new_V0; }

  const math::se3v::MatrixType& GetVdot0() const { return Vdot0; }

  // Vdot of root link in root frame
  void SetVdot0(const math::se3v::MatrixType& new_Vdot0) { Vdot0 = new_Vdot0; }

  void SetGravity(const math::se3v::MatrixType& gravity) { Vdot0 = -gravity; }

  ContainerType<std::string, DOF> GetJointNames() const { return joint_names; }

  [[nodiscard]] std::vector<std::string> GetLinkNames() const { return link_names; }

 private:
  explicit State(int dof) {
    if (!(DOF < 0 || dof == DOF)) {
      throw std::runtime_error("State initialization failed");
    }

    if constexpr (DOF < 0) {
      q.resize(dof);
      qdot.resize(dof);
      qddot.resize(dof);
      tau.resize(dof);
      E.resize(dof + 1, math::SE3::Identity());
      S.resize(6, dof);
      T.resize(dof + 1, math::SE3::Identity());
      V.resize(6, dof);
      Vdot.resize(6, dof);
      F.resize(6, dof);
      joint_names.resize(dof);
      utr_joint_map.resize(dof);
      rtu_joint_map.resize(dof);
    } else {
      E.fill(math::SE3::Identity());
      T.fill(math::SE3::Identity());
    }

    q.setZero();
    qdot.setZero();
    qddot.setZero();
    S.setZero();
    V.setZero();
    Vdot0.setZero();
    F.setZero();
  }

  template <int N>
  void Set(Eigen::Vector<double, DOF>& s, const Eigen::Vector<double, N>& i) {
    if (s.size() == i.size()) {
      s = i(rtu_joint_map);
    } else if (s.size() > i.size()) {
      s.template head<>(i.size()) = i(rtu_joint_map.template head<>(i.size()));
    } else {
      throw std::runtime_error("i.size cannot be greater than s.size");
    }
  }

  ContainerType<std::string, DOF> joint_names;
  Eigen::Vector<unsigned int, DOF> utr_joint_map;  // user joint idx -> robot joint idx
  Eigen::Vector<unsigned int, DOF> rtu_joint_map;  // robot joint idx -> user joint idx

  unsigned int base_link_user_idx;
  std::vector<std::string> link_names;
  std::vector<typename Robot<DOF>::LinkIdx_> utr_link_map;  // user link idx -> robot dummy/sub link idx

 public:
  Eigen::Vector<double, DOF> q;
  Eigen::Vector<double, DOF> qdot;
  Eigen::Vector<double, DOF> qddot;
  Eigen::Vector<double, DOF> tau;
  math::se3v::MatrixType V0{math::se3v::MatrixType::Zero()};
  math::se3v::MatrixType Vdot0{math::se3v::MatrixType::Zero()};

  ContainerType<math::SE3::MatrixType, DOF> E{};  // exponential mapping
  Eigen::Matrix<double, 6, DOF> S;                // S(q_0,...q_i) in root frame

  ContainerType<math::SE3::MatrixType, DOF> T{};  // product of exponential
  Eigen::Matrix<double, 6, DOF> V;
  Eigen::Matrix<double, 6, DOF> Vdot;
  Eigen::Matrix<double, 6, DOF> F;
};

}  // namespace rb::dyn
