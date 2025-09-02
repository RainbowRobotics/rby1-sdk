#pragma once

#include <Eigen/Core>
#include <memory>

#include "rby1-sdk/export.h"
#include "rby1-sdk/math/liegroup.h"

namespace rb::dyn {

template <int DOF>
class Robot;
class Link;
class Joint;

class RBY1_SDK_API Joint : public std::enable_shared_from_this<Joint> {
 public:
  template <int DOF>
  friend class Robot;

  static std::shared_ptr<Joint> Make(std::string name, math::se3v::MatrixType S);

  static std::shared_ptr<Joint> MakeRevoluteJoint(std::string name,
                                                  const math::SE3::MatrixType& T = math::SE3::Identity(),
                                                  const Eigen::Vector3d& axis = {0, 0, 1});

  static std::shared_ptr<Joint> MakePrismaticJoint(std::string name,
                                                   const math::SE3::MatrixType& T = math::SE3::Identity(),
                                                   const Eigen::Vector3d& axis = {0, 0, 1});

  static std::shared_ptr<Joint> MakeFixedJoint(std::string name);

  std::string GetName() const;

  void ConnectLinks(const std::shared_ptr<Link>& parent_link, const std::shared_ptr<Link>& child_link,
                    const math::SE3::MatrixType& T_pj = math::SE3::Identity(),
                    const math::SE3::MatrixType& T_jc = math::SE3::Identity());

  void Disconnect();

  void SetLimitQ(double lower, double upper);

  void SetLimitQdot(double lower, double upper);

  void SetLimitQddot(double lower, double upper);

  void SetLimitTorque(double value);

  double GetLimitQLower() const;

  double GetLimitQUpper() const;

  double GetLimitQdotLower() const;

  double GetLimitQdotUpper() const;

  double GetLimitQddotLower() const;

  double GetLimitQddotUpper() const;

  double GetLimitTorque() const;

  void SetLimitQLower(double val);

  void SetLimitQUpper(double val);

  void SetLimitQdotLower(double val);

  void SetLimitQdotUpper(double val);

  void SetLimitQddotLower(double val);

  void SetLimitQddotUpper(double val);

  std::weak_ptr<const Link> GetParentLink() const;

  std::weak_ptr<Link> GetParentLink();

  std::shared_ptr<Link> GetChildLink();

  std::shared_ptr<const Link> GetChildLink() const;

  bool IsFixed() const;

 private:
  explicit Joint(std::string name);

  Joint(std::string name, math::se3v::MatrixType S);

 private:
  std::string name_{};
  bool fixed_;
  math::se3v::MatrixType S_;
  double limit_torque_{(std::numeric_limits<double>::max)()};
  double limit_q_lower_{-(std::numeric_limits<double>::max)()};
  double limit_q_upper_{(std::numeric_limits<double>::max)()};
  double limit_qdot_lower_{-(std::numeric_limits<double>::max)()};
  double limit_qdot_upper_{(std::numeric_limits<double>::max)()};
  //  double limit_qddot_lower_{-(std::numeric_limits<double>::max)()};
  //  double limit_qddot_upper_{(std::numeric_limits<double>::max)()};
  double limit_qddot_lower_{-10.};
  double limit_qddot_upper_{10.};  // (rad/s^2)

  std::weak_ptr<Link> parent_link_;
  std::shared_ptr<Link> child_link_{nullptr};
  math::SE3::MatrixType T_pj_, T_jc_;
};

}  // namespace rb::dyn