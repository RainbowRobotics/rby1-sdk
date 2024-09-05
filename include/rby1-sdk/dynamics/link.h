#pragma once

#include <Eigen/Core>
#include <memory>

#include "rby1-sdk/dynamics/inertial.h"
#include "rby1-sdk/math/liegroup.h"

namespace rb::dyn {

template <int DOF>
class Robot;
class Link;
class Joint;
class Collision;
class Geom;
class GeomCapsule;

enum class GeomType { kCapsule = 0 };

class Link : public std::enable_shared_from_this<Link> {
 public:
  template <int DOF>
  friend class Robot;

  friend class Joint;

  static std::shared_ptr<Link> Make(std::string name, Inertial::MatrixType I = Inertial::I(1.));

  std::string GetName() const;

  std::weak_ptr<Joint> GetParentJoint();

  std::vector<std::shared_ptr<Joint>> GetChildJointList();

  const std::vector<std::shared_ptr<Joint>>& GetChildJointList() const;

  void AddCollision(const std::shared_ptr<Collision>& collision);

  std::vector<std::shared_ptr<Collision>> GetCollisions();

  const std::vector<std::shared_ptr<Collision>>& GetCollisions() const;

 private:
  Link(std::string name, Inertial::MatrixType I);

 private:
  std::string name_{};
  Inertial::MatrixType I_{};

  std::weak_ptr<Joint> parent_joint_;
  std::vector<std::shared_ptr<Joint>> child_joints_;

  std::vector<std::shared_ptr<Collision>> collisions_;
};

class Collision : public std::enable_shared_from_this<Collision> {
 public:
  explicit Collision(std::string name);

  void SetOrigin(const math::SE3::MatrixType& T);

  math::SE3::MatrixType GetOrigin() const;

  void AddGeom(const std::shared_ptr<Geom>& geom);

  std::vector<std::shared_ptr<Geom>> GetGeoms();

  const std::vector<std::shared_ptr<Geom>>& GetGeoms() const;

 private:
  std::string name_;
  math::SE3::MatrixType T_{math::SE3::Identity()};
  std::vector<std::shared_ptr<Geom>> geoms_;
};

class Geom : public std::enable_shared_from_this<Geom> {
 public:
  virtual GeomType GetType() const = 0;
};

class GeomCapsule : public Geom {
 public:
  GeomCapsule(Eigen::Vector3d sp, Eigen::Vector3d ep, double radius)
      : sp_(std::move(sp)), ep_(std::move(ep)), radius_(radius) {}

  GeomType GetType() const override { return GeomType::kCapsule; }

  Eigen::Vector3d GetStartPoint() const { return sp_; }

  Eigen::Vector3d GetEndPoint() const { return ep_; }

  double GetRadius() const { return radius_; }

 private:
  Eigen::Vector3d sp_;
  Eigen::Vector3d ep_;
  double radius_;
};

}  // namespace rb::dyn