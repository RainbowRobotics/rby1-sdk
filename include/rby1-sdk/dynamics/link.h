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
  Geom(unsigned int coltype = 0, unsigned int colaffinity = 0) : coltype_(coltype), colaffinity_(colaffinity) {}

  virtual ~Geom() {}

  virtual GeomType GetType() const = 0;

  unsigned int GetColtype() const { return coltype_; }

  unsigned int GetColaffinity() const { return colaffinity_; }

  virtual std::pair<bool, double> ComputeMinimumDistance(const math::SE3::MatrixType& T, const Geom& other_geom,
                                                         const math::SE3::MatrixType& other_T) const = 0;

  bool Filter(const Geom& other_geom) const { return Filter(*this, other_geom); }

  static bool Filter(const Geom& geom1, const Geom& geom2) {
    return (geom1.coltype_ & geom2.colaffinity_) || (geom2.coltype_ & geom1.colaffinity_);
  }

 protected:
  unsigned int coltype_;
  unsigned int colaffinity_;
};

class GeomCapsule : public Geom {
 public:
  GeomCapsule(double length, double radius, unsigned int coltype = 0, unsigned int colaffinity = 0);

  GeomCapsule(Eigen::Vector3d sp, Eigen::Vector3d ep, double radius, unsigned int coltype = 0,
              unsigned int colaffinity = 0);

  GeomType GetType() const override;

  std::pair<bool, double> ComputeMinimumDistance(const math::SE3::MatrixType& T, const Geom& other_geom,
                                                 const math::SE3::MatrixType& other_T) const override;

  Eigen::Vector3d GetStartPoint() const;

  Eigen::Vector3d GetEndPoint() const;

  double GetRadius() const;

 private:
  Eigen::Vector3d sp_;
  Eigen::Vector3d ep_;
  double radius_;
};

}  // namespace rb::dyn