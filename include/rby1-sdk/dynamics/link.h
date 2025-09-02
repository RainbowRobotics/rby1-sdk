#pragma once

#include <Eigen/Core>
#include <memory>

#include "rby1-sdk/dynamics/inertial.h"
#include "rby1-sdk/export.h"
#include "rby1-sdk/math/liegroup.h"

namespace rb::dyn {

template <int DOF>
class Robot;
class Link;
class Joint;
class Collision;
struct CollisionResult;
class Geom;
class GeomCapsule;

enum class GeomType { kCapsule = 0 };

class RBY1_SDK_API Link : public std::enable_shared_from_this<Link> {
 public:
  template <int DOF>
  friend class Robot;

  friend class Joint;

  static std::shared_ptr<Link> Make(std::string name, Inertial::MatrixType I = Inertial::I(1.));

  std::string GetName() const;

  std::weak_ptr<const Joint> GetParentJoint() const;

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

class RBY1_SDK_API Collision : public std::enable_shared_from_this<Collision> {
 public:
  explicit Collision(std::string name);

  std::string GetName() const;

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

class RBY1_SDK_API Geom : public std::enable_shared_from_this<Geom> {
 public:
  Geom(unsigned int coltype = 0, unsigned int colaffinity = 0) : coltype_(coltype), colaffinity_(colaffinity) {}

  virtual ~Geom() = default;

  virtual GeomType GetType() const = 0;

  unsigned int GetColtype() const { return coltype_; }

  unsigned int GetColaffinity() const { return colaffinity_; }

  virtual std::optional<CollisionResult> ComputeMinimumDistance(const math::SE3::MatrixType& T, const Geom& other_geom,
                                                                const math::SE3::MatrixType& other_T) const = 0;

  bool Filter(const Geom& other_geom) const {
    return (coltype_ & other_geom.colaffinity_) || (other_geom.coltype_ & colaffinity_);
  }

 protected:
  unsigned int coltype_;
  unsigned int colaffinity_;
};

class RBY1_SDK_API GeomCapsule : public Geom {
 public:
  GeomCapsule(double length, double radius, unsigned int coltype = 0, unsigned int colaffinity = 0);

  GeomCapsule(Eigen::Vector3d sp, Eigen::Vector3d ep, double radius, unsigned int coltype = 0,
              unsigned int colaffinity = 0);

  GeomType GetType() const override;

  std::optional<CollisionResult> ComputeMinimumDistance(const math::SE3::MatrixType& T, const Geom& other_geom,
                                                        const math::SE3::MatrixType& other_T) const override;

  Eigen::Vector3d GetStartPoint() const;

  Eigen::Vector3d GetEndPoint() const;

  double GetRadius() const;

 private:
  Eigen::Vector3d sp_;
  Eigen::Vector3d ep_;
  double radius_;
};

struct RBY1_SDK_API CollisionResult {
  std::string link1;
  std::string link2;
  Eigen::Vector3d position1;
  Eigen::Vector3d position2;
  double distance;
};

}  // namespace rb::dyn