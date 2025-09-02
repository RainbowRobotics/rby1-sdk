#include <utility>

#include "rby1-sdk/dynamics/link.h"

using namespace std;
using namespace rb::math;

namespace {

std::tuple<double, Eigen::Vector3d, Eigen::Vector3d> compute_min_distance_two_segments(
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& s1,  // first capsule start point, end point
    const std::pair<Eigen::Vector3d, Eigen::Vector3d>& s2   // second capsule start point, end point
) {
  Eigen::Vector3d u = s1.second - s1.first;
  Eigen::Vector3d v = s2.second - s2.first;
  Eigen::Vector3d w = s1.first - s2.first;

  double a = u.dot(u);
  double b = u.dot(v);
  double c = v.dot(v);
  double d = u.dot(w);
  double e = v.dot(w);

  double D = a * c - b * b;

  double sc, sN, sD = D;
  double tc, tN, tD = D;

  if (D < 0.000001) {
    // Segments are (nearly) parallel
    sN = 0.0;
    sD = 1.0;
    tN = e;
    tD = c;
  } else {
    // Compute the closest points on the infinite lines
    sN = (b * e - c * d);
    tN = (a * e - b * d);

    if (sN < 0.0) {
      sN = 0.0;
      tN = e;
      tD = c;
    } else if (sN > sD) {
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {
    tN = 0.0;
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = sD;
    } else {
      sN = -d;
      sD = a;
    }
  } else if (tN > tD) {
    tN = tD;
    if ((-d + b) < 0.0) {
      sN = 0.0;
    } else if ((-d + b) > a) {
      sN = sD;
    } else {
      sN = (-d + b);
      sD = a;
    }
  }

  // Calculate the closest points
  sc = (fabs(sN) < 1e-6 ? 0.0 : sN / sD);
  tc = (fabs(tN) < 1e-6 ? 0.0 : tN / tD);

  Eigen::Vector3d dP = w + sc * u - tc * v;  // Vector between the closest points

  // Return the shortest distance
  return {dP.norm(), s1.first + sc * u, s2.first + tc * v};
}

}  // namespace

namespace rb::dyn {

Collision::Collision(std::string name) : name_(std::move(name)) {}

std::string Collision::GetName() const {
  return name_;
}

void Collision::SetOrigin(const math::SE3::MatrixType& T) {
  T_ = T;
}

math::SE3::MatrixType Collision::GetOrigin() const {
  return T_;
}

void Collision::AddGeom(const std::shared_ptr<Geom>& geom) {
  geoms_.push_back(geom);
}

std::vector<std::shared_ptr<Geom>> Collision::GetGeoms() {
  return geoms_;
}

const std::vector<std::shared_ptr<Geom>>& Collision::GetGeoms() const {
  return geoms_;
}

shared_ptr<Link> Link::Make(string name, Inertial::MatrixType I) {
  return shared_ptr<Link>(new Link(std::move(name), std::move(I)));
}

Link::Link(string name, Inertial::MatrixType I) : name_(std::move(name)), I_(std::move(I)) {}

std::string Link::GetName() const {
  return name_;
}

weak_ptr<const Joint> Link::GetParentJoint() const {
  return parent_joint_;
}

weak_ptr<Joint> Link::GetParentJoint() {
  return parent_joint_;
}

std::vector<shared_ptr<Joint>> Link::GetChildJointList() {
  return child_joints_;
}

const std::vector<shared_ptr<Joint>>& Link::GetChildJointList() const {
  return child_joints_;
}

void Link::AddCollision(const std::shared_ptr<Collision>& collision) {
  collisions_.push_back(collision);
}

std::vector<std::shared_ptr<Collision>> Link::GetCollisions() {
  return collisions_;
}

const std::vector<std::shared_ptr<Collision>>& Link::GetCollisions() const {
  return collisions_;
}

GeomCapsule::GeomCapsule(double length, double radius, unsigned int coltype, unsigned int colaffinity)
    : GeomCapsule({0, 0, length / 2}, {0, 0, -length / 2}, radius, coltype, colaffinity) {}

GeomCapsule::GeomCapsule(Eigen::Vector3d sp, Eigen::Vector3d ep, double radius, unsigned int coltype,
                         unsigned int colaffinity)
    : Geom(coltype, colaffinity), sp_(std::move(sp)), ep_(std::move(ep)), radius_(radius) {}

GeomType GeomCapsule::GetType() const {
  return GeomType::kCapsule;
}

std::optional<CollisionResult> GeomCapsule::ComputeMinimumDistance(const math::SE3::MatrixType& T,
                                                                   const Geom& other_geom,
                                                                   const math::SE3::MatrixType& other_T) const {
  switch (other_geom.GetType()) {
    case GeomType::kCapsule: {
      const auto& other_capsule = dynamic_cast<const GeomCapsule&>(other_geom);
      auto dis = compute_min_distance_two_segments(
          {math::SE3::Multiply(T, GetStartPoint()), math::SE3::Multiply(T, GetEndPoint())},
          {math::SE3::Multiply(other_T, other_capsule.GetStartPoint()),
           math::SE3::Multiply(other_T, other_capsule.GetEndPoint())});
      CollisionResult rv;
      rv.link1 = "";
      rv.link2 = "";
      rv.position1 = std::get<1>(dis);
      rv.position2 = std::get<2>(dis);
      rv.distance = std::get<0>(dis) - (radius_ + other_capsule.GetRadius());
      return rv;
    }
  }

  return {};
}

Eigen::Vector3d GeomCapsule::GetStartPoint() const {
  return sp_;
}

Eigen::Vector3d GeomCapsule::GetEndPoint() const {
  return ep_;
}

double GeomCapsule::GetRadius() const {
  return radius_;
}

}  // namespace rb::dyn