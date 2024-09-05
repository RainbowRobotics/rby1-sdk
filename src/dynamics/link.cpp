#include <utility>

#include "rby1-sdk/dynamics/link.h"

using namespace std;
using namespace rb::math;

namespace rb::dyn {

Collision::Collision(std::string name) : name_(std::move(name)) {}

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

}  // namespace rb::dyn