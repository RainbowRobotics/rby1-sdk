#include <utility>

#include "rby1-sdk/dynamics/robot.h"

using namespace std;
using namespace rb::math;

namespace rb::dyn {

shared_ptr<Joint> Joint::Make(string name, se3v::MatrixType S) {
  return shared_ptr<Joint>(new Joint(std::move(name), std::move(S)));
}

shared_ptr<Joint> Joint::MakeRevoluteJoint(string name, const SE3::MatrixType& T, const Eigen::Vector3d& axis) {
  return Make(std::move(name), SE3::Ad(T, {axis(0), axis(1), axis(2), 0, 0, 0}));
}

shared_ptr<Joint> Joint::MakePrismaticJoint(std::string name, const math::SE3::MatrixType& T,
                                            const Eigen::Vector3d& axis) {
  return Make(std::move(name), SE3::Ad(T, {0, 0, 0, axis(0), axis(1), axis(2)}));
}

shared_ptr<Joint> Joint::MakeFixedJoint(string name) {
  return shared_ptr<Joint>(new Joint(std::move(name)));
}

std::string Joint::GetName() const {
  return name_;
}

void Joint::ConnectLinks(const shared_ptr<Link>& parent_link, const shared_ptr<Link>& child_link,
                         const SE3::MatrixType& T_pj, const SE3::MatrixType& T_jc) {
  Disconnect();

  if (!child_link->parent_joint_.expired()) {
    auto j = child_link->parent_joint_.lock();
    j->Disconnect();
  }

  parent_link_ = parent_link;
  child_link_ = child_link;

  T_pj_ = T_pj;
  T_jc_ = T_jc;

  parent_link->child_joints_.push_back(this->shared_from_this());
  child_link->parent_joint_ = this->weak_from_this();
}

void Joint::Disconnect() {
  if (!parent_link_.expired()) {
    auto parent_lock = parent_link_.lock();
    auto& joints = parent_lock->child_joints_;
    joints.erase(remove_if(joints.begin(), joints.end(), [n = name_](auto& j) { return j->name_ == n; }), joints.end());
  }
  if (child_link_) {
    child_link_->parent_joint_.reset();
  }
}

void Joint::SetLimitQ(double lower, double upper) {
  limit_q_lower_ = lower;
  limit_q_upper_ = upper;
}

void Joint::SetLimitQdot(double lower, double upper) {
  limit_qdot_lower_ = lower;
  limit_qdot_upper_ = upper;
}

void Joint::SetLimitQddot(double lower, double upper) {
  limit_qddot_lower_ = lower;
  limit_qddot_upper_ = upper;
}

void Joint::SetLimitTorque(double value) {
  limit_torque_ = value;
}

double Joint::GetLimitQLower() const {
  return limit_q_lower_;
}

double Joint::GetLimitQUpper() const {
  return limit_q_upper_;
}

double Joint::GetLimitQdotLower() const {
  return limit_qdot_lower_;
}

double Joint::GetLimitQdotUpper() const {
  return limit_qdot_upper_;
}

double Joint::GetLimitQddotLower() const {
  return limit_qddot_lower_;
}

double Joint::GetLimitQddotUpper() const {
  return limit_qddot_upper_;
}

double Joint::GetLimitTorque() const {
  return limit_torque_;
}

void Joint::SetLimitQLower(double val) {
  limit_q_lower_ = val;
}

void Joint::SetLimitQUpper(double val) {
  limit_q_upper_ = val;
}

void Joint::SetLimitQdotLower(double val) {
  limit_qdot_lower_ = val;
}

void Joint::SetLimitQdotUpper(double val) {
  limit_qdot_upper_ = val;
}

void Joint::SetLimitQddotLower(double val) {
  limit_qddot_lower_ = val;
}

void Joint::SetLimitQddotUpper(double val) {
  limit_qddot_upper_ = val;
}

weak_ptr<const Link> Joint::GetParentLink() const {
  return parent_link_;
}

weak_ptr<Link> Joint::GetParentLink() {
  return parent_link_;
}

shared_ptr<Link> Joint::GetChildLink() {
  return child_link_;
}

shared_ptr<const Link> Joint::GetChildLink() const {
  return child_link_;
}

bool Joint::IsFixed() const {
  return fixed_;
}

Joint::Joint(string name) : name_(std::move(name)), fixed_(true) {}

Joint::Joint(string name, se3v::MatrixType S) : name_(std::move(name)), S_(std::move(S)), fixed_(false) {}

}  // namespace rb::dyn
