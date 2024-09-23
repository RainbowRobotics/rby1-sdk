#include <chrono>
#include <iostream>
#include <random>
#include <tuple>
#include "rby1-sdk/dynamics/robot.h"

using namespace rb;
using namespace rb::dyn;
using namespace rb::math;

// 0 1 2 3 4 5 -> body
// 6 7 8 9 10 11 12 -> right arm
// 13 14 15 16 17 18 19 -> left arm

enum LinkName {
  BASE = 0,

  AR,
  AP,
  KN,
  HP,
  HR,
  HY,

  RSP,
  RSR,
  RSY,
  REB,
  RWY,
  RWP,
  RH,

  LSP,
  LSR,
  LSY,
  LEB,
  LWY,
  LWP,
  LH,

};

std::optional<LinkName> linkname_strig_to_enum(std::string str) {

  LinkName link_name_enum;

  if (str == "base") {
    link_name_enum = LinkName::BASE;
  } else if (str == "Link_1_Anckle_Roll") {
    link_name_enum = LinkName::AR;
  } else if (str == "Link_2_Anckle_Pitch") {
    link_name_enum = LinkName::AP;
  } else if (str == "Link_3_Knee_Pitch") {
    link_name_enum = LinkName::KN;
  } else if (str == "Link_4_Hip_Pitch") {
    link_name_enum = LinkName::HP;
  } else if (str == "Link_5_Hip_Roll") {
    link_name_enum = LinkName::HR;
  } else if (str == "Link_6_Hip_Yaw") {
    link_name_enum = LinkName::HY;
  } else if (str == "Link_7_Right_Shoulder_Pitch") {
    link_name_enum = LinkName::RSP;
  } else if (str == "Link_8_Right_Shoulder_Roll") {
    link_name_enum = LinkName::RSR;
  } else if (str == "Link_9_Right_Shoulder_Yaw") {
    link_name_enum = LinkName::RSY;
  } else if (str == "Link_10_Right_Elbow_Pitch") {
    link_name_enum = LinkName::REB;
  } else if (str == "Link_11_Right_Wrist_Yaw1") {
    link_name_enum = LinkName::RWY;
  } else if (str == "Link_12_Right_Wrist_Pitch") {
    link_name_enum = LinkName::RWP;
  } else if (str == "Link_13_Right_ToolFlange") {
    link_name_enum = LinkName::RH;
  } else if (str == "Link_14_Left_Shoulder_Pitch") {
    link_name_enum = LinkName::LSP;
  } else if (str == "Link_15_Left_Shoulder_Roll") {
    link_name_enum = LinkName::LSR;
  } else if (str == "Link_16_Left_Shoulder_Yaw") {
    link_name_enum = LinkName::LSY;
  } else if (str == "Link_17_Left_Elbow_Pitch") {
    link_name_enum = LinkName::LEB;
  } else if (str == "Link_18_Left_Wrist_Yaw1") {
    link_name_enum = LinkName::LWY;
  } else if (str == "Link_19_Left_Wrist_Pitch") {
    link_name_enum = LinkName::LWP;
  } else if (str == "Link_20_Left_ToolFlange") {
    link_name_enum = LinkName::LH;
  } else {
    return {};
  }

  return link_name_enum;
}

std::tuple<double, Eigen::Vector3d, Eigen::Vector3d> ComputeMinimumDistanceBetweenSegments(
    const std::pair<Eigen::Vector3d, Eigen::Vector3d> s1, const std::pair<Eigen::Vector3d, Eigen::Vector3d> s2) {

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

std::tuple<int, std::pair<LinkName, LinkName>, std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>>
ComputeSelfCollision(std::shared_ptr<Robot<22>> robot, std::shared_ptr<State<22>> state,
                     Eigen::Matrix<double, 22, 1> q_joint,
                     std::vector<std::tuple<LinkName, LinkName, double>> pair_and_margins) {
  state->SetQ(q_joint);
  robot->ComputeForwardKinematics(state);

  int is_collision = false;
  std::pair<LinkName, LinkName> minimum_distance_link_pair;
  std::tuple<double, Eigen::Vector3d, Eigen::Vector3d> minimum_distance_skeleton_info;
  double minimum_distance = 1e6;

  for (const auto& pair_and_margin : pair_and_margins) {

    auto [link_index1, link_index2, r_margin] = pair_and_margin;
    std::vector<std::pair<LinkName, std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>>> geom_first;
    std::vector<std::pair<LinkName, std::tuple<Eigen::Vector3d, Eigen::Vector3d, double>>> geom_second;

    for (const auto& collision : robot->GetLink(state, link_index1)->GetCollisions()) {
      for (const auto& geom : collision->GetGeoms()) {
        if (geom->GetType() == GeomType::kCapsule) {
          auto g = std::static_pointer_cast<GeomCapsule>(geom);
          Eigen::Matrix<double, 4, 4> T = robot->ComputeTransformation(state, LinkName::BASE, link_index1);
          Eigen::Vector3d sp = T.block(0, 0, 3, 3) * g->GetStartPoint() + T.block(0, 3, 3, 1);
          Eigen::Vector3d ep = T.block(0, 0, 3, 3) * g->GetEndPoint() + T.block(0, 3, 3, 1);
          geom_first.push_back(std::make_pair(link_index1, std::make_tuple(sp, ep, g->GetRadius())));
        }
      }
    }

    for (const auto& collision : robot->GetLink(state, link_index2)->GetCollisions()) {
      for (const auto& geom : collision->GetGeoms()) {
        if (geom->GetType() == GeomType::kCapsule) {
          auto g = std::static_pointer_cast<GeomCapsule>(geom);
          Eigen::Matrix<double, 4, 4> T = robot->ComputeTransformation(state, LinkName::BASE, link_index2);
          Eigen::Vector3d sp = T.block(0, 0, 3, 3) * g->GetStartPoint() + T.block(0, 3, 3, 1);
          Eigen::Vector3d ep = T.block(0, 0, 3, 3) * g->GetEndPoint() + T.block(0, 3, 3, 1);
          geom_second.push_back(std::make_pair(link_index2, std::make_tuple(sp, ep, g->GetRadius())));
        }
      }
    }

    for (const auto& geom1 : geom_first) {
      for (const auto& geom2 : geom_second) {
        auto [sp1, ep1, raidus1] = geom1.second;
        auto [sp2, ep2, raidus2] = geom2.second;

        auto [dist, p1, p2] = ComputeMinimumDistanceBetweenSegments(std::make_pair(sp1, ep1), std::make_pair(sp2, ep2));

        if (dist < raidus1 + raidus2 + r_margin) {
          is_collision = true;
        }

        if (dist < minimum_distance) {
          minimum_distance = dist - raidus1 - raidus2 - r_margin;
          minimum_distance_link_pair = std::make_pair(geom1.first, geom2.first);
          minimum_distance_skeleton_info = std::make_tuple(dist - raidus1 - raidus2 - r_margin, p1, p2);
        }
      }
    }
  }

  std::tuple<int, std::pair<LinkName, LinkName>, std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> ret;
  ret = std::make_tuple(is_collision, minimum_distance_link_pair, minimum_distance_skeleton_info);

  return ret;
}

int main() {
  auto robot = std::make_shared<Robot<22>>(LoadRobotFromURDF(PATH "/sample_model.urdf", "base"));
  auto state = robot->MakeState<std::vector<std::string>, std::vector<std::string>>(
      {"base",
       "Link_1_Anckle_Roll",
       "Link_2_Anckle_Pitch",
       "Link_3_Knee_Pitch",
       "Link_4_Hip_Pitch",
       "Link_5_Hip_Roll",
       "Link_6_Hip_Yaw",
       "Link_7_Right_Shoulder_Pitch",
       "Link_8_Right_Shoulder_Roll",
       "Link_9_Right_Shoulder_Yaw",
       "Link_10_Right_Elbow_Pitch",
       "Link_11_Right_Wrist_Yaw1",
       "Link_12_Right_Wrist_Pitch",
       "Link_13_Right_ToolFlange",
       "Link_14_Left_Shoulder_Pitch",
       "Link_15_Left_Shoulder_Roll",
       "Link_16_Left_Shoulder_Yaw",
       "Link_17_Left_Elbow_Pitch",
       "Link_18_Left_Wrist_Yaw1",
       "Link_19_Left_Wrist_Pitch",
       "Link_20_Left_ToolFlange",
       "wheel_r",
       "wheel_l",
       "ee_right",
       "ee_finger_r1",
       "ee_finger_r2",
       "ee_left",
       "ee_finger_l1",
       "ee_finger_l2"},
      {"torso_1",     "torso_2",     "torso_3",     "torso_4",     "torso_5",     "torso_6",     "right_arm_1",
       "right_arm_2", "right_arm_3", "right_arm_4", "right_arm_5", "right_arm_6", "right_arm_7", "left_arm_1",
       "left_arm_2",  "left_arm_3",  "left_arm_4",  "left_arm_5",  "left_arm_6",  "left_arm_7"});

  Eigen::Matrix<double, 22, 1> q_joint;
  q_joint.setZero();
  state->SetQ(q_joint);
  robot->ComputeForwardKinematics(state);

  std::vector<std::tuple<LinkName, LinkName, double>> collision_pair_and_margins;

  if (1) {
    // AR ↔ RWP LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AR, LinkName::RWP, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AR, LinkName::LWP, 0.01));

    // AP ↔ REB RWY1 RWP LEB LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AP, LinkName::REB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AP, LinkName::RWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AP, LinkName::RWP, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AP, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AP, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::AP, LinkName::LWP, 0.01));

    // NK ↔ REB RWY1 RWP LEB LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::KN, LinkName::RWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::KN, LinkName::REB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::KN, LinkName::RWP, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::KN, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::KN, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::KN, LinkName::LWP, 0.01));

    // HR ↔ REB RWY1 RWP LEB LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HR, LinkName::REB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HR, LinkName::RWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HR, LinkName::RWP, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HR, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HR, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HR, LinkName::LWP, 0.01));

    // HY ↔ REB RWY1 RWP LEB LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HY, LinkName::REB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HY, LinkName::RWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HY, LinkName::RWP, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HY, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HY, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::HY, LinkName::LWP, 0.01));

    // RSR ↔ LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RSR, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RSR, LinkName::LWP, 0.01));

    // RSY ↔ LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RSY, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RSY, LinkName::LWP, 0.01));

    // REB ↔ LEB LWY1 LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::REB, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::REB, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::REB, LinkName::LWP, 0.01));

    // RWY1 ↔ LSR, LSY, LEB, LWY1, LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWY, LinkName::LSR, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWY, LinkName::LSY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWY, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWY, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWY, LinkName::LWP, 0.01));

    // RWP ↔ LSR, LSY, LEB, LWY1, LWP
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWP, LinkName::LSR, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWP, LinkName::LSY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWP, LinkName::LEB, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWP, LinkName::LWY, 0.01));
    collision_pair_and_margins.push_back(std::make_tuple(LinkName::RWP, LinkName::LWP, 0.01));
  }

  std::cout << "number of collision pairs :" << collision_pair_and_margins.size() << std::endl;
  auto start = std::chrono::steady_clock::now();

  auto [is_collision, collision_pair, collision_info] =
      ComputeSelfCollision(robot, state, q_joint, collision_pair_and_margins);

  std::cout << "Duration: "
            << (double)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start)
                       .count() /
                   1.e6
            << " ms" << std::endl;

  std::cout << "is_collision : " << is_collision << std::endl;
  std::cout << "collision_pair : " << collision_pair.first << ", " << collision_pair.second << std::endl;

  auto [min, p1, p2] = collision_info;
  std::cout << "collision_info : \n";
  std::cout << "min : " << min << std::endl;
  std::cout << "p1 : " << p1.transpose() << std::endl;
  std::cout << "p2 : " << p2.transpose() << std::endl;

  return 0;
}
