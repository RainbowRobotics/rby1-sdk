#pragma once

#include "rby1-sdk/dynamics/robot.h"

class SampleRobot : public rb::dyn::Robot<6> {
 public:
  SampleRobot() {
    using namespace rb::dyn;
    using namespace rb::math;

    auto base = Link::Make("base");
    auto link1 = Link::Make("link1");
    auto link2 = Link::Make("link2");
    auto link3 = Link::Make("link3");
    auto link4 = Link::Make("link4");
    auto link5 = Link::Make("link5");
    auto link6 = Link::Make("link6");
    auto tooltip = Link::Make("tooltip", Inertial::I(0));

    Joint::MakeRevoluteJoint("joint1")->ConnectLinks(  //
        base, link1,                                   //
        SE3::T(Eigen::Vector3d{0, 0, 1}),              //
        SE3::T(Eigen::Vector3d{0, 0, 1}));
    Joint::MakeRevoluteJoint("joint2")->ConnectLinks(             //
        link1, link2,                                             //
        SE3::T(SO3::RotX(-kPiHalf), Eigen::Vector3d{0.2, 0, 0}),  //
        SE3::Identity());
    Joint::MakeRevoluteJoint("joint3")->ConnectLinks(  //
        link2, link3,                                  //
        SE3::T(Eigen::Vector3d{3, 0, 0}),              //
        SE3::Identity());
    Joint::MakeRevoluteJoint("joint4")->ConnectLinks(               //
        link3, link4,                                               //
        SE3::T(SO3::RotY(-kPiHalf), Eigen::Vector3d{0.4, 0.5, 0}),  //
        SE3::T(SO3::RotX(-kPiHalf), Eigen::Vector3d{0.4, 0.5, 0}));
    Joint::MakeRevoluteJoint("joint5")->ConnectLinks(  //
        link4, link5,                                  //
        SE3::T(SO3::RotX(kPiHalf)),                    //
        SE3::T(SO3::RotX(kPiHalf)));
    Joint::MakeRevoluteJoint("joint6")->ConnectLinks(  //
        link5, link6,                                  //
        SE3::T(SO3::RotX(-kPi)),                       //
        SE3::Identity());
    Joint::MakeFixedJoint("tooltip")->ConnectLinks(  //
        link6, tooltip,                              //
        SE3::T(Eigen::Vector3d{0.1, 0.1, 0}),        //
        SE3::T(Eigen::Vector3d{0, 0, 0.125}));

    RobotConfiguration rc;
    rc.name = "sample";
    rc.base_link = base;
    rc.mobile_base = nullptr;
    Build(rc);
  }

 private:
};