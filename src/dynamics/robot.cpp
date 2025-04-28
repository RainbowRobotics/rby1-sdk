#include <utility>

#include "tinyxml2.h"

#include "rby1-sdk/dynamics/robot.h"

using namespace std;
using namespace rb::math;

namespace {

using namespace rb;
using namespace rb::dyn;

RobotConfiguration _load_robot_from_urdf(tinyxml2::XMLDocument& doc, const std::string& base_link_name) {
  using namespace tinyxml2;

  const auto& ToDoubleArray3 = [](const std::string& str) {
    std::stringstream ss(str);
    std::array<double, 3> values{};
    ss >> values[0] >> values[1] >> values[2];
    return values;
  };

  const auto& GetOriginT = [&](XMLElement* origin_element) {
    if (origin_element) {
      std::array<double, 3> rpy{0, 0, 0};
      if (origin_element->FindAttribute("rpy")) {
        rpy = ToDoubleArray3(origin_element->Attribute("rpy"));
      }

      std::array<double, 3> xyz{0, 0, 0};
      if (origin_element->FindAttribute("xyz")) {
        xyz = ToDoubleArray3(origin_element->Attribute("xyz"));
      }

      return math::SE3::T(math::SO3::FromRPY({rpy[0], rpy[1], rpy[2]}), Eigen::Vector3d{xyz[0], xyz[1], xyz[2]});
    }
    return math::SE3::Identity();
  };

  std::string robot_name{};
  std::unordered_map<std::string, std::shared_ptr<Link>> links;
  auto mobile_base = std::make_shared<MobileBase>();
  mobile_base->type = MobileBaseType::kNone;

  XMLElement* robot = doc.FirstChildElement("robot");

  if (robot->FindAttribute("name")) {
    robot_name = robot->Attribute("name");
  }

  XMLElement* link = robot->FirstChildElement("link");
  while (link) {
    std::string name = link->Attribute("name");
    Inertial::MatrixType I;

    XMLElement* inertial_element = link->FirstChildElement("inertial");
    if (inertial_element) {
      double mass = 0.;
      math::SE3::MatrixType T{math::SE3::Identity()};
      double ixx{0.}, ixy{0.}, ixz{0.}, iyy{0.}, iyz{0.}, izz{0.};

      XMLElement* mass_element = inertial_element->FirstChildElement("mass");
      if (mass_element) {
        mass = mass_element->DoubleAttribute("value");
      }

      XMLElement* origin_element = inertial_element->FirstChildElement("origin");
      if (origin_element) {
        T = GetOriginT(origin_element);
      }

      XMLElement* inertia_element = inertial_element->FirstChildElement("inertia");
      if (inertia_element) {
        for (auto& [k, v] : std::vector<std::pair<std::string, double&>>{
                 {"ixx", ixx}, {"ixy", ixy}, {"ixz", ixz}, {"iyy", iyy}, {"iyz", iyz}, {"izz", izz}}) {
          if (inertia_element->FindAttribute(k.c_str())) {
            v = inertia_element->DoubleAttribute(k.c_str());
          }
        }
      }

      I = Inertial::I(mass, ixx, iyy, izz, ixy, ixz, iyz, T);
    }
    links[name] = Link::Make(name, I);

    XMLElement* collision_element = link->FirstChildElement("collision");
    while (collision_element) {
      std::string collision_name;
      if (collision_element->FindAttribute("name")) {
        collision_name = collision_element->Attribute("name");
      }

      math::SE3::MatrixType T{math::SE3::Identity()};
      XMLElement* origin_element = collision_element->FirstChildElement("origin");
      if (origin_element) {
        T = GetOriginT(origin_element);
      }

      std::shared_ptr<Geom> geom;

      XMLElement* geometry_element = collision_element->FirstChildElement("geometry");
      if (geometry_element) {
        // capsule
        XMLElement* capsule_element = geometry_element->FirstChildElement("capsule");
        if (capsule_element) {
          geom = std::make_shared<GeomCapsule>(capsule_element->DoubleAttribute("length", 0),        //
                                               capsule_element->DoubleAttribute("radius", 0),        //
                                               capsule_element->UnsignedAttribute("coltype", 0),     //
                                               capsule_element->UnsignedAttribute("colaffinity", 0)  //
          );
        }
      }

      if (geom) {
        auto collision = std::make_shared<Collision>(collision_name);
        collision->SetOrigin(T);
        collision->AddGeom(geom);
        links[name]->AddCollision(collision);
      }

      collision_element = collision_element->NextSiblingElement("collision");
    }

    link = link->NextSiblingElement("link");
  }

  XMLElement* joint = robot->FirstChildElement("joint");
  while (joint) {
    std::string name = joint->Attribute("name");

    std::string type = joint->Attribute("type");
    if(joint->FindAttribute("in_model_type")) {
      type = joint->Attribute("in_model_type");
    }

    std::string parent_name, child_name;
    math::SE3::MatrixType T{math::SE3::Identity()};

    XMLElement* parent_element = joint->FirstChildElement("parent");
    if (parent_element) {
      if (parent_element->FindAttribute("link")) {
        parent_name = parent_element->Attribute("link");
      } else {
        throw std::runtime_error("There is no link tag in parent element.");
      }
    } else {
      throw std::runtime_error("There is no parent tag in joint.");
    }

    XMLElement* child_element = joint->FirstChildElement("child");
    if (child_element) {
      if (child_element->FindAttribute("link")) {
        child_name = child_element->Attribute("link");
      } else {
        throw std::runtime_error("There is no link tag in child element.");
      }
    } else {
      throw std::runtime_error("There is no child tag in joint.");
    }

    XMLElement* origin_element = joint->FirstChildElement("origin");
    if (origin_element) {
      T = GetOriginT(origin_element);
    }

    XMLElement* axis_element = joint->FirstChildElement("axis");
    std::array<double, 3> axis{0, 0, 0};
    if (axis_element) {
      if (axis_element->FindAttribute("xyz")) {
        axis = ToDoubleArray3(axis_element->Attribute("xyz"));
      }
    } else {
      throw std::runtime_error("There is no axis element in joint.");
    }

    std::shared_ptr<Joint> joint_ptr;
    if (type == "revolute" || type == "continuous") {
      joint_ptr = Joint::MakeRevoluteJoint(name, math::SE3::Identity(), {axis[0], axis[1], axis[2]});
    } else if (type == "prismatic") {
      joint_ptr = Joint::MakePrismaticJoint(name, math::SE3::Identity(), {axis[0], axis[1], axis[2]});
    } else {
      joint_ptr = Joint::MakeFixedJoint(name);
    }
    if (links.find(parent_name) == links.end()) {
      std::stringstream ss;
      ss << "Cannot find the link name (" << parent_name << ")";
      throw std::runtime_error(ss.str());
    }
    if (links.find(child_name) == links.end()) {
      std::stringstream ss;
      ss << "Cannot find the link name (" << child_name << ")";
      throw std::runtime_error(ss.str());
    }
    joint_ptr->ConnectLinks(links[parent_name], links[child_name], T, math::SE3::Identity());

    XMLElement* limit_element = joint->FirstChildElement("limit");
    if (limit_element) {
      if (limit_element->FindAttribute("effort")) {
        joint_ptr->SetLimitTorque(limit_element->DoubleAttribute("effort"));
      }
      if (limit_element->FindAttribute("lower")) {
        joint_ptr->SetLimitQLower(limit_element->DoubleAttribute("lower"));
      }
      if (limit_element->FindAttribute("upper")) {
        joint_ptr->SetLimitQUpper(limit_element->DoubleAttribute("upper"));
      }
      if (limit_element->FindAttribute("velocity")) {
        double v = limit_element->DoubleAttribute("velocity");
        joint_ptr->SetLimitQdotLower(-v);
        joint_ptr->SetLimitQdotUpper(v);
      }
      if (limit_element->FindAttribute("acceleration")) {
        double a = limit_element->DoubleAttribute("acceleration");
        joint_ptr->SetLimitQddotLower(-a);
        joint_ptr->SetLimitQddotUpper(a);
      }
    }

    joint = joint->NextSiblingElement("joint");
  }

  XMLElement* mobile = robot->FirstChildElement("mobile");
  if (mobile) {
    if (mobile->FindAttribute("type")) {
      std::string type = mobile->Attribute("type");

      mobile_base->type = MobileBaseType::kNone;
      if (type == "differential") {
        mobile_base->type = MobileBaseType::kDifferential;
      } else if (type == "mecanum") {
        mobile_base->type = MobileBaseType::kMecanum;
      }
    } else {
      throw std::runtime_error("There is not type in mobile.");
    }

    mobile_base->T = GetOriginT(mobile);

    {
      std::string params = mobile->Attribute("joints");
      std::stringstream ss(params);
      std::string j;
      while (ss >> j) {
        mobile_base->joints.push_back(j);
      }
    }

    {
      std::string params = mobile->Attribute("params");
      std::stringstream ss(params);
      double param;
      while (ss >> param) {
        mobile_base->params.push_back(param);
      }
    }
  }

  if (links.find(base_link_name) == links.end()) {
    throw std::runtime_error("There is no base link name");
  }

  RobotConfiguration rc;

  rc.name = robot_name;
  rc.base_link = links[base_link_name];
  rc.mobile_base = mobile_base;

  return rc;
}

}  // namespace

namespace rb::dyn {

RobotConfiguration LoadRobotFromURDFData(const std::string& model, const std::string& base_link_name) {
  using namespace tinyxml2;

  XMLDocument doc;
  XMLError ok = doc.Parse(model.c_str());
  if (ok != XMLError::XML_SUCCESS) {
    throw std::runtime_error("Cannot parse model data");
  }

  return _load_robot_from_urdf(doc, base_link_name);
}

RobotConfiguration LoadRobotFromURDF(const std::string& path, const std::string& base_link_name) {
  using namespace tinyxml2;

  XMLDocument doc;
  XMLError ok = doc.LoadFile(path.c_str());
  if (ok != XMLError::XML_SUCCESS) {
    throw std::runtime_error("Load URDF failed");
  }

  return _load_robot_from_urdf(doc, base_link_name);
}

}  // namespace rb::dyn