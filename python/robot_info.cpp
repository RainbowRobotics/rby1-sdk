#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iomanip>
#include <string>
#include <sstream>

#include "common.h"
#include "rby1-sdk/robot_info.h"

namespace py = pybind11;
using namespace rb;

void bind_battery_info(py::module_& m) {
  py::class_<BatteryInfo>(m, "BatteryInfo")  //
      .def(py::init<>())
      .def("__repr__", [](const BatteryInfo& self) { return "BatteryInfo()"; });
}

void bind_power_info(py::module_& m) {
  py::class_<PowerInfo>(m, "PowerInfo")  //
      .def(py::init<>())
      .def_readonly("name", &PowerInfo::name)
      .def("__repr__", [](const PowerInfo& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "PowerInfo("                                       //
           << "name=" << self.name                               //
           << ")";
        return ss.str();
      });
}

void bind_emo_info(py::module_& m) {
  py::class_<EMOInfo>(m, "EMOInfo")  //
      .def(py::init<>())
      .def_readonly("name", &EMOInfo::name)
      .def("__repr__", [](const EMOInfo& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "EMOInfo("                                         //
           << "name=" << self.name                               //
           << ")";
        return ss.str();
      });
}

void bind_joint_info(py::module_& m) {
  py::class_<JointInfo>(m, "JointInfo")
      .def(py::init<>())
      .def_readonly("name", &JointInfo::name)
      .def_readonly("has_brake", &JointInfo::has_brake)
      .def_readonly("product_name", &JointInfo::product_name)
      .def_readonly("firmware_version", &JointInfo::firmware_version)
      .def("__repr__", [](const JointInfo& self) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)      //
           << "JointInfo("                                           //
           << "name=" << self.name                                   //
           << ", has_brake=" << (self.has_brake ? "True" : "False")  //
           << ", product_name=" << self.product_name  //
           << ", firmware_version=" << self.firmware_version  //
           << ")";
        return ss.str();
      });
}

void bind_robot_info(py::module_& m) {
  py::class_<RobotInfo>(m, "RobotInfo")
      .def(py::init<>())
      .def_readonly("robot_version", &RobotInfo::robot_version)
      .def_readonly("sdk_commit_id", &RobotInfo::sdk_commit_id)
      .def_readonly("battery_info", &RobotInfo::battery_info)
      .def_readonly("power_infos", &RobotInfo::power_infos)
      .def_readonly("emo_infos", &RobotInfo::emo_infos)
      .def_readonly("degree_of_freedom", &RobotInfo::degree_of_freedom)
      .def_readonly("joint_infos", &RobotInfo::joint_infos)
      .def_readonly("mobility_joint_idx", &RobotInfo::mobility_joint_idx)
      .def_readonly("body_joint_idx", &RobotInfo::body_joint_idx)
      .def_readonly("head_joint_idx", &RobotInfo::head_joint_idx)
      .def("__repr__", [](const RobotInfo& self) {
        const auto& print = [](std::stringstream& out, const std::vector<unsigned>& vec) {
          out << "[";
          for (int i = 0; i < (int)vec.size(); i++) {
            if (i != 0) {
              out << ", ";
            }
            out << vec[i];
          }
          out << "]";
        };
        std::stringstream ss;
        ss << std::fixed << std::setprecision(kDoublePrecision)  //
           << "RobotInfo("                                       //
           << "robot_version='" << self.robot_version << "'"     //
           << ", sdk_commit_id='" << self.sdk_commit_id << "'"   //
           << ", battery_info={}"                                //
           << ", num_power_infos=" << self.power_infos.size()    //
           << ", num_emo_infos=" << self.emo_infos.size()        //
           << ", degree_of_freedom=" << self.degree_of_freedom   //
           << ", num_joint_infos=" << self.joint_infos.size()    //
           << ", mobility_joint_idx=";
        print(ss, self.mobility_joint_idx);
        ss << ", body_joint_idx=";
        print(ss, self.body_joint_idx);
        ss << ", head_joint_idx=";
        print(ss, self.head_joint_idx);
        ss << ")";
        return ss.str();
      });
}

void pybind11_robot_info(py::module_& m) {
  bind_battery_info(m);
  bind_power_info(m);
  bind_emo_info(m);
  bind_joint_info(m);
  bind_robot_info(m);
}