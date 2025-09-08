#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iomanip>
#include <sstream>
#include <string>

#include "common.h"
#include "print_helper.h"
#include "rby1-sdk/robot_info.h"

namespace py = pybind11;
using namespace rb;

void bind_battery_info(py::module_& m) {
  py::class_<BatteryInfo>(m, "BatteryInfo", R"doc(
Battery information.

This class represents battery information for the robot.
**Currently contains no specific attributes.**
)doc")
      .def(py::init<>(), R"doc(
Construct an ``BatteryInfo`` instance.
)doc")
      .def("__repr__", [](const BatteryInfo&) { return std::string("BatteryInfo()"); })
      .def("__str__", [](const BatteryInfo&) { return std::string("BatteryInfo()"); });
}

void bind_power_info(py::module_& m) {
  py::class_<PowerInfo>(m, "PowerInfo", R"doc(
Power information

Information about robot-controlled external (auxiliary) power outputs.

Attributes
----------
name : str
    Name/label of an externally controllable power output (e.g., "5v", "12v", "24v").
)doc")
      .def(py::init<>(), R"doc(
Construct an ``PowerInfo`` instance.
)doc")
      .def_readonly("name", &PowerInfo::name)
      .def("__repr__",
           [](const PowerInfo& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             ss << "PowerInfo(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "name=" << inline_obj(py::cast(self.name));
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const PowerInfo& self) {
        std::ostringstream ss;
        ss << "PowerInfo(name=" << self.name << ")";
        return ss.str();
      });
}

void bind_emo_info(py::module_& m) {
  py::class_<EMOInfo>(m, "EMOInfo", R"doc(
Emergency stop button information.

This class represents information about an emergency stop
button connected to the robot.

Attributes
----------
name : str
    Name identifier for the emergency stop button.
)doc")
      .def(py::init<>(), R"doc(
Construct an ``EMOInfo`` instance with default values.
)doc")
      .def_readonly("name", &EMOInfo::name)
      .def("__repr__",
           [](const EMOInfo& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             ReprStream ss;

             ss << "EMOInfo(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "name=" << inline_obj(py::cast(self.name));
             ss << (ml ? "\n" : "");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const EMOInfo& self) {
        std::ostringstream ss;
        ss << "EMOInfo(name=" << self.name << ")";
        return ss.str();
      });
}

void bind_joint_info(py::module_& m) {
  py::class_<JointInfo>(m, "JointInfo", R"doc(
Joint information.

Stores hardware/firmware metadata for a single joint.

Attributes
----------
name : str
    Joint name (e.g., "right_arm_2").
has_brake : bool
    Whether the joint includes a brake.
product_name : str
    Actuator/driver product name.
firmware_version : str
    Firmware revision running on the motor driver / servo drive.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``JointInfo`` with default-initialized fields.
)doc")
      .def_readonly("name", &JointInfo::name)
      .def_readonly("has_brake", &JointInfo::has_brake)
      .def_readonly("product_name", &JointInfo::product_name)
      .def_readonly("firmware_version", &JointInfo::firmware_version)
      .def("__repr__",
           [](const JointInfo& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";
             std::ostringstream out;
             out << "JointInfo(" << FIRST                                                       //
                 << "name=" << inline_obj(py::cast(self.name)) << SEP                           //
                 << "has_brake=" << (self.has_brake ? "True" : "False") << SEP                  //
                 << "product_name=" << inline_obj(py::cast(self.product_name)) << SEP           //
                 << "firmware_version=" << inline_obj(py::cast(self.firmware_version)) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const JointInfo& self) {
        std::ostringstream ss;
        ss << "JointInfo(name=" << self.name << ", brake=" << (self.has_brake ? "True" : "False")
           << ", product=" << self.product_name << ", fw=" << self.firmware_version << ")";
        return ss.str();
      });
}

void bind_robot_info(py::module_& m) {
  py::class_<RobotInfo>(m, "RobotInfo", R"doc(
Robot information and configuration.

This class provides comprehensive information about the robot
including version details, hardware configuration, and joint
specifications.

Attributes
----------
version : str
    Robot software version.
sdk_version : str
    SDK version.
robot_version : str
    Robot version (deprecated, use robot_model_name instead).
robot_model_name : str
    Name of the robot model.
robot_model_version : str
    Version of the robot model.
sdk_commit_id : str
    SDK commit identifier (deprecated, use sdk_version instead).
battery_info : BatteryInfo
    Battery information.
power_infos : list[PowerInfo]
    Robot-controlled external (auxiliary) power outputs (e.g., "5V", "12V", "24V").
emo_infos : list[EMOInfo]
    List of emergency stop button information.
degree_of_freedom : int
    Total number of degrees of freedom.
joint_infos : list[JointInfo]
    List of joint information.
mobility_joint_idx : list[int]
    Indices of mobility joints (e.g., wheels).
body_joint_idx : list[int]
    Indices of body joints.
head_joint_idx : list[int]
    Indices of head joints.
torso_joint_idx : list[int]
    Indices of torso joints.
right_arm_joint_idx : list[int]
    Indices of right arm joints.
left_arm_joint_idx : list[int]
    Indices of left arm joints.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``RobotInfo`` instance with default values.
)doc")
      .def_readonly("version", &RobotInfo::version)
      .def_readonly("sdk_version", &RobotInfo::sdk_version)
      .def_readonly("robot_version", &RobotInfo::robot_version)
      .def_readonly("robot_model_name", &RobotInfo::robot_model_name)
      .def_readonly("robot_model_version", &RobotInfo::robot_model_version)
      .def_readonly("sdk_commit_id", &RobotInfo::sdk_commit_id)
      .def_readonly("battery_info", &RobotInfo::battery_info)
      .def_readonly("power_infos", &RobotInfo::power_infos)
      .def_readonly("emo_infos", &RobotInfo::emo_infos)
      .def_readonly("degree_of_freedom", &RobotInfo::degree_of_freedom)
      .def_readonly("joint_infos", &RobotInfo::joint_infos)
      .def_readonly("mobility_joint_idx", &RobotInfo::mobility_joint_idx)
      .def_readonly("body_joint_idx", &RobotInfo::body_joint_idx)
      .def_readonly("head_joint_idx", &RobotInfo::head_joint_idx)
      .def_readonly("torso_joint_idx", &RobotInfo::torso_joint_idx)
      .def_readonly("right_arm_joint_idx", &RobotInfo::right_arm_joint_idx)
      .def_readonly("left_arm_joint_idx", &RobotInfo::left_arm_joint_idx)
      .def("__repr__",
           [](const RobotInfo& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();

             auto vec_to_str = [](const std::vector<unsigned>& v) {
               std::ostringstream s;
               s << "[";
               for (size_t i = 0; i < v.size(); ++i) {
                 if (i) {
                   s << ", ";
                 }
                 s << v[i];
               }
               s << "]";
               return s.str();
             };

             ReprStream ss;

             ss << "RobotInfo(";
             ss << prefix(ml ? "  " : "");

             ss << (ml ? "\n" : "");

             ss << "version=" << inline_obj(py::cast(self.version)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "sdk_version=" << inline_obj(py::cast(self.sdk_version)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "robot_version=" << inline_obj(py::cast(self.robot_version)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "sdk_commit_id=" << inline_obj(py::cast(self.sdk_commit_id)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "robot_model_name=" << inline_obj(py::cast(self.robot_model_name)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "robot_model_version=" << inline_obj(py::cast(self.robot_model_version)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "battery_info=" << inline_obj(py::cast(self.battery_info)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "num_power_infos=" << inline_obj(py::cast(self.power_infos.size())) << ",";
             ss << (ml ? "\n" : " ");

             ss << "num_emo_infos=" << inline_obj(py::cast(self.emo_infos.size())) << ",";
             ss << (ml ? "\n" : " ");

             ss << "degree_of_freedom=" << self.degree_of_freedom << ",";
             ss << (ml ? "\n" : " ");

             ss << "joint_infos=" << inline_obj(py::cast(self.joint_infos)) << ",";
             ss << (ml ? "\n" : " ");

             ss << "mobility_joint_idx=" << vec_to_str(self.mobility_joint_idx) << ",";
             ss << (ml ? "\n" : " ");

             ss << "body_joint_idx=" << vec_to_str(self.body_joint_idx) << ",";
             ss << (ml ? "\n" : " ");

             ss << "head_joint_idx=" << vec_to_str(self.head_joint_idx) << ",";
             ss << (ml ? "\n" : " ");

             ss << "torso_joint_idx=" << vec_to_str(self.torso_joint_idx) << ",";
             ss << (ml ? "\n" : " ");

             ss << "right_arm_joint_idx=" << vec_to_str(self.right_arm_joint_idx) << ",";
             ss << (ml ? "\n" : " ");

             ss << "left_arm_joint_idx=" << vec_to_str(self.left_arm_joint_idx);
             ss << (ml ? "\n" : " ");

             ss << prefix("");
             ss << ")";

             return ss.str();
           })
      .def("__str__", [](const RobotInfo& self) {
        std::ostringstream ss;
        ss << "RobotInfo(model=" << self.robot_model_name << "@" << self.robot_model_version
           << ", dof=" << self.degree_of_freedom << ", joints=" << self.joint_infos.size()
           << ", powers=" << self.power_infos.size() << ", emos=" << self.emo_infos.size() << ")";
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