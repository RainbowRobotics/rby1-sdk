// Hello RB-Y1 Demo
// This example demonstrates basic connection to RB-Y1 robot and retrieves robot information
// in different string representation formats. See --help for arguments.
//
// Usage example:
//     ./example_01_hello_rby1.py --address 192.168.30.1:50051 --model a
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

std::string Quote(const std::string& v) {
  std::ostringstream ss;
  ss << std::quoted(v);
  return ss.str();
}

std::string NormalizeModelName(std::string model_name) {
  model_name = ToLower(model_name);
  if (model_name.rfind("rby1", 0) == 0) {
    model_name = model_name.substr(4);
  }
  return model_name;
}

std::string BoolStr(bool v) { return v ? "True" : "False"; }

std::string Repr(const BatteryInfo&) { return "BatteryInfo()"; }

std::string Repr(const PowerInfo& info) {
  std::ostringstream ss;
  ss << "PowerInfo(name=" << Quote(info.name) << ")";
  return ss.str();
}

std::string Repr(const EMOInfo& info) {
  std::ostringstream ss;
  ss << "EMOInfo(name=" << Quote(info.name) << ")";
  return ss.str();
}

std::string Repr(const JointInfo& info, bool multiline) {
  const char* first = multiline ? "\n  " : "";
  const char* sep = multiline ? ",\n  " : ", ";
  const char* last = multiline ? "\n" : "";

  std::ostringstream ss;
  ss << "JointInfo(" << first                                             //
     << "name=" << Quote(info.name) << sep                                 //
     << "has_brake=" << BoolStr(info.has_brake) << sep                     //
     << "product_name=" << Quote(info.product_name) << sep                 //
     << "firmware_version=" << Quote(info.firmware_version) << last << ")";  //
  return ss.str();
}

template <typename T>
std::string ReprList(const std::vector<T>& items, bool multiline) {
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i) {
      ss << ", ";
    }
    if constexpr (std::is_same_v<T, JointInfo>) {
      ss << Repr(items[i], multiline);
    } else if constexpr (std::is_same_v<T, PowerInfo>) {
      ss << Repr(items[i]);
    } else if constexpr (std::is_same_v<T, EMOInfo>) {
      ss << Repr(items[i]);
    } else {
      ss << items[i];
    }
  }
  ss << "]";
  return ss.str();
}

std::string ReprVectorU32(const std::vector<unsigned int>& v) {
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) {
      ss << ", ";
    }
    ss << v[i];
  }
  ss << "]";
  return ss.str();
}

std::string RobotInfoStr(const RobotInfo& info) {
  std::ostringstream ss;
  ss << "RobotInfo(model=" << info.robot_model_name << "@" << info.robot_model_version
     << ", dof=" << info.degree_of_freedom << ", joints=" << info.joint_infos.size()
     << ", powers=" << info.power_infos.size() << ", emos=" << info.emo_infos.size() << ")";
  return ss.str();
}

std::string RobotInfoRepr(const RobotInfo& info, bool multiline) {
  const char* indent = multiline ? "  " : "";
  const char* nl = multiline ? "\n" : "";
  const char* sep = multiline ? "\n" : " ";

  std::ostringstream ss;
  ss << "RobotInfo(" << nl;
  ss << indent << "version=" << Quote(info.version) << "," << sep;
  ss << indent << "sdk_version=" << Quote(info.sdk_version) << "," << sep;
  ss << indent << "robot_version=" << Quote(info.robot_version) << "," << sep;
  ss << indent << "sdk_commit_id=" << Quote(info.sdk_commit_id) << "," << sep;
  ss << indent << "robot_model_name=" << Quote(info.robot_model_name) << "," << sep;
  ss << indent << "robot_model_version=" << Quote(info.robot_model_version) << "," << sep;
  ss << indent << "battery_info=" << Repr(info.battery_info) << "," << sep;
  ss << indent << "num_power_infos=" << info.power_infos.size() << "," << sep;
  ss << indent << "num_emo_infos=" << info.emo_infos.size() << "," << sep;
  ss << indent << "degree_of_freedom=" << info.degree_of_freedom << "," << sep;
  ss << indent << "joint_infos=" << ReprList(info.joint_infos, multiline) << "," << sep;
  ss << indent << "mobility_joint_idx=" << ReprVectorU32(info.mobility_joint_idx) << "," << sep;
  ss << indent << "body_joint_idx=" << ReprVectorU32(info.body_joint_idx) << "," << sep;
  ss << indent << "head_joint_idx=" << ReprVectorU32(info.head_joint_idx) << "," << sep;
  ss << indent << "torso_joint_idx=" << ReprVectorU32(info.torso_joint_idx) << "," << sep;
  ss << indent << "right_arm_joint_idx=" << ReprVectorU32(info.right_arm_joint_idx) << "," << sep;
  ss << indent << "left_arm_joint_idx=" << ReprVectorU32(info.left_arm_joint_idx) << nl;
  ss << ")";
  return ss.str();
}

template <typename ModelT>
int RunHello(const std::string& address, const std::string& model_arg) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  const RobotInfo info = robot->GetRobotInfo();
  const std::string requested_model = NormalizeModelName(model_arg);

  if (info.mobility_joint_idx.size() == 2 && requested_model == "m") {
    std::cerr << "wrong model argument. this robot model is a" << std::endl;
    return 1;
  } else if (info.mobility_joint_idx.size() == 4 && requested_model == "a") {
    std::cerr << "wrong model argument. this robot model is m" << std::endl;
    return 1;
  }

  std::cout << "Hello, RB-Y1! (Robot model name: " << ModelT::kModelName << ")" << std::endl;

  std::cout << "\n== Robot info (__str__ format):" << std::endl;
  std::cout << RobotInfoStr(info) << std::endl;

  std::cout << "\n== Robot info (repr, single-line format):" << std::endl;
  std::cout << RobotInfoRepr(info, false) << std::endl;

  std::cout << "\n=== Robot info (repr, multi-line format):" << std::endl;
  std::cout << RobotInfoRepr(info, true) << std::endl;

  return 0;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (address.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return RunHello<y1_model::A>(address, model);
  }
  if (model == "m") {
    return RunHello<y1_model::M>(address, model);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
