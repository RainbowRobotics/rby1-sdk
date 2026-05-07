//  Robot State Demo
//  This example demonstrates how to connect to an RB-Y1 robot, ensure specified
//  power devices (by regex) are on, retrieve the current robot state, and print
//  it in a readable form.
// 
//  Usage example:
//    ./example_03_robot_state --address 192.168.30.1:50051 --model a --power ".*"
// 
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
// 
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

std::string BoolStr(bool v) { return v ? "True" : "False"; }

std::string TimespecStr(const timespec& ts) {
  std::ostringstream ss;
  ss << ts.tv_sec << "." << std::setw(9) << std::setfill('0') << ts.tv_nsec;
  return ss.str();
}

template <typename Derived>
std::string FormatEigen(const Eigen::MatrixBase<Derived>& m, int precision = 3) {
  Eigen::IOFormat fmt(precision, Eigen::DontAlignCols, " ", ", ", "[", "]", "[", "]");
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  if (m.cols() == 1 && m.rows() > 1) {
    ss << std::setprecision(precision) << m.transpose().format(fmt);
  } else {
    ss << std::setprecision(precision) << m.format(fmt);
  }
  return ss.str();
}

template <int N>
std::string FormatBoolVector(const Eigen::Vector<bool, N>& v) {
  std::ostringstream ss;
  ss << "[";
  for (int i = 0; i < v.size(); ++i) {
    if (i) {
      ss << ", ";
    }
    ss << BoolStr(v[i]);
  }
  ss << "]";
  return ss.str();
}

std::string PowerStateStr(const PowerState& ps, bool multiline) {
  std::ostringstream ss;
  if (multiline) {
    ss << "PowerState(\n  state=" << to_string(ps.state) << ",\n  voltage=" << std::fixed << std::setprecision(3)
       << ps.voltage << "\n)";
  } else {
    ss << "PowerState(state=" << to_string(ps.state) << ", voltage=" << std::fixed << std::setprecision(3)
       << ps.voltage << ")";
  }
  return ss.str();
}

std::string EMOStateStr(const EMOState& es, bool multiline) {
  std::ostringstream ss;
  if (multiline) {
    ss << "EMOState(\n  state=" << to_string(es.state) << "\n)";
  } else {
    ss << "EMOState(state=" << to_string(es.state) << ")";
  }
  return ss.str();
}

std::string JointStateStr(const JointState& js, int idx, bool multiline) {
  std::ostringstream ss;
  if (multiline) {
    ss << "JointState(\n"
       << "  index=" << idx << ",\n"
       << "  ready=" << BoolStr(js.is_ready) << ",\n"
       << "  fet=" << to_string(js.fet_state) << ",\n"
       << "  run=" << to_string(js.run_state) << ",\n"
       << "  init=" << to_string(js.init_state) << ",\n"
       << "  pos=" << std::fixed << std::setprecision(3) << js.position << ",\n"
       << "  vel=" << js.velocity << ",\n"
       << "  cur=" << js.current << ",\n"
       << "  tau=" << js.torque << ",\n"
       << "  tpos=" << js.target_position << ",\n"
       << "  tvel=" << js.target_velocity << ",\n"
       << "  tgain=" << js.target_feedback_gain << ",\n"
       << "  tff=" << js.target_feedforward_torque << ",\n"
       << "  temp=" << js.temperature << "\n)";
  } else {
    ss << "JointState(index=" << idx << ", ready=" << BoolStr(js.is_ready) << ", fet=" << to_string(js.fet_state)
       << ", run=" << to_string(js.run_state) << ", init=" << to_string(js.init_state)
       << ", pos=" << std::fixed << std::setprecision(3) << js.position << ", vel=" << js.velocity
       << ", cur=" << js.current << ", tau=" << js.torque << ", tpos=" << js.target_position
       << ", tvel=" << js.target_velocity << ", tgain=" << js.target_feedback_gain
       << ", tff=" << js.target_feedforward_torque << ", temp=" << js.temperature << ")";
  }
  return ss.str();
}

std::string ToolFlangeStr(const ToolFlangeState& tf, bool multiline) {
  std::ostringstream ss;
  if (multiline) {
    ss << "ToolFlangeState(\n"
       << "  t=" << TimespecStr(tf.time_since_last_update) << ",\n"
       << "  gyro=" << FormatEigen(tf.gyro) << ",\n"
       << "  acc=" << FormatEigen(tf.acceleration) << ",\n"
       << "  switch_A=" << BoolStr(tf.switch_A) << ",\n"
       << "  output_voltage=" << tf.output_voltage << ",\n"
       << "  dinA=" << BoolStr(tf.digital_input_A) << ",\n"
       << "  dinB=" << BoolStr(tf.digital_input_B) << ",\n"
       << "  doutA=" << BoolStr(tf.digital_output_A) << ",\n"
       << "  doutB=" << BoolStr(tf.digital_output_B) << "\n)";
  } else {
    ss << "ToolFlangeState(t=" << TimespecStr(tf.time_since_last_update) << ", gyro=" << FormatEigen(tf.gyro)
       << ", acc=" << FormatEigen(tf.acceleration) << ", switch_A=" << BoolStr(tf.switch_A)
       << ", output_voltage=" << tf.output_voltage << ", dinA=" << BoolStr(tf.digital_input_A)
       << ", dinB=" << BoolStr(tf.digital_input_B) << ", doutA=" << BoolStr(tf.digital_output_A)
       << ", doutB=" << BoolStr(tf.digital_output_B) << ")";
  }
  return ss.str();
}

std::string FTSensorStr(const FTSensorData& ft, bool multiline) {
  std::ostringstream ss;
  if (multiline) {
    ss << "FTSensorData(\n"
       << "  t=" << TimespecStr(ft.time_since_last_update) << ",\n"
       << "  force=" << FormatEigen(ft.force) << ",\n"
       << "  torque=" << FormatEigen(ft.torque) << "\n)";
  } else {
    ss << "FTSensorData(t=" << TimespecStr(ft.time_since_last_update) << ", force=" << FormatEigen(ft.force)
       << ", torque=" << FormatEigen(ft.torque) << ")";
  }
  return ss.str();
}

std::string CollisionStr(const dyn::CollisionResult& cr, bool multiline) {
  std::ostringstream ss;
  if (multiline) {
    ss << "CollisionResult(\n"
       << "  link1=" << cr.link1 << ",\n"
       << "  link2=" << cr.link2 << ",\n"
       << "  distance=" << std::fixed << std::setprecision(6) << cr.distance << "\n)";
  } else {
    ss << "CollisionResult(link1=" << cr.link1 << ", link2=" << cr.link2 << ", distance=" << std::fixed
       << std::setprecision(6) << cr.distance << ")";
  }
  return ss.str();
}

template <typename ModelT>
void PrintRobotState(const RobotState<ModelT>& rs, bool multiline) {
  if (!multiline) {
    std::ostringstream out;
    auto append_field = [&](const std::string& name, const std::string& value) {
      if (!out.str().empty()) {
        out << ", ";
      }
      out << name << "=" << value;
    };

    Eigen::Index ready = 0;
    for (Eigen::Index i = 0; i < rs.is_ready.size(); ++i) {
      if (rs.is_ready[i]) {
        ++ready;
      }
    }
    const Eigen::Index total = rs.is_ready.size();

    append_field("timestamp", TimespecStr(rs.timestamp));
    {
      std::ostringstream ss;
      ss << "SystemStat(cpu=" << std::fixed << std::setprecision(3) << rs.system_stat.cpu_usage
         << ", mem=" << rs.system_stat.memory_usage << ", uptime=" << rs.system_stat.uptime
         << ", prog_uptime=" << rs.system_stat.program_uptime << ")";
      append_field("system_stat", ss.str());
    }
    {
      std::ostringstream ss;
      ss << "BatteryState(v=" << rs.battery_state.voltage << ", i=" << rs.battery_state.current
         << ", level=" << rs.battery_state.level_percent << ")";
      append_field("battery_state", ss.str());
    }
    {
      std::ostringstream ss;
      ss << "[";
      for (size_t i = 0; i < rs.power_states.size(); ++i) {
        if (i) {
          ss << ", ";
        }
        ss << PowerStateStr(rs.power_states[i], false);
      }
      ss << "]";
      append_field("power_states", ss.str());
    }
    {
      std::ostringstream ss;
      ss << "[";
      for (size_t i = 0; i < rs.emo_states.size(); ++i) {
        if (i) {
          ss << ", ";
        }
        ss << EMOStateStr(rs.emo_states[i], false);
      }
      ss << "]";
      append_field("emo_states", ss.str());
    }
    {
      std::ostringstream ss;
      ss << "[";
      for (size_t i = 0; i < rs.joint_states.size(); ++i) {
        if (i) {
          ss << ", ";
        }
        ss << JointStateStr(rs.joint_states[i], static_cast<int>(i), false);
      }
      ss << "]";
      append_field("joint_states", ss.str());
    }

    append_field("tool_flange_right", ToolFlangeStr(rs.tool_flange_right, false));
    append_field("tool_flange_left", ToolFlangeStr(rs.tool_flange_left, false));
    append_field("ft_sensor_right", FTSensorStr(rs.ft_sensor_right, false));
    append_field("ft_sensor_left", FTSensorStr(rs.ft_sensor_left, false));

    append_field("is_ready", FormatBoolVector(rs.is_ready));
    append_field("position", FormatEigen(rs.position));
    append_field("velocity", FormatEigen(rs.velocity));
    append_field("current", FormatEigen(rs.current));
    append_field("torque", FormatEigen(rs.torque));
    append_field("target_position", FormatEigen(rs.target_position));
    append_field("target_velocity", FormatEigen(rs.target_velocity));
    append_field("target_feedback_gain", FormatEigen(rs.target_feedback_gain.template cast<double>()));
    append_field("target_feedforward_torque", FormatEigen(rs.target_feedforward_torque));
    append_field("odometry", FormatEigen(rs.odometry));
    append_field("center_of_mass", FormatEigen(rs.center_of_mass));

    {
      std::ostringstream ss;
      ss << "[";
      for (size_t i = 0; i < rs.collisions.size(); ++i) {
        if (i) {
          ss << ", ";
        }
        ss << CollisionStr(rs.collisions[i], false);
      }
      ss << "]";
      append_field("collisions", ss.str());
    }

    append_field("temperature", FormatEigen(rs.temperature.template cast<double>()));
    append_field("gravity", FormatEigen(rs.gravity));
    append_field("ready", std::to_string(ready) + "/" + std::to_string(total));

    std::cout << "RobotState_" << ModelT::kModelName << "(" << out.str() << ")" << std::endl;
    return;
  }

  auto print_field = [](const std::string& name, const std::string& value) {
    std::cout << "  " << name << "=" << value << ",\n";
  };

  std::cout << "RobotState_" << ModelT::kModelName << "(\n";

  print_field("timestamp", TimespecStr(rs.timestamp));

  {
    std::ostringstream ss;
    ss << "SystemStat(cpu=" << std::fixed << std::setprecision(3) << rs.system_stat.cpu_usage
       << ", mem=" << rs.system_stat.memory_usage << ", uptime=" << rs.system_stat.uptime
       << ", prog_uptime=" << rs.system_stat.program_uptime << ")";
    print_field("system_stat", ss.str());
  }

  {
    std::ostringstream ss;
    ss << "BatteryState(v=" << rs.battery_state.voltage << ", i=" << rs.battery_state.current
       << ", level=" << rs.battery_state.level_percent << ")";
    print_field("battery_state", ss.str());
  }

  {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < rs.power_states.size(); ++i) {
      if (i) {
        ss << ", ";
      }
      ss << PowerStateStr(rs.power_states[i], multiline);
    }
    ss << "]";
    print_field("power_states", ss.str());
  }

  {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < rs.emo_states.size(); ++i) {
      if (i) {
        ss << ", ";
      }
      ss << EMOStateStr(rs.emo_states[i], multiline);
    }
    ss << "]";
    print_field("emo_states", ss.str());
  }

  std::cout << "  joint_states=[\n";
  for (size_t i = 0; i < rs.joint_states.size(); ++i) {
    std::cout << "    " << JointStateStr(rs.joint_states[i], static_cast<int>(i), multiline);
    if (i + 1 < rs.joint_states.size()) {
      std::cout << ",";
    }
    std::cout << "\n";
  }
  std::cout << "  ],\n";

  print_field("tool_flange_right", ToolFlangeStr(rs.tool_flange_right, multiline));
  print_field("tool_flange_left", ToolFlangeStr(rs.tool_flange_left, multiline));
  print_field("ft_sensor_right", FTSensorStr(rs.ft_sensor_right, multiline));
  print_field("ft_sensor_left", FTSensorStr(rs.ft_sensor_left, multiline));

  print_field("is_ready", FormatBoolVector(rs.is_ready));
  print_field("position", FormatEigen(rs.position));
  print_field("velocity", FormatEigen(rs.velocity));
  print_field("current", FormatEigen(rs.current));
  print_field("torque", FormatEigen(rs.torque));
  print_field("target_position", FormatEigen(rs.target_position));
  print_field("target_velocity", FormatEigen(rs.target_velocity));
  print_field("target_feedback_gain", FormatEigen(rs.target_feedback_gain.template cast<double>()));
  print_field("target_feedforward_torque", FormatEigen(rs.target_feedforward_torque));
  print_field("odometry", FormatEigen(rs.odometry));
  print_field("center_of_mass", FormatEigen(rs.center_of_mass));

  {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < rs.collisions.size(); ++i) {
      if (i) {
        ss << ", ";
      }
      ss << CollisionStr(rs.collisions[i], multiline);
    }
    ss << "]";
    print_field("collisions", ss.str());
  }

  print_field("temperature", FormatEigen(rs.temperature.template cast<double>()));
  std::cout << "  gravity=" << FormatEigen(rs.gravity) << "\n";
  std::cout << ")\n";
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex]" << std::endl;
}

template <typename ModelT>
int RunRobotState(const std::string& address, const std::string& power_regex) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }
  if (!robot->IsPowerOn(power_regex)) {
    if (!robot->PowerOn(power_regex)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  const auto state = robot->GetState();

  PrintRobotState(state,false);

  PrintRobotState(state,true);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
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
    return RunRobotState<y1_model::A>(address, power);
  }
  if (model == "m") {
    return RunRobotState<y1_model::M>(address, power);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
