#pragma once

#include "export.h"

namespace rb {

struct RBY1_SDK_API ControlManagerState {
  enum class State {
    kUnknown = 0,
    kIdle,
    kEnabled,
    kMinorFault,
    kMajorFault,
  };

  enum class ControlState { kUnknown = 0, kIdle, kExecuting, kSwitching };

  State state{State::kUnknown};
  double time_scale{0.};
  ControlState control_state{ControlState::kUnknown};
  std::vector<unsigned int> enabled_joint_idx{};
  bool unlimited_mode_enabled{false};
};

RBY1_SDK_API inline std::string to_string(ControlManagerState::State s) {
  switch (s) {
    case ControlManagerState::State::kUnknown:
      return "Unknown";
    case ControlManagerState::State::kIdle:
      return "Idle";
    case ControlManagerState::State::kEnabled:
      return "Enabled";
    case ControlManagerState::State::kMinorFault:
      return "MinorFault";
    case ControlManagerState::State::kMajorFault:
      return "MajorFault";
  }
  return "";
}

RBY1_SDK_API inline std::string to_string(ControlManagerState::ControlState s) {
  switch (s) {
    case ControlManagerState::ControlState::kUnknown:
      return "Unknown";
    case ControlManagerState::ControlState::kIdle:
      return "Idle";
    case ControlManagerState::ControlState::kExecuting:
      return "Executing";
    case ControlManagerState::ControlState::kSwitching:
      return "Switching";
  }
  return "";
}

}  // namespace rb