#pragma once

namespace rb {

struct ControlManagerState {
  enum class State {
    kUnknown = 0,
    kIdle,
    kEnabled,
    kMinorFault,
    kMajorFault,
  };

  enum class ControlState { kUnknown = 0, kIdle, kExecuting, kSwitching };

  State state;
  double time_scale;
  ControlState control_state;
  std::vector<unsigned int> enabled_joint_idx;
  bool unlimited_mode_enabled;
};

}  // namespace rb