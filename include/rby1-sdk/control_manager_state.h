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
};

}  // namespace rb