#pragma once

#include <string>
#include "rby1-sdk/export.h"

namespace rb::upc {

const char* const kGripperDeviceName = "/dev/rby1_gripper";

const char* const kLeaderArmDeviceName = "/dev/rby1_leader_arm";

/// @deprecated Use kLeaderArmDeviceName instead.
const char* const kMasterArmDeviceName = kLeaderArmDeviceName;

/// @brief Legacy device path kept for backward compatibility.
const char* const kLegacyLeaderArmDeviceName = "/dev/rby1_master_arm";

/// @brief Returns kLeaderArmDeviceName if the device exists, otherwise falls back to kLegacyLeaderArmDeviceName.
RBY1_SDK_API std::string ResolveLeaderArmDeviceName();

RBY1_SDK_API void InitializeDevice(const std::string& device_name);

}  // namespace rb::upc