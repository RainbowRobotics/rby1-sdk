#pragma once

#include <string>
#include "rby1-sdk/export.h"

namespace rb::upc {

const char* const kGripperDeviceName = "/dev/rby1_gripper";

const char* const kMasterArmDeviceName = "/dev/rby1_master_arm";

RBY1_SDK_API void InitializeDevice(const std::string& device_name);

}  // namespace rb::upc