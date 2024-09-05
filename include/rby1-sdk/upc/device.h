#pragma once

#include <string>

namespace rb::upc {

const char* const kGripperDeviceName = "/dev/rby1_gripper";

const char* const kMasterArmDeviceName = "/dev/rby1_master_arm";

void InitializeDevice(const std::string& device_name);

}  // namespace rb::upc