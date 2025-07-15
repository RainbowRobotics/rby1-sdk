#pragma once

#include <string>
#include <vector>

#include "rby1-sdk/export.h"

namespace rb {

struct RBY1_SDK_API WifiNetwork {
  std::string ssid;
  int signal_strength;
  bool secured;
};

struct RBY1_SDK_API WifiStatus {
  std::string ssid;
  std::string ip_address;
  std::string gateway;
  std::vector<std::string> dns;
  bool connected;
};

}  // namespace rb