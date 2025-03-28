#pragma once

#include <string>
#include <vector>

namespace rb {

struct WifiNetwork {
  std::string ssid;
  int signal_strength;
  bool secured;
};

struct WifiStatus {
  std::string ssid;
  std::string ip_address;
  std::string gateway;
  std::vector<std::string> dns;
  bool connected;
};

}  // namespace rb