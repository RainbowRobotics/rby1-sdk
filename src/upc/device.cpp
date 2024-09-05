#include "rby1-sdk/upc/device.h"

#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {

std::string ResolveSymlink(const std::string& symlink) {
  char buf[1024];
  ssize_t len = readlink(symlink.c_str(), buf, sizeof(buf) - 1);
  if (len != -1) {
    buf[len] = '\0';
    return {buf};
  }
  return "";
}

}  // namespace

namespace rb::upc {

void InitializeDevice(const std::string& device_name) {
  std::string real_path = ResolveSymlink(device_name);

  if (real_path.empty()) {
    real_path = device_name;
  }
  std::string device_path =
      "/sys/bus/usb-serial/devices/" + real_path.substr(real_path.find_last_of('/') + 1) + "/latency_timer";

  std::ofstream file(device_path);
  if (file.is_open()) {
    file << "1";
    file.close();
  } else {
    throw std::runtime_error("ttyUSB for gripper, Error: Unable to open latency_timer file");
  }
}

}  // namespace rb::upc