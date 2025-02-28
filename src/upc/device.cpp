#include "rby1-sdk/upc/device.h"

#include <csignal>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace std::chrono_literals;

namespace {

#if defined(_WIN32)
std::string ResolveSymlink(const std::string& symlink) {
  // TODO Check whether it works on Windows
  HANDLE hFile =
      CreateFile(symlink.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (hFile == INVALID_HANDLE_VALUE) {
    return "";
  }

  char buf[MAX_PATH];
  DWORD len = GetFinalPathNameByHandle(hFile, buf, MAX_PATH, FILE_NAME_NORMALIZED);
  CloseHandle(hFile);

  if (len > 0 && len < MAX_PATH) {
    return std::string(buf);
  } else {
    return "";
  }
}
#else
std::string ResolveSymlink(const std::string& symlink) {
  char buf[1024];
  ssize_t len = readlink(symlink.c_str(), buf, sizeof(buf) - 1);
  if (len != -1) {
    buf[len] = '\0';
    return {buf};
  }
  return "";
}
#endif

}  // namespace

namespace rb::upc {

void InitializeDevice(const std::string& device_name) {
#if defined(_WIN32)
  throw std::runtime_error("Not implemented: Unable to initialize device on Windows");
#else
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
#endif
}

}  // namespace rb::upc