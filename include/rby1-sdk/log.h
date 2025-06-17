#pragma once

#include <ctime>
#include <iomanip>
#include <ostream>
#include <string>

#include "export.h"

namespace rb {

struct RBY1_SDK_API Log {
  enum class Level {
    kTrace = 0,  //
    kDebug,      //
    kInfo,       //
    kWarn,       //
    kError,      //
    kCritical    //
  };

  struct timespec timestamp {};

  struct timespec robot_system_timestamp {};

  Level level;

  std::string message;
};

RBY1_SDK_API inline std::string to_string(const Log::Level& level) {
  switch (level) {
    case Log::Level::kTrace:
      return "Trace";
    case Log::Level::kDebug:
      return "Debug";
    case Log::Level::kInfo:
      return "Info";
    case Log::Level::kWarn:
      return "Warn";
    case Log::Level::kError:
      return "Error";
    case Log::Level::kCritical:
      return "Critical";
  }
  return "";
}

}  // namespace rb

RBY1_SDK_API inline std::ostream& operator<<(std::ostream& out, const rb::Log::Level& level) {
  out << rb::to_string(level);

  return out;
}

RBY1_SDK_API inline std::ostream& operator<<(std::ostream& out, const rb::Log& log) {
  out << "[" << log.timestamp.tv_sec << ".";
  out << std::setw(9) << std::setfill('0') << log.timestamp.tv_nsec << "] ";

  out << "[" << log.robot_system_timestamp.tv_sec << ".";
  out << std::setw(9) << std::setfill('0') << log.robot_system_timestamp.tv_nsec << "] ";

  out << "[" << log.level << "] ";

  out << log.message;

  return out;
}