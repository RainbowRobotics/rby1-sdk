#pragma once

#include <ctime>
#include <iomanip>
#include <ostream>
#include <string>

namespace rb {

struct Log {
  enum class Level {
    kTrace = 0,  //
    kDebug,      //
    kInfo,       //
    kWarn,       //
    kError,      //
    kCritical    //
  };

  struct timespec timestamp {};

  Level level;

  std::string message;

  std::string logger_name;
};

inline std::string ToString(const Log::Level& level) {
  switch (level) {
    case Log::Level::kTrace:
      return "trace";
    case Log::Level::kDebug:
      return "debug";
    case Log::Level::kInfo:
      return "info";
    case Log::Level::kWarn:
      return "warn";
    case Log::Level::kError:
      return "error";
    case Log::Level::kCritical:
      return "critical";
  }
  return "";
}

}  // namespace rb

inline std::ostream& operator<<(std::ostream& out, const rb::Log::Level& level) {
  out << rb::ToString(level);

  return out;
}

inline std::ostream& operator<<(std::ostream& out, const rb::Log& log) {
  out << "[" << log.timestamp.tv_sec << ".";
  out << std::setw(9) << std::setfill('0') << log.timestamp.tv_nsec << "] ";

  out << "[" << log.logger_name << "] ";

  out << "[" << log.level << "] ";

  out << log.message;

  return out;
}