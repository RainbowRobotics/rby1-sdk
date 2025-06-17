#pragma once

#include <chrono>
#include <ctime>
#include <iostream>

#include "rby1-sdk/export.h"

namespace rb {

constexpr int64_t kNanosecondsInSecond = 1000000000;

/**
 * @brief Calculates the duration between two timespec values in nanoseconds.
 *
 * @param start The start time as a timespec structure.
 * @param end The end time as a timespec structure.
 * @return int64_t The duration between start and end in nanoseconds.
 */
RBY1_SDK_API inline long GetDurationInNs(struct timespec start, struct timespec end) {
  return (static_cast<int64_t>(end.tv_sec) * kNanosecondsInSecond + end.tv_nsec) -
         (static_cast<int64_t>(start.tv_sec) * kNanosecondsInSecond + start.tv_nsec);
}

/**
 * @brief Calculates the duration between two timespec values as a timespec structure.
 *
 * @param start The start time as a timespec structure.
 * @param end The end time as a timespec structure.
 * @return timespec The duration between start and end as a timespec structure.
 */
RBY1_SDK_API inline struct timespec GetDurationInTimespec(struct timespec start, struct timespec end) {
  int64_t duration_ns = GetDurationInNs(start, end);

  struct timespec duration{};

  duration.tv_sec = duration_ns / kNanosecondsInSecond;
  duration.tv_nsec = duration_ns % kNanosecondsInSecond;
  return duration;
}

/**
 * @brief Gets the current time using the CLOCK_MONOTONIC clock.
 *
 * @return timespec The current time as a timespec structure.
 *
 * @note CLOCK_MONOTONIC is used to retrieve the time since the system boot, unaffected by system clock changes.
 *       In case of failure, an empty timespec structure is returned.
 */
RBY1_SDK_API inline struct timespec GetCurrentTime() {
  struct timespec current_time{};

#ifdef __linux__
  if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0) {
    std::cerr << "Failed to get current time" << std::endl;
  }
#else
  auto ct = std::chrono::steady_clock::now();
  int64_t ct_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(ct.time_since_epoch()).count();
  current_time.tv_sec = ct_ns / kNanosecondsInSecond;
  current_time.tv_nsec = ct_ns % kNanosecondsInSecond;
#endif

  return current_time;
}

/**
 * @brief Converts a timespec structure to a 64-bit integer representing the time in nanoseconds.
 *
 * @param ts The timespec structure to convert.
 * @return int64_t The time in nanoseconds.
 */
RBY1_SDK_API inline int64_t TimespecInNs(const struct timespec& ts) {
  return static_cast<int64_t>(ts.tv_sec) * kNanosecondsInSecond + static_cast<int64_t>(ts.tv_nsec);
}

/**
 * @brief Converts a std::chrono::time_point to a timespec structure.
 *
 * @param tp The time_point to convert, with the system clock and nanosecond precision.
 * @return timespec A timespec structure representing the same time as the given time_point.
 */
RBY1_SDK_API inline struct timespec TimepointToTimespec(
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp) {
  using namespace std::chrono;

  auto secs = time_point_cast<seconds>(tp);
  auto ns = time_point_cast<nanoseconds>(tp) - time_point_cast<nanoseconds>(secs);

  return timespec{secs.time_since_epoch().count(), static_cast<long>(ns.count())};
}

/**
 * @brief Converts a timespec structure to a double value representing the time in seconds.
 *
 * @param ts The timespec structure to convert.
 * @return double The time in seconds, including the fractional part from nanoseconds.
 */
RBY1_SDK_API inline double TimespecToDouble(const struct timespec& ts) {
  return static_cast<double>(TimespecInNs(ts)) / static_cast<double>(kNanosecondsInSecond);
}

/**
 * @brief Normalizes a timespec structure by ensuring the nanosecond field is less than one second.
 *
 * @param ts The timespec structure to normalize.
 * @return timespec The normalized timespec structure.
 *
 * @note This function ensures that the tv_nsec value is within the range [0, kNanosecondsInSecond).
 *       If tv_nsec is larger than or equal to kNanosecondsInSecond, it carries over to the seconds field.
 */
RBY1_SDK_API inline timespec NormalizeTimespec(timespec ts) {
  ts.tv_sec += ts.tv_nsec / kNanosecondsInSecond;
  ts.tv_nsec %= kNanosecondsInSecond;
  return ts;
}

class RBY1_SDK_API TimeWatch {
 public:
  TimeWatch() : ts_(GetCurrentTime()) {}

  void Record() { ts_ = GetCurrentTime(); }

  long GetDurationInNs(bool record = false) {
    const auto& cts = GetCurrentTime();
    const auto& duration = rb::GetDurationInNs(ts_, cts);
    if (record) {
      ts_ = cts;
    }
    return duration;
  }

 private:
  struct timespec ts_;
};

}  // namespace rb