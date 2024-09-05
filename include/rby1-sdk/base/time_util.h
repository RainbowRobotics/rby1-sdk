#pragma once

#include <ctime>

namespace rb {

inline long GetDurationInNs(struct timespec start, struct timespec end) {
  return ((long)end.tv_sec * 1000000000 + (long)end.tv_nsec) -
         ((long)start.tv_sec * 1000000000 + (long)start.tv_nsec);
}

inline struct timespec GetDurationInTimespec(struct timespec start, struct timespec end) {
  struct timespec d {};

  long duration = GetDurationInNs(start, end);
  d.tv_nsec = duration % 1000000000;
  d.tv_sec = (duration - d.tv_nsec) / 1000000000;
  return d;
}

inline struct timespec GetCurrentTime() {
  struct timespec ts {};

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts;
}

inline double TimespecToDouble(const struct timespec& ts) {
  return (double)ts.tv_sec + ((double)ts.tv_nsec) / 1.e9;
}

class TimeWatch {
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