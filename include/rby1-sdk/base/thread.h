#pragma once

#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

namespace rb {

class Thread {
 public:
  using Functor = std::function<void()>;

  Thread() : thread_(), name_(), cpuid_(-1), priority_(0), policy_(SCHED_OTHER), started_(false), running_(false) {}

  ~Thread() {
    if (started_) {
      if (thread_.joinable()) {
        thread_.join();
      }
    }
  }

  void SetName(const std::string& name) { name_ = name; }

  void SetAffinity(int cpuid) { cpuid_ = cpuid; }

  void SetOSPriority(int priority, int policy) {
    priority_ = priority;
    policy_ = policy;
  }

  void StartFunc(const Functor& func) {
    if (started_) {
      throw std::runtime_error("Thread already started");
    }

    thread_ = std::thread([this, func]() {
      running_ = true;

      if (!name_.empty()) {
        pthread_setname_np(pthread_self(), name_.c_str());
      }

      if (cpuid_ != -1) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpuid_, &cpuset);

        int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        if (rc != 0) {
          throw std::runtime_error("Error calling pthread_setaffinity_np");
        }
      }

      if (priority_ != 0) {
        struct sched_param param;
        param.sched_priority = priority_;

        int rc = pthread_setschedparam(pthread_self(), policy_, &param);
        if (rc != 0) {
          throw std::runtime_error("Error calling pthread_setschedparam");
        }
      }

      func();
      running_ = false;
    });

    started_ = true;
  }

  bool IsRunning() const { return running_; }

  void Join() {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  std::thread thread_;
  std::string name_;
  int cpuid_;
  int priority_;
  int policy_;
  bool started_;
  std::atomic<bool> running_;
};

}  // namespace rb