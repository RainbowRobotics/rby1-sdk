#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rby1-sdk/export.h"

#if defined(_WIN32)
#include <Windows.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#else
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

#if defined(_WIN32)
#define POLICY_DEFAULT_VALUE 0
#else
#define POLICY_DEFAULT_VALUE SCHED_OTHER
#endif

namespace rb {

class RBY1_SDK_API Thread {
 public:
  using Functor = std::function<void()>;

  Thread(std::string name = "", int cpuid = -1, int priority = 0, int policy = POLICY_DEFAULT_VALUE)
      : thread_(),
        name_(std::move(name)),
        cpuid_(cpuid),
        priority_(priority),
        policy_(policy),
        started_(false),
        running_(false) {}

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

      SetThreadName();
      SetThreadAffinity();
      SetThreadPriority();

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
  std::atomic<bool> running_{};

  void SetThreadName() {
    if (!name_.empty()) {
#if defined(_WIN32)
      const DWORD MS_VC_EXCEPTION = 0x406D1388;

#pragma pack(push, 8)

      typedef struct tagTHREADNAME_INFO {
        DWORD dwType;      // must be 0x1000
        LPCSTR szName;     // thread name
        DWORD dwThreadID;  // thread ID (-1 = caller thread)
        DWORD dwFlags;     // reserved for future use, must be zero
      } THREADNAME_INFO;

#pragma pack(pop)

      THREADNAME_INFO info;
      info.dwType = 0x1000;
      info.szName = name_.c_str();
      info.dwThreadID = GetCurrentThreadId();
      info.dwFlags = 0;

      __try {
        RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
      } __except (EXCEPTION_EXECUTE_HANDLER) {}
#elif defined(__APPLE__)
      pthread_setname_np(name_.c_str());
#else
      pthread_setname_np(pthread_self(), name_.c_str());
#endif
    }
  }

  void SetThreadAffinity() {  // NOLINT
    if (cpuid_ != -1) {
#if defined(_WIN32)
      DWORD_PTR mask = 1 << cpuid_;
      SetThreadAffinityMask(GetCurrentThread(), mask);
#elif defined(__APPLE__)
      thread_affinity_policy_data_t policy = {cpuid_};
      thread_port_t mach_thread = pthread_mach_thread_np(pthread_self());
      kern_return_t ret = thread_policy_set(mach_thread, THREAD_AFFINITY_POLICY, (thread_policy_t)&policy, 1);
      if (ret != KERN_SUCCESS) {
        throw std::runtime_error("Error setting thread affinity on macOS");
      }
#else
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(cpuid_, &cpuset);

      int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
      if (rc != 0) {
        throw std::runtime_error("Error calling pthread_setaffinity_np");
      }
#endif
    }
  }

  void SetThreadPriority() {  // NOLINT
    if (priority_ != 0) {
#if defined(_WIN32)
      int win_priority = THREAD_PRIORITY_NORMAL;
      if (priority_ > 0) {
        win_priority = THREAD_PRIORITY_HIGHEST;
      } else if (priority_ < 0) {
        win_priority = THREAD_PRIORITY_LOWEST;
      }
      ::SetThreadPriority(GetCurrentThread(), win_priority);
#else
      struct sched_param param{};

      param.sched_priority = priority_;

      int rc = pthread_setschedparam(pthread_self(), policy_, &param);
      if (rc != 0) {
        throw std::runtime_error("Error calling pthread_setschedparam");
      }
#endif
    }
  }
};

}  // namespace rb