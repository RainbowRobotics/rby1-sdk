#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <queue>

#include "rby1-sdk/base/thread.h"
#include "time_util.h"

namespace rb {

class RBY1_SDK_API EventLoop {
 public:
  EventLoop() : EventLoop(std::make_unique<Thread>()) {}

  explicit EventLoop(std::unique_ptr<Thread> thd) : thd_(std::move(thd)) {
    worker_running_ = true;

    thd_->StartFunc([this]() { this->worker_(); });
  }

  ~EventLoop() noexcept {
    if (thd_->IsRunning()) {
      Pause();
      WaitForTasks();
      PurgeTasks();
      Stop();
    }
  }

  void PushCyclicTask(const std::function<void()>& cb, std::chrono::nanoseconds period,
                      std::chrono::nanoseconds offset = std::chrono::nanoseconds{0}) {
    long period_ns = period.count();
    long offset_ns = offset.count();

#ifdef __linux__
    const auto& get_next_wakeup_time = [period_ns](struct timespec ts) {
      ts.tv_nsec += period_ns;
      while (ts.tv_nsec >= kNanosecondsInSecond) {
        ++ts.tv_sec;
        ts.tv_nsec -= kNanosecondsInSecond;
      }
      while (ts.tv_nsec < 0) {
        --ts.tv_sec;
        ts.tv_nsec += kNanosecondsInSecond;
      }
      return ts;
    };

    const auto& cyclic_task = [=](struct timespec wakeup_time, const auto& self) -> void {
      struct timespec current_ts{};

      clock_gettime(CLOCK_MONOTONIC, &current_ts);

      while (true) {
        if (current_ts.tv_sec * kNanosecondsInSecond + current_ts.tv_nsec <
            wakeup_time.tv_sec * kNanosecondsInSecond + wakeup_time.tv_nsec) {
          break;
        }
        wakeup_time = get_next_wakeup_time(wakeup_time);
      }

      while (true) {
        int rv = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, nullptr);
        if (rv == EINTR) {
          continue;
        }
        break;
      }
      if (cb) {
        cb();
      }

      PushTask([=] { self(wakeup_time, self); });
    };

    struct timespec ts{};

    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t c = (ts.tv_sec * kNanosecondsInSecond) + ts.tv_nsec;
    c = (uint64_t)(c / period_ns + 1) * period_ns + offset_ns;
    ts.tv_sec = (long)(c / kNanosecondsInSecond);
    ts.tv_nsec = (long)(c % kNanosecondsInSecond);
    PushTask([=] { cyclic_task(ts, cyclic_task); });
#else
    const auto& cyclic_task = [=](std::chrono::steady_clock::time_point next_wakeup_time, const auto& self) -> void {
      auto current_ts = std::chrono::steady_clock::now();
      while (true) {
        if (current_ts < next_wakeup_time) {
          break;
        }
        next_wakeup_time = next_wakeup_time + period;
      }
      std::this_thread::sleep_until(next_wakeup_time);
      if (cb) {
        cb();
      }

      PushTask([=] { self(next_wakeup_time, self); });
    };
    uint64_t c = 0;
    {
      auto ts = std::chrono::steady_clock::now();
      c = std::chrono::duration_cast<std::chrono::nanoseconds>(ts.time_since_epoch()).count();
      c = (uint64_t)(c / period_ns + 1) * period_ns + offset_ns;
    }
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> ts((std::chrono::nanoseconds(c)));
    PushTask([=] { cyclic_task(ts, cyclic_task); });
#endif
  }

  template <typename F, typename... A>
  void PushLoopTask(F&& task, A&&... args) {
    const auto& loop = [task_function = std::bind(std::forward<F>(task), std::forward<A>(args)...),
                        this](const auto& self) -> void {
      std::invoke(task_function);
      this->PushTask(self, self);
    };
    PushTask(loop, loop);
  }

  //  EventLoop(const EventLoop&) = delete;

  template <typename F, typename... A>
  void PushTask(F&& task, A&&... args) {
    {
      const std::scoped_lock tasks_lock(tasks_mutex_);
      tasks_.push(std::bind(std::forward<F>(task), std::forward<A>(args)...));
    }
    task_available_cv_.notify_one();
  }

  template <typename F, typename... A, typename R = std::invoke_result_t<std::decay_t<F>, std::decay_t<A>...>>
  [[nodiscard]] std::future<R> Submit(F&& task, A&&... args) {
    std::shared_ptr<std::promise<R>> task_promise = std::make_shared<std::promise<R>>();
    PushTask([task_function = std::bind(std::forward<F>(task), std::forward<A>(args)...), task_promise] {
      try {
        if constexpr (std::is_void_v<R>) {
          std::invoke(task_function);
          task_promise->set_value();
        } else {
          task_promise->set_value(std::invoke(task_function));
        }
      } catch (...) {
        try {
          task_promise->set_exception(std::current_exception());
        } catch (...) {}
      }
    });
    return task_promise->get_future();
  }

  template <typename F, typename... A, typename R = std::invoke_result_t<std::decay_t<F>, std::decay_t<A>...>>
  [[nodiscard]] R DoTask(F&& task, A&&... args) {
    return Submit(std::forward<F>(task), std::forward<A>(args)...).get();
  }

  void Stop() {
    {
      const std::scoped_lock tasks_lock(tasks_mutex_);
      worker_running_ = false;
    }
    task_available_cv_.notify_all();
    thd_->Join();
  }

  void Pause() {
    const std::scoped_lock tasks_lock(tasks_mutex_);
    paused_ = true;
  }

  void Unpause() {
    const std::scoped_lock tasks_lock(tasks_mutex_);
    paused_ = false;
  }

  void PurgeTasks() {
    const std::scoped_lock tasks_lock(tasks_mutex_);
    while (!tasks_.empty()) {
      tasks_.pop();
    }
  }

  void WaitForTasks() noexcept {
    std::unique_lock tasks_lock(tasks_mutex_);
    waiting_ = true;
    tasks_done_cv_.wait(tasks_lock,
                        [this] { return !worker_running_ || (!tasks_running_ && (paused_ || tasks_.empty())); });
    waiting_ = false;
  }

 private:
  void worker_() {
    std::function<void()> task;
    while (true) {
      std::unique_lock tasks_lock(tasks_mutex_);
      task_available_cv_.wait(tasks_lock, [this] { return !tasks_.empty() || !worker_running_; });
      if (!worker_running_) {
        break;
      }
      if (paused_) {
        continue;
      }
      task = std::move(tasks_.front());
      tasks_.pop();
      ++tasks_running_;
      tasks_lock.unlock();
      task();
      tasks_lock.lock();
      --tasks_running_;
      if (waiting_ && !tasks_running_ && (paused_ || tasks_.empty())) {
        tasks_done_cv_.notify_all();
      }
    }
  }

 private:
  std::unique_ptr<Thread> thd_;

  bool paused_ = false;

  std::condition_variable task_available_cv_ = {};

  std::condition_variable tasks_done_cv_ = {};

  std::queue<std::function<void()>> tasks_ = {};

  size_t tasks_running_ = 0;

  mutable std::mutex tasks_mutex_ = {};

  bool waiting_ = false;

  bool worker_running_ = false;
};

}  // namespace rb