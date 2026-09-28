// handoff.h
// Bounded handoffs between workers, and the per-worker counters that make
// their behavior visible.
//
// LatestSlot is a single-entry, latest-replacement handoff for work whose
// only interesting instance is the newest one (a sensor snapshot for the
// field worker). Putting into a full slot replaces the pending entry and
// counts the replacement; nothing accumulates. It is the wrong tool for
// motion increments, which must never be replaced; those are consumed on
// the thread that acquires them.
//
// WorkerStats is written by one worker and read by anyone through a copy.

#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace navigatr
{

template <typename T>
class LatestSlot
{
public:
    // Replaces any pending entry.
    void put(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_.has_value()) {
                ++replaced_;
            }
            pending_ = std::move(value);
            ++offered_;
        }
        cv_.notify_one();
    }

    // The pending entry, if any, without waiting.
    bool take(T& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pending_.has_value()) {
            return false;
        }
        out = std::move(*pending_);
        pending_.reset();
        ++taken_;
        return true;
    }

    // Waits up to timeout for an entry. False on timeout or after stop().
    bool waitTake(T& out, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, timeout, [&] { return stopped_ || pending_.has_value(); });
        if (!pending_.has_value()) {
            return false;
        }
        out = std::move(*pending_);
        pending_.reset();
        ++taken_;
        return true;
    }

    // Wakes a waiting consumer; later waits return immediately until
    // resume().
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    void resume() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = false;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.reset();
    }

    bool pending() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return pending_.has_value();
    }
    uint64_t offered() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return offered_;
    }
    uint64_t taken() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return taken_;
    }
    // Entries a newer one displaced before any consumer saw them.
    uint64_t replaced() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return replaced_;
    }

private:
    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    std::optional<T>        pending_;
    bool                    stopped_  = false;
    uint64_t                offered_  = 0;
    uint64_t                taken_    = 0;
    uint64_t                replaced_ = 0;
};

struct WorkerStatsSnapshot {
    std::string name;
    bool        running          = false;
    uint64_t    cycles           = 0;
    double      rate_hz          = 0.0;   // cycles completed over the last window
    double      last_cycle_ms    = 0.0;   // work time of the newest cycle
    double      mean_cycle_ms    = 0.0;   // exponential average of work time
    double      max_cycle_ms     = 0.0;   // since start or reset
    double      period_target_ms = 0.0;   // 0 when event driven
    uint64_t    overruns         = 0;     // cycles whose work exceeded the period
    uint64_t    dropped          = 0;     // inputs displaced before this worker saw them
    uint64_t    pending          = 0;     // inputs waiting right now
    int64_t     last_cycle_host_ms = -1;  // host clock when the newest cycle finished
};

// One writer (the worker), any reader through snapshot().
class WorkerStats
{
public:
    explicit WorkerStats(std::string name) { snapshot_.name = std::move(name); }

    void setPeriodTarget(double ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.period_target_ms = ms;
    }

    void setRunning(bool running) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.running = running;
        if (running) {
            window_start_ = std::chrono::steady_clock::now();
            window_count_ = 0;
        }
    }

    // Called by the worker at the end of each cycle with the work time.
    void cycleDone(double work_ms, int64_t host_ms, uint64_t dropped, uint64_t pending) {
        const auto                  now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.cycles;
        snapshot_.last_cycle_ms = work_ms;
        snapshot_.mean_cycle_ms = snapshot_.cycles == 1
                                      ? work_ms
                                      : 0.9 * snapshot_.mean_cycle_ms + 0.1 * work_ms;
        if (work_ms > snapshot_.max_cycle_ms) {
            snapshot_.max_cycle_ms = work_ms;
        }
        if (snapshot_.period_target_ms > 0.0 && work_ms > snapshot_.period_target_ms) {
            ++snapshot_.overruns;
        }
        snapshot_.dropped            = dropped;
        snapshot_.pending            = pending;
        snapshot_.last_cycle_host_ms = host_ms;
        ++window_count_;
        const double window_s =
            std::chrono::duration<double>(now - window_start_).count();
        if (window_s >= 1.0) {
            snapshot_.rate_hz = window_count_ / window_s;
            window_start_     = now;
            window_count_     = 0;
        }
    }

    void resetCounters() {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string name    = snapshot_.name;
        const bool        running = snapshot_.running;
        const double      period  = snapshot_.period_target_ms;
        snapshot_                  = WorkerStatsSnapshot{};
        snapshot_.name             = name;
        snapshot_.running          = running;
        snapshot_.period_target_ms = period;
        window_start_              = std::chrono::steady_clock::now();
        window_count_              = 0;
    }

    WorkerStatsSnapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        WorkerStatsSnapshot         copy = snapshot_;
        // a stalled worker must not keep advertising its last good rate, and
        // a young one reports what it has measured so far
        const double since_window_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - window_start_)
                .count();
        if (since_window_s > 2.0 || (snapshot_.rate_hz == 0.0 && since_window_s > 0.05)) {
            copy.rate_hz = window_count_ / since_window_s;
        }
        return copy;
    }

private:
    mutable std::mutex                    mutex_;
    WorkerStatsSnapshot                   snapshot_;
    std::chrono::steady_clock::time_point window_start_ = std::chrono::steady_clock::now();
    uint64_t                              window_count_ = 0;
};

} // namespace navigatr
