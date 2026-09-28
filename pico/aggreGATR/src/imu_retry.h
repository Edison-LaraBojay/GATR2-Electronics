// imu_retry.h
// IMU bring-up retry bookkeeping and the IMU fields of the Pico status frame,
// shared by both IMU drivers. No hardware dependencies.

#pragma once
#include <stdint.h>

namespace imu
{

constexpr uint8_t  kQuickAttempts = 5;
constexpr uint32_t kBackoffMinUs  = 500000;
constexpr uint32_t kBackoffMaxUs  = 8000000;
constexpr uint32_t kSlowRetryUs   = 30000000;
constexpr uint32_t kStableUs      = 1000000; // up this long ends an episode

// Attempts to bring one IMU up. An episode starts at enable, at a reinit
// request, or when a stable device fails, and ends when an attempt stays up
// for kStableUs. After failed attempt n the next one waits 0.5 s * 2^(n-1),
// capped at 8 s, while n < kQuickAttempts; after that the IMU is failed and
// retries every kSlowRetryUs. Losing a stable device waits kBackoffMinUs.
class RetryPolicy {
  public:
    // New episode: no attempts, no failure.
    void restart();

    // An attempt starts. Returns its number in the episode.
    uint16_t attempt();

    // The current attempt, or a running device, failed. reason is a
    // translagatr::PicoImuReason. Returns the wait before the next attempt.
    uint32_t failed(uint8_t reason);

    // The device stayed up for kStableUs. Idempotent.
    void stable();

    uint16_t attempts() const { return attempts_; }
    uint8_t  reason() const { return reason_; }       // last failure in this episode
    bool     exhausted() const { return exhausted_; } // quick attempts used, not stable since

  private:
    bool     open_      = false;
    bool     exhausted_ = false;
    uint16_t attempts_  = 0;
    uint8_t  reason_    = 0;
};

// What a driver knows about its device right now.
struct DeviceView {
    bool enabled = false;
    bool up      = false; // running, reports flowing
    bool ready   = false; // yaw output available
    bool waiting = false; // between attempts
};

// translagatr::PicoImuState. Aligning is up but not ready.
uint8_t wireState(const DeviceView& d, const RetryPolicy& retry);

// translagatr::PicoImuReason: the last failure, none while disabled or up.
uint8_t wireReason(const DeviceView& d, const RetryPolicy& retry);

} // namespace imu
