// imu_retry.cpp

#include "imu_retry.h"

#include "imu_status.h"

namespace imu
{

void RetryPolicy::restart() {
    open_      = true;
    exhausted_ = false;
    attempts_  = 0;
    reason_    = imu::kReasonNone;
}

uint16_t RetryPolicy::attempt() {
    if (!open_) {
        restart();
    }
    if (attempts_ < UINT16_MAX) {
        ++attempts_;
    }
    return attempts_;
}

uint32_t RetryPolicy::failed(uint8_t reason) {
    if (!open_) {
        restart();
        reason_ = reason;
        return kBackoffMinUs;
    }
    reason_          = reason;
    const uint16_t n = attempts_ > 0 ? attempts_ : 1;
    if (n >= kQuickAttempts) {
        exhausted_ = true;
        return kSlowRetryUs;
    }
    uint32_t wait = kBackoffMinUs;
    for (uint16_t i = 1; i < n && wait < kBackoffMaxUs; ++i) {
        wait *= 2;
    }
    return wait < kBackoffMaxUs ? wait : kBackoffMaxUs;
}

void RetryPolicy::stable() {
    open_      = false;
    exhausted_ = false;
    reason_    = imu::kReasonNone;
}

uint8_t statusState(const DeviceView& d, const RetryPolicy& retry) {
    if (!d.enabled) {
        return imu::kStateDisabled;
    }
    if (d.ready) {
        return imu::kStateReady;
    }
    if (d.up) {
        return imu::kStateAligning;
    }
    if (retry.exhausted()) {
        return imu::kStateFailed;
    }
    if (d.waiting && retry.reason() != imu::kReasonNone) {
        return imu::kStateRetrying;
    }
    return imu::kStateInitializing;
}

uint8_t statusReason(const DeviceView& d, const RetryPolicy& retry) {
    return d.enabled && !d.up ? retry.reason() : static_cast<uint8_t>(imu::kReasonNone);
}

} // namespace imu
