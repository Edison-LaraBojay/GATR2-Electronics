// imu_retry.cpp

#include "imu_retry.h"

#include "frames.h"

namespace imu
{

void RetryPolicy::restart() {
    open_      = true;
    exhausted_ = false;
    attempts_  = 0;
    reason_    = translagatr::kPicoImuReasonNone;
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
    reason_    = translagatr::kPicoImuReasonNone;
}

uint8_t wireState(const DeviceView& d, const RetryPolicy& retry) {
    if (!d.enabled) {
        return translagatr::kPicoImuDisabled;
    }
    if (d.ready) {
        return translagatr::kPicoImuReady;
    }
    if (d.up) {
        return translagatr::kPicoImuAligning;
    }
    if (retry.exhausted()) {
        return translagatr::kPicoImuFailed;
    }
    if (d.waiting && retry.reason() != translagatr::kPicoImuReasonNone) {
        return translagatr::kPicoImuRetrying;
    }
    return translagatr::kPicoImuInitializing;
}

uint8_t wireReason(const DeviceView& d, const RetryPolicy& retry) {
    return d.enabled && !d.up ? retry.reason() : static_cast<uint8_t>(translagatr::kPicoImuReasonNone);
}

} // namespace imu
