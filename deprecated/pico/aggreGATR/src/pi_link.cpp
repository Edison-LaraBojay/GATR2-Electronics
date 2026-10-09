// pi_link.cpp

#include "pi_link.h"

namespace pilink
{

uint16_t makeBootId(uint32_t entropy, uint64_t time_us) {
    uint32_t x = entropy ^ static_cast<uint32_t>(time_us) ^
                 (static_cast<uint32_t>(time_us >> 32) * 0x9E3779B9u);
    // murmur3 finalizer
    x ^= x >> 16;
    x *= 0x85EBCA6Bu;
    x ^= x >> 13;
    x *= 0xC2B2AE35u;
    x ^= x >> 16;
    const uint16_t id = static_cast<uint16_t>(x ^ (x >> 16));
    return id != 0 ? id : static_cast<uint16_t>(0xA5A5u);
}

StatusSchedule::StatusSchedule(uint32_t period_us, uint32_t min_gap_us, uint32_t guard_us)
    : period_us_(period_us), min_gap_us_(min_gap_us), guard_us_(guard_us) {}

bool StatusSchedule::ready(uint32_t now_us, uint32_t next_tick_us, bool tx_empty) const {
    if (!tx_empty) {
        return false;
    }
    if (static_cast<int32_t>(next_tick_us - now_us) < static_cast<int32_t>(guard_us_)) {
        return false;
    }
    if (!sent_) {
        return true;
    }
    const uint32_t since = now_us - last_us_;
    return since >= period_us_ || (requested_ && since >= min_gap_us_);
}

void StatusSchedule::sent(uint32_t now_us) {
    sent_      = true;
    last_us_   = now_us;
    requested_ = false;
}

void DiagSchedule::setRate(uint8_t hz) {
    if (hz != hz_ && hz != 0) {
        due_ = true;
    }
    hz_ = hz;
}

bool DiagSchedule::ready(uint32_t now_us, uint32_t next_tick_us, bool tx_empty) const {
    if (hz_ == 0 || !tx_empty) {
        return false;
    }
    if (static_cast<int32_t>(next_tick_us - now_us) < static_cast<int32_t>(guard_us_)) {
        return false;
    }
    return due_ || now_us - last_us_ >= 1000000u / hz_;
}

void DiagSchedule::sent(uint32_t now_us) {
    last_us_ = now_us;
    due_     = false;
}

} // namespace pilink
