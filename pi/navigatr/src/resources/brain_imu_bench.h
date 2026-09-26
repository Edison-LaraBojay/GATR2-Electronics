#pragma once
#include <cstdint>
#include "core/time.h"

namespace navigatr {
// Opt-in bench mailbox, written by Commands and read by Localization on the
// same worker. Brain stamps detect duplicates/restarts only; they are never
// interpreted as Pico timestamps or precision-synchronized measurement time.
struct BrainImuBench {
    bool valid = false;
    uint32_t session = 0, stamp_ms = 0;
    int32_t rotation_mdeg = 0;
    uint64_t sequence = 0, epoch = 0;
    MonotonicTime received;

    void reset(uint32_t next_session = 0) {
        valid = false;
        session = next_session;
        received = {};
        ++epoch;
    }
    void accept(uint32_t s, uint8_t flags, uint32_t stamp, int32_t rotation,
                MonotonicTime now) {
        if (session != s) reset(s);
        if ((flags & 1u) == 0) {
            if (valid) reset(s);
            return;
        }
        if (valid && stamp == stamp_ms) return; // retained sample cannot refresh health
        if (valid && stamp < stamp_ms) reset(s);
        valid = true;
        stamp_ms = stamp;
        rotation_mdeg = rotation;
        received = now;
        ++sequence;
    }
};
} // namespace navigatr
