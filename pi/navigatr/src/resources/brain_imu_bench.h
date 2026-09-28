#pragma once
#include <cstdint>
#include "core/time.h"

namespace navigatr {
// Opt-in bench mailbox, written by Commands and read by Localization on the
// same worker. Brain stamps detect duplicates/restarts only; they are never
// interpreted as Pico timestamps or precision-synchronized measurement time.
//
// The attitude part holds the newest Brain VEX IMU tilt from TELEMETRY, in
// the robot frame (roll about +x, left side up; pitch about +y, nose down;
// the Brain applies its IMU mounting), stamped with the Pi arrival time. It
// is display attitude only: localization folds it as tilt, never as motion.
struct BrainImuBench {
    bool valid = false;
    uint32_t session = 0, stamp_ms = 0;
    int32_t rotation_mdeg = 0;
    uint64_t sequence = 0, epoch = 0;
    MonotonicTime received;

    bool attitude_valid = false;
    int16_t roll_cdeg = 0, pitch_cdeg = 0;
    uint32_t attitude_stamp_ms = 0;
    uint64_t attitude_sequence = 0;
    MonotonicTime attitude_received;

    void reset(uint32_t next_session = 0) {
        valid = false;
        session = next_session;
        received = {};
        attitude_valid = false;
        attitude_received = {};
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
    // One TELEMETRY report of the current session; a repeated Brain stamp
    // is not new. A report without the attitude group clears the tilt here
    // and publishes nothing, so the estimator keeps the last published tilt:
    // measured until it is max_age_ms old, then stale.
    void acceptAttitude(bool present, uint32_t stamp, int16_t roll, int16_t pitch,
                        MonotonicTime now) {
        if (!present) {
            attitude_valid = false;
            return;
        }
        if (attitude_valid && stamp == attitude_stamp_ms) return;
        attitude_valid = true;
        attitude_stamp_ms = stamp;
        roll_cdeg = roll;
        pitch_cdeg = pitch;
        attitude_received = now;
        ++attitude_sequence;
    }
};
} // namespace navigatr
