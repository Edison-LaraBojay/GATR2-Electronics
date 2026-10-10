// pose_tracker.h
// The receiver rules of PROTOCOL.md for decoded lines: drops counted per reason, a new session or
// epoch restarts the trail, and within one session and epoch only a newer seq (modulo 2^32) is
// accepted; a seq more than 64 behind is a restarted publisher whose id collided. Each accepted
// pose gets the receiver's own clock; the publisher's t_ms is passed on,
// never compared with it. Pure: no clock, no I/O.

#pragma once
#include <cstdint>
#include <optional>

#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::receiver::feed
{

using RxMillis = double; // receiver monotonic clock, ms

struct TrackerCounts {
    uint64_t poses        = 0; // accepted
    uint64_t bad_crc      = 0;
    uint64_t malformed    = 0;
    uint64_t out_of_range = 0; // includes non-finite values a publisher could not encode
    uint64_t repeated     = 0; // same seq again
    uint64_t out_of_order = 0; // older seq
    uint64_t sessions     = 0; // publisher runs seen
};

struct TrackedPose {
    protocol::PoseRecord record;
    bool                 restart = false; // first pose of a new session or epoch
    RxMillis             rx_ms   = 0;
};

enum class Verdict : uint8_t {
    kAccepted,
    kBadCrc,
    kMalformed,
    kOutOfRange,
    kRepeated,
    kOutOfOrder
};

class PoseTracker {
  public:
    // One decoded line, received at rx_ms. The pose when accepted.
    std::optional<TrackedPose> offer(const protocol::DecodeResult& decoded, RxMillis rx_ms);

    // Why the last offer was or was not accepted.
    Verdict lastVerdict() const { return verdict_; }

    // A replayed stream starts over: the next pose is accepted as a restart.
    void forgetOrdering() { last_.reset(); }

    const TrackerCounts&    counts() const { return counts_; }
    std::optional<RxMillis> lastRx() const { return last_rx_; }

  private:
    struct Position {
        protocol::SessionId session = 0;
        protocol::Epoch     epoch   = 0;
        protocol::Sequence  seq     = 0;
    };

    std::optional<Position> last_;
    std::optional<RxMillis> last_rx_;
    TrackerCounts           counts_;
    Verdict                 verdict_ = Verdict::kAccepted;
};

// int32(seq - last) as PROTOCOL.md defines newer: positive is newer, across the u32 wrap.
int32_t sequenceDistance(protocol::Sequence seq, protocol::Sequence last);

} // namespace spectagatr::receiver::feed
