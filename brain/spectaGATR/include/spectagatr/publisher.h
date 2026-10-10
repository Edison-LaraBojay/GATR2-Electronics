// publisher.h
// Portable pose publisher. updatePose keeps only the newest pose in a
// lock-free slot; one transport task pulls SGP1 lines (spectaGATR/protocol)
// from it at a capped rate. No PROS code here; the PROS transport is
// pros_publisher.h.
//
// updatePose and resetTrail: any task, any rate, never block, never allocate.
// Concurrent calls are safe; a pose torn by two overlapping writers is never
// sent (the next update heals the slot).
// begin, nextLine, recordWrite: the one transport task only.

#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "spectagatr/pose.h"
#include "spectagatr/protocol/pose_line.h"

namespace spectagatr
{

using protocol::Epoch;
using protocol::Sequence;
using protocol::SessionId;

// Publisher clock in milliseconds; pros::millis on the Brain.
using Clock = Milliseconds (*)();

constexpr Hertz       kDefaultRateHz = 10;
constexpr Hertz       kMinRateHz     = 1;
constexpr Hertz       kMaxRateHz     = 50;
constexpr Meters      kMaxCoordinate = protocol::kMaxCoordinateMm / 1000.0; // 2000 m
constexpr std::size_t kLineCapacity  = protocol::kMaxLineLength + 1;        // line plus NUL

struct PublisherStats {
    uint32_t updates      = 0; // accepted updatePose calls
    uint32_t rejected     = 0; // non-finite or out of range; the previous pose stays
    uint32_t lines        = 0; // lines handed to the transport
    uint32_t overwritten  = 0; // accepted poses replaced before a line carried them
    uint32_t write_errors = 0; // failed or short transport writes
};

// Nonzero session id hashed (splitmix64) from values that differ between runs.
SessionId makeSessionId(const uint64_t* inputs, std::size_t count);

class Publisher {
  public:
    explicit Publisher(Clock clock, Hertz rate = kDefaultRateHz);

    Publisher(const Publisher&)            = delete;
    Publisher& operator=(const Publisher&) = delete;

    // Heading only: stored as Attitude{0, 0, heading}, full attitude flag off.
    void updatePose(Meters x, Meters y, Radians heading);
    // Full attitude, flag on.
    void updatePose(Meters x, Meters y, const Attitude& attitude);

    // Explicit pose reset or re-placement: poses supplied after this call
    // carry the next epoch and the viewer starts a new trail.
    void resetTrail();

    // Clamped to [kMinRateHz, kMaxRateHz]; non-finite gives the default.
    void         setRate(Hertz rate);
    Milliseconds period() const; // minimum spacing of lines, round(1000 / rate)
    Hertz        rate() const;   // 1000 / period

    PublisherStats stats() const;
    SessionId      session() const; // 0 before begin

    // Transport task. Sets the session (nonzero) and the next line's seq.
    void begin(SessionId session, Sequence first_seq = 0);

    // Writes the next line, '\n' included, and returns its length; 0 when
    // there is no session yet, no new pose since the last line, the period
    // has not elapsed, a writer is busy, or cap < kLineCapacity.
    std::size_t nextLine(Milliseconds now, char* out, std::size_t cap);

    // Transport task, after writing a line: false for a failed or short write.
    void recordWrite(bool ok);

  private:
    static constexpr std::size_t kSlotWords = 8; // 7 data words and a checksum

    void store(Meters x, Meters y, const Attitude& attitude, bool full_attitude);
    bool load(uint32_t& version, uint32_t (&words)[kSlotWords]) const;

    Clock clock_;

    // Slot: seqlock over relaxed atomic words; odd while a writer is inside.
    std::atomic<uint32_t> slot_seq_{0};
    std::atomic<uint32_t> slot_[kSlotWords]{};

    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint32_t> period_ms_{0};
    std::atomic<uint32_t> session_{0};

    std::atomic<uint32_t> updates_{0};
    std::atomic<uint32_t> rejected_{0};
    std::atomic<uint32_t> lines_{0};
    std::atomic<uint32_t> overwritten_{0};
    std::atomic<uint32_t> write_errors_{0};

    // Transport task only.
    Sequence     next_seq_     = 0;
    uint32_t     sent_version_ = 0; // slot version of the last line, 0 = none
    bool         scheduled_    = false;
    Milliseconds due_          = 0;
};

} // namespace spectagatr
