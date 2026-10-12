// pose_history.h
// Time-ordered ring of recent odometry poses on the host clock, owned by
// localization. Capacity is fixed at construction from the retention
// target and the expected publication rate; the ring overwrites its oldest
// entry when full, and everything older than the retention window counts
// as expired. Lookups binary-search the logical order across the physical
// wrap.
//
// Rules, all explicit:
//   append   a later timestamp appends; the same timestamp revises the
//            newest entry; an earlier timestamp is rejected; a different
//            odometry epoch clears the ring first (poses across an epoch
//            share no frame)
//   poseAt   exact hit returns the entry; a bracketed time interpolates
//            position linearly and yaw along the shortest arc, only when
//            the bracket is no wider than the gap gate; older than the
//            ring is expired; newer than the newest is pending (a consumer
//            may wait boundedly, never clamp); a wrong epoch is refused
//   rate     yaw rate from the bracket pair under the same gates
//   attitude interpolated by slerp between two valid, same-epoch neighbours
//            within the gate; unavailable otherwise

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/time.h"
#include "math/transforms.h"
#include "state/attitude.h"

namespace navigatr
{

struct PoseHistoryConfig {
    int64_t     retention_ms             = 5000;
    std::size_t capacity                 = 1024;   // entries; 5 s at 200 Hz
    int64_t     max_interpolation_gap_ms = 100;    // widest usable bracket
    int64_t     attitude_gap_ms          = 100;    // widest attitude bracket
};

struct PoseHistoryEntry {
    MonotonicTime at;   // host clock
    Pose2D        odom_pose;
    uint64_t      odometry_epoch = 0;
    Attitude      attitude;   // valid only when measured near at
};

enum class AppendResult : uint8_t {
    kAppended,
    kRevised,      // same timestamp as the newest entry
    kOutOfOrder,   // earlier than the newest entry; rejected
    kEpochChange,  // ring cleared, entry appended
    kUnset,        // timestamp not on the host clock; rejected
};

enum class LookupStatus : uint8_t {
    kOk = 0,
    kEmpty,           // nothing retained
    kExpired,         // older than the retained window
    kPending,         // newer than the newest entry; may arrive later
    kGap,             // bracket wider than the gate
    kEpochMismatch,   // requested epoch differs from the retained one
    kUnavailable,     // attitude not measured around the time
    kUnset,           // request not on the host clock
};

inline const char* lookupStatusName(LookupStatus s) {
    switch (s) {
    case LookupStatus::kOk: return "ok";
    case LookupStatus::kEmpty: return "empty";
    case LookupStatus::kExpired: return "expired";
    case LookupStatus::kPending: return "pending";
    case LookupStatus::kGap: return "gap";
    case LookupStatus::kEpochMismatch: return "epoch_mismatch";
    case LookupStatus::kUnavailable: return "unavailable";
    case LookupStatus::kUnset: return "unset";
    }
    return "unknown";
}

struct PoseLookupResult {
    LookupStatus status = LookupStatus::kEmpty;
    Pose2D       odom_pose;
    uint64_t     odometry_epoch = 0;
    bool         exact          = false;   // hit an entry, no interpolation
};

struct RateLookupResult {
    LookupStatus status         = LookupStatus::kEmpty;
    double       yaw_rate_rad_s = 0.0;
    uint64_t     odometry_epoch = 0;
};

struct AttitudeLookupResult {
    LookupStatus status = LookupStatus::kEmpty;
    Attitude     attitude;   // valid only with status kOk
};

struct PoseSampleLookupResult {
    PoseLookupResult     pose;
    RateLookupResult     yaw_rate;
    AttitudeLookupResult attitude;
};

// Read side shared by every consumer of localization history.
class PoseLookup
{
public:
    virtual ~PoseLookup() = default;

    virtual PoseLookupResult     poseAt(MonotonicTime t) const     = 0;
    virtual RateLookupResult     yawRateAt(MonotonicTime t) const  = 0;
    virtual AttitudeLookupResult attitudeAt(MonotonicTime t) const = 0;
    // Mutable publications override this to hold one lock for every part.
    virtual PoseSampleLookupResult sampleAt(MonotonicTime t) const {
        return {poseAt(t), yawRateAt(t), attitudeAt(t)};
    }
};

class PoseHistory : public PoseLookup
{
public:
    explicit PoseHistory(PoseHistoryConfig config = {});

    AppendResult append(const PoseHistoryEntry& entry);
    AppendResult appendAttitude(const Attitude& attitude, uint64_t odometry_epoch);

    void clear();

    PoseLookupResult     poseAt(MonotonicTime t) const override;
    RateLookupResult     yawRateAt(MonotonicTime t) const override;
    AttitudeLookupResult attitudeAt(MonotonicTime t) const override;

    std::size_t size() const { return count_; }
    bool        empty() const { return count_ == 0; }
    std::size_t capacity() const { return ring_.size(); }

    // Logical order, oldest first.
    const PoseHistoryEntry& entry(std::size_t logical_index) const {
        return ring_[(head_ + logical_index) % ring_.size()];
    }
    const PoseHistoryEntry& newest() const { return entry(count_ - 1); }
    const PoseHistoryEntry& oldest() const { return entry(0); }

    // Newest-first copy of up to max_entries, for inspection trails.
    std::vector<PoseHistoryEntry> recent(std::size_t max_entries) const;

    const PoseHistoryConfig& config() const { return config_; }

private:
    // Logical index of the first entry with at >= t, or count_ when none.
    std::size_t lowerBound(MonotonicTime t) const;
    const Attitude& attitudeEntry(std::size_t i) const {
        return attitude_ring_[(attitude_head_ + i) % attitude_ring_.size()];
    }
    bool selectEpoch(uint64_t epoch);

    PoseHistoryConfig             config_;
    std::vector<PoseHistoryEntry> ring_;
    std::size_t                   head_  = 0;   // physical index of the oldest entry
    std::size_t                   count_ = 0;
    std::vector<Attitude>          attitude_ring_;
    std::size_t                   attitude_head_ = 0;
    std::size_t                   attitude_count_ = 0;
    bool                          have_epoch_ = false;
    uint64_t                      epoch_ = 0;
};

} // namespace navigatr
