// robot_state_feed.h
// Publication side of localization. One writer (the localization stage),
// any number of readers on any thread: the latest RobotState snapshot, the
// localization status, and timestamped history lookups. Every read copies
// under the lock or answers the query under the lock; no reference into
// the mutable history ever escapes.

#pragma once
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "state/pose_history.h"
#include "state/robot_state.h"

namespace navigatr
{

// Per observation function readiness, for publishers and inspection.
struct ObservationFunctionStatus {
    std::string id;
    std::string type;
    bool        ready = false;   // calibration complete, producing
    std::string note;            // calibrating, waiting for wheels, ...
};

struct LocalizationStatus {
    std::vector<ObservationFunctionStatus> functions;
    std::string                            estimator_type;
    uint64_t                               updates      = 0;   // accepted estimator advances
    uint64_t                               history_size = 0;
    bool                                   clock_mapped = false;   // device to host mapping valid

    bool allReady() const {
        for (const auto& f : functions) {
            if (!f.ready) {
                return false;
            }
        }
        return true;
    }

    const ObservationFunctionStatus* find(const std::string& id) const {
        for (const auto& f : functions) {
            if (f.id == id) {
                return &f;
            }
        }
        return nullptr;
    }
};

struct LocalizationSnapshot {
    RobotState robot;
    LocalizationStatus status;
    uint64_t publication = 0;
    std::vector<PoseHistoryEntry> trail;
};

struct LocalizationSampleSnapshot {
    LocalizationSnapshot current;
    PoseSampleLookupResult sample;
};

class RobotStateFeed : public PoseLookup
{
public:
    explicit RobotStateFeed(PoseHistoryConfig history = {}) : history_(history) {}

    // Writer side.
    void publish(const RobotState& state, const LocalizationStatus& status,
                 bool append_history, uint64_t publication);
    void resetHistory();

    // Reader side.
    RobotState         latest() const;
    LocalizationStatus status() const;
    uint64_t           publication() const;   // increments on every publish
    LocalizationSnapshot snapshot(std::size_t max_trail_entries = 0) const;
    LocalizationSampleSnapshot sampleSnapshotAt(MonotonicTime t) const;

    PoseLookupResult     poseAt(MonotonicTime t) const override;
    RateLookupResult     yawRateAt(MonotonicTime t) const override;
    AttitudeLookupResult attitudeAt(MonotonicTime t) const override;
    PoseSampleLookupResult sampleAt(MonotonicTime t) const override;

    std::size_t                   historySize() const;
    const PoseHistoryConfig&      historyConfig() const { return history_.config(); }

private:
    mutable std::mutex mutex_;
    RobotState         latest_;
    LocalizationStatus status_;
    PoseHistory        history_;
    uint64_t           publication_ = 0;
};

} // namespace navigatr
