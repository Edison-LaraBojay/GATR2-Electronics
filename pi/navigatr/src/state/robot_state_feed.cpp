// robot_state_feed.cpp

#include "state/robot_state_feed.h"

namespace navigatr
{

void RobotStateFeed::publish(const RobotState& state, const LocalizationStatus& status,
                             bool append_history, uint64_t publication) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Epoch changes invalidate both histories even if the estimator has no
    // usable host pose yet. Reset and publication occur under this one lock.
    if (state.odometry_epoch != latest_.odometry_epoch) history_.clear();
    latest_      = state;
    status_      = status;
    publication_ = publication;
    if (append_history && state.valid && state.measuredAtHost.isSet()) {
        PoseHistoryEntry entry;
        entry.at             = state.measuredAtHost;
        entry.odom_pose      = state.odom_pose;
        entry.odometry_epoch = state.odometry_epoch;
        entry.attitude       = state.attitude;
        history_.append(entry);
    }
    history_.appendAttitude(state.attitude, state.odometry_epoch);
    status_.history_size = history_.size();
}

void RobotStateFeed::resetHistory() {
    std::lock_guard<std::mutex> lock(mutex_);
    history_.clear();
    status_.history_size = 0;
}

RobotState RobotStateFeed::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

LocalizationStatus RobotStateFeed::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

uint64_t RobotStateFeed::publication() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return publication_;
}

LocalizationSnapshot RobotStateFeed::snapshot(std::size_t max_trail_entries) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {latest_, status_, publication_, history_.recent(max_trail_entries)};
}

LocalizationSampleSnapshot RobotStateFeed::sampleSnapshotAt(MonotonicTime t) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {{latest_, status_, publication_, {}}, history_.sampleAt(t)};
}

PoseSampleLookupResult RobotStateFeed::sampleAt(MonotonicTime t) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.sampleAt(t);
}

PoseLookupResult RobotStateFeed::poseAt(MonotonicTime t) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.poseAt(t);
}

RateLookupResult RobotStateFeed::yawRateAt(MonotonicTime t) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.yawRateAt(t);
}

AttitudeLookupResult RobotStateFeed::attitudeAt(MonotonicTime t) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.attitudeAt(t);
}

std::vector<PoseHistoryEntry> RobotStateFeed::recentHistory(std::size_t max_entries) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.recent(max_entries);
}

std::size_t RobotStateFeed::historySize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.size();
}

} // namespace navigatr
