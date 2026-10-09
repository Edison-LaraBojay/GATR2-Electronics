// robot_state_feed.cpp

#include "state/robot_state_feed.h"

#include "diagnostics/hub.h"

namespace navigatr
{
namespace
{

DiagRobotState diagRecordOf(const RobotState& state, bool stationary, bool advanced,
                            uint64_t publication) {
    DiagRobotState d;
    d.publication = publication;
    d.advanced    = advanced;
    if (state.measuredAt.isSet()) {
        d.measured_ms    = state.measuredAt.ms;
        d.measured_clock = state.measuredAt.domain == ClockDomain::kHost ? DiagClock::kPiHost
                                                                         : DiagClock::kPico;
    }
    d.measured_host_ms =
        state.measuredAtHost.domain == ClockDomain::kHost ? state.measuredAtHost.ms : -1;
    d.odom_x_m          = state.odom_pose.x_m;
    d.odom_y_m          = state.odom_pose.y_m;
    d.odom_heading_rad  = state.odom_pose.heading_rad;
    const Pose2D field  = state.fieldPose();
    d.field_x_m         = field.x_m;
    d.field_y_m         = field.y_m;
    d.field_heading_rad = field.heading_rad;
    d.vx_m_s            = static_cast<float>(state.vx_m_s);
    d.vy_m_s            = static_cast<float>(state.vy_m_s);
    d.yaw_rate_rad_s    = static_cast<float>(state.yaw_rate_rad_s);
    d.confidence        = static_cast<float>(state.confidence);
    if (state.attitude.valid) {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        attitudeEuler(state.attitude, roll, pitch, yaw);
        d.roll_rad  = static_cast<float>(roll);
        d.pitch_rad = static_cast<float>(pitch);
    }
    d.odometry_epoch         = state.odometry_epoch;
    d.anchor_revision        = state.anchor_revision;
    d.placement_sequence     = state.placement_sequence;
    d.placement_session      = state.placement_session;
    d.valid                  = state.valid;
    d.initialized            = state.initialized;
    d.attitude_valid         = state.attitude.valid;
    d.attitude_assumed_level = !state.attitude.valid && state.attitude.assumed_level;
    d.stationary             = stationary;
    return d;
}

} // namespace

void RobotStateFeed::setDiagnostics(std::shared_ptr<DiagnosticsHub> hub) {
    hub_        = std::move(hub);
    hub_source_ = hub_ != nullptr ? hub_->sourceId("localization") : 0;
}

void RobotStateFeed::publish(const RobotState& state, const LocalizationStatus& status,
                             bool advanced, uint64_t publication) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Epoch changes invalidate both histories even if the estimator has no
        // usable host pose yet. Reset and publication occur under this one lock.
        if (state.odometry_epoch != latest_.odometry_epoch) history_.clear();
        latest_      = state;
        status_      = status;
        publication_ = publication;
        if (advanced && state.valid && state.measuredAtHost.isSet()) {
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
    // outside the feed lock: readers never wait on the hub
    if (hub_ != nullptr && hub_->wants(DiagKind::kRobotState)) {
        DiagRecord record;
        record.kind    = DiagKind::kRobotState;
        record.source  = hub_source_;
        record.payload = diagRecordOf(state, status.stationary(), advanced, publication);
        hub_->post(std::move(record));
    }
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

std::size_t RobotStateFeed::historySize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return history_.size();
}

} // namespace navigatr
