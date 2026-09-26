// driver.cpp

#include "communigatr/driver.h"

#include <cmath>
#include <cstdint>

namespace communigatr
{

namespace
{

using investigatr::LandmarkSource;
using investigatr::LandmarkStatus;

constexpr double kMmPerMeter = 1000.0;
constexpr double kCdegPerRad = 18000.0 / investigatr::kPi;

// 0 = nothing to select on the Pi.
uint8_t wireId(const investigatr::InputRequest& request) {
    if (!request.landmark || request.landmark_id == 0 || request.landmark_id > 255) {
        return 0;
    }
    return static_cast<uint8_t>(request.landmark_id);
}

investigatr::Pose fromWire(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg) {
    investigatr::Pose pose;
    pose.x       = x_mm / kMmPerMeter;
    pose.y       = y_mm / kMmPerMeter;
    pose.heading = investigatr::wrapAngle(heading_cdeg / kCdegPerRad);
    return pose;
}

// Nearest integer. False when not finite or outside int32.
bool toWire(double value, int32_t& out) {
    const double rounded = std::round(value);
    if (!std::isfinite(rounded) || rounded < INT32_MIN || rounded > INT32_MAX) {
        return false;
    }
    out = static_cast<int32_t>(rounded);
    return true;
}

Seconds fromMs(uint16_t ms) { return ms / 1000.0; }

// Time since the Pi took the reply's ages: round trip plus time since receipt.
Seconds transit(const StateSample& sample, Seconds now) {
    return sample.round_trip + (now - sample.received_at);
}

} // namespace

Driver::Driver(Client& client, const DriverConfig& config) : client_(client), config_(config) {}

void Driver::request(const investigatr::InputRequest& request) {
    request_ = request;
    if (!request_.landmark) {
        request_.landmark_id = 0;
    }
    client_.selectLandmark(wireId(request_));
}

investigatr::InputSnapshot Driver::latest(Seconds now) {
    investigatr::InputSnapshot snapshot;
    const StateSample&         sample = client_.state();

    snapshot.frame     = frameOf(sample);
    snapshot.connected = client_.connected(now);
    snapshot.link_age  = client_.linkAge(now);
    snapshot.landmark  = landmark(sample, snapshot.connected, now);
    if (sample.valid && robotValid(sample.state)) {
        const gatr2::BrainState& s = sample.state;
        snapshot.robot.valid       = true;
        snapshot.robot.pose        = fromWire(s.x_mm, s.y_mm, s.heading_cdeg);
        snapshot.robot.age         = fromMs(s.robot_age_ms) + transit(sample, now);
    }
    return snapshot;
}

PlacementTicket Driver::submitPlacement(const investigatr::Pose& pose) {
    int32_t x_mm         = 0;
    int32_t y_mm         = 0;
    int32_t heading_cdeg = 0;
    if (!std::isfinite(pose.heading) || !toWire(pose.x * kMmPerMeter, x_mm) ||
        !toWire(pose.y * kMmPerMeter, y_mm) ||
        !toWire(investigatr::wrapAngle(pose.heading) * kCdegPerRad, heading_cdeg)) {
        return 0;
    }
    if (heading_cdeg == -18000) {
        heading_cdeg = 18000; // (-18000, 18000]
    }
    return client_.submitPlacement(x_mm, y_mm, heading_cdeg);
}

PlacementResult Driver::placementResult(PlacementTicket ticket) const {
    return client_.placementResult(ticket);
}

PlacementStatus Driver::placementStatus(PlacementTicket ticket) const {
    return client_.placementStatus(ticket);
}

bool Driver::placementPending() const { return client_.placementPending(); }

// A new generation for every (pi_instance, session, epoch, anchor); 0 while
// there is no state of the current session.
investigatr::FrameGeneration Driver::frameOf(const StateSample& sample) {
    if (!sample.valid) {
        return 0;
    }
    const Identity current{sample.pi_instance, sample.session, sample.state.odometry_epoch,
                           sample.state.anchor_revision};

    const bool changed = current.pi_instance != identity_.pi_instance ||
                         current.session != identity_.session ||
                         current.odometry_epoch != identity_.odometry_epoch ||
                         current.anchor_revision != identity_.anchor_revision;
    if (generation_ == 0 || changed) {
        identity_   = current;
        generation_ = generation_ == UINT32_MAX ? 1 : generation_ + 1;
    }
    return generation_;
}

// Status bits describe the estimate; they never acknowledge anything. A
// pending placement makes the cached pose unusable until its anchor shows.
bool Driver::robotValid(const gatr2::BrainState& state) const {
    const uint8_t flags = state.robot_flags;
    const bool    anchor =
        (flags & gatr2::kRobotAnchorCommand) != 0 ||
        (config_.accept_configured_anchor && (flags & gatr2::kRobotAnchorConfigured) != 0);
    return (flags & gatr2::kRobotPoseValid) != 0 && (flags & gatr2::kRobotLocalized) != 0 &&
           (flags & gatr2::kRobotAgeKnown) != 0 && anchor && !client_.placementPending();
}

investigatr::LandmarkEstimate Driver::landmark(const StateSample& sample, bool connected,
                                               Seconds now) const {
    investigatr::LandmarkEstimate estimate;
    if (!request_.landmark) {
        return estimate;
    }
    estimate.id = request_.landmark_id;
    if (wireId(request_) == 0) {
        estimate.status = LandmarkStatus::kUnknownLandmark;
        return estimate;
    }
    if (!connected) {
        estimate.status = LandmarkStatus::kStale;
        return estimate;
    }
    switch (client_.selection()) {
    case SelectionState::kUnknownLandmark:
        estimate.status = LandmarkStatus::kUnknownLandmark;
        return estimate;
    case SelectionState::kUnsupported:
        estimate.status = LandmarkStatus::kUnsupported;
        return estimate;
    case SelectionState::kActive:
        break;
    default:
        estimate.status = LandmarkStatus::kPending;
        return estimate;
    }

    const gatr2::BrainState& s = sample.state;
    if (s.landmark_source == gatr2::kLandmarkSourceNominal) {
        estimate.source = LandmarkSource::kNominal;
    } else if (s.landmark_source == gatr2::kLandmarkSourceObserved) {
        estimate.source    = LandmarkSource::kObserved;
        estimate.age_known = true;
        estimate.age       = fromMs(s.landmark_age_ms) + transit(sample, now);
    } else {
        estimate.status = LandmarkStatus::kUnavailable;
        return estimate;
    }
    estimate.status = LandmarkStatus::kAvailable;
    estimate.pose   = fromWire(s.lm_x_mm, s.lm_y_mm, s.lm_heading_cdeg);
    return estimate;
}

} // namespace communigatr
