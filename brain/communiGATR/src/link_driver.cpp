// link_driver.cpp

#include "communigatr/link_driver.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "translaGATR/link_documents.h"

namespace communigatr
{

namespace
{

using investigatr::EstimateSource;
using investigatr::Pose;
using investigatr::RobotStatus;

constexpr double kMmPerMeter = 1000.0;
constexpr double kCdegPerRad = 18000.0 / investigatr::kPi;

investigatr::Radians fromCdeg(int32_t cdeg) { return investigatr::wrapAngle(cdeg / kCdegPerRad); }

Pose fromWire(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg) {
    return Pose{x_mm / kMmPerMeter, y_mm / kMmPerMeter, fromCdeg(heading_cdeg)};
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

EstimateSource sourceOf(uint8_t code) {
    switch (code) {
    case translagatr::kEstimateSourceNominal: return EstimateSource::kNominal;
    case translagatr::kEstimateSourceObserved: return EstimateSource::kObserved;
    default: return EstimateSource::kNone;
    }
}

} // namespace

LinkDriver::LinkDriver(Client& client, const LinkDriverConfig& config)
    : client_(client), config_(config) {}

investigatr::RobotState LinkDriver::robot(Seconds now) {
    investigatr::RobotState out;
    out.connected = client_.connected(now);
    out.link_age  = client_.linkAge(now);
    if (!out.connected) {
        out.status = RobotStatus::kNoLink;
        return out;
    }
    if (client_.profileConfigured() && !client_.profileApplied()) {
        out.status = RobotStatus::kNoProfile;
        return out;
    }
    const StateSample&       sample = client_.state();
    const translagatr::BrainState& s      = sample.state;
    if (s.calibration == translagatr::kCalibrationRunning ||
        s.calibration == translagatr::kCalibrationWaitingStill ||
        s.calibration == translagatr::kCalibrationWaitingData) {
        out.status = RobotStatus::kCalibrating;
        return out;
    }
    const uint8_t flags  = s.robot_flags;
    const bool    anchor = (flags & translagatr::kRobotAnchorCommand) != 0 ||
                        (config_.accept_configured_anchor &&
                         (flags & translagatr::kRobotAnchorConfigured) != 0);
    // A pending placement is about to replace the anchor.
    if ((flags & translagatr::kRobotLocalized) == 0 || !anchor || client_.placementPending()) {
        out.status = RobotStatus::kUnplaced;
        return out;
    }
    if ((flags & translagatr::kRobotPoseValid) == 0 || (flags & translagatr::kRobotAgeKnown) == 0) {
        out.status = RobotStatus::kNoPose;
        return out;
    }
    out.status = RobotStatus::kValid;
    out.pose   = fromWire(s.x_mm, s.y_mm, s.heading_cdeg);
    out.age    = s.robot_age_ms / 1000.0 + sample.round_trip + (now - sample.received_at);
    out.frame  = generationOf(
        Identity{sample.pi_instance, sample.session, s.odometry_epoch, s.anchor_revision});
    return out;
}

bool LinkDriver::field(investigatr::Field& out) {
    const FieldPublication& p = client_.field();
    if (p.generation == 0) {
        return false;
    }
    if (out.generation == p.generation) {
        return true;
    }
    const uint16_t map_len = static_cast<uint16_t>(p.map.size());
    const uint16_t est_len = static_cast<uint16_t>(p.estimate.size());
    translagatr::FieldMapHeader      map;
    translagatr::FieldEstimateHeader estimate;
    if (!translagatr::decodeFieldMapHeader(p.map.data(), map_len, map) ||
        !translagatr::decodeFieldEstimateHeader(p.estimate.data(), est_len, estimate) ||
        estimate.object_count != map.object_count) {
        return false; // the client publishes only validated pairs
    }

    out.generation   = p.generation;
    out.map.id       = p.map_id;
    out.map.revision = map.revision;
    out.bounds       = investigatr::Bounds{map.min_x_mm / kMmPerMeter, map.min_y_mm / kMmPerMeter,
                                     map.max_x_mm / kMmPerMeter, map.max_y_mm / kMmPerMeter};
    out.frame        = generationOf(
        Identity{p.pi_instance, p.session, estimate.odometry_epoch, estimate.anchor_revision});
    out.received_at = p.completed_at;

    // Upper bound of the time between the snapshot and completion.
    const Seconds transit = p.completed_at - p.snapshot_after;
    out.objects.resize(map.object_count);
    for (uint16_t i = 0; i < map.object_count; ++i) {
        translagatr::FieldObjectRecord   r;
        translagatr::FieldEstimateRecord e;
        translagatr::decodeFieldObjectRecord(p.map.data(), map_len, i, r);
        translagatr::decodeFieldEstimateRecord(p.estimate.data(), est_len, i, e);

        investigatr::FieldObject& o = out.objects[i];
        o                           = investigatr::FieldObject{};
        o.id                        = r.object_id;
        o.kind      = r.kind == translagatr::kObjectLandmark ? investigatr::ObjectKind::kLandmark
                                                       : investigatr::ObjectKind::kFixed;
        o.obstacle  = (r.flags & translagatr::kObjectObstacle) != 0;
        o.estimated = (r.flags & translagatr::kObjectEstimated) != 0;
        o.reference = (r.flags & translagatr::kObjectReference) != 0;
        o.nominal   = fromWire(r.x_mm, r.y_mm, r.heading_cdeg);
        o.box.center = Pose{r.box_x_mm / kMmPerMeter, r.box_y_mm / kMmPerMeter,
                            fromCdeg(r.box_heading_cdeg)};
        o.box.length = r.box_length_mm / kMmPerMeter;
        o.box.width  = r.box_width_mm / kMmPerMeter;

        o.source = sourceOf(e.source);
        o.valid  = (e.flags & translagatr::kEstimateValid) != 0;
        if (o.valid) {
            o.pose = fromWire(e.x_mm, e.y_mm, e.heading_cdeg);
        }
        if (o.source == EstimateSource::kObserved) {
            o.age_known = true;
            o.age       = e.age_ms / 1000.0 + transit;
        }
    }
    return true;
}

void LinkDriver::reportPath(investigatr::CommandId command, const investigatr::Path& path) {
    uint8_t mode = translagatr::kPathNone;
    if (!path.empty()) {
        mode = path.mode == investigatr::PlanMode::kAvoiding ? uint8_t{translagatr::kPathAvoiding}
                                                             : uint8_t{translagatr::kPathDirect};
    }
    std::vector<translagatr::PathPoint> points;
    points.reserve(path.segments.size() + 1);
    const auto add = [&points](const Pose& p) {
        translagatr::PathPoint point;
        if (!toWire(p.x * kMmPerMeter, point.x_mm) || !toWire(p.y * kMmPerMeter, point.y_mm)) {
            return false;
        }
        if (points.empty() || points.back().x_mm != point.x_mm ||
            points.back().y_mm != point.y_mm) {
            points.push_back(point);
        }
        return true;
    };
    for (const investigatr::PathSegment& s : path.segments) {
        if (!add(s.start) || (s.kind == investigatr::SegmentKind::kTranslate && !add(s.end))) {
            return;
        }
    }
    client_.reportPath(command, mode, points.data(), points.size());
}

PlacementTicket LinkDriver::place(const investigatr::Pose& pose) {
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

bool LinkDriver::setProfile(const RobotProfile& profile) {
    const ProfileDocument doc = makeProfileDocument(profile);
    const bool            ok  = client_.setProfile(doc);
    const ProfileDocument& held = client_.config().profile;
    if (held.len == doc.len && held.reason == doc.reason && held.detail == doc.detail &&
        std::memcmp(held.bytes, doc.bytes, doc.len) == 0) {
        profile_     = profile;
        has_profile_ = true;
    }
    return ok;
}

// Equal identities share a number, so a field and a robot pose under the
// same anchor compare equal. New identities count up; a forgotten one comes
// back with a new number, which only ever reads as a frame change.
investigatr::FrameGeneration LinkDriver::generationOf(const Identity& id) {
    for (const Numbered& n : frames_) {
        const Identity& k = n.identity;
        if (n.generation != 0 && k.pi_instance == id.pi_instance && k.session == id.session &&
            k.odometry_epoch == id.odometry_epoch && k.anchor_revision == id.anchor_revision) {
            return n.generation;
        }
    }
    last_generation_ = last_generation_ == UINT32_MAX ? 1 : last_generation_ + 1;
    for (int i = 3; i > 0; --i) {
        frames_[i] = frames_[i - 1];
    }
    frames_[0] = Numbered{id, last_generation_};
    return last_generation_;
}

} // namespace communigatr
