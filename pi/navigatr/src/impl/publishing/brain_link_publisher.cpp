// brain_link_publisher.cpp

#include "impl/publishing/brain_link_publisher.h"

#include <algorithm>
#include <cmath>

#include "math/angles.h"
#include "resources/resource_store.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

namespace
{

int32_t toWireMm(double meters) {
    const double mm = std::round(meters * 1000.0);
    if (std::isnan(mm)) {
        return 0;
    }
    return static_cast<int32_t>(std::clamp(mm, -2147483648.0, 2147483647.0));
}

uint16_t clampAgeMs(int64_t ms) {
    return static_cast<uint16_t>(std::clamp<int64_t>(ms, 0, 65535));
}

bool hostSet(MonotonicTime t) { return t.domain == ClockDomain::kHost; }

} // namespace

std::unique_ptr<Publishing> BrainLinkPublisher::create(const ConfigNode& node,
                                                       SlotInitializationContext& context,
                                                       std::string& err) {
    auto publisher = std::make_unique<BrainLinkPublisher>();

    const ResourceId link_id{node.child("Serial").attr("resource_id")};
    if (link_id.empty()) {
        err = node.path() + ": needs <Serial resource_id=.../>";
        return nullptr;
    }
    if (context.commands_type.value != "brain_link" || context.commands_serial != link_id) {
        err = node.path() + ": brain_link publishing answers brain_link commands on the "
                            "same Serial resource (" + link_id.value + ")";
        return nullptr;
    }
    if (context.resources == nullptr || context.sensors == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    std::string inner;
    publisher->link_ = context.resources->require<SerialLink>(link_id, inner);
    if (publisher->link_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }
    publisher->diagnostics_id_ = link_id.value;
    publisher->world_noop_     = context.world_estimation_noop;

    const ConfigNode health = node.child("Health");
    if (health.valid()) {
        if (!health.getInt("fresh_ms", 150, publisher->fresh_ms_, err)) {
            return nullptr;
        }
        if (publisher->fresh_ms_ <= 0) {
            err = health.path() + ": fresh_ms must be positive";
            return nullptr;
        }
        bool ok = true;
        health.forEach("Encoder", [&](const ConfigNode& e) {
            if (!ok) {
                return;
            }
            const SensorId id{e.attr("sensor_id")};
            if (context.sensors->payloadOf(id) == nullptr) {
                err = e.path() + ": references sensor " + id.value +
                      " which is not configured";
                ok = false;
                return;
            }
            publisher->encoder_health_.push_back(id);
        });
        if (!ok) {
            return nullptr;
        }
        const ConfigNode gyro = health.child("Gyro");
        if (gyro.valid()) {
            publisher->gyro_health_ = SensorId{gyro.attr("sensor_id")};
            if (context.sensors->payloadOf(publisher->gyro_health_) == nullptr) {
                err = gyro.path() + ": references sensor " +
                      publisher->gyro_health_.value + " which is not configured";
                return nullptr;
            }
        }
        const ConfigNode bias = health.child("BiasCal");
        if (bias.valid()) {
            publisher->bias_cal_function_ = bias.attr("function_id");
            if (publisher->bias_cal_function_.empty()) {
                err = bias.path() + ": BiasCal needs function_id";
                return nullptr;
            }
            if (!context.requireObservationFunction(publisher->bias_cal_function_,
                                                    bias.path(), err)) {
                return nullptr;
            }
        }
    }

    bool ok = true;
    node.forEach("FieldObject", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        WireObject wire;
        wire.object = FieldObjectId{w.attr("object_id")};
        long id     = -1;
        if (wire.object.empty() || !w.getInt("wire_id", -1, id, err)) {
            if (err.empty()) {
                err = w.path() + ": FieldObject needs object_id and wire_id";
            }
            ok = false;
            return;
        }
        if (id < 1 || id > 255) {
            err = w.path() + ": wire_id must be 1..255";
            ok  = false;
            return;
        }
        wire.wire_id = static_cast<uint8_t>(id);
        for (const WireObject& seen : publisher->wire_objects_) {
            if (seen.wire_id == wire.wire_id || seen.object == wire.object) {
                err = w.path() + ": duplicate FieldObject mapping";
                ok  = false;
                return;
            }
        }
        publisher->wire_objects_.push_back(wire);
    });
    if (!ok) {
        return nullptr;
    }
    return publisher;
}

void BrainLinkPublisher::reset() { bias_cal_seen_ = false; }

bool BrainLinkPublisher::sensorFresh(const SensorMap& results, const SensorId& id,
                                     MonotonicTime now) const {
    const auto it = results.find(id);
    if (it == results.end() || it->second.state != SourceState::kValid ||
        !it->second.latest.has_value()) {
        return false;
    }
    return (now - it->second.latest->receivedAt) <= fresh_ms_;
}

const BrainLinkPublisher::WireObject* BrainLinkPublisher::findWire(uint8_t wire_id) const {
    for (const WireObject& wire : wire_objects_) {
        if (wire.wire_id == wire_id) {
            return &wire;
        }
    }
    return nullptr;
}

uint8_t BrainLinkPublisher::selectResult(const BrainReplyContext& ctx) const {
    if ((ctx.select_flags & gatr2::kSelectFlagSelected) == 0) {
        return gatr2::kResultOk;   // a release always succeeds
    }
    if (world_noop_) {
        return gatr2::kResultLandmarkUnsupported;
    }
    if (findWire(ctx.landmark_id) == nullptr) {
        return gatr2::kResultUnknownLandmark;
    }
    return gatr2::kResultOk;
}

gatr2::BrainState BrainLinkPublisher::state(const PublishingInput& in) const {
    const RobotState& robot = in.robot;
    gatr2::BrainState s;

    if (robot.valid) {
        s.robot_flags |= gatr2::kRobotPoseValid;
    }
    if (robot.initialized) {
        s.robot_flags |= gatr2::kRobotLocalized;
    }
    if (hostSet(robot.measuredAtHost) && hostSet(in.now)) {
        s.robot_flags |= gatr2::kRobotAgeKnown;
        s.robot_age_ms = clampAgeMs(in.now - robot.measuredAtHost);
    }
    if (robot.placement_origin == "command") {
        s.robot_flags |= gatr2::kRobotAnchorCommand;
    } else if (robot.placement_origin == "configuration") {
        s.robot_flags |= gatr2::kRobotAnchorConfigured;
    }
    const Pose2D field_pose = robot.fieldPose();
    s.x_mm                  = toWireMm(field_pose.x_m);
    s.y_mm                  = toWireMm(field_pose.y_m);
    s.heading_cdeg          = radToCdeg(field_pose.heading_rad);
    s.odometry_epoch        = static_cast<uint32_t>(robot.odometry_epoch);
    s.anchor_revision       = static_cast<uint32_t>(robot.anchor_revision);

    if (!encoder_health_.empty()) {
        bool all_fresh = true;
        for (const SensorId& id : encoder_health_) {
            all_fresh = all_fresh && sensorFresh(in.sensors, id, in.now);
        }
        if (all_fresh) {
            s.health |= gatr2::kHealthEncodersFresh;
        }
    }
    if (!gyro_health_.empty() && sensorFresh(in.sensors, gyro_health_, in.now)) {
        s.health |= gatr2::kHealthGyroFresh;
    }
    if (!in.observations.empty()) {
        s.health |= gatr2::kHealthVisionAlive;
    }
    if (bias_cal_seen_) {
        s.health |= gatr2::kHealthBiasCalibrated;
    }

    s.landmark_id = in.command.object_requested ? in.command.object_wire_id : 0;
    if (s.landmark_id == 0 || world_noop_) {
        return s;
    }
    const WireObject* wire = findWire(s.landmark_id);
    if (wire == nullptr) {
        return s;
    }
    const auto it = in.field.objects.find(wire->object);
    if (it == in.field.objects.end() || !it->second.valid) {
        return s;
    }
    const FieldObjectState& o = it->second;
    Pose2D                  landmark;
    if (o.source == EstimateSource::kObserved) {
        if (o.odometry_epoch != robot.odometry_epoch || !hostSet(o.lastObservedAt)) {
            return s;   // measured in another odometry frame: no usable estimate
        }
        landmark          = compose(robot.field_from_odom, o.T_odom_object);
        s.landmark_source = gatr2::kLandmarkSourceObserved;
        s.landmark_age_ms = clampAgeMs(in.now - o.lastObservedAt);
    } else if (o.source == EstimateSource::kFieldMap) {
        landmark          = o.pose.pose;
        s.landmark_source = gatr2::kLandmarkSourceNominal;
    } else {
        return s;
    }
    s.lm_x_mm         = toWireMm(landmark.x_m);
    s.lm_y_mm         = toWireMm(landmark.y_m);
    s.lm_heading_cdeg = radToCdeg(landmark.heading_rad);
    return s;
}

PublishingOutput BrainLinkPublisher::run(const PublishingInput& in) {
    PublishingOutput out;
    if (!bias_cal_function_.empty()) {
        const ObservationFunctionStatus* f = in.localization.find(bias_cal_function_);
        if (f != nullptr && f->ready) {
            bias_cal_seen_ = true;
        }
    }

    const BrainReplyContext& ctx = in.command.reply;
    if (!ctx.pending) {
        return out;   // the brain initiates; nothing is unsolicited
    }

    gatr2::BrainReply reply;
    reply.op          = ctx.op;
    reply.session     = ctx.session;
    reply.request_id  = ctx.request_id;
    reply.result      = ctx.result;
    reply.pi_instance = ctx.pi_instance;
    reply.nonce       = ctx.nonce;
    if (ctx.op == gatr2::kOpSetPose && ctx.result == gatr2::kResultPending) {
        // Ok only once localization applied exactly this placement
        const bool applied = in.robot.placement_origin == "command" &&
                             in.robot.placement_session == ctx.session &&
                             in.robot.placement_sequence == ctx.placement_sequence;
        reply.result          = applied ? gatr2::kResultOk : gatr2::kResultPending;
        reply.odometry_epoch  = static_cast<uint32_t>(in.robot.odometry_epoch);
        reply.anchor_revision = static_cast<uint32_t>(in.robot.anchor_revision);
    } else if (ctx.op == gatr2::kOpSelectLandmark && ctx.result == gatr2::kResultOk) {
        reply.result       = selectResult(ctx);
        reply.landmark_id  = ctx.landmark_id;
        reply.select_flags = ctx.select_flags;
    } else if ((ctx.op == gatr2::kOpGetState || ctx.op == gatr2::kOpGetStateWithImu) &&
               ctx.result == gatr2::kResultOk) {
        reply.state = state(in);
    }

    LinkStats* stats =
        in.diagnostics != nullptr ? &in.diagnostics->links[diagnostics_id_] : nullptr;
    uint8_t        buf[gatr2::kMaxFrameLen];
    const uint16_t len = gatr2::encodeBrainReply(reply, buf, sizeof(buf));
    if (len == 0) {
        out.status = FunctionStatus::kFault;
        return out;
    }
    const SerialWriteResult written = link_->write(ByteSpan{buf, len}, ctx.window);
    if (stats != nullptr) {
        if (written.ok) {
            ++stats->replies;
        } else if (written.expired) {
            ++stats->expired;
        } else if (written.input_pending) {
            ++stats->input_pending;
        } else {
            ++stats->tx_errors;
        }
        if (written.late_release) {
            ++stats->late_release;
        }
    }
    if (!written.ok && !written.expired && !written.input_pending) {
        out.status = FunctionStatus::kFault;
    }
    return out;
}

} // namespace navigatr
