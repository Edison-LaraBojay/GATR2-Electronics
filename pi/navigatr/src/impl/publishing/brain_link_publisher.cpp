// brain_link_publisher.cpp

#include "impl/publishing/brain_link_publisher.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "math/angles.h"
#include "resources/resource_store.h"
#include "runtime/pico_control_ref.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

namespace
{

static_assert(static_cast<uint8_t>(BiasCalibration::kNone) == gatr2::kCalibrationNone &&
                  static_cast<uint8_t>(BiasCalibration::kRunning) == gatr2::kCalibrationRunning &&
                  static_cast<uint8_t>(BiasCalibration::kDone) == gatr2::kCalibrationDone &&
                  static_cast<uint8_t>(BiasCalibration::kWaitingStill) ==
                      gatr2::kCalibrationWaitingStill &&
                  static_cast<uint8_t>(BiasCalibration::kWaitingData) ==
                      gatr2::kCalibrationWaitingData &&
                  static_cast<uint8_t>(BiasCalibration::kFailed) == gatr2::kCalibrationFailed,
              "BiasCalibration values are the wire CalibrationState values");

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

bool knownChildren(const ConfigNode& node, std::string& err) {
    for (ConfigNode c = node.child(); c.valid(); c = c.next()) {
        const std::string name = c.name();
        if (name == "Serial" || name == "Health" || name == "Field" || name == "Pico") {
            continue;
        }
        if (name == "FieldObject") {
            err = c.path() + ": FieldObject was removed with brain link v4; the Brain reads "
                             "every landmark from the field documents";
        } else {
            err = node.path() + " has unknown element " + name;
        }
        return false;
    }
    return true;
}

bool makeFieldDocuments(const ConfigNode& publishing, const ResourceStore& resources,
                        std::unique_ptr<FieldDocuments>& out, std::string& err) {
    if (!publishing.atMostOne("Field", err)) {
        return false;
    }
    const ConfigNode node = publishing.child("Field");
    if (!node.valid()) {
        return true;
    }
    std::string id;
    long        period_ms = 200;
    if (!node.onlyAttributes({"resource_id", "estimate_period_ms"}, err) ||
        !node.onlyChildren({}, err) || !node.requireAttr("resource_id", id, err) ||
        !node.getInt("estimate_period_ms", 200, period_ms, err)) {
        return false;
    }
    if (period_ms <= 0) {
        err = node.path() + ": estimate_period_ms must be positive";
        return false;
    }
    std::string inner;
    const auto  map = resources.require<const FieldMap>(ResourceId{id}, inner);
    if (map == nullptr) {
        err = node.path() + ": " + inner;
        return false;
    }
    FieldMapDocument doc;
    if (!buildFieldMapDocument(*map, doc, inner)) {
        err = node.path() + ": field " + id + ": " + inner;
        return false;
    }
    out = std::make_unique<FieldDocuments>(std::move(doc), period_ms);
    return true;
}

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
    if (!knownChildren(node, err)) {
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
    publisher->profile_host_   = context.brain_profile;

    const ConfigNode health = node.child("Health");
    if (health.valid() && publisher->profile_host_ != nullptr &&
        (!health.onlyChildren({}, err) || !health.onlyAttributes({"fresh_ms"}, err))) {
        err += "; with a Brain profile the health references follow the profile";
        return nullptr;
    }
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
    if (!makeFieldDocuments(node, *context.resources, publisher->documents_, err)) {
        return nullptr;
    }
    if (!parsePicoReference(node, *context.resources, publisher->pico_, err)) {
        return nullptr;
    }
    return publisher;
}

void BrainLinkPublisher::reset() {
    bias_cal_seen_ = false;
    if (documents_ != nullptr) {
        documents_->reset();
    }
}

bool BrainLinkPublisher::sensorFresh(const SensorMap& results, const SensorId& id,
                                     MonotonicTime now) const {
    const auto it = results.find(id);
    if (it == results.end() || it->second.state != SourceState::kValid ||
        !it->second.latest.has_value()) {
        return false;
    }
    return (now - it->second.latest->receivedAt) <= fresh_ms_;
}

uint8_t BrainLinkPublisher::calibration(const PublishingInput& in,
                                        const ProfileBinding* profile) const {
    const std::string& function =
        profile_host_ != nullptr
            ? (profile != nullptr ? profile->bias_function : std::string())
            : bias_cal_function_;
    if (function.empty()) {
        return gatr2::kCalibrationNone;
    }
    const ObservationFunctionStatus* f = in.localization.find(function);
    if (f == nullptr) {
        return gatr2::kCalibrationNone;
    }
    if (f->stillness.monitored) {
        // BiasCalibration values are the wire CalibrationState values
        return static_cast<uint8_t>(f->stillness.calibration);
    }
    return f->ready ? gatr2::kCalibrationDone : gatr2::kCalibrationRunning;
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

    const std::shared_ptr<const ProfileBinding> profile =
        profile_host_ != nullptr ? profile_host_->applied() : nullptr;
    const std::vector<SensorId>& encoders =
        profile_host_ != nullptr ? (profile != nullptr ? profile->encoders : encoder_health_)
                                 : encoder_health_;
    if (!encoders.empty()) {
        bool all_fresh = true;
        for (const SensorId& id : encoders) {
            all_fresh = all_fresh && sensorFresh(in.sensors, id, in.now);
        }
        if (all_fresh) {
            s.health |= gatr2::kHealthEncodersFresh;
        }
    }
    bool gyro_fresh = false;
    if (profile_host_ == nullptr) {
        gyro_fresh = !gyro_health_.empty() && sensorFresh(in.sensors, gyro_health_, in.now);
    } else if (profile != nullptr && !profile->imu.empty()) {
        gyro_fresh = sensorFresh(in.sensors, profile->imu, in.now);
    } else if (profile != nullptr && profile->bench_imu != nullptr) {
        const BrainImuBench& bench = *profile->bench_imu;
        gyro_fresh = bench.valid && hostSet(bench.received) && hostSet(in.now) &&
                     (in.now - bench.received) <= fresh_ms_;
    }
    if (gyro_fresh) {
        s.health |= gatr2::kHealthGyroFresh;
    }
    if (!in.observations.empty()) {
        s.health |= gatr2::kHealthVisionAlive;
    }
    if (pico_ != nullptr) {
        const PicoLinkState link = pico_->link();
        if (link.frames_fresh) {
            s.health |= gatr2::kHealthPicoLink;
        }
        if (link.status_known) {
            switch (link.status.imu_state) {
            case gatr2::kPicoImuInitializing:
            case gatr2::kPicoImuAligning:
            case gatr2::kPicoImuRetrying: s.health |= gatr2::kHealthImuInitializing; break;
            case gatr2::kPicoImuFailed: s.health |= gatr2::kHealthImuFailed; break;
            default: break;
            }
        }
    }
    if (in.localization.stationary()) {
        s.health |= gatr2::kHealthStationary;
    }
    s.calibration = calibration(in, profile.get());
    const bool bias_calibrated = profile_host_ != nullptr
                                     ? s.calibration == gatr2::kCalibrationDone
                                     : bias_cal_seen_;
    if (bias_calibrated) {
        s.health |= gatr2::kHealthBiasCalibrated;
    }

    const ProfileStatus& status = in.command.profile;
    s.profile_state             = status.state;
    s.profile_reason            = status.reason;
    s.profile_detail            = status.detail;
    s.profile_id                = status.id;

    if (documents_ != nullptr) {
        s.map_id      = documents_->mapId();
        s.estimate_id = documents_->estimateId();
    }
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
    if (documents_ != nullptr) {
        documents_->update(in.field, in.robot, in.now);
    }
    out.brain_state = state(in);

    const BrainReplyContext& ctx = in.command.reply;
    if (!ctx.pending) {
        return out;   // the brain initiates; nothing is unsolicited
    }

    gatr2::BrainReply reply;
    reply.op             = ctx.op;
    reply.session        = ctx.session;
    reply.request_id     = ctx.request_id;
    reply.result         = ctx.result;
    reply.pi_instance    = ctx.pi_instance;
    reply.nonce          = ctx.nonce;
    reply.profile_id     = ctx.profile_id;
    reply.received       = ctx.received;
    reply.profile_state  = ctx.profile_state;
    reply.profile_reason = ctx.profile_reason;
    reply.profile_detail = ctx.profile_detail;
    reply.action         = ctx.action;
    switch (ctx.op) {
    case gatr2::kOpSetPose:
        if (ctx.result == gatr2::kResultPending) {
            // Ok only once localization applied exactly this placement
            const bool applied = in.robot.placement_origin == "command" &&
                                 in.robot.placement_session == ctx.session &&
                                 in.robot.placement_sequence == ctx.placement_sequence;
            reply.result          = applied ? gatr2::kResultOk : gatr2::kResultPending;
            reply.odometry_epoch  = static_cast<uint32_t>(in.robot.odometry_epoch);
            reply.anchor_revision = static_cast<uint32_t>(in.robot.anchor_revision);
        }
        break;
    case gatr2::kOpGetState:
        if (ctx.result == gatr2::kResultOk) {
            reply.state = *out.brain_state;
        }
        break;
    case gatr2::kOpReadDoc:
        if (ctx.result == gatr2::kResultOk) {
            reply.result = documents_ == nullptr
                               ? static_cast<uint8_t>(gatr2::kResultUnavailable)
                               : documents_->read(ctx.doc_kind, ctx.doc_id, ctx.doc_offset,
                                                  ctx.doc_max_len, reply);
        }
        break;
    case gatr2::kOpControl: {
        const std::shared_ptr<const ProfileBinding> profile =
            profile_host_ != nullptr ? profile_host_->applied() : nullptr;
        reply.calibration    = calibration(in, profile.get());
        reply.control_detail = ctx.control_detail;
        break;
    }
    case gatr2::kOpReadWheels:
        if (ctx.result == gatr2::kResultOk) {
            reply.wheel_count = ctx.wheel_count;
            for (uint8_t i = 0; i < ctx.wheel_count && i < gatr2::kWheelReadingsMax; ++i) {
                reply.wheels[i] = ctx.wheels[i];
            }
        }
        break;
    default: break;
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
