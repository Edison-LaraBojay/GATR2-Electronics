// brain_link_publisher.cpp

#include "impl/publishing/brain_link_publisher.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <cstdio>

#include "diagnostics/hub.h"
#include "impl/commands/brain_link_commands.h"
#include "math/angles.h"
#include "resources/resource_store.h"
#include "runtime/pico_control_ref.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

namespace
{

static_assert(static_cast<uint8_t>(BiasCalibration::kNone) == translagatr::kCalibrationNone &&
                  static_cast<uint8_t>(BiasCalibration::kRunning) == translagatr::kCalibrationRunning &&
                  static_cast<uint8_t>(BiasCalibration::kDone) == translagatr::kCalibrationDone &&
                  static_cast<uint8_t>(BiasCalibration::kWaitingStill) ==
                      translagatr::kCalibrationWaitingStill &&
                  static_cast<uint8_t>(BiasCalibration::kWaitingData) ==
                      translagatr::kCalibrationWaitingData &&
                  static_cast<uint8_t>(BiasCalibration::kFailed) == translagatr::kCalibrationFailed,
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
    if (context.diagnostics != nullptr) {
        publisher->hub_       = context.diagnostics;
        publisher->source_id_ = context.diagnostics->sourceId(link_id.value);
        // the commands slot, built first, named the monitor and its kind
        publisher->monitor_ = context.diagnostics->links().monitor(link_id.value, "brain_serial");
    }

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

// The reply as the link took it: attempted versus accepted bytes, the
// decoded summary, and the processed request's record completed with the
// result and length actually sent (0 when nothing went out).
void BrainLinkPublisher::noteReply(const translagatr::BrainReply& reply, const uint8_t* frame,
                                   uint16_t len, const SerialWriteResult& written) {
    if (monitor_ == nullptr) {
        return;
    }
    // a SerialLink write is whole or nothing: a refused one (expired window,
    // input pending, error) was attempted and accepted nothing
    monitor_->tx(frame, len, written.ok ? len : 0);
    std::string fields;
    if (monitor_->decodedOn()) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "rid=%u result=%s%s", static_cast<unsigned>(reply.request_id),
                      brainResultName(reply.result),
                      written.ok              ? ""
                      : written.expired       ? " not sent: window expired"
                      : written.input_pending ? " not sent: input pending"
                                              : " write failed");
        fields = buf;
    }
    monitor_->frame(false, brainOpName(reply.op), fields);

    DiagBrainRequest r;
    if (hub_ == nullptr || !monitor_->takeStagedRequest(r)) {
        return;
    }
    if (r.request_id == reply.request_id && r.op == reply.op) {
        r.result    = reply.result;
        r.reply_len = written.ok ? static_cast<uint8_t>(len) : 0;
    }
    DiagRecord record;
    record.kind    = DiagKind::kBrainRequest;
    record.source  = source_id_;
    record.payload = r;
    hub_->post(std::move(record));
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
        return translagatr::kCalibrationNone;
    }
    const ObservationFunctionStatus* f = in.localization.find(function);
    if (f == nullptr) {
        return translagatr::kCalibrationNone;
    }
    if (f->stillness.monitored) {
        // BiasCalibration values are the wire CalibrationState values
        return static_cast<uint8_t>(f->stillness.calibration);
    }
    return f->ready ? translagatr::kCalibrationDone : translagatr::kCalibrationRunning;
}

translagatr::BrainState BrainLinkPublisher::state(const PublishingInput& in) const {
    const RobotState& robot = in.robot;
    translagatr::BrainState s;

    if (robot.valid) {
        s.robot_flags |= translagatr::kRobotPoseValid;
    }
    if (robot.initialized) {
        s.robot_flags |= translagatr::kRobotLocalized;
    }
    if (hostSet(robot.measuredAtHost) && hostSet(in.now)) {
        s.robot_flags |= translagatr::kRobotAgeKnown;
        s.robot_age_ms = clampAgeMs(in.now - robot.measuredAtHost);
    }
    if (robot.placement_origin == "command") {
        s.robot_flags |= translagatr::kRobotAnchorCommand;
    } else if (robot.placement_origin == "configuration") {
        s.robot_flags |= translagatr::kRobotAnchorConfigured;
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
            s.health |= translagatr::kHealthEncodersFresh;
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
        s.health |= translagatr::kHealthGyroFresh;
    }
    if (!in.observations.empty()) {
        s.health |= translagatr::kHealthVisionAlive;
    }
    if (pico_ != nullptr) {
        const PicoLinkState link = pico_->link();
        if (link.frames_fresh) {
            s.health |= translagatr::kHealthPicoLink;
        }
        if (link.status_known) {
            switch (link.status.imu_state) {
            case translagatr::kPicoImuInitializing:
            case translagatr::kPicoImuAligning:
            case translagatr::kPicoImuRetrying: s.health |= translagatr::kHealthImuInitializing; break;
            case translagatr::kPicoImuFailed: s.health |= translagatr::kHealthImuFailed; break;
            default: break;
            }
        }
    }
    if (in.localization.stationary()) {
        s.health |= translagatr::kHealthStationary;
    }
    s.calibration = calibration(in, profile.get());
    const bool bias_calibrated = profile_host_ != nullptr
                                     ? s.calibration == translagatr::kCalibrationDone
                                     : bias_cal_seen_;
    if (bias_calibrated) {
        s.health |= translagatr::kHealthBiasCalibrated;
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

    translagatr::BrainReply reply;
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
    case translagatr::kOpSetPose:
        if (ctx.result == translagatr::kResultPending) {
            // Ok only once localization applied exactly this placement
            const bool applied = in.robot.placement_origin == "command" &&
                                 in.robot.placement_session == ctx.session &&
                                 in.robot.placement_sequence == ctx.placement_sequence;
            reply.result          = applied ? translagatr::kResultOk : translagatr::kResultPending;
            reply.odometry_epoch  = static_cast<uint32_t>(in.robot.odometry_epoch);
            reply.anchor_revision = static_cast<uint32_t>(in.robot.anchor_revision);
        }
        break;
    case translagatr::kOpGetState:
        if (ctx.result == translagatr::kResultOk) {
            reply.state = *out.brain_state;
        }
        break;
    case translagatr::kOpReadDoc:
        if (ctx.result == translagatr::kResultOk) {
            reply.result = documents_ == nullptr
                               ? static_cast<uint8_t>(translagatr::kResultUnavailable)
                               : documents_->read(ctx.doc_kind, ctx.doc_id, ctx.doc_offset,
                                                  ctx.doc_max_len, reply);
        }
        break;
    case translagatr::kOpControl: {
        const std::shared_ptr<const ProfileBinding> profile =
            profile_host_ != nullptr ? profile_host_->applied() : nullptr;
        reply.calibration    = calibration(in, profile.get());
        reply.control_detail = ctx.control_detail;
        break;
    }
    case translagatr::kOpReadWheels:
        if (ctx.result == translagatr::kResultOk) {
            reply.wheel_count = ctx.wheel_count;
            for (uint8_t i = 0; i < ctx.wheel_count && i < translagatr::kWheelReadingsMax; ++i) {
                reply.wheels[i] = ctx.wheels[i];
            }
        }
        break;
    default: break;
    }

    LinkStats* stats =
        in.diagnostics != nullptr ? &in.diagnostics->links[diagnostics_id_] : nullptr;
    uint8_t        buf[translagatr::kMaxFrameLen];
    const uint16_t len = translagatr::encodeBrainReply(reply, buf, sizeof(buf));
    if (len == 0) {
        out.status = FunctionStatus::kFault;
        return out;
    }
    const SerialWriteResult written = link_->write(ByteSpan{buf, len}, ctx.window);
    noteReply(reply, buf, len, written);
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
