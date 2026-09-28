// brain_imu_wheel_bench.cpp

#include "impl/localization/brain_imu_wheel_bench.h"

#include <cmath>
#include <vector>

#include "math/angles.h"
#include "math/quaternion.h"
#include "payloads/robot_observations.h"
#include "resources/resource_store.h"
#include "resources/wheel_geometry.h"

namespace navigatr
{

std::unique_ptr<RobotObservationFunction>
BrainImuWheelBench::createParallel(const ConfigNode& node,
                                   RobotObservationInitializationContext& context,
                                   std::string& err) {
    return create(node, context, err, false);
}

std::unique_ptr<RobotObservationFunction>
BrainImuWheelBench::createPlanar(const ConfigNode& node,
                                 RobotObservationInitializationContext& context,
                                 std::string& err) {
    return create(node, context, err, true);
}

std::unique_ptr<RobotObservationFunction>
BrainImuWheelBench::create(const ConfigNode& node, RobotObservationInitializationContext& context,
                           std::string& err, bool planar) {
    auto model     = std::make_unique<BrainImuWheelBench>();
    model->planar_ = planar;
    model->type_   = planar ? "brain_imu_planar_bench" : "brain_imu_parallel_bench";
    model->id_     = ObservationFunctionId{node.attr("id")};
    model->output_ = ObservationId{node.child("Output").attr("observation_id")};
    if (!context.resources || !context.sensors || model->output_.empty()) {
        err = node.path() + ": needs resources, sensors and Output observation_id";
        return nullptr;
    }
    model->imu_ = context.resources->require<BrainImuBench>(
        ResourceId{node.child("Imu").attr("resource_id")}, err);
    if (!model->imu_) {
        return nullptr;
    }
    const ConfigNode attitude = node.child("Attitude");
    if (attitude.valid()) {
        model->attitude_output_ = ObservationId{attitude.attr("observation_id")};
        if (model->attitude_output_.empty() || model->attitude_output_ == model->output_) {
            err = attitude.path() + ": Attitude needs its own observation_id";
            return nullptr;
        }
    }

    // Inline TrackingWheel elements or a Wheels reference to wheel_geometry.
    std::vector<WheelDecl> declared;
    bool                   ok = true;
    node.forEach("TrackingWheel", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        WheelDecl   decl;
        std::string sensor, direction;
        ok = w.requireAttr("sensor_id", sensor, err) &&
             w.requireDouble("radius_m", decl.radius_m, err) &&
             w.requireDouble("position_x_m", decl.position_x_m, err) &&
             w.requireDouble("position_y_m", decl.position_y_m, err) &&
             w.requireDouble("measurement_angle_deg", decl.measurement_angle_deg, err) &&
             w.requireAttr("direction", direction, err) &&
             w.getDouble("travel_scale", 1.0, decl.travel_scale, err);
        if (ok && direction != "positive" && direction != "negative") {
            err = w.path() + ": direction must be positive or negative";
            ok  = false;
        }
        decl.sensor             = SensorId{sensor};
        decl.label              = w.attr("label");
        decl.direction_positive = direction == "positive";
        declared.push_back(decl);
    });
    if (!ok) {
        return nullptr;
    }
    const ConfigNode wheels_ref = node.child("Wheels");
    if (wheels_ref.valid()) {
        if (!declared.empty()) {
            err = wheels_ref.path() + ": use either inline TrackingWheel elements or a Wheels "
                                      "reference, not both";
            return nullptr;
        }
        const auto geometry = context.resources->require<const WheelGeometryMap>(
            ResourceId{wheels_ref.attr("resource_id")}, err);
        if (!geometry) {
            return nullptr;
        }
        wheels_ref.forEach("Use", [&](const ConfigNode& use) {
            if (!ok) {
                return;
            }
            const auto* w = geometry->find(use.attr("wheel_id"));
            if (!w) {
                err = node.path() + ": bench model needs exactly two declared wheels";
                ok  = false;
                return;
            }
            declared.push_back(*w);
        });
        if (!ok) {
            return nullptr;
        }
    }
    if (declared.size() > 2) {
        err = node.path() + ": bench model needs exactly two declared wheels";
        return nullptr;
    }
    std::size_t count = 0;
    for (const WheelDecl& w : declared) {
        const double angle = degToRad(w.measurement_angle_deg);
        // forward wheels measure along +x or -x; the travel projects onto x
        if (!planar && std::fabs(std::sin(angle)) > 1e-9) {
            err = node.path() + ": bench model needs exactly two forward (0 or 180 degree) wheels";
            return nullptr;
        }
        if (w.radius_m <= 0.0 || w.travel_scale <= 0.0) {
            err = node.path() + ": radius_m and travel_scale must be positive";
            return nullptr;
        }
        auto& dest = model->wheels_[count++];
        if (!context.sensors->bind<EncoderSample>(w.sensor, node.path(), dest.binding, err)) {
            return nullptr;
        }
        dest.radius = w.radius_m;
        dest.scale  = w.travel_scale;
        dest.ux     = std::cos(angle);
        dest.uy     = std::sin(angle);
        dest.k      = w.position_x_m * dest.uy - w.position_y_m * dest.ux;
        dest.sign   = w.direction_positive ? 1.0 : -1.0;
    }
    if (count != 2 || model->wheels_[0].binding.id == model->wheels_[1].binding.id) {
        err = node.path() + ": bench model needs two distinct encoder sensors";
        return nullptr;
    }
    const auto& a       = model->wheels_[0];
    const auto& b       = model->wheels_[1];
    model->determinant_ = a.ux * b.uy - a.uy * b.ux;
    if (planar && std::fabs(model->determinant_) < 1e-3) {
        err = node.path() + ": planar bench needs two nonparallel wheel directions; "
                            "use brain_imu_parallel_bench only when sideways travel is assumed "
                            "zero";
        return nullptr;
    }

    const ConfigNode still_node = node.child("Stillness");
    StillnessConfig  still;
    long             samples = still.min_samples;
    if (!still_node.getInt("samples", samples, samples, err) ||
        !still_node.getDouble("still_travel_m", still.still_travel_m, still.still_travel_m,
                              err) ||
        !readStillness(still_node, still, err)) {
        return nullptr;
    }
    if (samples < 1 || !(still.still_travel_m > 0.0)) {
        err = still_node.path() + ": samples and still_travel_m must be positive";
        return nullptr;
    }
    still.min_samples = samples;
    model->window_.configure(still);

    const ConfigNode freshness = node.child("Freshness");
    if (!freshness.getInt("max_age_ms", 200, model->max_age_ms_, err)) {
        return nullptr;
    }
    if (model->max_age_ms_ <= 0) {
        err = freshness.path() + ": max_age_ms must be positive";
        return nullptr;
    }
    for (Wheel& w : model->wheels_) {
        w.source = model->window_.addSource(StationaryWindow::Kind::kWheel,
                                            "wheel " + w.binding.id.value);
    }
    model->rotation_source_ =
        model->window_.addSource(StationaryWindow::Kind::kRotation, "VEX IMU rotation");

    if (context.warnings) {
        context.warnings->push_back(
            node.path() + ": BENCH ONLY: pairs latest wheels and Brain IMU by Pi arrival time; " +
            (planar ? "forward and sideways motion from configured wheel directions"
                    : "sideways motion assumed zero"));
    }
    return model;
}

std::vector<RobotObservationOutputDecl> BrainImuWheelBench::outputs() const {
    std::vector<RobotObservationOutputDecl> out = {
        {output_, PayloadDescriptor::of<BodyMotionIncrement>(payload_names::kBodyMotionIncrement)}};
    if (!attitude_output_.empty()) {
        out.push_back({attitude_output_, PayloadDescriptor::of<AttitudeObservation>(
                                             payload_names::kAttitudeObservation)});
    }
    return out;
}

void BrainImuWheelBench::publishAttitude(RobotObservationMap& out) {
    if (attitude_output_.empty() || !imu_->attitude_valid ||
        imu_->attitude_sequence == attitude_sequence_ ||
        imu_->attitude_received.domain != ClockDomain::kHost) {
        return;
    }
    attitude_sequence_ = imu_->attitude_sequence;
    AttitudeObservation a;
    a.q_reference_body = quaternionFromEuler(cdegToRad(imu_->roll_cdeg),
                                             cdegToRad(imu_->pitch_cdeg), 0.0);
    a.reference  = "gravity";
    a.has_yaw    = false;
    a.measuredAt = imu_->attitude_received;   // Pi arrival: the bench time base
    a.source     = Provenance{"brain_vex_imu", "brain.vex_imu.attitude", "host",
                          imu_->attitude_sequence, imu_->epoch};
    a.quality    = 1.0;   // measured by the VEX IMU; no quality estimate exists
    RobotObservationRecord record;
    record.measuredAt = a.measuredAt;
    record.receivedAt = a.measuredAt;
    record.payload    = TypedPayload::store(std::move(a), payload_names::kAttitudeObservation);
    out[attitude_output_] = std::move(record);
}

ObservationReadiness BrainImuWheelBench::readiness() const {
    ObservationReadiness r;
    r.ready             = ready_;
    r.note              = note_;
    r.stillness         = stillnessOf(window_, nullptr);
    r.dropped_intervals = dropped_;
    r.dropped_why       = dropped_why_;
    return r;
}

void BrainImuWheelBench::lostMotion(const std::string& why) {
    ++dropped_;
    dropped_why_ = why;
}

void BrainImuWheelBench::reset() {
    baseline_ = false;
    offered_  = false;
    ready_    = false;
    window_.restart(StationaryWindow::Phase::kWaitingData, "reset");
}

void BrainImuWheelBench::observeStillness(const std::array<const StoredSample*, 2>&  stored,
                                          const std::array<const EncoderSample*, 2>& samples,
                                          MonotonicTime                              now) {
    for (std::size_t i = 0; i < 2; ++i) {
        if (stored[i] == nullptr || samples[i] == nullptr) {
            continue;
        }
        const Wheel& w = wheels_[i];
        StillSample  evidence;
        evidence.sequence      = stored[i]->sequence;
        evidence.epoch         = stored[i]->epoch;
        evidence.discontinuity = samples[i]->discontinuity_epoch;
        evidence.at            = stored[i]->receivedAt;   // arrival time, like the IMU
        evidence.received      = stored[i]->receivedAt;
        evidence.value         = samples[i]->angle_rad * w.radius * w.scale;
        window_.add(w.source, evidence);
    }
    if (imu_->valid) {
        StillSample evidence;
        evidence.sequence = imu_->sequence;
        evidence.epoch    = imu_->epoch;
        evidence.at       = imu_->received;
        evidence.received = imu_->received;
        evidence.value    = degToRad(imu_->rotation_mdeg / 1000.0);
        window_.add(rotation_source_, evidence);
    }
    window_.poll(now);
    StationaryWindow::Qualified ignored;
    window_.takeQualified(ignored);   // status only; the VEX IMU has no Pi bias
}

FunctionStatus BrainImuWheelBench::run(const RobotObservationInput& in, RobotObservationMap& out) {
    publishAttitude(out);   // independent of the motion step below
    const auto fresh = [&](MonotonicTime t) {
        return t.domain == ClockDomain::kHost && in.context.now.domain == ClockDomain::kHost &&
               in.context.now.ms >= t.ms && in.context.now.ms - t.ms <= max_age_ms_;
    };
    std::array<const StoredSample*, 2>  stored{};
    std::array<const EncoderSample*, 2> samples{};
    std::string                         missing;   // why no step can be measured now
    if (!imu_->valid) {
        missing = "VEX IMU sample invalid";
    } else if (!fresh(imu_->received)) {
        missing = "VEX IMU stale";
    }
    for (std::size_t i = 0; i < 2; ++i) {
        stored[i]  = wheels_[i].binding.freshStored(in.sensors);
        samples[i] = stored[i] ? stored[i]->payload.get<EncoderSample>() : nullptr;
        if (missing.empty() && !(stored[i] && samples[i] && fresh(stored[i]->receivedAt) &&
                                 std::isfinite(samples[i]->angle_rad))) {
            missing = "encoder " + wheels_[i].binding.id.value + " missing or stale";
        }
    }
    observeStillness(stored, samples, in.context.now);
    if (!missing.empty()) {
        if (baseline_) {
            lostMotion(missing);   // the travel from here to the next baseline is lost
        }
        baseline_ = false;
        ready_    = false;
        note_     = "bench: " + missing + "; hold pose and rebaseline";
        return FunctionStatus::kNoData;
    }
    if (offered_) {
        return FunctionStatus::kNoData;
    }
    if (baseline_ && imu_->sequence == imu_sequence_ && imu_->epoch == imu_epoch_) {
        return FunctionStatus::kNoData;
    }
    // a rebaseline after a baseline existed discards the interval since it
    std::string dropped;
    if (baseline_ && imu_->epoch != imu_epoch_) {
        dropped = "VEX IMU restarted";
    } else if (baseline_ && imu_->received.ms <= previous_.ms) {
        dropped = "VEX IMU receipt did not advance";
    } else if (baseline_ && imu_->received.ms - previous_.ms > max_age_ms_) {
        dropped = "no VEX IMU and wheel pair for " +
                  std::to_string(imu_->received.ms - previous_.ms) + " ms";
    }
    for (std::size_t i = 0; i < 2; ++i) {
        const Wheel& w = wheels_[i];
        if (baseline_ && dropped.empty() &&
            (stored[i]->epoch != w.epoch || samples[i]->discontinuity_epoch != w.discontinuity)) {
            dropped = "encoder " + w.binding.id.value + " restarted";
        }
    }
    bool rebase = !baseline_ || !dropped.empty();
    // Do not emit heading-only updates while wheel records are retained. A
    // later new wheel sample captures the whole cumulative travel instead.
    if (!rebase && (stored[0]->sequence == wheels_[0].sequence ||
                    stored[1]->sequence == wheels_[1].sequence)) {
        return FunctionStatus::kNoData;
    }

    BodyMotionIncrement motion;
    motion.startAt    = previous_;
    motion.endAt      = imu_->received;
    motion.dt_s       = (motion.endAt.ms - motion.startAt.ms) / 1000.0;
    motion.dtheta_rad = degToRad((static_cast<double>(imu_->rotation_mdeg) - rotation_) / 1000.0);
    // Catch unannounced zeroing or discontinuous angles rather than teleporting.
    if (!rebase && std::fabs(motion.dtheta_rad) > 12.0 * motion.dt_s + 0.1) {
        rebase  = true;
        dropped = "VEX IMU rotation jumped";
    }
    std::array<double, 2> travel{};
    for (std::size_t i = 0; i < 2; ++i) {
        auto& w = wheels_[i];
        // Wheel travel = ux*dx + uy*dy + (x*uy - y*ux)*dtheta.
        // Remove travel caused by the wheel's offset before solving translation.
        travel[i] = (samples[i]->angle_rad - w.angle) * w.radius * w.scale * w.sign -
                    w.k * motion.dtheta_rad;
        Provenance source = stored[i]->upstream;
        source.source     = w.binding.id.value;
        source.sequence   = stored[i]->sequence;
        source.epoch      = stored[i]->epoch;
        motion.sources.push_back(std::move(source));   // preserve actual Pico lineage/clock
        w.angle         = samples[i]->angle_rad;
        w.sequence      = stored[i]->sequence;
        w.epoch         = stored[i]->epoch;
        w.discontinuity = samples[i]->discontinuity_epoch;
    }
    if (planar_) {
        const auto& a = wheels_[0];
        const auto& b = wheels_[1];
        motion.dx_m   = (travel[0] * b.uy - a.uy * travel[1]) / determinant_;
        motion.dy_m   = (a.ux * travel[1] - travel[0] * b.ux) / determinant_;
    } else {
        // both wheels along +-x: the mean forward travel
        motion.dx_m = 0.5 * (wheels_[0].ux * travel[0] + wheels_[1].ux * travel[1]);
    }
    motion.sources.push_back(Provenance{"brain_imu_bench", "brain.vex_imu", "brain",
                                        imu_->sequence, imu_->epoch});
    rotation_     = imu_->rotation_mdeg;
    imu_sequence_ = imu_->sequence;
    imu_epoch_    = imu_->epoch;
    previous_     = imu_->received;
    baseline_     = true;
    ready_        = true;
    note_         = planar_ ? "BENCH: arrival-time pairing; forward and sideways wheel solve"
                            : "BENCH: arrival-time pairing; sideways motion unmeasured";
    if (rebase) {
        if (!dropped.empty()) {
            lostMotion(dropped);
        }
        return FunctionStatus::kNoData;
    }
    RobotObservationRecord record;
    record.measuredAt = motion.endAt;
    record.receivedAt = motion.endAt;
    record.payload = TypedPayload::store(std::move(motion), payload_names::kBodyMotionIncrement);
    out[output_]   = std::move(record);
    offered_       = true;
    return FunctionStatus::kOk;
}

} // namespace navigatr
