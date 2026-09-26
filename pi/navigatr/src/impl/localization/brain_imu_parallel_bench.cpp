#include "impl/localization/brain_imu_parallel_bench.h"
#include <cmath>
#include "math/angles.h"
#include "payloads/robot_observations.h"
#include "resources/resource_store.h"
#include "resources/wheel_geometry.h"

namespace navigatr {
std::unique_ptr<RobotObservationFunction> BrainImuParallelBench::create(
    const ConfigNode& node, RobotObservationInitializationContext& context, std::string& err) {
    auto model = std::make_unique<BrainImuParallelBench>();
    model->id_ = ObservationFunctionId{node.attr("id")};
    model->output_ = ObservationId{node.child("Output").attr("observation_id")};
    if (!context.resources || !context.sensors || model->output_.empty()) {
        err = node.path() + ": needs resources, sensors and Output observation_id";
        return nullptr;
    }
    model->imu_ = context.resources->require<BrainImuBench>(
        ResourceId{node.child("Imu").attr("resource_id")}, err);
    const auto geometry = context.resources->require<const WheelGeometryMap>(
        ResourceId{node.child("Wheels").attr("resource_id")}, err);
    if (!model->imu_ || !geometry) return nullptr;
    std::size_t count = 0;
    bool ok = true;
    node.child("Wheels").forEach("Use", [&](const ConfigNode& use) {
        if (!ok) return;
        const auto* w = geometry->find(use.attr("wheel_id"));
        if (!w || count == 2 || std::fabs(w->measurement_angle_deg) > 1e-6) {
            err = node.path() + ": bench model needs exactly two forward (0 degree) wheels";
            ok = false;
            return;
        }
        auto& dest = model->wheels_[count++];
        ok = context.sensors->bind<EncoderSample>(w->sensor, node.path(), dest.binding, err);
        dest.radius = w->radius_m;
        dest.y = w->position_y_m;
        dest.sign = w->direction_positive ? 1.0 : -1.0;
    });
    if (!ok) return nullptr;
    if (count != 2 || model->wheels_[0].binding.id == model->wheels_[1].binding.id) {
        err = node.path() + ": bench model needs two distinct forward wheels";
        return nullptr;
    }
    if (context.warnings) context.warnings->push_back(
        node.path() + ": BENCH ONLY: pairs latest wheels and Brain IMU by Pi arrival time; sideways motion assumed zero");
    return model;
}

std::vector<RobotObservationOutputDecl> BrainImuParallelBench::outputs() const {
    return {{output_, PayloadDescriptor::of<BodyMotionIncrement>(payload_names::kBodyMotionIncrement)}};
}

FunctionStatus BrainImuParallelBench::run(const RobotObservationInput& in, RobotObservationMap& out) {
    const auto fresh = [&](MonotonicTime t) {
        return t.domain == ClockDomain::kHost && in.context.now.domain == ClockDomain::kHost &&
               in.context.now.ms >= t.ms && in.context.now.ms - t.ms <= 200;
    };
    std::array<const StoredSample*, 2> stored{};
    std::array<const EncoderSample*, 2> samples{};
    bool valid = imu_->valid && fresh(imu_->received);
    for (std::size_t i = 0; i < 2; ++i) {
        stored[i] = wheels_[i].binding.freshStored(in.sensors);
        samples[i] = stored[i] ? stored[i]->payload.get<EncoderSample>() : nullptr;
        valid = valid && stored[i] && samples[i] && fresh(stored[i]->receivedAt) &&
                std::isfinite(samples[i]->angle_rad);
    }
    if (!valid) {
        baseline_ = ready_ = false;
        note_ = "bench: VEX IMU or encoder missing/stale; hold pose and rebaseline";
        return FunctionStatus::kNoData;
    }
    if (offered_) return FunctionStatus::kNoData;
    if (baseline_ && imu_->sequence == imu_sequence_ && imu_->epoch == imu_epoch_)
        return FunctionStatus::kNoData;
    bool rebase = !baseline_ || imu_->epoch != imu_epoch_ ||
                  imu_->received.ms <= previous_.ms || imu_->received.ms - previous_.ms > 200;
    for (std::size_t i = 0; i < 2; ++i) {
        const auto& w = wheels_[i];
        rebase = rebase || stored[i]->epoch != w.epoch ||
                 samples[i]->discontinuity_epoch != w.discontinuity;
    }
    // Do not emit heading-only updates while wheel records are retained. A
    // later new wheel sample captures the whole cumulative travel instead.
    if (!rebase && (stored[0]->sequence == wheels_[0].sequence ||
                    stored[1]->sequence == wheels_[1].sequence)) return FunctionStatus::kNoData;

    BodyMotionIncrement motion;
    motion.startAt = previous_;
    motion.endAt = imu_->received;
    motion.dt_s = (motion.endAt.ms - motion.startAt.ms) / 1000.0;
    motion.dtheta_rad = degToRad((static_cast<double>(imu_->rotation_mdeg) - rotation_) / 1000.0);
    // Catch unannounced zeroing/discontinuous angles rather than teleporting.
    if (!rebase && std::fabs(motion.dtheta_rad) > 12.0 * motion.dt_s + 0.1) rebase = true;
    for (std::size_t i = 0; i < 2; ++i) {
        auto& w = wheels_[i];
        motion.dx_m += 0.5 * ((samples[i]->angle_rad - w.angle) * w.radius * w.sign +
                              w.y * motion.dtheta_rad);
        Provenance source = stored[i]->upstream;
        source.source = w.binding.id.value;
        source.sequence = stored[i]->sequence;
        source.epoch = stored[i]->epoch;
        motion.sources.push_back(std::move(source)); // preserve actual Pico lineage/clock
        w.angle = samples[i]->angle_rad;
        w.sequence = stored[i]->sequence;
        w.epoch = stored[i]->epoch;
        w.discontinuity = samples[i]->discontinuity_epoch;
    }
    motion.sources.push_back(Provenance{"brain_imu_bench", "brain.vex_imu", "brain",
                                        imu_->sequence, imu_->epoch});
    rotation_ = imu_->rotation_mdeg;
    imu_sequence_ = imu_->sequence;
    imu_epoch_ = imu_->epoch;
    previous_ = imu_->received;
    baseline_ = ready_ = true;
    note_ = "BENCH: arrival-time pairing; sideways motion unmeasured";
    if (rebase) return FunctionStatus::kNoData;
    RobotObservationRecord record;
    record.measuredAt = motion.endAt;
    record.receivedAt = motion.endAt;
    record.payload = TypedPayload::store(std::move(motion), payload_names::kBodyMotionIncrement);
    out[output_] = std::move(record);
    offered_ = true;
    return FunctionStatus::kOk;
}
} // namespace navigatr
