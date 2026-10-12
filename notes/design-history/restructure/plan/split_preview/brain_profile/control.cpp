// control.cpp

#include "runtime/system.h"

#include <algorithm>
#include <cmath>

#include "payloads/pico_telemetry_samples.h"
#include "payloads/sensor_samples.h"

namespace navigatr
{

uint8_t System::controlProfile(uint8_t action, uint8_t, MonotonicTime now, uint8_t& detail) {
    detail = translagatr::kControlDetailNone;
    const std::shared_ptr<const ProfileBinding> binding = profileBinding();
    if (binding == nullptr) {
        return translagatr::kResultNotReady;
    }
    const auto stationary = [&](const char* name) {
        std::string why;
        if (precheck_.still(now, &why)) {
            return true;
        }
        noteEvent(now, std::string(name) + " refused, not stationary: " + why);
        return false;
    };
    switch (action) {
    case translagatr::kControlRecalibrate: {
        if (!stationary("recalibrate")) {
            return translagatr::kResultNotStationary;
        }
        const bool any = execute_localization_.recalibrate();
        noteEvent(now, any ? "IMU bias recalibration started; the pose holds while the robot "
                             "stays still"
                           : "recalibrate: nothing to calibrate on the Pi for this profile");
        return translagatr::kResultOk;
    }
    case translagatr::kControlReinitialize:
        if (!stationary("reinitialize")) {
            return translagatr::kResultNotStationary;
        }
        execute_localization_.reset();
        placement_floor_ = command_.init_sequence;
        noteEvent(now, "localization reinitialized: odometry epoch " +
                           std::to_string(execute_localization_.state().odometry_epoch) +
                           ", placement withdrawn, bias recalibrating");
        return translagatr::kResultOk;
    case translagatr::kControlReinitImu: {
        if (binding->profile.imu_source != translagatr::kImuSourcePico) {
            detail = translagatr::kControlDetailImuUnused;
            noteEvent(now, "IMU reinitialization refused: the profile does not use the Pico IMU");
            return translagatr::kResultFailed;
        }
        const uint32_t handle =
            submitPico(translagatr::kPicoOpReinitImu, binding->profile.imu_port, now, detail);
        if (handle == 0) {
            return translagatr::kResultFailed;
        }
        pico_op_         = PicoOperation{};
        pico_op_.active  = true;
        pico_op_.action  = action;
        pico_op_.handle  = handle;
        pico_op_.started = now;
        pico_op_.result  = translagatr::kResultPending;
        noteEvent(now, "Pico IMU reinitialization requested");
        return translagatr::kResultPending;
    }
    case translagatr::kControlRestartAcquisition: {
        if (!stationary("acquisition restart")) {
            return translagatr::kResultNotStationary;
        }
        const PicoLinkState link = pico_ != nullptr ? pico_->link() : PicoLinkState{};
        const uint32_t handle = submitPico(translagatr::kPicoOpRestartAcquisition, 0, now, detail);
        if (handle == 0) {
            return translagatr::kResultFailed;
        }
        pico_op_           = PicoOperation{};
        pico_op_.active    = true;
        pico_op_.action    = action;
        pico_op_.handle    = handle;
        pico_op_.started   = now;
        pico_op_.acq_epoch = link.acq_epoch;
        pico_op_.restarts  = link.restarts;
        pico_op_.result    = translagatr::kResultPending;
        noteEvent(now, "Pico acquisition restart requested");
        return translagatr::kResultPending;
    }
    default: return translagatr::kResultInvalidArgument;
    }
}

uint8_t System::readWheels(MonotonicTime now, uint8_t& count, translagatr::WheelReading* wheels) const {
    count = 0;
    const std::shared_ptr<const ProfileBinding> binding = profileBinding();
    if (binding == nullptr) {
        return translagatr::kResultNotReady;
    }
    const SensorMap&   sensors   = execute_sensors_.retained();
    const ResourceMap& resources = execute_resources_.retained();
    const auto         resource  = resources.find(profile_config_.encoder_resource);
    for (uint8_t i = 0; i < binding->profile.wheel_count && i < translagatr::kWheelReadingsMax; ++i) {
        const translagatr::ProfileWheel& w = binding->profile.wheels[i];
        translagatr::WheelReading        r;
        r.port = w.encoder_port;
        if (resource != resources.end() && w.encoder_port < kProfileEncoderPorts) {
            const auto out = resource->second.outputs.find(profile_config_.ports[w.encoder_port]);
            if (out != resource->second.outputs.end() && out->second.latest.has_value()) {
                const PicoEncoderCounts* raw =
                    out->second.latest->payload.get<PicoEncoderCounts>();
                r.counts = raw != nullptr ? raw->counts : 0;
            }
        }
        const auto it = i < binding->encoders.size() ? sensors.find(binding->encoders[i])
                                                     : sensors.end();
        if (it != sensors.end() && it->second.latest.has_value()) {
            const StoredSample&  stored = *it->second.latest;
            const EncoderSample* sample = stored.payload.get<EncoderSample>();
            if (sample != nullptr) {
                r.flags |= translagatr::kWheelValid;
                // wheel angle already carries counts per revolution, gearing
                // and polarity; the radius makes it raw travel, no travel scale
                const double um = std::round(sample->angle_rad * w.radius_um);
                r.travel_um = static_cast<int32_t>(std::clamp(um, -2147483648.0, 2147483647.0));
                r.discontinuity =
                    static_cast<uint16_t>((sample->discontinuity_epoch + stored.epoch) & 0xFFFF);
                const bool host = stored.receivedAt.domain == ClockDomain::kHost &&
                                  now.domain == ClockDomain::kHost;
                const int64_t age = host ? now - stored.receivedAt : 0;
                r.age_ms = static_cast<uint16_t>(std::clamp<int64_t>(age, 0, 65535));
                if (it->second.state == SourceState::kValid && host && age <= kWheelFreshMs) {
                    r.flags |= translagatr::kWheelFresh;
                }
            }
        }
        wheels[count++] = r;
    }
    return translagatr::kResultOk;
}

} // namespace navigatr
