// pico_channels.cpp

#include "impl/sensors/pico_channels.h"

#include "impl/resources/pico_telemetry.h"
#include "math/angles.h"
#include "resources/resource_store.h"

namespace navigatr
{

namespace
{

std::shared_ptr<PicoTelemetry> telemetryFrom(const ConfigNode& node,
                                             SensorInitializationContext& context,
                                             ConfigNode& source_out, std::string& err) {
    source_out = node.child("Source");
    const ResourceId telemetry_id{source_out.attr("resource_id")};
    if (telemetry_id.empty()) {
        err = node.path() + ": needs <Source resource_id=.../>";
        return nullptr;
    }
    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    std::string inner;
    auto        telemetry = context.resources->require<PicoTelemetry>(telemetry_id, inner);
    if (telemetry == nullptr) {
        err = node.path() + ": " + inner;
    }
    return telemetry;
}

} // namespace

std::unique_ptr<Sensor> PicoEncoderChannelSensor::create(
    const ConfigNode& node, SensorInitializationContext& context, std::string& err) {
    auto sensor = std::make_unique<PicoEncoderChannelSensor>();

    ConfigNode source;
    sensor->telemetry_ = telemetryFrom(node, context, source, err);
    if (sensor->telemetry_ == nullptr) {
        return nullptr;
    }

    long channel = -1;
    if (!source.getInt("channel", -1, channel, err)) {
        return nullptr;
    }
    if (channel < 0 || channel >= PicoTelemetry::kEncoderChannels) {
        err = source.path() + ": channel must be 0.." +
              std::to_string(PicoTelemetry::kEncoderChannels - 1);
        return nullptr;
    }
    sensor->channel_ = static_cast<int>(channel);

    const ConfigNode calibration = node.child("Calibration");
    double           cpr         = 0.0;
    bool             invert      = false;
    if (!calibration.getDouble("counts_per_revolution", 0.0, cpr, err) ||
        !calibration.getBool("invert", false, invert, err)) {
        return nullptr;
    }
    if (cpr <= 0.0) {
        err = node.path() + ": Calibration needs positive counts_per_revolution";
        return nullptr;
    }
    sensor->radians_per_count_ = (2.0 * kPi / cpr) * (invert ? -1.0 : 1.0);

    sensor->payload_ =
        PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample);
    return sensor;
}

void PicoEncoderChannelSensor::reset() {
    have_prev_    = false;
    angle_rad_    = 0.0;
    last_updates_ = 0;
    if (telemetry_ != nullptr) {
        telemetry_->reset();
    }
}

SensorPollResult PicoEncoderChannelSensor::poll(const SensorExecutionInput& input) {
    telemetry_->refresh(input.cycle, input.diagnostics);

    SensorPollResult result;

    const PicoTelemetry::Channel& channel = telemetry_->encoder(channel_);
    if (channel.updates == last_updates_) {
        // data decoded before a link death still counts; the fault shows on
        // the first poll with nothing new
        if (telemetry_->linkDead()) {
            result.state      = SensorState::kFault;
            result.diagnostic = "telemetry link dead";
            return result;
        }
        result.state =
            channel.present ? SensorState::kValid : SensorState::kNoDataYet;
        return result;
    }
    last_updates_ = channel.updates;

    const int32_t counts = channel.value[0];
    if (have_prev_) {
        // modular unsigned subtraction survives count wraparound
        const int32_t delta = static_cast<int32_t>(static_cast<uint32_t>(counts) -
                                                   static_cast<uint32_t>(prev_counts_));
        angle_rad_ += delta * radians_per_count_;
    }
    have_prev_   = true;
    prev_counts_ = counts;

    result.state = SensorState::kValid;
    result.publication =
        SensorPublication{channel.measuredAt,
                          TypedPayload::store(EncoderSample{angle_rad_},
                                              payload_names::kEncoderSample)};
    return result;
}

std::unique_ptr<Sensor> PicoImuChannelSensor::create(const ConfigNode& node,
                                                     SensorInitializationContext& context,
                                                     std::string& err) {
    auto sensor = std::make_unique<PicoImuChannelSensor>();

    ConfigNode source;
    sensor->telemetry_ = telemetryFrom(node, context, source, err);
    if (sensor->telemetry_ == nullptr) {
        return nullptr;
    }
    const std::string channel = source.attr("channel");
    if (channel != "imu") {
        err = source.path() + ": pico_imu_channel supports channel=\"imu\"";
        return nullptr;
    }
    const ConfigNode calibration = node.child("Calibration");
    if (!calibration.getBool("invert", false, sensor->invert_, err)) {
        return nullptr;
    }

    sensor->payload_ = PayloadDescriptor::of<ImuSample>(payload_names::kImuSample);
    return sensor;
}

void PicoImuChannelSensor::reset() {
    last_updates_ = 0;
    if (telemetry_ != nullptr) {
        telemetry_->reset();
    }
}

SensorPollResult PicoImuChannelSensor::poll(const SensorExecutionInput& input) {
    telemetry_->refresh(input.cycle, input.diagnostics);

    SensorPollResult result;

    const PicoTelemetry::Channel& gyro = telemetry_->gyro();
    if (gyro.updates == last_updates_) {
        if (telemetry_->linkDead()) {
            result.state      = SensorState::kFault;
            result.diagnostic = "telemetry link dead";
            return result;
        }
        result.state = gyro.present ? SensorState::kValid : SensorState::kNoDataYet;
        return result;
    }
    last_updates_ = gyro.updates;

    // wire unit is millidegrees per second
    const double sign = invert_ ? -1.0 : 1.0;
    ImuSample    sample;
    sample.yaw_rate_rad_s = sign * degToRad(gyro.value[0] / 1000.0);

    result.state = SensorState::kValid;
    result.publication =
        SensorPublication{gyro.measuredAt,
                          TypedPayload::store(sample, payload_names::kImuSample)};
    return result;
}

} // namespace navigatr
