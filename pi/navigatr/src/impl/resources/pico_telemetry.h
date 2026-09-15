// pico_telemetry.h
// Shared decoder for one Pico telemetry stream, and the resource that
// publishes its configured channels. The Pico packs several physical
// sensors into one packet; the resource drains and decodes the serial link
// once per cycle and publishes each configured channel as a named output
// in the ResourceMap, so every channel sensor reads from the same decoded
// packet and nobody re-drains the UART. All Pico wire knowledge lives here
// and in common/, nowhere else in navigatr.
//
//   <Resource id="pico_telemetry" type="pico_telemetry">
//       <Serial resource_id="pico_uart"/>
//       <Output id="encoder_a" channel="0"/>
//       <Output id="encoder_b" channel="1"/>
//       <Output id="imu" channel="imu"/>
//   </Resource>
//
// channel is 0..2 for an encoder counter (pico.encoder_counts) or imu for
// the yaw gyro (pico.gyro_rate). Output ids are configuration; sensors
// bind to them by name.
//
// Thread safety: single-threaded; refresh and channel reads happen on the
// pipeline thread.

#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "common/frame_codec.h"
#include "core/time.h"
#include "resources/resource_store.h"
#include "resources/serial_link.h"

namespace navigatr
{

struct Diagnostics;

class PicoTelemetry
{
public:
    PicoTelemetry(std::shared_ptr<SerialLink> link, std::string diagnostics_id);

    // Drains and decodes at most once per cycle; later calls in the same
    // cycle see the identical snapshot.
    void refresh(uint64_t cycle, Diagnostics* diagnostics);

    struct Channel {
        bool          present = false;
        int32_t       value[2] = {0, 0};
        MonotonicTime measuredAt;   // device clock
        uint64_t      updates = 0;  // counts distinct decoded publications
    };

    static constexpr int kEncoderChannels = 3;

    const Channel& encoder(int channel) const { return encoders_[channel]; }
    const Channel& gyro() const { return gyro_; }
    const Channel& accel() const { return accel_; }

    // Yaw integrated over every decoded packet in raw wire units times
    // seconds (millidegrees), so a drained batch loses no rotation the way
    // a latest-rate snapshot would. Intervals longer than kGyroGapMs are
    // dropped and reseeded, never integrated; every dropped interval bumps
    // the accumulator epoch so consumers can tell a discontinuity from
    // zero rotation.
    double   gyroAccumulatedRaw() const { return gyro_accum_raw_; }
    uint64_t gyroAccumulatedEpoch() const { return gyro_accum_epoch_; }

    static constexpr int64_t kGyroGapMs = 250;

    bool linkDead() const { return link_dead_; }
    bool anyPacket() const { return have_seq_; }

    // Device restart generation: bumps when the device clock runs backwards
    // (a Pico reboot); every packet counter and accumulator baseline
    // restarts with it.
    uint64_t deviceEpoch() const { return device_epoch_; }
    uint64_t packetsDecoded() const { return packets_decoded_; }
    const std::string& clockId() const { return diagnostics_id_; }

    void reset();

private:
    void applyPacket(const gatr2::SensorSample& s, Diagnostics* diagnostics);

    std::shared_ptr<SerialLink> link_;
    std::string                 diagnostics_id_;
    gatr2::FrameReader          reader_;

    Channel encoders_[kEncoderChannels];
    Channel gyro_;
    Channel accel_;

    double        gyro_accum_raw_   = 0.0;   // mdeg (mdps integrated over s)
    uint64_t      gyro_accum_epoch_ = 0;     // bumps on every dropped interval
    bool          gyro_have_prev_   = false;
    int32_t       gyro_prev_raw_    = 0;
    MonotonicTime gyro_prev_stamp_;

    bool     have_seq_        = false;
    uint8_t  last_seq_        = 0;
    bool     have_stamp_      = false;
    MonotonicTime last_stamp_;
    uint64_t device_epoch_    = 0;
    uint64_t packets_decoded_ = 0;
    uint64_t last_poll_cycle_ = 0;
    bool     polled_once_     = false;
    bool     link_dead_       = false;
};

ResourceInstance make_pico_telemetry(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err);

} // namespace navigatr
