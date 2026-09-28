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
// Identity. v2 sensor frames carry boot_id, acq_epoch and imu_epoch. A new
// boot or acquisition moves the encoder epoch, a new boot or IMU
// initialization the IMU epoch; each is the upstream epoch of its outputs,
// so channel sensors rebase instead of differencing across a restart. v1
// frames carry no identity: a device clock regression is taken as a reboot,
// and commands are refused.
//
// Commands (PicoControl) go out on the same serial link from refresh(),
// one frame at a time, and are resent until a status frame names them.
//
// Thread safety: refresh and the channel reads run on the pipeline thread.
// The PicoControl part is safe from any thread.

#pragma once
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "common/frame_codec.h"
#include "core/execution_context.h"
#include "core/time.h"
#include "resources/pico_control.h"
#include "resources/resource_store.h"
#include "resources/serial_link.h"

namespace navigatr
{

struct Diagnostics;
struct LinkStats;

// Consumers of the Pico link require<PicoTelemetry> by resource id and use
// it through PicoControl.
class PicoTelemetry : public PicoControl
{
public:
    // first_request_id 0 draws a random request id base, so a restarted Pi
    // never repeats ids the Pico has recorded; tests pass a fixed one.
    PicoTelemetry(std::shared_ptr<SerialLink> link, std::string diagnostics_id,
                  uint16_t first_request_id = 0);

    // Drains and decodes at most once per cycle, then sends at most one due
    // command; later calls in the same cycle see the identical snapshot.
    void refresh(const ExecutionContext& context);

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
    // a latest-rate snapshot would. Intervals longer than kGyroGapMs, and
    // any IMU or device restart, are dropped and reseeded, never integrated;
    // every dropped interval bumps the accumulator epoch so consumers can
    // tell a discontinuity from zero rotation.
    double   gyroAccumulatedRaw() const { return gyro_accum_raw_; }
    uint64_t gyroAccumulatedEpoch() const { return gyro_accum_epoch_; }

    static constexpr int64_t kGyroGapMs    = 250;
    static constexpr int64_t kFrameFreshMs = 250; // PicoLinkState::frames_fresh
    static constexpr int64_t kResendMs     = 200; // per command, until reported
    static constexpr int64_t kCommandGapMs = 100; // between any two command frames
    static constexpr std::size_t kRequestsKept = 16;

    bool linkDead() const { return link_dead_; }
    bool anyPacket() const { return packets_decoded_ > 0; } // since reset()
    bool identity() const { return have_frame_ && last_identity_; }

    // Restart generations, the upstream epochs of the published outputs.
    // Encoder: Pico reboot (boot_id, frame version or clock regression) or
    // acquisition restart. IMU: Pico reboot or IMU initialization.
    uint64_t encoderEpoch() const { return encoder_epoch_; }
    uint64_t imuEpoch() const { return imu_epoch_; }
    uint64_t packetsDecoded() const { return packets_decoded_; }
    const std::string& clockId() const { return diagnostics_id_; }

    // Decoder and channels back to power-on. Identity, epochs and commands
    // survive: they describe the Pico, not this pipeline.
    void reset();

    PicoLinkState     link() const override;
    uint32_t          submit(uint8_t op, uint8_t arg, MonotonicTime now, double timeout_s) override;
    PicoRequestStatus request(uint32_t handle) const override;

private:
    struct Request {
        uint32_t          handle = 0;
        uint16_t          request_id = 0;
        uint8_t           op = 0;
        uint8_t           arg = 0;
        uint16_t          target = 0; // boot_id at submit
        MonotonicTime     deadline;   // host clock
        MonotonicTime     last_sent;  // unset until first sent
        bool              reported = false; // a status frame named it
        PicoRequestStatus status;
    };

    void applyPacket(const gatr2::SensorSample& s, MonotonicTime now, LinkStats* stats);
    void applyStatus(const gatr2::PicoStatus& s, MonotonicTime now);
    void sendDue(MonotonicTime now, LinkStats* stats);

    // Callers hold state_mutex_.
    void failStale(bool identity, uint16_t boot_id);
    bool namedByStatus(const Request& r) const;
    uint16_t nextRequestId();
    void publishLinkState(MonotonicTime now);

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

    // newest sensor frame; kept across reset()
    bool          have_frame_    = false;
    bool          last_identity_ = false;
    uint16_t      last_boot_     = 0;
    uint8_t       last_acq_      = 0;
    uint8_t       last_imu_      = 0;
    MonotonicTime last_stamp_;    // device clock
    MonotonicTime last_frame_at_; // host clock
    bool          have_seq_ = false;
    uint8_t       last_seq_ = 0;

    uint64_t encoder_epoch_   = 0;
    uint64_t imu_epoch_       = 0;
    uint64_t reboots_         = 0;
    uint64_t restarts_        = 0;
    uint64_t imu_restarts_    = 0;
    uint64_t packets_decoded_ = 0;
    uint64_t last_poll_cycle_ = 0;
    bool     polled_once_     = false;
    bool     link_dead_       = false;

    // Guards everything below; the pipeline thread writes, anyone reads.
    mutable std::mutex  state_mutex_;
    PicoLinkState       link_state_; // published at the end of each refresh
    bool                have_status_ = false;
    gatr2::PicoStatus   status_;     // newest status frame, any boot
    MonotonicTime       status_at_;  // host clock
    std::deque<Request> requests_;   // oldest first, at most kRequestsKept
    uint32_t            next_handle_ = 1;
    uint16_t            next_request_id_ = 1;
    MonotonicTime       last_command_at_; // host clock, unset before the first
};

ResourceInstance make_pico_telemetry(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err);

} // namespace navigatr
