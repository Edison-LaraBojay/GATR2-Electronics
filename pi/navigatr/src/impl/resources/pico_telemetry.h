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
//       <Diagnostics hz="1"/>        optional, 1..5: diagnostic frames
//   </Resource>
//
// channel is 0..2 for an encoder counter (pico.encoder_counts) or imu for
// the yaw gyro (pico.gyro_rate). Output ids are configuration; sensors
// bind to them by name.
//
// Diagnostics asks every Pico boot, through a DIAGNOSTICS command, for the
// optional diagnostic frame (0x14) at hz; the latest is kept for live
// instrumentation. Firmware that answers UnknownOp marks diagnostics
// unavailable ("firmware") until the Pico reboots. A Pico keeps its rate
// until it reboots, so without the element the Pi asks for nothing, except
// that diagnostic frames of the current boot (asked for by an earlier Pi
// process) are turned off once with DIAGNOSTICS 0. Diagnostic frames never
// feed localization.
//
// Instrumentation: with the System's DiagnosticsHub the resource posts
// every decoded status and diagnostic frame, sensor frames only while a
// consumer wants them, and feeds the link's LinkMonitor from the reads and
// writes it already makes (never a second reader).
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
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "translaGATR/frame_codec.h"
#include "core/execution_context.h"
#include "core/time.h"
#include "resources/pico_control.h"
#include "resources/resource_store.h"
#include "resources/serial_link.h"

namespace navigatr
{

struct Diagnostics;
struct LinkStats;
class DiagnosticsHub;
class LinkMonitor;
class ReopeningLink;

// Why Pico diagnostic frames are or are not arriving.
enum class PicoDiagState : uint8_t {
    kNotRequested, // no <Diagnostics>: nothing asked
    kNoLink,       // no fresh Pico frames
    kNoIdentity,   // v1 frames: the firmware takes no commands
    kRequesting,   // DIAGNOSTICS sent, not answered yet (retried)
    kActive,       // the Pico accepted the rate
    kFirmware,     // the Pico answered UnknownOp: firmware without diagnostics
    kRefused,      // the Pico refused the rate (BadBody or another failure)
};

const char* picoDiagStateName(PicoDiagState s);

// One encoder channel as instrumentation shows it: raw counts before any
// profile polarity. No change over the window cannot tell a stationary
// encoder from a disconnected one.
struct PicoEncoderView {
    bool    present   = false; // the channel bit was in a decoded frame
    int32_t counts    = 0;     // newest value; old once fresh is false
    int64_t updated_host_us = -1; // HostClock of the newest update, -1 never
    bool    fresh       = false;  // updated within kFrameFreshMs of the refresh
    bool    delta_known = false;  // fresh, and two samples of one encoder epoch
    int32_t delta     = 0;        // counts change over span_ms
    int64_t span_ms   = 0;        // about 1000 once a second of samples exists
};

struct PicoInstrumentation {
    uint8_t       diag_hz    = 0;  // configured rate, 0 not requested
    PicoDiagState diag_state = PicoDiagState::kNotRequested;
    bool          have_diag  = false;
    translagatr::PicoDiag diag;          // newest diagnostic frame, any boot
    int64_t       diag_host_us  = -1;    // HostClock at its decode
    uint64_t      diag_frames   = 0;
    uint64_t      sensor_frames = 0;
    uint64_t      status_frames = 0;
    std::array<PicoEncoderView, 3> encoders;

    // The serial device under the link, when it is a reopening device link.
    bool        serial_reopening = false;
    bool        serial_open      = false;
    uint64_t    serial_attempts  = 0;   // reopen attempts, not the first open
    uint64_t    serial_reopens   = 0;
    uint64_t    serial_closes    = 0;
    std::string serial_error;           // why the last open failed
};

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

    // Instrumentation, set at build before the first refresh. hub may be
    // null (outside a System): nothing is posted or monitored then.
    void attachDiagnostics(DiagnosticsHub* hub);
    // 0 off (the default), 1..5 Hz; larger values are clamped to 5.
    void setDiagnosticsRate(uint8_t hz);

    static constexpr uint8_t kMaxDiagHz        = 5;
    static constexpr double  kDiagTimeoutS     = 2.0;  // per DIAGNOSTICS attempt
    static constexpr int64_t kDiagRetryMs      = 5000; // after an unanswered attempt
    static constexpr int64_t kEncoderWindowMs  = 1000; // delta window

    // Live view for the instrumentation panel; any thread.
    PicoInstrumentation instrumentation() const;

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
        uint8_t           pico_detail = 0;  // Pico detail of a reported failure
        PicoRequestStatus status;
    };

    void applyPacket(const translagatr::SensorSample& s, MonotonicTime now, LinkStats* stats);
    void applyStatus(const translagatr::PicoStatus& s, MonotonicTime now);
    void applyDiag(const translagatr::PicoDiag& d, const uint8_t* payload);
    void sendDue(MonotonicTime now, LinkStats* stats);
    void requestDiagnostics(MonotonicTime now);
    void driveDiagnostics(MonotonicTime now);
    void noteEncoders(MonotonicTime now, uint16_t mask, bool rebase);
    void noteFrame(const char* name, const std::string& fields);
    void postSensor(const translagatr::SensorSample& s);

    // Callers hold state_mutex_.
    void failStale(bool identity, uint16_t boot_id);
    bool namedByStatus(const Request& r) const;
    uint16_t nextRequestId();
    void publishLinkState(MonotonicTime now);

    std::shared_ptr<SerialLink> link_;
    ReopeningLink*              reopening_ = nullptr;   // link_ itself, when it is one
    std::string                 diagnostics_id_;
    translagatr::FrameReader          reader_;

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
    translagatr::PicoStatus   status_;     // newest status frame, any boot
    MonotonicTime       status_at_;  // host clock
    std::deque<Request> requests_;   // oldest first, at most kRequestsKept
    uint32_t            next_handle_ = 1;
    uint16_t            next_request_id_ = 1;
    MonotonicTime       last_command_at_; // host clock, unset before the first
    PicoInstrumentation view_;            // published at the end of each refresh

    // Instrumentation, pipeline thread only.
    DiagnosticsHub*              hub_       = nullptr;
    uint16_t                     source_id_ = 0;
    std::shared_ptr<LinkMonitor> monitor_;
    uint8_t                      diag_hz_     = 0;
    uint32_t                     diag_handle_ = 0;     // DIAGNOSTICS in flight, 0 none
    uint16_t                     diag_boot_   = 0;     // boot it was sent to
    bool                         diag_settled_ = false; // answered for diag_boot_
    MonotonicTime                diag_retry_at_;
    PicoDiagState                diag_state_ = PicoDiagState::kNotRequested;
    bool                         have_diag_  = false;
    translagatr::PicoDiag        diag_;
    int64_t                      diag_host_us_ = -1;
    uint64_t                     diag_frames_ = 0, sensor_frames_ = 0, status_frames_ = 0;
    struct EncoderHistory {
        std::deque<std::pair<int64_t, int32_t>> samples; // (host ms, counts), oldest first
        int64_t updated_ms      = -1;  // pipeline host ms of the newest update
        int64_t updated_host_us = -1;
    };
    std::array<EncoderHistory, kEncoderChannels> encoder_history_;
};

ResourceInstance make_pico_telemetry(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err);

} // namespace navigatr
