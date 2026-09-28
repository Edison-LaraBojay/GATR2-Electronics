// records.h
// What producers post to the DiagnosticsHub: small fixed-size copies of data
// the runtime already has, taken on the producing thread. No formatting and
// no allocation there; the recorder and the viewer format on their own
// threads. Every record keeps the source's own timestamp and clock next to
// the Pi host arrival time, so nothing has to be inferred later.

#pragma once
#include <cstdint>
#include <variant>

#include "translaGATR/frames.h"

namespace navigatr
{

enum class DiagKind : uint8_t {
    kRobotState     = 0, // every localization publication
    kPicoSensor     = 1, // one decoded Pico sensor frame
    kPicoStatus     = 2, // one decoded Pico status frame
    kPicoDiag       = 3, // one Pico diagnostic frame (raw payload)
    kVexImu         = 4, // one Brain bench IMU sample carried by GET_STATE
    kBrainRequest   = 5, // one Brain request with the result the Pi answered
    kBrainTelemetry = 6, // one Brain TELEMETRY report (raw body)
    kPath           = 7, // one PATH_REPORT
    kEvent          = 8, // one runtime event
    kBytes          = 9, // raw transport bytes, only while raw capture is on
};
constexpr int kDiagKindCount = 10;

constexpr uint32_t diagBit(DiagKind k) { return 1u << static_cast<uint8_t>(k); }
constexpr uint32_t kDiagAllKinds = (1u << kDiagKindCount) - 1u;

const char* diagKindName(DiagKind k);

// Clock of a source timestamp.
enum class DiagClock : uint8_t {
    kNone   = 0,
    kPiHost = 1, // HostClock, milliseconds since the Pi process started
    kPico   = 2, // Pico stamp_ms, per Pico boot
    kBrain  = 3, // Brain millis, per Brain program start
};

struct DiagRobotState {
    uint64_t publication       = 0;  // feed publication counter
    int64_t  measured_ms       = 0;  // source clock of the newest measurement
    DiagClock measured_clock   = DiagClock::kNone;
    int64_t  measured_host_ms  = -1; // mapped to the Pi host clock, -1 unknown
    double   odom_x_m          = 0;
    double   odom_y_m          = 0;
    double   odom_heading_rad  = 0;
    double   field_x_m         = 0;
    double   field_y_m         = 0;
    double   field_heading_rad = 0;
    float    vx_m_s            = 0;
    float    vy_m_s            = 0;
    float    yaw_rate_rad_s    = 0;
    float    confidence        = 0;
    float    roll_rad          = 0; // with attitude_valid
    float    pitch_rad         = 0;
    uint64_t odometry_epoch    = 0;
    uint64_t anchor_revision   = 0;
    uint64_t placement_sequence = 0;
    uint32_t placement_session = 0;
    bool     valid             = false;
    bool     initialized       = false; // placed (field anchor set)
    bool     attitude_valid    = false;
    bool     attitude_assumed_level = false;
    bool     stationary        = false;
    bool     advanced          = false; // this publication carries a new measurement
};

struct DiagPicoSensor {
    uint8_t  version    = 0; // sensor frame 1 or 2
    uint16_t boot_id    = 0; // v2
    uint8_t  acq_epoch  = 0; // v2
    uint8_t  imu_epoch  = 0; // v2
    uint8_t  seq        = 0;
    uint8_t  mask       = 0; // translagatr::SensorMaskBit
    uint32_t stamp_ms   = 0; // Pico clock
    int32_t  enc[3]     = {0, 0, 0};
    int32_t  gyro_z_mdps = 0;
};

struct DiagPicoStatus {
    translagatr::PicoStatus status;
};

// Raw payload of a Pico diagnostic frame; decoded by the consumer with the
// translaGATR codec.
struct DiagPicoDiag {
    uint8_t len          = 0;
    uint8_t payload[32]  = {};
};

struct DiagVexImu {
    uint32_t session       = 0;
    uint16_t request_id    = 0;
    uint8_t  flags         = 0; // translagatr::BenchImuFlagBit
    uint32_t stamp_ms      = 0; // Brain clock
    int32_t  rotation_mdeg = 0;
    bool     accepted      = false; // the bench mailbox took it as a new sample
};

struct DiagBrainRequest {
    uint32_t session    = 0;
    uint16_t request_id = 0;
    uint8_t  op         = 0;
    uint8_t  result     = 0;     // the reply result the Pi sent
    uint8_t  request_len = 0;    // frame bytes
    uint8_t  reply_len   = 0;    // frame bytes, 0 when nothing was sent
    bool     duplicate  = false; // answered from the dedupe record
};

// Raw TELEMETRY body; decoded by the consumer with the translaGATR codec.
struct DiagBrainTelemetry {
    uint32_t session  = 0;
    uint8_t  len      = 0;
    uint8_t  body[96] = {};
};

struct DiagPath {
    uint32_t session     = 0;
    uint32_t command_id  = 0;
    uint8_t  mode        = 0; // translagatr::PathMode
    uint8_t  count       = 0;
    translagatr::PathPoint points[translagatr::kPathReportMaxPoints];
};

struct DiagEvent {
    char text[120] = {}; // NUL terminated, truncated
};

enum class DiagDirection : uint8_t {
    kRx          = 0, // bytes read from the link
    kTxAttempted = 1, // bytes handed to write()
    kTxAccepted  = 2, // bytes the link reported written
};

struct DiagBytes {
    DiagDirection dir = DiagDirection::kRx;
    uint8_t       len = 0;
    uint8_t       data[64] = {};
};

using DiagPayload = std::variant<DiagRobotState, DiagPicoSensor, DiagPicoStatus, DiagPicoDiag,
                                 DiagVexImu, DiagBrainRequest, DiagBrainTelemetry, DiagPath,
                                 DiagEvent, DiagBytes>;

struct DiagRecord {
    DiagKind    kind    = DiagKind::kEvent;
    uint16_t    source  = 0; // DiagnosticsHub::sourceId of the producer
    int64_t     host_us = 0; // Pi host clock when posted (HostClock::nowUs)
    DiagPayload payload;
};

} // namespace navigatr
