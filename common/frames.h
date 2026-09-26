// frames.h
// Wire format shared by the Pico, Pi, and brain.
// No hardware dependencies. Must compile on all three targets.

#pragma once
#include <stdint.h>

namespace gatr2
{

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;

// Bounded so every parser can use a static buffer.
constexpr uint16_t kMaxFrameLen = 128;

// 0x02 and 0x03 are retired (pose and command frames of the one-way link).
enum FrameType : uint8_t {
    kFrameSensor       = 0x01, // Pico  -> Pi
    kFrameBrainRequest = 0x10, // brain -> Pi
    kFrameBrainReply   = 0x11, // Pi    -> brain
};

// ---------------------------------------------------------------------------
// Sensor frame (Pico -> Pi)
//
//   sync0 sync1 | type | seq u8 | stamp_ms u32 | mask u16
//   | payload, present sensors only, in ascending bit order
//   | xor u8
//
// A cleared bit means the sensor did not report.
// ---------------------------------------------------------------------------
enum SensorBit : uint16_t {
    kSensorEnc0    = 1u << 0,   // bit 0, parallel tracking wheel
    kSensorEnc1    = 1u << 1,   // bit 1, perpendicular tracking wheel
    kSensorEnc2    = 1u << 2,   // bit 2, spare / second parallel
    kSensorGyroZ   = 1u << 3,   // bit 3, yaw rate
    kSensorAccelXY = 1u << 4,   // bit 4, reserved
};

// Payload width in bytes for each bit, ascending. A parser walks the frame
// using only this table, with no knowledge of what a sensor means.
constexpr uint8_t kSensorWidth[] = {
    4,  // kSensorEnc0     int32, counts
    4,  // kSensorEnc1     int32, counts
    4,  // kSensorEnc2     int32, counts
    4,  // kSensorGyroZ    int32, millidegrees per second
    8,  // kSensorAccelXY  2 x int32, milli-g
};
constexpr uint8_t kSensorBitCount = sizeof(kSensorWidth);

struct SensorSample {
    uint8_t  seq;
    uint32_t stamp_ms;   // Pico acquisition clock; the Pi maps it to host time
    uint16_t mask;
    int32_t  enc[3];
    int32_t  gyro_z;     // mdeg/s, raw, bias not removed
    int32_t  accel[2];   // mg
};

// ---------------------------------------------------------------------------
// Brain link v3 (brain request, Pi reply)
//
//   sync0 sync1 | type | len u8 | payload[len] | crc u16
//
// crc is CRC-16/CCITT-FALSE over type, len and payload. The brain is the only
// initiator and has at most one request outstanding.
//
// Request payload:  version u8 | op u8 | session u32 | request_id u16 | body
// Reply payload:    version u8 | op u8 | session u32 | request_id u16
//                   | result u8 | pi_instance u32 | body
//
// These header offsets are frozen for every version.
// ---------------------------------------------------------------------------

constexpr uint8_t kBrainLinkVersion = 3;

enum BrainOp : uint8_t {
    kOpHello          = 1, // body nonce u32
    kOpSetPose        = 2, // body x_mm i32, y_mm i32, heading_cdeg i32
    kOpSelectLandmark = 3, // body landmark_id u8, flags u8
    kOpGetState       = 4, // no body
};

enum BrainResult : uint8_t {
    kResultOk                  = 0, // done; for SET_POSE localization applied it
    kResultPending             = 1, // SET_POSE accepted in this session, not applied yet
    kResultUnknownSession      = 2,
    kResultUnsupportedVersion  = 3, // reply header carries the Pi version
    kResultUnsupportedOp       = 4,
    kResultInvalidArgument     = 5, // request_id 0, or reused id with a different op/body
    kResultUnknownLandmark     = 6, // no configured mapping for the id
    kResultLandmarkUnsupported = 7, // world estimation is noop
    kResultStale               = 8, // old request id, or HELLO reusing a recent nonce
};

enum SelectFlagBit : uint8_t {
    kSelectFlagSelected = 1u << 0, // clear releases the selection
};

// State block robot_flags. Never acknowledgements.
enum RobotFlagBit : uint8_t {
    kRobotPoseValid         = 1u << 0, // estimator produced a pose
    kRobotLocalized         = 1u << 1, // field anchor set by a placement
    kRobotAgeKnown          = 1u << 2, // robot_age_ms is meaningful
    kRobotAnchorCommand     = 1u << 3, // anchor from a brain SET_POSE
    kRobotAnchorConfigured  = 1u << 4, // anchor from the configured initial placement
};

// State block health. Information only, never acknowledgements.
enum HealthBit : uint8_t {
    kHealthEncodersFresh  = 1u << 0,
    kHealthGyroFresh      = 1u << 1,
    kHealthVisionAlive    = 1u << 2,
    kHealthBiasCalibrated = 1u << 3,
};

enum LandmarkSourceCode : uint8_t {
    kLandmarkSourceNone     = 0,
    kLandmarkSourceNominal  = 1,
    kLandmarkSourceObserved = 2,
};

// Brain -> Pi. Body fields are used only by their op.
struct BrainRequest {
    uint8_t  version    = kBrainLinkVersion;
    uint8_t  op         = 0;
    uint32_t session    = 0; // 0 in HELLO
    uint16_t request_id = 0; // per brain boot, 1..65535, never 0

    uint32_t nonce        = 0; // HELLO
    int32_t  x_mm         = 0; // SET_POSE, field frame
    int32_t  y_mm         = 0;
    int32_t  heading_cdeg = 0;
    uint8_t  landmark_id  = 0; // SELECT_LANDMARK
    uint8_t  select_flags = 0;
};

// GET_STATE Ok body. Robot and landmark poses share the same field anchor.
struct BrainState {
    uint8_t  robot_flags     = 0;
    int32_t  x_mm            = 0; // robot origin, field frame
    int32_t  y_mm            = 0;
    int32_t  heading_cdeg    = 0; // CCW from +x, (-18000, 18000]
    uint16_t robot_age_ms    = 0; // Pi cycle time minus measurement time, clamped
    uint32_t odometry_epoch  = 0; // low 32 bits
    uint32_t anchor_revision = 0; // low 32 bits
    uint8_t  health          = 0;
    uint8_t  landmark_id     = 0; // selected landmark, 0 when none
    uint8_t  landmark_source = 0; // LandmarkSourceCode
    int32_t  lm_x_mm         = 0; // physical landmark pose, field frame
    int32_t  lm_y_mm         = 0;
    int32_t  lm_heading_cdeg = 0;
    uint16_t landmark_age_ms = 0; // observed only, clamped; 0 otherwise
};

// Pi -> brain. Body fields are used only by their (op, result).
struct BrainReply {
    uint8_t  version     = kBrainLinkVersion;
    uint8_t  op          = 0; // echo
    uint32_t session     = 0; // echo, or the opened session for HELLO Ok
    uint16_t request_id  = 0; // echo
    uint8_t  result      = kResultOk;
    uint32_t pi_instance = 0; // random nonzero id per Pi process start and reset

    uint32_t   nonce           = 0; // HELLO, every result
    uint32_t   odometry_epoch  = 0; // SET_POSE Ok and Pending, current values
    uint32_t   anchor_revision = 0;
    uint8_t    landmark_id     = 0; // SELECT_LANDMARK Ok, echo
    uint8_t    select_flags    = 0;
    BrainState state;               // GET_STATE Ok
};

// ---------------------------------------------------------------------------

inline uint8_t checksum(const uint8_t* data, uint16_t len) {
    uint8_t x = 0;
    for (uint16_t i = 0; i < len; ++i) {
        x ^= data[i];
    }
    return x;
}

} // namespace gatr2
