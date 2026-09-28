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
    kFrameSensor       = 0x01, // Pico  -> Pi, no acquisition identity (older firmware)
    kFrameSensorV2     = 0x04, // Pico  -> Pi, with acquisition identity
    kFrameBrainRequest = 0x10, // brain -> Pi
    kFrameBrainReply   = 0x11, // Pi    -> brain
    kFramePicoCommand  = 0x12, // Pi    -> Pico
    kFramePicoStatus   = 0x13, // Pico  -> Pi
};

// ---------------------------------------------------------------------------
// Sensor frame (Pico -> Pi)
//
//   v1 (0x01)  sync0 sync1 | type | seq u8 | stamp_ms u32 | mask u16
//   v2 (0x04)  sync0 sync1 | type | seq u8 | stamp_ms u32 | boot_id u16
//              | acq_epoch u8 | imu_epoch u8 | mask u16
//   both       | payload, present sensors only, in ascending bit order
//              | xor u8
//
// A cleared bit means the sensor did not report. Encoder bit k is logical
// encoder port k (HAT v2: port 0 J2, port 1 J3, port 2 J4). What a port
// measures is set by the Brain robot profile, not here.
//
// v2 identity: boot_id is random and nonzero per Pico boot. acq_epoch counts
// explicit acquisition restarts in this boot (counters zeroed), imu_epoch
// counts IMU (re)initializations in this boot. Any change means the matching
// measurements are not continuous with earlier ones.
// ---------------------------------------------------------------------------
enum SensorBit : uint16_t {
    kSensorEnc0    = 1u << 0, // bit 0, encoder port 0
    kSensorEnc1    = 1u << 1, // bit 1, encoder port 1
    kSensorEnc2    = 1u << 2, // bit 2, encoder port 2
    kSensorGyroZ   = 1u << 3, // bit 3, yaw rate of IMU port 0
    kSensorAccelXY = 1u << 4, // bit 4, reserved
};

// Payload width in bytes for each bit, ascending. A parser walks the frame
// using only this table, with no knowledge of what a sensor means.
constexpr uint8_t kSensorWidth[] = {
    4, // kSensorEnc0     int32, counts
    4, // kSensorEnc1     int32, counts
    4, // kSensorEnc2     int32, counts
    4, // kSensorGyroZ    int32, millidegrees per second
    8, // kSensorAccelXY  2 x int32, milli-g
};
constexpr uint8_t kSensorBitCount = sizeof(kSensorWidth);

struct SensorSample {
    uint8_t  seq;
    uint32_t stamp_ms; // Pico acquisition clock; the Pi maps it to host time
    uint16_t mask;
    int32_t  enc[3];
    int32_t  gyro_z;   // mdeg/s, raw, bias not removed
    int32_t  accel[2]; // mg

    bool     identity; // v2: the fields below are set
    uint16_t boot_id;
    uint8_t  acq_epoch;
    uint8_t  imu_epoch;
};

// ---------------------------------------------------------------------------
// Brain link v4 (brain request, Pi reply)
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
// These header offsets are frozen for every version. Bodies are in
// frame_codec.h next to their lengths; documents moved in chunks are in
// link_documents.h.
// ---------------------------------------------------------------------------

constexpr uint8_t kBrainLinkVersion = 4;

// 3 (SELECT_LANDMARK) and 5 (GET_STATE_WITH_IMU) are retired v3 ops.
enum BrainOp : uint8_t {
    kOpHello        = 1,  // nonce u32
    kOpSetPose      = 2,  // x_mm i32, y_mm i32, heading_cdeg i32
    kOpGetState     = 4,  // imu_flags u8, imu_stamp_ms u32, imu_rotation_mdeg i32
    kOpProfileWrite = 6,  // profile_id u32, total_len u16, offset u16, data
    kOpProfileApply = 7,  // profile_id u32, total_len u16
    kOpReadDoc      = 8,  // doc_kind u8, doc_id u32, offset u16, max_len u8
    kOpControl      = 9,  // action u8, arg u8
    kOpPathReport   = 10, // command_id u32, path_mode u8, count u8, count x (x_mm i32, y_mm i32)
    kOpReadWheels   = 11, // no body; raw wheel readings for calibration
};

enum BenchImuFlagBit : uint8_t {
    kBenchImuValid = 1u << 0,
};

// 6 and 7 are retired v3 landmark selection results.
enum BrainResult : uint8_t {
    kResultOk                 = 0,  // done; for SET_POSE localization applied it
    kResultPending            = 1,  // SET_POSE or PROFILE_APPLY accepted, not applied yet
    kResultUnknownSession     = 2,
    kResultUnsupportedVersion = 3,  // reply header carries the Pi version
    kResultUnsupportedOp      = 4,
    kResultInvalidArgument    = 5,  // bad body, request_id 0, or reused id with another op/body
    kResultStale              = 8,  // old request id, HELLO reusing a recent nonce, doc replaced
    kResultNotReady           = 9,  // needs an applied robot profile first
    kResultProfileRejected    = 10, // PROFILE_APPLY: the Pi cannot run this profile
    kResultUnavailable        = 11, // READ_DOC: no such document on this Pi
    kResultNotStationary      = 12, // CONTROL: robot moved in the stationary window
    kResultFailed             = 13, // CONTROL: ran and failed, see ControlDetail
};

// State block robot_flags. Never acknowledgements.
enum RobotFlagBit : uint8_t {
    kRobotPoseValid        = 1u << 0, // estimator produced a pose
    kRobotLocalized        = 1u << 1, // field anchor set by a placement
    kRobotAgeKnown         = 1u << 2, // robot_age_ms is meaningful
    kRobotAnchorCommand    = 1u << 3, // anchor from a brain SET_POSE, any session
    kRobotAnchorConfigured = 1u << 4, // anchor from the configured initial placement
};

// State block health. Information only, never acknowledgements.
enum HealthBit : uint8_t {
    kHealthEncodersFresh   = 1u << 0, // every profile encoder
    kHealthGyroFresh       = 1u << 1, // the profile IMU source, Pico or Brain bench
    kHealthVisionAlive     = 1u << 2,
    kHealthBiasCalibrated  = 1u << 3,
    kHealthPicoLink        = 1u << 4, // Pico frames arriving
    kHealthImuInitializing = 1u << 5, // Pico IMU initializing or aligning
    kHealthImuFailed       = 1u << 6, // Pico IMU failed; retries continue while enabled
    kHealthStationary      = 1u << 7, // qualified stationary window right now
};

// Robot profile on the Pi, per state block.
enum ProfileState : uint8_t {
    kProfileNone     = 0, // nothing applied; localization is waiting
    kProfileApplying = 1, // accepted, applied at the next boundary
    kProfileApplied  = 2, // profile_id is running
    kProfileRejected = 3, // profile_id was refused, see ProfileReason
};

// Why a profile was refused. detail names the wheel or camera index when
// the reason is about one.
enum ProfileReason : uint8_t {
    kProfileReasonNone           = 0,
    kProfileReasonFormat         = 1,  // not a decodable profile document
    kProfileReasonTopology       = 2,  // unknown localization topology
    kProfileReasonWheelCount     = 3,  // wrong wheel count for the topology
    kProfileReasonEncoderPort    = 4,  // port not wired on this Pi, or used twice
    kProfileReasonWheelGeometry  = 5,  // radius, counts, offsets, angle or flags out of range
    kProfileReasonObservability  = 6,  // wheel directions cannot resolve the motion
    kProfileReasonImuSource      = 7,  // IMU source unknown or unavailable, or flags it cannot take
    kProfileReasonImuPort        = 8,  // no such Pico IMU port
    kProfileReasonImuCombination = 9,  // topology and IMU source cannot be combined
    kProfileReasonCamera         = 10, // unknown camera slot or bad mount
    kProfileReasonFootprint      = 11, // footprint side out of range, or zero width or length
    kProfileReasonBuild          = 12, // Pi failed to build the localization; see its log
    kProfileReasonNotAccepted    = 13, // this Pi configuration takes no Brain profile
    kProfileReasonCalibration    = 14, // calibration settings out of range
};

// Pi side IMU bias calibration, per state block.
enum CalibrationState : uint8_t {
    kCalibrationNone         = 0, // nothing to calibrate on the Pi for this profile
    kCalibrationRunning      = 1, // collecting a stationary window
    kCalibrationDone         = 2,
    kCalibrationWaitingStill = 3, // movement seen; the window restarts when still
    kCalibrationWaitingData  = 4, // IMU or wheel samples missing, stale or discontinuous
    kCalibrationFailed       = 5, // no qualified window within the bound; recalibrate retries
};

enum ControlAction : uint8_t {
    kControlRecalibrate        = 1, // restart Pi IMU bias calibration; pose holds
    kControlReinitialize       = 2, // new odometry epoch, placement required again
    kControlReinitImu          = 3, // reinitialize the Pico IMU, then recalibrate; pose holds
    kControlRestartAcquisition = 4, // Pico zeroes its counters under a new acq_epoch; pose holds
};

// CONTROL reply detail, with Failed, and with Pending while waiting on the Pico.
enum ControlDetail : uint8_t {
    kControlDetailNone        = 0,
    kControlDetailPicoLink    = 1, // no Pico frames, or its firmware takes no commands
    kControlDetailImuAbsent   = 2, // Pico IMU did not initialize within its attempts
    kControlDetailImuUnused   = 3, // the profile does not use the Pico IMU
    kControlDetailPicoRefused = 4, // the Pico rejected the command
    kControlDetailTimedOut    = 5, // no completion within the bound
    kControlDetailCalibration = 6, // no qualified stationary window within the bound
};

// READ_WHEELS reading flags.
enum WheelReadingFlag : uint8_t {
    kWheelFresh = 1u << 0, // received within the Pi freshness window
    kWheelValid = 1u << 1, // the encoder sensor has a value
};

enum DocKind : uint8_t {
    kDocFieldMap      = 1, // doc_id is the map_id
    kDocFieldEstimate = 2, // doc_id is the estimate_id
};

enum PathMode : uint8_t {
    kPathNone     = 0, // clears the reported path
    kPathDirect   = 1,
    kPathAvoiding = 2,
};

enum EstimateSourceCode : uint8_t {
    kEstimateSourceNone     = 0,
    kEstimateSourceNominal  = 1,
    kEstimateSourceObserved = 2,
};

// Chunk and list capacities, set by kMaxFrameLen.
constexpr uint8_t kProfileChunkMax     = 106; // PROFILE_WRITE data bytes
constexpr uint8_t kDocChunkMax         = 96;  // READ_DOC reply data bytes
constexpr uint8_t kPathReportMaxPoints = 13;
constexpr uint8_t kWheelReadingsMax    = 4;   // READ_WHEELS records

// One profile wheel, raw. travel_um applies counts per revolution, gearing,
// polarity and radius, never the travel scale, and stays continuous across
// source restarts; discontinuity changes whenever motion may have been lost.
struct WheelReading {
    uint8_t  port          = 0;
    uint8_t  flags         = 0;
    uint16_t discontinuity = 0; // low 16 bits of the encoder discontinuity epoch
    int32_t  counts        = 0; // as last received from the Pico
    int32_t  travel_um     = 0;
    uint16_t age_ms        = 0; // Pi cycle time minus receipt, clamped
};

struct PathPoint {
    int32_t x_mm = 0;
    int32_t y_mm = 0;
};

// Brain -> Pi. Body fields are used only by their op.
struct BrainRequest {
    uint8_t  version    = kBrainLinkVersion;
    uint8_t  op         = 0;
    uint32_t session    = 0; // 0 in HELLO
    uint16_t request_id = 0; // per brain boot, 1..65535, never 0

    uint32_t nonce = 0; // HELLO

    int32_t x_mm         = 0; // SET_POSE, field frame
    int32_t y_mm         = 0;
    int32_t heading_cdeg = 0;

    uint8_t  imu_flags         = 0; // GET_STATE, 0 when no bench IMU sample
    uint32_t imu_stamp_ms      = 0; // Brain acquisition clock, not the Pico clock
    int32_t  imu_rotation_mdeg = 0; // continuous rotation, CCW positive

    uint32_t profile_id = 0; // PROFILE_WRITE, PROFILE_APPLY: crc32 of the document
    uint16_t total_len  = 0;
    uint16_t offset     = 0; // PROFILE_WRITE
    uint8_t  data_len   = 0;
    uint8_t  data[kProfileChunkMax] = {};

    uint8_t  doc_kind   = 0; // READ_DOC
    uint32_t doc_id     = 0; // 0 = current
    uint16_t doc_offset = 0;
    uint8_t  max_len    = 0;

    uint8_t action     = 0; // CONTROL
    uint8_t action_arg = 0;

    uint32_t  command_id  = 0; // PATH_REPORT
    uint8_t   path_mode   = kPathNone;
    uint8_t   point_count = 0;
    PathPoint points[kPathReportMaxPoints];
};

// GET_STATE Ok body. The robot pose and the field estimate share the anchor
// named by odometry_epoch and anchor_revision.
struct BrainState {
    uint8_t  robot_flags     = 0;
    int32_t  x_mm            = 0; // robot origin, field frame
    int32_t  y_mm            = 0;
    int32_t  heading_cdeg    = 0; // CCW from +x, (-18000, 18000]
    uint16_t robot_age_ms    = 0; // Pi cycle time minus measurement time, clamped
    uint32_t odometry_epoch  = 0; // low 32 bits
    uint32_t anchor_revision = 0; // low 32 bits
    uint8_t  health          = 0;
    uint8_t  profile_state   = kProfileNone;
    uint8_t  profile_reason  = kProfileReasonNone;
    uint8_t  profile_detail  = 0;
    uint32_t profile_id      = 0; // applied, applying or rejected profile, 0 = none
    uint32_t map_id          = 0; // field map document, 0 = no field
    uint32_t estimate_id     = 0; // newest field estimate document, 0 = none yet
    uint8_t  calibration     = kCalibrationNone;
};

// Pi -> brain. Body fields are used only by their (op, result).
struct BrainReply {
    uint8_t  version     = kBrainLinkVersion;
    uint8_t  op          = 0; // echo
    uint32_t session     = 0; // echo, or the opened session for HELLO Ok
    uint16_t request_id  = 0; // echo
    uint8_t  result      = kResultOk;
    uint32_t pi_instance = 0; // random nonzero id per Pi process start and reset

    uint32_t nonce = 0; // HELLO, every result

    uint32_t odometry_epoch  = 0; // SET_POSE Ok and Pending, current values
    uint32_t anchor_revision = 0;

    uint32_t profile_id     = 0; // PROFILE_WRITE Ok, PROFILE_APPLY Ok/Pending/Rejected
    uint16_t received       = 0; // PROFILE_WRITE Ok: contiguous bytes held
    uint8_t  profile_state  = kProfileNone; // PROFILE_APPLY
    uint8_t  profile_reason = kProfileReasonNone;
    uint8_t  profile_detail = 0;

    uint8_t  doc_kind      = 0; // READ_DOC Ok
    uint32_t doc_id        = 0;
    uint16_t doc_total_len = 0;
    uint32_t doc_crc32     = 0; // over the whole document
    uint16_t doc_offset    = 0;
    uint8_t  data_len      = 0;
    uint8_t  data[kDocChunkMax] = {};

    uint8_t action         = 0; // CONTROL Ok, Pending, Failed: echo
    uint8_t calibration    = kCalibrationNone;
    uint8_t control_detail = kControlDetailNone;

    uint8_t      wheel_count = 0; // READ_WHEELS Ok
    WheelReading wheels[kWheelReadingsMax];

    BrainState state; // GET_STATE Ok
};

// ---------------------------------------------------------------------------
// Pico link control (Pi -> Pico command, Pico -> Pi status)
//
//   sync0 sync1 | type | len u8 | payload[len] | crc u16
//
// Same envelope and CRC as the brain link. The Pi sends commands; the Pico
// answers only through status frames, sent periodically and after every
// command state change, between sensor frames.
//
// Command payload: version u8 | op u8 | request_id u16 | target_boot_id u16 | body
// Status payload:  version u8 | boot_id u16 | acq_epoch u8 | imu_epoch u8
//                  | uptime_ms u32 | imu_state u8 | imu_reason u8
//                  | imu_attempts u16 | flags u8 | last_request_id u16
//                  | last_op u8 | last_status u8 | last_detail u8 | firmware u8
//
// A command runs only when target_boot_id is the current boot_id. A request
// id seen with the same op and body is never run again; its recorded status
// is reported. last_* reports the newest command.
// ---------------------------------------------------------------------------

constexpr uint8_t kPicoLinkVersion = 1;

enum PicoOp : uint8_t {
    kPicoOpConfigure          = 1, // imu_enabled u8
    kPicoOpReinitImu          = 2, // imu_port u8
    kPicoOpRestartAcquisition = 3, // no body; zeroes counters, acq_epoch + 1
};

enum PicoCommandStatus : uint8_t {
    kPicoCommandNone      = 0,
    kPicoCommandRunning   = 1, // accepted, not finished (IMU reinitializing)
    kPicoCommandCompleted = 2,
    kPicoCommandFailed    = 3, // see PicoCommandDetail
};

enum PicoCommandDetail : uint8_t {
    kPicoDetailNone        = 0,
    kPicoDetailWrongTarget = 1, // target_boot_id is not this boot
    kPicoDetailUnknownOp   = 2,
    kPicoDetailBadBody     = 3,
    kPicoDetailImuAbsent   = 4, // did not initialize within its attempts
    kPicoDetailImuDisabled = 5, // reinit asked while the IMU is disabled
    kPicoDetailNoSuchPort  = 6,
};

enum PicoImuState : uint8_t {
    kPicoImuDisabled     = 0,
    kPicoImuInitializing = 1,
    kPicoImuAligning     = 2, // BNO08X startup gravity alignment, hold still and level
    kPicoImuReady        = 3,
    kPicoImuRetrying     = 4, // an attempt failed; next attempt after a backoff
    kPicoImuFailed       = 5, // quick attempts used up; slow retries continue while enabled
};

enum PicoImuReason : uint8_t {
    kPicoImuReasonNone       = 0,
    kPicoImuReasonNoResponse = 1, // no device answered
    kPicoImuReasonBoot       = 2, // boot timeout
    kPicoImuReasonFeatures   = 3, // reports not acknowledged
    kPicoImuReasonStream     = 4, // reports stopped
};

enum PicoStatusFlag : uint8_t {
    kPicoImuEnabled = 1u << 0,
};

enum PicoFirmware : uint8_t {
    kPicoFirmwareUnknown = 0,
    kPicoFirmwareBno08x  = 1,
    kPicoFirmwareAsm330  = 2,
};

struct PicoCommand {
    uint8_t  version        = kPicoLinkVersion;
    uint8_t  op             = 0;
    uint16_t request_id     = 0; // never 0
    uint16_t target_boot_id = 0;
    uint8_t  imu_enabled    = 0; // CONFIGURE
    uint8_t  imu_port       = 0; // REINIT_IMU
};

struct PicoStatus {
    uint8_t  version         = kPicoLinkVersion;
    uint16_t boot_id         = 0;
    uint8_t  acq_epoch       = 0;
    uint8_t  imu_epoch       = 0;
    uint32_t uptime_ms       = 0;
    uint8_t  imu_state       = kPicoImuDisabled;
    uint8_t  imu_reason      = kPicoImuReasonNone;
    uint16_t imu_attempts    = 0; // in the current episode
    uint8_t  flags           = 0;
    uint16_t last_request_id = 0; // 0 = no command yet
    uint8_t  last_op         = 0;
    uint8_t  last_status     = kPicoCommandNone;
    uint8_t  last_detail     = kPicoDetailNone;
    uint8_t  firmware        = kPicoFirmwareUnknown;
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
