// frames.h
// Wire format shared by the Pico, Pi, and brain.
// No hardware dependencies. Must compile on all three targets.

#pragma once
#include <stdint.h>

namespace translagatr
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
    kFramePicoDiag     = 0x14, // Pico  -> Pi, only after kPicoOpDiagnostics asked for it
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
    kOpTelemetry    = 12, // BrainTelemetry body; best effort, display and recording only
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
    kControlReinitImu          = 3, // reinit the Pico IMU, recalibrate; pose invalid if used
    kControlRestartAcquisition = 4, // Pico zeroes its counters, new acq_epoch; pose invalid
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

// TELEMETRY flags: which groups of a BrainTelemetry are present.
enum TelemetryFlagBit : uint8_t {
    kTelemetryAttitude = 1u << 0, // roll_cdeg, pitch_cdeg: robot frame, Brain VEX IMU
    kTelemetryMotion   = 1u << 1, // the movement group
    kTelemetryWheels   = 1u << 2, // wheel_count and wheel_rpm: motor velocity targets
    kTelemetryTarget   = 1u << 3, // target_* holds a resolved destination (else ignore it)
};

// Chunk and list capacities, set by kMaxFrameLen.
constexpr uint8_t kProfileChunkMax     = 106; // PROFILE_WRITE data bytes
constexpr uint8_t kDocChunkMax         = 96;  // READ_DOC reply data bytes
constexpr uint8_t kPathReportMaxPoints = 13;
constexpr uint8_t kWheelReadingsMax    = 4;   // READ_WHEELS records
constexpr uint8_t kTelemetryWheelsMax  = 6;   // TELEMETRY wheel groups

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

// TELEMETRY body (Brain -> Pi). Never changes Pi state: the Pi records it
// and shows it; localization never reads it. Fixed layout, 54 bytes:
//   flags u8 | stamp_ms u32
//   | roll_cdeg i16 | pitch_cdeg i16
//   | command_id u32 | motion_state u8 | motion_reason u8 | plan_mode u8
//   | segment u8 | segment_count u8
//   | target_x_mm i32 | target_y_mm i32 | target_heading_cdeg i16
//   | cmd_vx_mm_s i16 | cmd_vy_mm_s i16 | cmd_omega_cdeg_s i16
//   | cross_track_mm i16 | distance_error_mm i16 | heading_error_cdeg i16
//   | drive_fault u8 | wheel_count u8 | wheel_rpm_x10 i16 x kTelemetryWheelsMax
// Groups whose flag bit is clear are zero on the wire and ignored.
// Units: attitude and headings in centidegrees, CCW positive about +z; roll
// is about robot +x (forward), positive left side up; pitch is about robot
// +y (left), positive nose down. Speeds are body frame (+x forward, +y left).
// motion_state, motion_reason and plan_mode are actugatr MotionState,
// MotionReason and investigatr PlanMode values; drive_fault is actugatr
// DriveFault. target_* is the field-frame destination of the command.
// Distances saturate at +-32767 mm.
struct BrainTelemetry {
    uint8_t  flags               = 0;
    uint32_t stamp_ms            = 0; // Brain clock when the values were taken
    int16_t  roll_cdeg           = 0;
    int16_t  pitch_cdeg          = 0;
    uint32_t command_id          = 0;
    uint8_t  motion_state        = 0;
    uint8_t  motion_reason       = 0;
    uint8_t  plan_mode           = 0;
    uint8_t  segment             = 0;
    uint8_t  segment_count       = 0;
    int32_t  target_x_mm         = 0;
    int32_t  target_y_mm         = 0;
    int16_t  target_heading_cdeg = 0;
    int16_t  cmd_vx_mm_s         = 0;
    int16_t  cmd_vy_mm_s         = 0;
    int16_t  cmd_omega_cdeg_s    = 0;
    int16_t  cross_track_mm      = 0;
    int16_t  distance_error_mm   = 0;
    int16_t  heading_error_cdeg  = 0;
    uint8_t  drive_fault         = 0;
    uint8_t  wheel_count         = 0;
    int16_t  wheel_rpm_x10[kTelemetryWheelsMax] = {};
};
constexpr uint8_t kTelemetryBodyLen = 54;

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

    BrainTelemetry telemetry; // TELEMETRY
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
    kPicoOpDiagnostics        = 4, // diag_hz u8 (0 off, 1..5); older firmware: UnknownOp
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
    uint8_t  diag_hz        = 0; // DIAGNOSTICS
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
// Pico diagnostic frame (Pico -> Pi, type 0x14), optional
//
//   sync0 sync1 | type | len u8 | payload[26] | crc u16     (32 bytes)
//
// Payload: version u8 | boot_id u16 | seq u8 | firmware u8
//          | pins u16 | pins_known u16
//          | imu_rx u16 | imu_bad u16 | imu_resets u8 | imu_error i8
//          | reports_ok u16 | reports_rejected u16 | report_age_ms u16
//          | link_rx_bad u16 | ticks_skipped u16 | flags u8
//
// Sent only while a DIAGNOSTICS command set a nonzero rate, and only when the
// UART TX FIFO is empty and the next sensor tick is not due: sensor frames
// always come first. Counters are free-running and wrap.
//
// pins holds digital pin states read back from the pad (HIGH = 1). They are
// logic levels sampled when the frame was built, not voltages, and a
// sample can miss fast transitions. A pin whose bit is clear in pins_known
// was not sampled or does not exist on this build. Reading a pin never
// changes its mode or function.
// ---------------------------------------------------------------------------

enum PicoDiagPin : uint16_t {
    kPicoPinImuInt  = 1u << 0,  // IMU INT (input)
    kPicoPinImuRst  = 1u << 1,  // IMU RST (driven by the Pico)
    kPicoPinImuWake = 1u << 2,  // IMU WAKE/PS0 (driven by the Pico)
    kPicoPinImuCs   = 1u << 3,  // IMU chip select (driven by the Pico)
    kPicoPinEnc0A   = 1u << 4,
    kPicoPinEnc0B   = 1u << 5,
    kPicoPinEnc1A   = 1u << 6,
    kPicoPinEnc1B   = 1u << 7,
    kPicoPinEnc2A   = 1u << 8,
    kPicoPinEnc2B   = 1u << 9,
    kPicoPinPiRx    = 1u << 10, // UART RX from the Pi; idles HIGH
};

enum PicoDiagFlag : uint8_t {
    kPicoDiagImuPresent = 1u << 0, // this build drives an IMU
};

constexpr uint8_t kPicoDiagLen = 26;

struct PicoDiag {
    uint8_t  version          = kPicoLinkVersion;
    uint16_t boot_id          = 0;
    uint8_t  seq              = 0;
    uint8_t  firmware         = kPicoFirmwareUnknown;
    uint16_t pins             = 0;
    uint16_t pins_known       = 0;
    uint16_t imu_rx           = 0; // IMU packets or reads completed
    uint16_t imu_bad          = 0; // bad headers or failed reads
    uint8_t  imu_resets       = 0; // hub resets or chip restarts seen
    int8_t   imu_error        = 0; // last driver error code, 0 none
    uint16_t reports_ok       = 0; // sensor reports accepted
    uint16_t reports_rejected = 0;
    uint16_t report_age_ms    = 0; // since the last accepted report, 0xFFFF none
    uint16_t link_rx_bad      = 0; // Pi -> Pico candidate frames rejected (length or crc)
    uint16_t ticks_skipped    = 0; // sensor ticks skipped for a busy TX FIFO
    uint8_t  flags            = 0;
};

// ---------------------------------------------------------------------------

inline uint8_t checksum(const uint8_t* data, uint16_t len) {
    uint8_t x = 0;
    for (uint16_t i = 0; i < len; ++i) {
        x ^= data[i];
    }
    return x;
}

} // namespace translagatr
