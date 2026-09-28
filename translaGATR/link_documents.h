// link_documents.h
// Documents moved over the brain link in chunks: the Brain robot profile
// (PROFILE_WRITE) and the Pi field map and field estimate (READ_DOC).
// Fixed layout records with explicit counts, little endian, no heap.
// Brain and Pi only; the Pico does not build this.

#pragma once
#include <stdint.h>

#include "frames.h"

namespace translagatr
{

// CRC-32/ISO-HDLC (zlib): reflected poly 0xEDB88320, init and xorout 0xFFFFFFFF.
uint32_t crc32(const uint8_t* data, uint32_t len);

enum class DocError : uint8_t {
    kNone,
    kLength,      // byte count does not match the header
    kFormat,      // unknown format or record length
    kCount,       // record count over the cap, or not the map's count
    kOrder,       // object ids not strictly increasing, or not the map's ids
    kKind,        // unknown object kind or estimate source
    kFlags,       // unknown flag bits, or flags that contradict the kind
    kBox,         // obstacle without a box, or a box without the obstacle flag
    kBounds,      // empty boundary
    kRange,       // angle out of (-18000, 18000], or reserved bytes set
    kMapMismatch, // estimate names another map
};

// ---------------------------------------------------------------------------
// Robot profile, Brain -> Pi. profile_id is the crc32 of the document bytes.
//
//   header 32  format u8 | topology u8 | wheel_count u8 | camera_count u8
//              | imu_source u8 | imu_port u8 | vex_smart_port u8 | imu_flags u8
//              | footprint_front_um i32 | footprint_back_um i32
//              | footprint_left_um i32 | footprint_right_um i32
//              | calibration_window_ms u16 | still_rate_cdps u16
//              | still_travel_um u16 | reserved u16
//   wheel 32   encoder_port u8 | flags u8 | reserved u16 | counts_per_rev u32
//              | radius_um u32 | x_um i32 | y_um i32 | angle_mdeg i32
//              | gear_micro u32 | travel_scale_ppm u32
//   camera 28  slot u8 | reserved 3 | x_um i32 | y_um i32 | z_um i32
//              | roll_mdeg i32 | pitch_mdeg i32 | yaw_mdeg i32
//
// Robot frame: +x forward, +y left, origin the reported robot point.
// Footprint is the distance from the origin to each side of the enclosing
// rectangle.
//
// Each wheel keeps three kinds of value apart, each applied once on the Pi:
//   encoder    counts_per_rev (encoder shaft), polarity (kWheelReversed),
//              gear_micro = encoder revolutions per wheel revolution x 1e6
//   geometry   radius, mounting position, measuring direction (angle, CCW
//              from +x)
//   empirical  travel_scale_ppm, a measured distance correction x 1e6,
//              1000000 when uncalibrated
// Calibration settings 0 mean the Pi defaults: the stationary window for IMU
// bias, and the gyro rate and wheel travel limits that still count as still.
// ---------------------------------------------------------------------------
constexpr uint8_t  kProfileFormat     = 1;
constexpr uint8_t  kProfileMaxWheels  = 3;
constexpr uint8_t  kProfileMaxCameras = 4;
constexpr uint16_t kProfileHeaderLen  = 32;
constexpr uint16_t kProfileWheelLen   = 32;
constexpr uint16_t kProfileCameraLen  = 28;
constexpr uint16_t kProfileMaxLen     = kProfileHeaderLen + kProfileMaxWheels * kProfileWheelLen +
                                    kProfileMaxCameras * kProfileCameraLen;

constexpr uint32_t kUnitMicro = 1000000; // gear_micro and travel_scale_ppm of 1.0

enum LocalizationTopology : uint8_t {
    kTopologyTwoWheelImu        = 1, // two independent wheel directions, IMU heading
    kTopologyThreeWheel         = 2, // three wheels, optional independent IMU fused
    kTopologyTwoForwardWheelImu = 3, // two forward wheels, IMU heading, no sideways travel
};

enum ImuSourceCode : uint8_t {
    kImuSourceNone     = 0,
    kImuSourcePico     = 1, // Pico IMU port imu_port
    kImuSourceBrainVex = 2, // Brain VEX IMU on vex_smart_port, bench arrival timing
};

enum ProfileWheelFlag : uint8_t {
    kWheelReversed = 1u << 0, // encoder polarity: positive counts mean travel against angle
};

enum ProfileImuFlag : uint8_t {
    kImuInvert = 1u << 0, // Pico IMU yaw sign flipped
};

struct ProfileWheel {
    uint8_t  encoder_port     = 0;
    uint8_t  flags            = 0;
    uint32_t counts_per_rev   = 0;
    uint32_t radius_um        = 0;
    int32_t  x_um             = 0;
    int32_t  y_um             = 0;
    int32_t  angle_mdeg       = 0;
    uint32_t gear_micro       = kUnitMicro;
    uint32_t travel_scale_ppm = kUnitMicro;
};

struct ProfileCamera {
    uint8_t slot       = 0;
    int32_t x_um       = 0;
    int32_t y_um       = 0;
    int32_t z_um       = 0;
    int32_t roll_mdeg  = 0;
    int32_t pitch_mdeg = 0;
    int32_t yaw_mdeg   = 0;
};

struct RobotProfileDoc {
    uint8_t format         = kProfileFormat;
    uint8_t topology       = 0;
    uint8_t wheel_count    = 0;
    uint8_t camera_count   = 0;
    uint8_t imu_source     = kImuSourceNone;
    uint8_t imu_port       = 0;
    uint8_t vex_smart_port = 0; // 1..21 with kImuSourceBrainVex, else 0
    uint8_t imu_flags      = 0;

    int32_t footprint_front_um = 0;
    int32_t footprint_back_um  = 0;
    int32_t footprint_left_um  = 0;
    int32_t footprint_right_um = 0;

    uint16_t calibration_window_ms = 0; // 0 = Pi default
    uint16_t still_rate_cdps       = 0; // centidegrees per second, 0 = Pi default
    uint16_t still_travel_um       = 0; // per wheel over the window, 0 = Pi default

    ProfileWheel  wheels[kProfileMaxWheels];
    ProfileCamera cameras[kProfileMaxCameras];
};

// Document length for the counts, 0 when a count is over its cap.
uint16_t robotProfileLen(const RobotProfileDoc& in);

// Bytes written, 0 when a count is over its cap or it does not fit.
uint16_t encodeRobotProfile(const RobotProfileDoc& in, uint8_t* buf, uint16_t cap);

// Structural decode: format, counts within caps, exact length, reserved zero.
bool decodeRobotProfile(const uint8_t* buf, uint16_t len, RobotProfileDoc& out);

// The rules every Pi applies before its own capability checks (wired ports,
// IMU availability, camera slots): known topology and IMU source, wheel
// count, distinct ports, radius, counts and offsets in range, observable
// wheel directions, footprint, distinct camera slots, no unknown flag bits,
// gearing in 0.1..10, travel scale in 0.9..1.1, calibration settings.
// On false, reason is a ProfileReason and detail the wheel or camera index.
bool validateRobotProfile(const RobotProfileDoc& in, uint8_t& reason, uint8_t& detail);

// ---------------------------------------------------------------------------
// Field map, Pi -> Brain. map_id is the crc32 of the document bytes.
//
//   header 24  format u8 | record_len u8 | revision u16 | object_count u16
//              | reserved u16 | min_x_mm i32 | min_y_mm i32 | max_x_mm i32
//              | max_y_mm i32
//   object 28  object_id u16 | kind u8 | flags u8 | x_mm i32 | y_mm i32
//              | heading_cdeg i32 | box_x_mm i16 | box_y_mm i16
//              | box_heading_cdeg i16 | box_length_mm u16 | box_width_mm u16
//              | reserved u16
//
// Poses are nominal, field frame. The box is in the object frame: center
// (box_x, box_y), rotated by box_heading, length along its own x. Records
// are sorted by object_id, ids 1..65535. The boundary is the region the
// robot footprint must stay inside.
// ---------------------------------------------------------------------------
constexpr uint8_t  kFieldMapFormat    = 1;
constexpr uint16_t kFieldMaxObjects   = 128;
constexpr uint16_t kFieldMapHeaderLen = 24;
constexpr uint16_t kFieldMapRecordLen = 28;
constexpr uint16_t kFieldMapMaxLen    = kFieldMapHeaderLen + kFieldMaxObjects * kFieldMapRecordLen;

enum FieldObjectKind : uint8_t {
    kObjectLandmark = 1, // physical landmark, may be estimated
    kObjectFixed    = 2, // fixed field element, never estimated
};

enum FieldObjectFlag : uint8_t {
    kObjectObstacle  = 1u << 0, // box is a planning obstacle
    kObjectEstimated = 1u << 1, // world estimation may correct its pose
    kObjectReference = 1u << 2, // usable as a movement reference
};

struct FieldMapHeader {
    uint8_t  format       = kFieldMapFormat;
    uint16_t revision     = 0; // declared in the field definition
    uint16_t object_count = 0;
    int32_t  min_x_mm     = 0;
    int32_t  min_y_mm     = 0;
    int32_t  max_x_mm     = 0;
    int32_t  max_y_mm     = 0;
};

struct FieldObjectRecord {
    uint16_t object_id        = 0;
    uint8_t  kind             = 0;
    uint8_t  flags            = 0;
    int32_t  x_mm             = 0;
    int32_t  y_mm             = 0;
    int32_t  heading_cdeg     = 0;
    int16_t  box_x_mm         = 0;
    int16_t  box_y_mm         = 0;
    int16_t  box_heading_cdeg = 0;
    uint16_t box_length_mm    = 0;
    uint16_t box_width_mm     = 0;
};

// Document length for a count, 0 over kFieldMaxObjects.
uint16_t fieldMapLen(uint16_t object_count);

// Writers for a document of fieldMapLen(header.object_count) bytes in a
// buffer of cap bytes. False when the index or count is out of range or the
// document does not fit in cap.
bool encodeFieldMapHeader(const FieldMapHeader& in, uint8_t* doc, uint16_t cap);
bool encodeFieldObjectRecord(const FieldObjectRecord& in, uint16_t index, uint8_t* doc,
                             uint16_t cap);

// Readers. The header check covers format, record length, count and exact
// document length. Run validateFieldMap before trusting records.
bool decodeFieldMapHeader(const uint8_t* doc, uint16_t len, FieldMapHeader& out);
bool decodeFieldObjectRecord(const uint8_t* doc, uint16_t len, uint16_t index,
                             FieldObjectRecord& out);

DocError validateFieldMap(const uint8_t* doc, uint16_t len);

// ---------------------------------------------------------------------------
// Field estimate, Pi -> Brain. A complete snapshot: one record per map
// object, in map order. estimate_id counts snapshots per Pi instance.
//
//   header 24  format u8 | record_len u8 | object_count u16 | map_id u32
//              | estimate_id u32 | odometry_epoch u32 | anchor_revision u32
//              | reserved u32
//   object 20  object_id u16 | source u8 | flags u8 | x_mm i32 | y_mm i32
//              | heading_cdeg i32 | age_ms u16 | reserved u16
//
// Poses are field frame under the anchor named by odometry_epoch and
// anchor_revision. age_ms is the observation age when the snapshot was
// taken, clamped, and 0 for nominal records.
// ---------------------------------------------------------------------------
constexpr uint8_t  kFieldEstimateFormat    = 1;
constexpr uint16_t kFieldEstimateHeaderLen = 24;
constexpr uint16_t kFieldEstimateRecordLen = 20;
constexpr uint16_t kFieldEstimateMaxLen =
    kFieldEstimateHeaderLen + kFieldMaxObjects * kFieldEstimateRecordLen;

enum EstimateFlag : uint8_t {
    kEstimateValid = 1u << 0,
};

struct FieldEstimateHeader {
    uint8_t  format          = kFieldEstimateFormat;
    uint16_t object_count    = 0;
    uint32_t map_id          = 0;
    uint32_t estimate_id     = 0;
    uint32_t odometry_epoch  = 0;
    uint32_t anchor_revision = 0;
};

struct FieldEstimateRecord {
    uint16_t object_id    = 0;
    uint8_t  source       = kEstimateSourceNone;
    uint8_t  flags        = 0;
    int32_t  x_mm         = 0;
    int32_t  y_mm         = 0;
    int32_t  heading_cdeg = 0;
    uint16_t age_ms       = 0;
};

uint16_t fieldEstimateLen(uint16_t object_count);

bool encodeFieldEstimateHeader(const FieldEstimateHeader& in, uint8_t* doc, uint16_t cap);
bool encodeFieldEstimateRecord(const FieldEstimateRecord& in, uint16_t index, uint8_t* doc,
                               uint16_t cap);

bool decodeFieldEstimateHeader(const uint8_t* doc, uint16_t len, FieldEstimateHeader& out);
bool decodeFieldEstimateRecord(const uint8_t* doc, uint16_t len, uint16_t index,
                               FieldEstimateRecord& out);

// Checks the estimate against a valid map document and its map_id: same
// count and ids in the same order, known sources, objects without
// kObjectEstimated nominal, source none exactly when not valid, age 0 unless
// observed.
DocError validateFieldEstimate(const uint8_t* doc, uint16_t len, const uint8_t* map_doc,
                               uint16_t map_len, uint32_t map_id);

} // namespace translagatr
