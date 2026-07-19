// frames.h
// Wire format shared by the Pico, Pi, and brain.
// No hardware dependencies. Must compile on all three targets.

#pragma once
#include <stdint.h>

namespace gatr2 {

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;

// Bounded so every parser can use a static buffer.
constexpr uint16_t kMaxFrameLen = 128;

enum FrameType : uint8_t {
    kFrameSensor = 0x01,  // Pico -> Pi
    kFramePose   = 0x02,  // Pi   -> brain
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
    uint32_t stamp_ms;   // Pico clock, the reference for all fusion timing
    uint16_t mask;
    int32_t  enc[3];
    int32_t  gyro_z;     // mdeg/s, raw, bias not removed
    int32_t  accel[2];   // mg
};

// ---------------------------------------------------------------------------
// Pose frame (Pi -> brain)
//
//   sync0 sync1 | type | seq u8 | stamp_ms u32
//   | x_mm i32 | y_mm i32 | heading_cdeg i32
//   | status u8 | n_landmarks u8
//   | n x { id u8, dx_mm i16, dy_mm i16, bearing_cdeg i16, quality u8 }
//   | xor u8
//
// Pose is relative to the zero point. Landmark entries are the live relative
// transform, for closing the loop directly on a target.
// ---------------------------------------------------------------------------

enum StatusBit : uint8_t {
    kStatusPoseValid   = 1u << 0,
    kStatusEncHealthy  = 1u << 1,
    kStatusGyroHealthy = 1u << 2,
    kStatusVisionAlive = 1u << 3,
    kStatusBiasCal     = 1u << 4,  // init bias calibration completed cleanly
};

constexpr uint8_t kMaxLandmarks = 8;

struct LandmarkObs {
    uint8_t id;
    int16_t dx_mm;
    int16_t dy_mm;
    int16_t bearing_cdeg;
    uint8_t quality;  // 0-255, derived from range and viewing angle
};

struct PoseFrame {
    uint8_t     seq;
    uint32_t    stamp_ms;
    int32_t     x_mm;
    int32_t     y_mm;
    int32_t     heading_cdeg;
    uint8_t     status;
    uint8_t     n_landmarks;
    LandmarkObs landmarks[kMaxLandmarks];
};

// ---------------------------------------------------------------------------

inline uint8_t checksum(const uint8_t* data, uint16_t len) {
    uint8_t x = 0;
    for (uint16_t i = 0; i < len; ++i) x ^= data[i];
    return x;
}

}  // namespace gatr2