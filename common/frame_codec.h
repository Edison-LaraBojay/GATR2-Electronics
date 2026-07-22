// frame_codec.h
// Maps between frame structs and wire bytes. No hardware dependencies.
// The structs are not the wire layout, only this codec crosses that boundary.

#pragma once
#include <stdint.h>

#include "frames.h"

namespace gatr2
{

// Fixed header sizes, sync through the last field before the variable part.
constexpr uint16_t kSensorHeaderLen = 10;
constexpr uint16_t kPoseHeaderLen   = 22;
constexpr uint16_t kLandmarkLen     = 8;

// True if every set bit in the mask has a width entry.
bool sensorMaskValid(uint16_t mask);

// Payload bytes implied by a mask. Caller must check sensorMaskValid first.
uint16_t sensorPayloadLen(uint16_t mask);

// Total frame length, sync through checksum.
uint16_t sensorFrameLen(uint16_t mask);
uint16_t poseFrameLen(uint8_t n_landmarks);

// Encode into buf. Returns bytes written, or 0 if the input is not encodable
// or would not fit in cap.
uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap);
uint16_t encodePoseFrame(const PoseFrame& in, uint8_t* buf, uint16_t cap);

// Decode one whole frame. False on bad sync, wrong length, unknown type,
// unknown mask bit, too many landmarks, or checksum mismatch.
bool decodeSensorFrame(const uint8_t* buf, uint16_t len, SensorSample& out);
bool decodePoseFrame(const uint8_t* buf, uint16_t len, PoseFrame& out);

// Byte stream reader. Scans for the sync pair, buffers one frame, validates
// the checksum, and re-locks on a later sync pair after garbage.
class FrameReader
{
public:
    void reset();

    // Feed one byte. True when a complete valid frame is buffered.
    bool push(uint8_t b);

    const uint8_t* frame() const { return buf_; }
    uint16_t       frameLen() const { return len_; }

    // Valid only while a frame is buffered.
    uint8_t frameType() const { return len_ >= 3 ? buf_[2] : 0; }

private:
    // Total length once the header is readable, 0 while still unknown,
    // and greater than kMaxFrameLen for anything unparseable.
    uint16_t expectedLen() const;

    uint8_t  buf_[kMaxFrameLen] = {};
    uint16_t len_               = 0;
    bool     ready_             = false;
};

} // namespace gatr2
