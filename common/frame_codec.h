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

// Brain link: sync0 sync1 type len, then crc u16 after the payload.
constexpr uint16_t kLinkEnvelopeLen = 6;

// Brain link payload sizes.
constexpr uint8_t kBrainRequestHeaderLen = 8;
constexpr uint8_t kBrainRequestMaxLen    = 32;
constexpr uint8_t kBrainReplyHeaderLen   = 13;
constexpr uint8_t kBrainReplyMaxLen      = 64;
constexpr uint8_t kBrainStateLen         = 40;

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0.
uint16_t crc16(const uint8_t* data, uint16_t len);

// True if every set bit in the mask has a width entry.
bool sensorMaskValid(uint16_t mask);

// Payload bytes implied by a mask. Caller must check sensorMaskValid first.
uint16_t sensorPayloadLen(uint16_t mask);

// Total frame length, sync through checksum.
uint16_t sensorFrameLen(uint16_t mask);

// v3 payload length for an op, or for an (op, result) reply.
// 0 when v3 does not define it (unknown op, or unknown result except HELLO).
uint8_t brainRequestLen(uint8_t op);
uint8_t brainReplyLen(uint8_t op, uint8_t result);

// Encode into buf. Returns bytes written, or 0 if the input is not encodable
// or would not fit in cap.
// Brain link encoders write version as given and the v3 body for the op
// (header only when brainRequestLen/brainReplyLen is 0).
uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap);
uint16_t encodeBrainRequest(const BrainRequest& in, uint8_t* buf, uint16_t cap);
uint16_t encodeBrainReply(const BrainReply& in, uint8_t* buf, uint16_t cap);

// Decode one whole frame. False on bad sync, wrong length, unknown type,
// unknown mask bit, or checksum mismatch.
bool decodeSensorFrame(const uint8_t* buf, uint16_t len, SensorSample& out);

// Brain link decode. False on a bad envelope, len out of range for the type,
// or CRC mismatch. Version other than kBrainLinkVersion: header only, true.
// v3: a known op (reply: op and result) with the wrong length is false; an
// unknown one is true with the body ignored.
bool decodeBrainRequest(const uint8_t* buf, uint16_t len, BrainRequest& out);
bool decodeBrainReply(const uint8_t* buf, uint16_t len, BrainReply& out);

// Byte stream reader for sensor and brain link frames. Scans for the sync
// pair, buffers one frame and validates its length and checksum or CRC. On a
// rejected candidate it rescans the buffered bytes after that sync byte.
class FrameReader
{
public:
    void reset();

    // Feed one byte. True when a complete valid frame is buffered.
    // Bytes buffered behind a frame are kept for the following pushes.
    bool push(uint8_t b);

    const uint8_t* frame() const { return buf_; }
    uint16_t       frameLen() const { return frame_len_; }

    // Valid only while a frame is buffered.
    uint8_t frameType() const { return frame_len_ >= 3 ? buf_[2] : 0; }

private:
    // Total length once the header is readable, 0 while still unknown,
    // and greater than kMaxFrameLen for anything unparseable.
    uint16_t expectedLen() const;
    bool     frameValid(uint16_t len) const;
    void     drop(uint16_t n);

    uint8_t  buf_[kMaxFrameLen] = {};
    uint16_t len_               = 0; // bytes buffered
    uint16_t frame_len_         = 0; // complete frame at buf_, 0 when none
};

} // namespace gatr2
