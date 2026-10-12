// frame_codec.h
// Maps between frame structs and wire bytes. No hardware dependencies.
// The structs are not the wire layout, only this codec crosses that boundary.

#pragma once
#include <stdint.h>

#include "frames.h"

namespace translagatr
{

// Fixed header sizes, sync through the last field before the variable part.
constexpr uint16_t kSensorHeaderLen   = 10;
constexpr uint16_t kSensorV2HeaderLen = 14;

// Brain link: sync0 sync1 type len, then crc u16 after the payload.
constexpr uint16_t kLinkEnvelopeLen = 6;

// Brain link payload sizes.
constexpr uint8_t kBrainPayloadMax       = kMaxFrameLen - kLinkEnvelopeLen; // 122
constexpr uint8_t kBrainRequestHeaderLen = 8;
constexpr uint8_t kBrainRequestMaxLen    = kBrainPayloadMax;
constexpr uint8_t kBrainReplyHeaderLen   = 13;
constexpr uint8_t kBrainReplyMaxLen      = kBrainPayloadMax;
constexpr uint8_t kBrainStateLen         = 40;

// v4 bodies, bytes after the header.
//
// Requests
//   HELLO          nonce u32                                         4
//   SET_POSE       x_mm i32, y_mm i32, heading_cdeg i32              12
//   GET_STATE      imu_flags u8, imu_stamp_ms u32, imu_rotation i32  9
//   PROFILE_WRITE  profile_id u32, total_len u16, offset u16, data   8 + 1..106
//   PROFILE_APPLY  profile_id u32, total_len u16                     6
//   READ_DOC       doc_kind u8, doc_id u32, offset u16, max_len u8   8
//   CONTROL        action u8, arg u8                                 2
//   PATH_REPORT    command_id u32, path_mode u8, count u8,           6 + 8 x 0..13
//                  count x (x_mm i32, y_mm i32)
//   READ_WHEELS    none                                              0
//   TELEMETRY      BrainTelemetry, see frames.h                      54
//
// Replies (other results and ops are header only, TELEMETRY always)
//   HELLO          any result: nonce u32                             4
//   SET_POSE       Ok, Pending: odometry_epoch u32, anchor_rev u32   8
//   GET_STATE      Ok: state block                                   40
//   PROFILE_WRITE  Ok: profile_id u32, received u16                  6
//   PROFILE_APPLY  Ok, Pending, ProfileRejected: profile_id u32,     7
//                  profile_state u8, reason u8, detail u8
//   READ_DOC       Ok: doc_kind u8, doc_id u32, total_len u16,       13 + 1..96
//                  crc32 u32, offset u16, data
//   CONTROL        Ok, Pending, Failed: action u8, calibration u8,   3
//                  detail u8
//   READ_WHEELS    Ok: count u8, count x (port u8, flags u8,         1 + 16 x 0..4
//                  discontinuity u16, counts i32, travel_um i32,
//                  age_ms u16, reserved u16)
constexpr uint8_t kProfileWriteHeaderLen = 8;
constexpr uint8_t kReadDocReplyHeaderLen = 13;
constexpr uint8_t kPathReportHeaderLen   = 6;
constexpr uint8_t kWheelReadingLen       = 16;

// Pico link payload sizes. The diagnostic payload is kPicoDiagLen (frames.h).
constexpr uint8_t kPicoCommandHeaderLen = 6;
constexpr uint8_t kPicoCommandMaxLen    = 8;
constexpr uint8_t kPicoStatusLen        = 20;

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0.
uint16_t crc16(const uint8_t* data, uint16_t len);

// True if every set bit in the mask has a width entry.
bool sensorMaskValid(uint16_t mask);

// Payload bytes implied by a mask. Caller must check sensorMaskValid first.
uint16_t sensorPayloadLen(uint16_t mask);

// Total frame length, sync through checksum.
uint16_t sensorFrameLen(uint16_t mask);
uint16_t sensorV2FrameLen(uint16_t mask);

// Pico command payload length for an op, 0 for an unknown op.
uint8_t picoCommandLen(uint8_t op);

// v4 payload length range for a request op, or for an (op, result) reply.
// Fixed bodies have min == max. 0 when v4 does not define it (unknown op, or
// unknown result except HELLO).
uint8_t brainRequestMinLen(uint8_t op);
uint8_t brainRequestMaxLen(uint8_t op);
uint8_t brainReplyMinLen(uint8_t op, uint8_t result);
uint8_t brainReplyMaxLen(uint8_t op, uint8_t result);

// Encode into buf. Returns bytes written, or 0 if the input is not encodable
// (a variable body out of range) or would not fit in cap.
// Brain link encoders write version as given and the v4 body for the op and
// result (header only when v4 defines no body).
// TELEMETRY writes zeros for every group whose flag bit is clear, and
// refuses a wheels group with wheel_count over kTelemetryWheelsMax.
// Sensor frames: v2 when in.identity, else v1.
uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap);
uint16_t encodeBrainRequest(const BrainRequest& in, uint8_t* buf, uint16_t cap);
uint16_t encodeBrainReply(const BrainReply& in, uint8_t* buf, uint16_t cap);

// Decode one whole frame, v1 or v2 (out.identity tells which). False on bad
// sync, wrong length, unknown type, unknown mask bit, or checksum mismatch.
bool decodeSensorFrame(const uint8_t* buf, uint16_t len, SensorSample& out);

// Pico link. Encoders return bytes written or 0. Decoders are false on a bad
// envelope, CRC, length for the op, or version; a command with an unknown op
// decodes its header and returns true so the Pico can report it.
uint16_t encodePicoCommand(const PicoCommand& in, uint8_t* buf, uint16_t cap);
uint16_t encodePicoStatus(const PicoStatus& in, uint8_t* buf, uint16_t cap);
uint16_t encodePicoDiag(const PicoDiag& in, uint8_t* buf, uint16_t cap);
bool     decodePicoCommand(const uint8_t* buf, uint16_t len, PicoCommand& out);
bool     decodePicoStatus(const uint8_t* buf, uint16_t len, PicoStatus& out);
bool     decodePicoDiag(const uint8_t* buf, uint16_t len, PicoDiag& out);

// Brain link decode. False on a bad envelope, len out of range for the type,
// or CRC mismatch. Version other than kBrainLinkVersion: header only, true.
// v4: a known op (reply: op and result) with a length outside its range, or a
// variable body inconsistent with its length, is false; an unknown one is
// true with the body ignored. TELEMETRY: groups whose flag bit is clear
// decode as zero whatever the wire holds; a wheels group with wheel_count
// over kTelemetryWheelsMax is false; unknown flag bits are kept in flags.
bool decodeBrainRequest(const uint8_t* buf, uint16_t len, BrainRequest& out);
bool decodeBrainReply(const uint8_t* buf, uint16_t len, BrainReply& out);

// Bodies alone, for records that keep the raw bytes (the Pi recorder and
// viewer): the TELEMETRY body (kTelemetryBodyLen, same rules as
// decodeBrainRequest) and the Pico diagnostic payload (kPicoDiagLen, from
// the version byte). False on a wrong length or body.
bool decodeTelemetryBody(const uint8_t* body, uint16_t len, BrainTelemetry& out);
bool decodePicoDiagPayload(const uint8_t* payload, uint16_t len, PicoDiag& out);

// Byte stream reader for sensor, brain link and Pico link frames. Scans for the sync
// pair, buffers one frame and validates its length and checksum or CRC. On a
// rejected candidate it rescans the buffered bytes after that sync byte, so
// one push can leave more complete frames buffered behind the reported one.
//
//   if (r.push(b)) { do { use(r.frame()); } while (r.next()); }
// What a FrameReader threw away. Every discarded byte is counted once, in
// sync_dropped, including the sync byte of a rejected candidate; the rest of
// a rejected candidate is rescanned, not discarded. So without reset(),
// bytes = bytes of reported frames + sync_dropped + bytes still buffered.
// length_errors and check_errors count rejected candidates, not bytes.
struct FrameReaderStats {
    uint32_t bytes         = 0; // bytes pushed
    uint32_t frames        = 0; // valid frames reported
    uint32_t sync_dropped  = 0; // bytes discarded while looking for a valid frame
    uint32_t length_errors = 0; // candidate frames with an impossible length, type or mask
    uint32_t check_errors  = 0; // candidate frames failing their CRC or checksum
};

class FrameReader
{
public:
    // Clears the buffer, not the statistics.
    void reset();

    const FrameReaderStats& stats() const { return stats_; }

    // Feed one byte. True when a complete valid frame is buffered.
    bool push(uint8_t b);

    // Release the reported frame and report the next complete frame already
    // buffered, without a new byte. False when none is complete yet.
    bool next();

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
    bool     scan();

    uint8_t  buf_[kMaxFrameLen] = {};
    uint16_t len_               = 0; // bytes buffered
    uint16_t frame_len_         = 0; // complete frame at buf_, 0 when none
    FrameReaderStats stats_;
};

} // namespace translagatr
