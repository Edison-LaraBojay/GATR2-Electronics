// pose_line.h
// spectaGATR wire format, version 1: one pose per text line.
//
//   SGP1 <session> <epoch> <seq> <t_ms> <x_mm> <y_mm> <yaw_cdeg> <roll_cdeg> <pitch_cdeg>
//   <flags>*<crc>\n
//
//   session     8 uppercase hex digits, nonzero, one per publisher run
//   epoch       decimal 0..65535, bumped by an explicit pose reset
//   seq         decimal u32, +1 per line, wraps
//   t_ms        decimal u32, publisher clock when the pose was supplied, wraps
//   x_mm, y_mm  decimal, field frame, millimeters, |v| <= kMaxCoordinateMm
//   yaw, roll,  decimal centidegrees, each in (-18000, 18000]
//   pitch
//   flags       decimal u8; bit 0 = full attitude supplied (else roll = pitch = 0)
//   crc         4 uppercase hex digits, CRC-16/CCITT-FALSE over every byte
//               from the 'S' of the marker up to, not including, '*'
//
// Single spaces between fields. A trailing '\r' before '\n' is ignored.
// Lines contain no 0x00 byte and stay under kMaxLineLength, so one line
// written in one write survives PROS COBS framing unchanged and can be found
// by a marker scan in raw console bytes, COBS packets or controller reads.
// Conventions and units: spectaGATR/docs/PROTOCOL.md.

#pragma once
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace spectagatr::protocol
{

using SessionId    = uint32_t;
using Epoch        = uint16_t;
using Sequence     = uint32_t;
using SourceMillis = uint32_t; // publisher clock, unrelated to any receiver clock
using Millimeters  = int32_t;
using Centidegrees = int32_t;

constexpr std::string_view kMarker          = "SGP1 ";
constexpr std::size_t      kMaxLineLength   = 120; // marker through '\n'
constexpr Millimeters      kMaxCoordinateMm = 2'000'000;
constexpr Centidegrees     kHalfTurnCdeg    = 18000;

enum PoseFlag : uint8_t {
    kFullAttitude = 1u << 0,
};

// One pose as it travels: fixed point, field frame.
struct PoseRecord {
    SessionId    session    = 0;
    Epoch        epoch      = 0;
    Sequence     seq        = 0;
    SourceMillis t_ms       = 0;
    Millimeters  x_mm       = 0;
    Millimeters  y_mm       = 0;
    Centidegrees yaw_cdeg   = 0;
    Centidegrees roll_cdeg  = 0;
    Centidegrees pitch_cdeg = 0;
    uint8_t      flags      = 0;
};

bool operator==(const PoseRecord& a, const PoseRecord& b);

// Every field inside the ranges above and session nonzero.
bool valid(const PoseRecord& record);

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
uint16_t crc16(std::string_view bytes);

// Writes the whole line including '\n' into out. Returns its length, or 0
// when the record is invalid or cap is too small. No allocation.
std::size_t encodeLine(const PoseRecord& record, char* out, std::size_t cap);

enum class DecodeError : uint8_t {
    kNone,
    kMalformed,  // wrong marker, field count, digits or separators
    kBadCrc,     // well formed, checksum mismatch
    kOutOfRange, // checksum fine, a value outside its range
};

struct DecodeResult {
    DecodeError error = DecodeError::kMalformed;
    PoseRecord  record;
};

// line: from the marker up to, not including, '\n'; one trailing '\r' is
// ignored.
DecodeResult decodeLine(std::string_view line);

const char* toString(DecodeError error);

// Finds marker lines in an arbitrary byte stream (raw console text, COBS
// packets, controller FIFO reads) fed in chunks of any size. Bytes outside a
// marker line are skipped. A line is dropped as truncated when a 0x00 or a
// new marker arrives before its '\n', and as too long past kMaxLineLength.
class LineScanner {
  public:
    struct Stats {
        uint64_t lines     = 0; // complete candidate lines handed out
        uint64_t truncated = 0;
        uint64_t too_long  = 0;
        uint64_t garbage   = 0; // control byte inside a line
    };

    // Calls on_line(std::string_view) per complete line, marker included,
    // '\n' excluded. The view is valid only during the call.
    template <typename OnLine> void feed(const uint8_t* data, std::size_t size, OnLine&& on_line) {
        for (std::size_t i = 0; i < size; ++i) {
            if (push(data[i])) {
                on_line(std::string_view(line_, length_));
                length_ = 0;
            }
        }
    }

    // Forget a partial line, for example after the source reconnected.
    void reset();

    const Stats& stats() const { return stats_; }

  private:
    // True when byte completed a line in line_[0, length_).
    bool push(uint8_t byte);

    char        line_[kMaxLineLength] = {};
    std::size_t length_               = 0; // collected bytes, 0 = searching
    std::size_t matched_              = 0; // marker prefix seen so far
    bool        collecting_           = false;
    Stats       stats_;
};

} // namespace spectagatr::protocol
