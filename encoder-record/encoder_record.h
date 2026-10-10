// encoder_record.h
// Encoder record for the encoder localization demo: Pico -> Pi -> Brain.
// Fixed 16 bytes, integers little endian.
//
//   offset  size  field
//   0       2     magic     0xE7 0x5A
//   2       2     run_id    u16, random nonzero, new on every Pico boot
//   4       2     seq       u16, +1 per record, wraps
//   6       4     channel0  i32, cumulative x4 quadrature count, encoder channel 0
//   10      4     channel1  i32, cumulative x4 quadrature count, encoder channel 1
//   14      2     crc       u16, CRC-16/CCITT-FALSE over bytes 0..13
//
// Pico -> Pi UART: the 16 bytes back to back.
// Pi -> Brain USB user channel: one text line per record,
// "ENC1:" + 32 uppercase hex digits + "\n". The PROS serial daemon treats some
// raw bytes from the host as kernel commands, so binary never goes to stdin.
// Portable, no I/O.

#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace encoder_record
{

constexpr std::size_t kRecordSize = 16;
constexpr uint8_t     kMagic0     = 0xE7;
constexpr uint8_t     kMagic1     = 0x5A;

struct Record {
    uint16_t run_id   = 0;
    uint16_t seq      = 0;
    int32_t  channel0 = 0;
    int32_t  channel1 = 0;
};

using RecordBytes = std::array<uint8_t, kRecordSize>;

uint16_t    crc16(const uint8_t* data, std::size_t size);
RecordBytes encode(const Record& record);

// False on a wrong magic or checksum. bytes holds kRecordSize bytes.
bool decode(const uint8_t* bytes, Record& record);

struct ScanStats {
    uint32_t records = 0; // valid records found
    uint32_t skipped = 0; // bytes (raw) or lines (text) that were not a valid record
};

// Raw byte stream to records. Resyncs on the magic and checksum, so it can
// start mid record and recovers from corrupted or missing bytes.
class RecordScanner {
  public:
    // True when this byte completed a valid record.
    bool push(uint8_t byte);
    void reset() { used_ = 0; }

    const RecordBytes& bytes() const { return last_bytes_; }
    const Record&      record() const { return last_; }
    const ScanStats&   stats() const { return stats_; }

  private:
    void drop(std::size_t count);

    RecordBytes window_{};
    std::size_t used_ = 0;
    RecordBytes last_bytes_{};
    Record      last_;
    ScanStats   stats_;
};

// Text line envelope.
constexpr char        kLineMarker[] = "ENC1:";
constexpr std::size_t kMarkerSize   = sizeof(kLineMarker) - 1;
constexpr std::size_t kLineSize     = kMarkerSize + 2 * kRecordSize + 1; // newline included
constexpr std::size_t kLineLimit    = 96; // longer lines are skipped

using Line = std::array<char, kLineSize>;

Line encodeLine(const RecordBytes& bytes);

// Text stream to records. The last marker in a line counts and text before it
// is ignored; a trailing "\r" is stripped. Lines without exactly one valid
// record after the marker are skipped, including other console text.
class LineScanner {
  public:
    // True when this character ended a line carrying a valid record.
    bool push(char c);
    void reset();

    const Record&    record() const { return last_; }
    const ScanStats& stats() const { return stats_; }

  private:
    bool finish();

    std::array<char, kLineLimit> line_{};
    std::size_t                  used_       = 0;
    bool                         discarding_ = false;
    Record                       last_;
    ScanStats                    stats_;
};

} // namespace encoder_record
