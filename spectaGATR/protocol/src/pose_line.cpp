// pose_line.cpp

#include "spectagatr/protocol/pose_line.h"

#include <cstdio>

namespace spectagatr::protocol
{
namespace
{

constexpr std::size_t kFieldCount = 10; // after the marker, before '*'
constexpr std::size_t kCrcDigits  = 4;

bool angleInRange(Centidegrees v) { return v > -kHalfTurnCdeg && v <= kHalfTurnCdeg; }

bool coordinateInRange(Millimeters v) { return v >= -kMaxCoordinateMm && v <= kMaxCoordinateMm; }

int hexValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

// Exactly digits uppercase hex digits.
bool parseHex(std::string_view text, std::size_t digits, uint32_t& out) {
    if (text.size() != digits) {
        return false;
    }
    uint32_t value = 0;
    for (char c : text) {
        const int v = hexValue(c);
        if (v < 0) {
            return false;
        }
        value = (value << 4) | static_cast<uint32_t>(v);
    }
    out = value;
    return true;
}

// 1 to 10 decimal digits, value <= max.
bool parseUnsigned(std::string_view text, uint64_t max, uint64_t& out) {
    if (text.empty() || text.size() > 10) {
        return false;
    }
    uint64_t value = 0;
    for (char c : text) {
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + static_cast<uint64_t>(c - '0');
    }
    if (value > max) {
        return false;
    }
    out = value;
    return true;
}

// Optional '-' then 1 to 10 digits, within int32.
bool parseSigned(std::string_view text, int32_t& out) {
    const bool negative = !text.empty() && text.front() == '-';
    if (negative) {
        text.remove_prefix(1);
    }
    uint64_t magnitude = 0;
    if (!parseUnsigned(text, negative ? 2147483648ull : 2147483647ull, magnitude)) {
        return false;
    }
    out = negative ? static_cast<int32_t>(-static_cast<int64_t>(magnitude))
                   : static_cast<int32_t>(magnitude);
    return true;
}

// Splits on single spaces; false unless exactly count fields, none empty.
bool split(std::string_view text, std::string_view* fields, std::size_t count) {
    std::size_t n = 0;
    while (true) {
        const std::size_t      space = text.find(' ');
        const std::string_view field = text.substr(0, space);
        if (field.empty() || n == count) {
            return false;
        }
        fields[n++] = field;
        if (space == std::string_view::npos) {
            break;
        }
        text.remove_prefix(space + 1);
    }
    return n == count;
}

} // namespace

bool operator==(const PoseRecord& a, const PoseRecord& b) {
    return a.session == b.session && a.epoch == b.epoch && a.seq == b.seq && a.t_ms == b.t_ms &&
           a.x_mm == b.x_mm && a.y_mm == b.y_mm && a.yaw_cdeg == b.yaw_cdeg &&
           a.roll_cdeg == b.roll_cdeg && a.pitch_cdeg == b.pitch_cdeg && a.flags == b.flags;
}

bool valid(const PoseRecord& r) {
    return r.session != 0 && coordinateInRange(r.x_mm) && coordinateInRange(r.y_mm) &&
           angleInRange(r.yaw_cdeg) && angleInRange(r.roll_cdeg) && angleInRange(r.pitch_cdeg);
}

uint16_t crc16(std::string_view bytes) {
    uint16_t crc = 0xFFFF;
    for (char c : bytes) {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(c) << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::size_t encodeLine(const PoseRecord& r, char* out, std::size_t cap) {
    if (!valid(r) || out == nullptr || cap == 0) {
        return 0;
    }
    char      body[kMaxLineLength];
    const int n =
        std::snprintf(body, sizeof(body), "SGP1 %08lX %u %lu %lu %ld %ld %ld %ld %ld %u",
                      static_cast<unsigned long>(r.session), static_cast<unsigned>(r.epoch),
                      static_cast<unsigned long>(r.seq), static_cast<unsigned long>(r.t_ms),
                      static_cast<long>(r.x_mm), static_cast<long>(r.y_mm),
                      static_cast<long>(r.yaw_cdeg), static_cast<long>(r.roll_cdeg),
                      static_cast<long>(r.pitch_cdeg), static_cast<unsigned>(r.flags));
    if (n <= 0 || static_cast<std::size_t>(n) >= sizeof(body)) {
        return 0;
    }
    const uint16_t    crc   = crc16(std::string_view(body, static_cast<std::size_t>(n)));
    const std::size_t total = static_cast<std::size_t>(n) + 1 + kCrcDigits + 1;
    if (total > kMaxLineLength || total >= cap) {
        return 0;
    }
    const int written = std::snprintf(out, cap, "%s*%04X\n", body, static_cast<unsigned>(crc));
    return written == static_cast<int>(total) ? total : 0;
}

DecodeResult decodeLine(std::string_view line) {
    DecodeResult result;
    if (!line.empty() && line.back() == '\r') {
        line.remove_suffix(1);
    }
    if (line.size() < kMarker.size() + 1 + kCrcDigits ||
        line.substr(0, kMarker.size()) != kMarker) {
        return result;
    }
    const std::size_t star = line.size() - kCrcDigits - 1;
    if (line[star] != '*') {
        return result;
    }
    uint32_t sent_crc = 0;
    if (!parseHex(line.substr(star + 1), kCrcDigits, sent_crc)) {
        return result;
    }
    const std::string_view body = line.substr(0, star);
    if (crc16(body) != sent_crc) {
        result.error = DecodeError::kBadCrc;
        return result;
    }

    std::string_view f[kFieldCount];
    if (!split(body.substr(kMarker.size()), f, kFieldCount)) {
        return result;
    }
    PoseRecord r;
    uint64_t   epoch = 0, seq = 0, t_ms = 0, flags = 0;
    if (!parseHex(f[0], 8, r.session) || !parseUnsigned(f[1], 0xFFFF, epoch) ||
        !parseUnsigned(f[2], 0xFFFFFFFFull, seq) || !parseUnsigned(f[3], 0xFFFFFFFFull, t_ms) ||
        !parseSigned(f[4], r.x_mm) || !parseSigned(f[5], r.y_mm) ||
        !parseSigned(f[6], r.yaw_cdeg) || !parseSigned(f[7], r.roll_cdeg) ||
        !parseSigned(f[8], r.pitch_cdeg) || !parseUnsigned(f[9], 0xFF, flags)) {
        return result;
    }
    r.epoch       = static_cast<Epoch>(epoch);
    r.seq         = static_cast<Sequence>(seq);
    r.t_ms        = static_cast<SourceMillis>(t_ms);
    r.flags       = static_cast<uint8_t>(flags);
    result.record = r;
    result.error  = valid(r) ? DecodeError::kNone : DecodeError::kOutOfRange;
    return result;
}

const char* toString(DecodeError error) {
    switch (error) {
    case DecodeError::kNone:
        return "none";
    case DecodeError::kMalformed:
        return "malformed";
    case DecodeError::kBadCrc:
        return "bad crc";
    case DecodeError::kOutOfRange:
        return "out of range";
    }
    return "unknown";
}

void LineScanner::reset() {
    length_     = 0;
    matched_    = 0;
    collecting_ = false;
}

bool LineScanner::push(uint8_t byte) {
    // The marker is looked for in every byte, so a cut line followed by a new
    // marker loses only the cut line.
    if (byte == static_cast<uint8_t>(kMarker[matched_])) {
        ++matched_;
    } else {
        matched_ = byte == static_cast<uint8_t>(kMarker[0]) ? 1 : 0;
    }
    if (matched_ == kMarker.size()) {
        if (collecting_) {
            ++stats_.truncated;
        }
        for (std::size_t i = 0; i < kMarker.size(); ++i) {
            line_[i] = kMarker[i];
        }
        length_     = kMarker.size();
        matched_    = 0;
        collecting_ = true;
        return false;
    }
    if (!collecting_) {
        return false;
    }

    if (byte == '\n') {
        collecting_ = false;
        if (length_ > 0 && line_[length_ - 1] == '\r') {
            --length_;
        }
        ++stats_.lines;
        return true;
    }
    if (byte == 0x00) {
        ++stats_.truncated; // COBS packet or read ended inside the line
        reset();
        return false;
    }
    if ((byte < 0x20 && byte != '\r') || byte >= 0x7F) {
        ++stats_.garbage;
        reset();
        return false;
    }
    if (length_ + 1 >= kMaxLineLength) { // keep room for the '\n'
        ++stats_.too_long;
        reset();
        return false;
    }
    line_[length_++] = static_cast<char>(byte);
    return false;
}

} // namespace spectagatr::protocol
