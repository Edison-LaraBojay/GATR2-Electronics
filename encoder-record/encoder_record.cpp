// encoder_record.cpp

#include "encoder_record.h"

#include <cstring>

namespace encoder_record
{

namespace
{

constexpr std::size_t kCrcOffset = kRecordSize - 2;
constexpr char        kDigits[]  = "0123456789ABCDEF";

void putU16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
}

void putU32(uint8_t* out, uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out[i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

uint16_t getU16(const uint8_t* in) {
    return static_cast<uint16_t>(in[0] | (in[1] << 8));
}

uint32_t getU32(const uint8_t* in) {
    uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        value |= static_cast<uint32_t>(in[i]) << (8 * i);
    }
    return value;
}

int hexDigit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

} // namespace

uint16_t crc16(const uint8_t* data, std::size_t size) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i] << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

RecordBytes encode(const Record& record) {
    RecordBytes out{};
    out[0] = kMagic0;
    out[1] = kMagic1;
    putU16(&out[2], record.run_id);
    putU16(&out[4], record.seq);
    putU32(&out[6], static_cast<uint32_t>(record.channel0));
    putU32(&out[10], static_cast<uint32_t>(record.channel1));
    putU16(&out[kCrcOffset], crc16(out.data(), kCrcOffset));
    return out;
}

bool decode(const uint8_t* bytes, Record& record) {
    if (bytes[0] != kMagic0 || bytes[1] != kMagic1) {
        return false;
    }
    if (getU16(&bytes[kCrcOffset]) != crc16(bytes, kCrcOffset)) {
        return false;
    }
    record.run_id   = getU16(&bytes[2]);
    record.seq      = getU16(&bytes[4]);
    record.channel0 = static_cast<int32_t>(getU32(&bytes[6]));
    record.channel1 = static_cast<int32_t>(getU32(&bytes[10]));
    return true;
}

void RecordScanner::drop(std::size_t count) {
    std::memmove(window_.data(), window_.data() + count, used_ - count);
    used_ -= count;
    stats_.skipped += static_cast<uint32_t>(count);
}

bool RecordScanner::push(uint8_t byte) {
    window_[used_++] = byte;
    while (true) {
        // Keep the window starting at a possible magic.
        while (used_ > 0 && window_[0] != kMagic0) {
            drop(1);
        }
        if (used_ >= 2 && window_[1] != kMagic1) {
            drop(1);
            continue;
        }
        if (used_ < kRecordSize) {
            return false;
        }
        Record record;
        if (decode(window_.data(), record)) {
            last_bytes_ = window_;
            last_       = record;
            used_       = 0;
            ++stats_.records;
            return true;
        }
        drop(1);
    }
}

Line encodeLine(const RecordBytes& bytes) {
    Line line{};
    std::memcpy(line.data(), kLineMarker, kMarkerSize);
    for (std::size_t i = 0; i < kRecordSize; ++i) {
        line[kMarkerSize + 2 * i]     = kDigits[bytes[i] >> 4];
        line[kMarkerSize + 2 * i + 1] = kDigits[bytes[i] & 0x0F];
    }
    line[kLineSize - 1] = '\n';
    return line;
}

bool LineScanner::push(char c) {
    if (c == '\n') {
        const bool got = !discarding_ && finish();
        if (discarding_) {
            ++stats_.skipped;
        }
        used_       = 0;
        discarding_ = false;
        return got;
    }
    if (discarding_) {
        return false;
    }
    if (used_ >= line_.size()) {
        discarding_ = true;
        return false;
    }
    line_[used_++] = c;
    return false;
}

void LineScanner::reset() {
    used_       = 0;
    discarding_ = false;
}

bool LineScanner::finish() {
    std::size_t end = used_;
    if (end > 0 && line_[end - 1] == '\r') {
        --end;
    }
    std::size_t start = 0;
    bool        found = false;
    for (std::size_t i = 0; i + kMarkerSize <= end; ++i) {
        if (std::memcmp(&line_[i], kLineMarker, kMarkerSize) == 0) {
            start = i + kMarkerSize;
            found = true;
        }
    }
    if (!found || end - start != 2 * kRecordSize) {
        ++stats_.skipped;
        return false;
    }
    RecordBytes bytes{};
    for (std::size_t i = 0; i < kRecordSize; ++i) {
        const int high = hexDigit(line_[start + 2 * i]);
        const int low  = hexDigit(line_[start + 2 * i + 1]);
        if (high < 0 || low < 0) {
            ++stats_.skipped;
            return false;
        }
        bytes[i] = static_cast<uint8_t>((high << 4) | low);
    }
    Record record;
    if (!decode(bytes.data(), record)) {
        ++stats_.skipped;
        return false;
    }
    last_ = record;
    ++stats_.records;
    return true;
}

} // namespace encoder_record
