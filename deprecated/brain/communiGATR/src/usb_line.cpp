// usb_line.cpp

#include "communigatr/usb_line.h"

#include <cstring>

namespace communigatr
{

namespace
{

constexpr char kMarker[] = "NG1:";
constexpr char kDigits[] = "0123456789ABCDEF";

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

std::size_t encodeUsbLine(const uint8_t* frame, std::size_t len, char* out, std::size_t cap) {
    const std::size_t line = kUsbMarkerLen + 2 * len + 1;
    if (len == 0 || len > translagatr::kMaxFrameLen || cap < line) {
        return 0;
    }
    std::memcpy(out, kMarker, kUsbMarkerLen);
    for (std::size_t i = 0; i < len; ++i) {
        out[kUsbMarkerLen + 2 * i]     = kDigits[frame[i] >> 4];
        out[kUsbMarkerLen + 2 * i + 1] = kDigits[frame[i] & 0x0F];
    }
    out[line - 1] = '\n';
    return line;
}

bool UsbLineDecoder::push(char c) {
    length_ = 0;
    if (c == '\n') {
        const bool whole = !discarding_;
        const bool got   = whole && finish();
        if (!whole) {
            ++stats_.dropped;
        }
        used_       = 0;
        discarding_ = false;
        return got;
    }
    if (discarding_) {
        return false;
    }
    if (used_ >= kUsbLineLimit) {
        used_       = 0;
        discarding_ = true;
        return false;
    }
    line_[used_++] = c;
    return false;
}

void UsbLineDecoder::reset() {
    used_       = 0;
    discarding_ = false;
    length_     = 0;
}

bool UsbLineDecoder::finish() {
    std::size_t end = used_;
    if (end > 0 && line_[end - 1] == '\r') {
        --end;
    }
    // Last marker in the line.
    std::size_t start = 0;
    bool        found = false;
    for (std::size_t i = 0; i + kUsbMarkerLen <= end; ++i) {
        if (std::memcmp(line_ + i, kMarker, kUsbMarkerLen) == 0) {
            start = i + kUsbMarkerLen;
            found = true;
        }
    }
    if (!found) {
        ++stats_.ignored;
        return false;
    }
    const std::size_t digits = end - start;
    if (digits == 0 || digits % 2 != 0 || digits > 2 * translagatr::kMaxFrameLen) {
        ++stats_.dropped;
        return false;
    }
    for (std::size_t i = 0; i < digits; i += 2) {
        const int high = hexDigit(line_[start + i]);
        const int low  = hexDigit(line_[start + i + 1]);
        if (high < 0 || low < 0) {
            ++stats_.dropped;
            return false;
        }
        bytes_[i / 2] = static_cast<uint8_t>((high << 4) | low);
    }
    length_ = digits / 2;
    ++stats_.frames;
    return true;
}

} // namespace communigatr
