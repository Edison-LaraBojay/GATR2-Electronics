// cdc2.cpp

#include "receiver/sources/cdc2.h"

#include <algorithm>

namespace spectagatr::receiver::sources
{

namespace
{

constexpr Byte        kRequestHeader[] = {0xC9, 0x36, 0xB8, 0x47};
constexpr Byte        kReplyFirst      = 0xAA;
constexpr Byte        kReplySecond     = 0x55;
constexpr std::size_t kSimpleHeadBytes = 4; // AA 55 cmd len
constexpr std::size_t kLongHeadBytes   = 5; // AA 55 56 size-hi size-lo
constexpr Byte        kLongSizeFlag    = 0x80;
constexpr std::size_t kCrcBytes        = 2;
constexpr std::size_t kMinExtended     = 1 + kCrcBytes; // ecmd + crc

Bytes extendedRequest(Byte ecmd, const Bytes& payload) {
    Bytes packet(std::begin(kRequestHeader), std::end(kRequestHeader));
    packet.push_back(kCommandExtended);
    packet.push_back(ecmd);
    if (payload.size() >= kLongSizeFlag) {
        packet.push_back(static_cast<Byte>((payload.size() >> 8) | kLongSizeFlag));
    }
    packet.push_back(static_cast<Byte>(payload.size() & 0xFF));
    packet.insert(packet.end(), payload.begin(), payload.end());
    const uint16_t crc = crc16Xmodem(packet.data(), packet.size());
    packet.push_back(static_cast<Byte>(crc >> 8));
    packet.push_back(static_cast<Byte>(crc & 0xFF));
    return packet;
}

// First AA 55 at or after 0; buffer size when none.
std::size_t findHeader(const Bytes& buffer) {
    for (std::size_t i = 0; i + 1 < buffer.size(); ++i) {
        if (buffer[i] == kReplyFirst && buffer[i + 1] == kReplySecond) {
            return i;
        }
    }
    return buffer.size();
}

} // namespace

uint16_t crc16Xmodem(const Byte* data, std::size_t size) {
    uint16_t crc = 0;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i] << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                      : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

Bytes systemVersionRequest() {
    Bytes packet(std::begin(kRequestHeader), std::end(kRequestHeader));
    packet.push_back(kCommandSystemVersion);
    return packet;
}

Bytes userReadRequest(Byte channel, Byte length) {
    return extendedRequest(kExtendedUserRead, Bytes{channel, length});
}

Bytes radioStatusRequest() { return extendedRequest(kExtendedRadioStatus, Bytes{}); }

Bytes radioChannelRequest(bool download) {
    return extendedRequest(kExtendedFileControl, Bytes{0x01, static_cast<Byte>(download ? 1 : 0)});
}

CdcScan CdcReplyScanner::push(const Byte* data, std::size_t size) {
    CdcScan scan;
    buffer_.insert(buffer_.end(), data, data + size);
    while (std::optional<CdcReply> reply = next(scan)) {
        scan.replies.push_back(std::move(*reply));
    }
    return scan;
}

void CdcReplyScanner::reset() { buffer_.clear(); }

std::optional<CdcReply> CdcReplyScanner::next(CdcScan& scan) {
    while (true) {
        std::size_t start = findHeader(buffer_);
        if (start == buffer_.size() && !buffer_.empty() && buffer_.back() == kReplyFirst) {
            start = buffer_.size() - 1; // maybe the first header byte
        }
        scan.stray_bytes += static_cast<uint32_t>(start);
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start));
        if (buffer_.size() < kSimpleHeadBytes) {
            return std::nullopt;
        }
        const Byte command  = buffer_[2];
        const bool extended = command == kCommandExtended;
        if (!extended && command != kCommandSystemVersion) {
            ++scan.stray_bytes; // no such reply is asked for
            buffer_.erase(buffer_.begin());
            continue;
        }
        std::size_t head = kSimpleHeadBytes;
        std::size_t size = buffer_[3];
        if (extended && (buffer_[3] & kLongSizeFlag) != 0) {
            if (buffer_.size() < kLongHeadBytes) {
                return std::nullopt;
            }
            head = kLongHeadBytes;
            size = (static_cast<std::size_t>(buffer_[3] & 0x7F) << 8) | buffer_[4];
        }
        const std::size_t total    = head + size;
        const bool        bad_size = size > kMaxReplyBytes || (extended && size < kMinExtended);
        if (!bad_size && buffer_.size() < total) {
            return std::nullopt;
        }
        if (bad_size || (extended && crc16Xmodem(buffer_.data(), total) != 0)) {
            ++scan.bad_replies;
            buffer_.erase(buffer_.begin()); // rescan after this AA
            continue;
        }
        CdcReply          reply;
        const std::size_t end = extended ? total - kCrcBytes : total;
        reply.command         = command;
        reply.payload.assign(buffer_.begin() + static_cast<std::ptrdiff_t>(head),
                             buffer_.begin() + static_cast<std::ptrdiff_t>(end));
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
        return reply;
    }
}

std::optional<SystemVersion> parseSystemVersion(const CdcReply& reply) {
    constexpr std::size_t kVersionBytes = 8;
    if (reply.command != kCommandSystemVersion || reply.payload.size() < kVersionBytes) {
        return std::nullopt;
    }
    SystemVersion version;
    version.product = reply.payload[5];
    version.flags   = reply.payload[6];
    return version;
}

LinkKind controllerLink(const SystemVersion& version) {
    if (version.product != kProductController) {
        return LinkKind::kNone;
    }
    if ((version.flags & kLinkedByRadio) != 0) {
        return LinkKind::kRadio;
    }
    if ((version.flags & kLinkedByCable) != 0) {
        return LinkKind::kCable;
    }
    return LinkKind::kNone;
}

UserRead parseUserRead(const CdcReply& reply) {
    constexpr std::size_t kAckAt     = 1;
    constexpr std::size_t kDataStart = 3; // ecmd, ack, channel
    UserRead              read;
    if (reply.command != kCommandExtended || reply.payload.size() <= kAckAt) {
        read.error = UserReadError::kShort;
        return read;
    }
    if (reply.payload[0] != kExtendedUserRead) {
        read.error = UserReadError::kNotUserRead;
        return read;
    }
    read.ack = reply.payload[kAckAt];
    if (read.ack != kAck) {
        read.error = UserReadError::kNack;
        return read;
    }
    if (reply.payload.size() < kDataStart) {
        read.error = UserReadError::kShort;
        return read;
    }
    read.data.assign(reply.payload.begin() + kDataStart, reply.payload.end());
    return read;
}

std::optional<RadioStatus> parseRadioStatus(const CdcReply& reply) {
    constexpr std::size_t kAckAt       = 1;
    constexpr std::size_t kStatusStart = 2;
    constexpr std::size_t kStatusBytes = 7;
    if (reply.command != kCommandExtended || reply.payload.size() < kStatusStart + kStatusBytes ||
        reply.payload[0] != kExtendedRadioStatus || reply.payload[kAckAt] != kAck) {
        return std::nullopt;
    }
    const Byte* at = reply.payload.data() + kStatusStart;
    RadioStatus status;
    status.device   = at[0];
    status.quality  = static_cast<uint16_t>(at[1] | (at[2] << 8));
    status.strength = static_cast<int16_t>(static_cast<uint16_t>(at[3] | (at[4] << 8)));
    status.channel  = at[5];
    status.timeslot = at[6];
    return status;
}

const char* userReadErrorName(UserReadError error) {
    switch (error) {
    case UserReadError::kNone:
        return "none";
    case UserReadError::kNotUserRead:
        return "not_user_read";
    case UserReadError::kNack:
        return "nack";
    case UserReadError::kShort:
        return "short";
    }
    return "unknown";
}

const char* linkKindName(LinkKind kind) {
    switch (kind) {
    case LinkKind::kNone:
        return "none";
    case LinkKind::kCable:
        return "cable";
    case LinkKind::kRadio:
        return "radio";
    }
    return "unknown";
}

} // namespace spectagatr::receiver::sources
