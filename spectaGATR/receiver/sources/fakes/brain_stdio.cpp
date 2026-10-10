// brain_stdio.cpp

#include "receiver/sources/fakes/brain_stdio.h"

#include <array>
#include <utility>

namespace spectagatr::receiver::sources::fakes
{

FakeBrainStdio::FakeBrainStdio(protocol::SessionId session, BrainStdioOptions options)
    : session_(session), options_(std::move(options)) {}

Bytes FakeBrainStdio::pose(protocol::Millimeters x_mm, protocol::Millimeters y_mm,
                           protocol::Centidegrees yaw_cdeg, protocol::SourceMillis t_ms) {
    protocol::PoseRecord record;
    record.session  = session_;
    record.epoch    = epoch_;
    record.seq      = seq_++;
    record.t_ms     = t_ms;
    record.x_mm     = x_mm;
    record.y_mm     = y_mm;
    record.yaw_cdeg = yaw_cdeg;
    return write(record);
}

Bytes FakeBrainStdio::write(const protocol::PoseRecord& record) const {
    std::array<char, protocol::kMaxLineLength + 1> line{};
    const std::size_t length = protocol::encodeLine(record, line.data(), line.size());
    if (length == 0) {
        return {};
    }
    return framed(std::string_view(line.data(), length));
}

Bytes FakeBrainStdio::consoleText(std::string_view text) const { return framed(text); }

void FakeBrainStdio::restart(protocol::SessionId session) {
    session_ = session;
    epoch_   = 0;
    seq_     = 0;
}

void FakeBrainStdio::resetPose() { ++epoch_; }

Bytes FakeBrainStdio::framed(std::string_view text) const {
    return options_.cobs ? cobsPacket(text, options_.stream_id) : bytesOf(text);
}

Bytes cobsPacket(std::string_view payload, std::string_view stream_id) {
    std::string source(stream_id);
    source.append(payload);
    Bytes       out(source.size() + source.size() / 254 + 2);
    std::size_t write   = 1;
    std::size_t code_at = 0;
    Byte        code    = 1;
    for (char c : source) {
        const Byte b = static_cast<Byte>(c);
        if (b == 0) {
            out[code_at] = code;
            code         = 1;
            code_at      = write++;
            continue;
        }
        out[write++] = b;
        if (++code == 0xFF) {
            out[code_at] = code;
            code         = 1;
            code_at      = write++;
        }
    }
    out[code_at] = code;
    out.resize(write);
    out.push_back(0);
    return out;
}

Bytes withNuls(const Bytes& bytes, std::size_t every) {
    Bytes out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        out.push_back(bytes[i]);
        if (every > 0 && (i + 1) % every == 0) {
            out.push_back(0x00);
        }
    }
    return out;
}

Bytes bytesOf(std::string_view text) { return Bytes(text.begin(), text.end()); }

} // namespace spectagatr::receiver::sources::fakes
