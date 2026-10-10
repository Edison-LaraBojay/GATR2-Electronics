// brain_stdio.h
// The bytes a Brain publisher puts on stdout: one SGP1 line per write, raw or as a PROS COBS
// packet (stream id, then the line), with console text in between. For host tests.

#pragma once
#include <string>
#include <string_view>

#include "receiver/sources/chunk.h"
#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::receiver::sources::fakes
{

struct BrainStdioOptions {
    bool        cobs      = false; // PROS COBS on (the PROS default)
    std::string stream_id = "sout";
};

class FakeBrainStdio {
  public:
    explicit FakeBrainStdio(protocol::SessionId session, BrainStdioOptions options = {});

    // The next pose line: seq from 0 per session, heading only.
    Bytes pose(protocol::Millimeters x_mm, protocol::Millimeters y_mm,
               protocol::Centidegrees yaw_cdeg, protocol::SourceMillis t_ms);

    // Exactly this record, as one write. Empty when it does not encode.
    Bytes write(const protocol::PoseRecord& record) const;

    Bytes consoleText(std::string_view text) const;

    void restart(protocol::SessionId session); // new publisher run: seq from 0
    void resetPose();                          // epoch + 1
    void skip(protocol::Sequence count) { seq_ += count; }

    protocol::SessionId session() const { return session_; }
    protocol::Epoch     epoch() const { return epoch_; }
    protocol::Sequence  nextSeq() const { return seq_; }

  private:
    Bytes framed(std::string_view text) const;

    protocol::SessionId session_;
    BrainStdioOptions   options_;
    protocol::Epoch     epoch_ = 0;
    protocol::Sequence  seq_   = 0;
};

// Port of PROS 4.2.2 cobs_encode: stream id first, 0x00 delimiter after.
Bytes cobsPacket(std::string_view payload, std::string_view stream_id);

// Every byte, with a NUL after each group of `every` bytes.
Bytes withNuls(const Bytes& bytes, std::size_t every);

Bytes bytesOf(std::string_view text);

} // namespace spectagatr::receiver::sources::fakes
