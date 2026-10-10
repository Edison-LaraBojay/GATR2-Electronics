// chunk.h
// What a source hands on: bytes as they arrived, stamped on the receiver clock. std only, so the
// feed needs no port code.

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace spectagatr::receiver::sources
{

using ReceiverUs = int64_t; // receiver steady clock, us
using DurationUs = int64_t;
using Byte       = std::uint8_t;
using Bytes      = std::vector<Byte>;

enum class SourceKind : uint8_t {
    kNone,       // auto mode, nothing found yet
    kBrainUsb,   // Brain USB user port, read raw
    kController, // V5 controller, CDC2 USER_READ polling
    kSim,        // deterministic simulated poses
    kFile,       // bytes replayed from a file or stdin
};

// Nothing dropped or added.
struct SourceChunk {
    SourceKind src   = SourceKind::kNone;
    ReceiverUs rx_us = 0;
    Bytes      bytes;
};
using SourceChunks = std::vector<SourceChunk>;

// The browser feed's names: "none", "brain", "controller", "sim", "file".
const char* sourceKindName(SourceKind kind);

} // namespace spectagatr::receiver::sources
