// source.h
// A receiver source: one device read on its own thread, handing on chunks and port state.
// Port: one open device, a real serial port or a fake in tests.

#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "receiver/platform/serial_port.h"
#include "receiver/sources/chunk.h"

namespace spectagatr::receiver::sources
{

using Milliseconds  = platform::Milliseconds;
using IoResult      = platform::IoResult;
using IoStatus      = platform::IoStatus;
using ReceiverClock = std::function<ReceiverUs()>;

enum class PortState : uint8_t {
    kSearching,     // no device, or the open failed; retried
    kOpen,          // reading
    kNoBrainLinked, // controller: open, but no Brain on the radio or cable
    kClosed,        // unplugged or failed; retried
    kStopped,
};

// Controller RADIO_STATUS, as vexide decodes it (layout unverified on hardware).
struct RadioStatus {
    uint8_t  device   = 0; // 0 none, 2 V5 controller Bluetooth, 4 V5 controller VEXnet
    uint16_t quality  = 0; // 0..100
    int16_t  strength = 0;
    uint8_t  channel  = 0; // VEXnet 5 = download, 9 = reconnecting
    uint8_t  timeslot = 0;
};

struct TransportStats {
    uint32_t                   reads            = 0; // brain: reads with bytes
    uint32_t                   reads_of_512     = 0; // brain: a multiple of 512 bytes
    uint32_t                   holdback_suspect = 0; // brain: such a read, then a timeout
    uint32_t                   polls            = 0; // controller USER_READ
    uint32_t                   empty_polls      = 0;
    uint32_t                   timeouts         = 0;
    uint32_t                   bad_replies      = 0; // crc, NACK, unexpected reply
    uint32_t                   late_replies     = 0; // answers to earlier, timed-out polls
    uint32_t                   stray_bytes      = 0; // outside any reply
    uint32_t                   padding_bytes    = 0; // controller: trailing NULs dropped
    uint32_t                   data_bytes       = 0; // controller: Brain bytes delivered
    uint32_t                   channel_switches = 0; // controller: radio moved to download
    std::optional<DurationUs>  rtt_us;               // last USER_READ round trip
    std::optional<RadioStatus> radio;                // last RADIO_STATUS
};

struct SourceStatus {
    SourceKind     src   = SourceKind::kNone;
    PortState      state = PortState::kSearching;
    ReceiverUs     at_us = 0;
    std::string    port;   // COM7, /dev/ttyACM1, a file; empty when none
    std::string    detail; // what is linked, or why not
    TransportStats stats;
    bool           stream_restarted = false; // replay rewound: ordering starts over
};

using ChunkCallback  = std::function<void(const SourceChunk&)>;
using StatusCallback = std::function<void(const SourceStatus&)>;

using PortRead =
    std::function<IoResult(Byte* destination, std::size_t capacity, Milliseconds wait)>;
using PortWrite =
    std::function<IoResult(const Byte* source, std::size_t size, Milliseconds budget)>;

// One open device. Closes when the last copy goes.
struct Port {
    PortRead    read;
    PortWrite   write;
    SourceKind  kind = SourceKind::kBrainUsb; // how to talk to it
    std::string path;                         // for status: COM7
    std::string name;                         // for status text: COM7 (controller, vex:0503@...)
};

// Finds and opens the device; nullopt with err when there is none.
using PortOpener = std::function<std::optional<Port>(std::string& err)>;

struct SourceHooks {
    PortOpener     open_port; // serial sources only
    ReceiverClock  clock;
    ChunkCallback  on_chunk;
    StatusCallback on_status;
};

const char* portStateName(PortState state);

} // namespace spectagatr::receiver::sources
