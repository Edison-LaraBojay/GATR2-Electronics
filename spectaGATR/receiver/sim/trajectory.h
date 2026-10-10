// trajectory.h
// Deterministic simulated poses, so rendering and the transport-independent receiver rules can be
// checked without hardware. Everything is a pure function of the tick index k (time k / rate)
// and the seed: the same k gives the same bytes on every run. Poses stay inside the 3.5664 m
// Override field (origin bottom left, +x right, +y up, heading counterclockwise from +x).

#pragma once
#include <cstdint>
#include <optional>
#include <string_view>

#include "receiver/sources/chunk.h"
#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::receiver::sim
{

using Meters  = double;
using Radians = double;
using Seconds = double;
using Hertz   = double;
using Tick    = uint64_t;

constexpr Meters  kFieldSize    = 3.5664;
constexpr Meters  kFieldCenter  = kFieldSize / 2.0;
constexpr Hertz   kMinRateHz    = 1.0;
constexpr Hertz   kMaxRateHz    = 50.0;
constexpr Seconds kSessionSpan  = 15.0; // restart: a new session this often
constexpr Seconds kEpochAt      = 7.5;  // restart: the epoch bump inside each session
constexpr Seconds kEpochJump    = 5.0;  // restart: motion skips ahead this far at the bump
constexpr Seconds kDropCycle    = 10.0; // dropout: cycle length
constexpr Seconds kDropFrom     = 7.0;  // dropout: silent from here to the cycle end (3 s)
constexpr Tick    kConsoleEvery = 50;   // console text before every 50th line

enum class SimMode : uint8_t {
    kFigure8,  // 20 s figure eight, heading along the path
    kCircle,   // 1 m radius, 12 s, counterclockwise
    kSquare,   // 2 m square, counterclockwise, 90 degree turns in place
    kAttitude, // standing at the center: yaw +90, then roll +20, then pitch +20 degrees and back
    kRestart,  // figure eight; a new session every 15 s, an epoch bump 7.5 s into each
    kDropout,  // circle; nothing sent for 3 s of every 10 s
};

struct SimConfig {
    SimMode  mode    = SimMode::kFigure8;
    Hertz    rate_hz = 10.0;
    uint32_t seed    = 1;
};

struct SimPose {
    Meters  x        = 0;
    Meters  y        = 0;
    Radians yaw      = 0;
    Radians roll     = 0;
    Radians pitch    = 0;
    bool    attitude = false; // roll and pitch supplied
};

// The motion at t seconds into it (restart uses the figure eight, dropout the circle).
SimPose simMotion(SimMode mode, Seconds t);

// Nonzero session id for a seed and the index of the publisher run.
protocol::SessionId simSession(uint32_t seed, uint32_t run);

// The record tick k sends, or none while a dropout is silent (its seq is lost, as on a radio).
std::optional<protocol::PoseRecord> simRecord(const SimConfig& config, Tick k);

// Every byte tick k writes: maybe a console line, then the pose line.
sources::Bytes simBytes(const SimConfig& config, Tick k);

Seconds simTime(const SimConfig& config, Tick k);

std::optional<SimMode> simModeOf(std::string_view name);
const char*            simModeName(SimMode mode);

} // namespace spectagatr::receiver::sim
