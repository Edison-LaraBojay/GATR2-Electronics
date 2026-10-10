// trajectory.cpp

#include "receiver/sim/trajectory.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "spectagatr/protocol/units.h"

namespace spectagatr::receiver::sim
{

namespace
{

using protocol::kPi;

constexpr Seconds kFigure8Period = 20.0;
constexpr Meters  kFigure8HalfX  = 1.2;
constexpr Meters  kFigure8HalfY  = 0.6;
constexpr Seconds kCirclePeriod  = 12.0;
constexpr Meters  kCircleRadius  = 1.0;
constexpr Meters  kSquareHalf    = 1.0;
constexpr Seconds kSquareDrive   = 4.0; // per edge, 0.5 m/s
constexpr Seconds kSquareTurn    = 1.0; // per corner
constexpr Seconds kSweep         = 6.0; // attitude: one axis out and back
constexpr Seconds kAttitudeCycle = 20.0;
constexpr Radians kYawSweep      = kPi / 2.0; // 90 degrees
constexpr Radians kTiltSweep     = kPi / 9.0; // 20 degrees
constexpr double  kStartMs       = 1000.0;    // publisher clock at the first pose

constexpr std::string_view kConsoleLine = "sim: console text between pose lines\n";

Radians degrees(double value) { return value * kPi / 180.0; }

Seconds cycle(Seconds t, Seconds period) {
    const Seconds phase = std::fmod(t, period);
    return phase < 0.0 ? phase + period : phase;
}

SimPose figure8(Seconds t) {
    const double w = 2.0 * kPi / kFigure8Period;
    SimPose      pose;
    pose.x          = kFieldCenter + kFigure8HalfX * std::sin(w * t);
    pose.y          = kFieldCenter + kFigure8HalfY * std::sin(2.0 * w * t);
    const double dx = kFigure8HalfX * w * std::cos(w * t);
    const double dy = 2.0 * kFigure8HalfY * w * std::cos(2.0 * w * t);
    pose.yaw        = std::atan2(dy, dx);
    return pose;
}

SimPose circle(Seconds t) {
    const Radians angle = 2.0 * kPi * cycle(t, kCirclePeriod) / kCirclePeriod;
    SimPose       pose;
    pose.x   = kFieldCenter + kCircleRadius * std::cos(angle);
    pose.y   = kFieldCenter + kCircleRadius * std::sin(angle);
    pose.yaw = angle + kPi / 2.0;
    return pose;
}

// Counterclockwise from the bottom left corner, facing +x.
SimPose square(Seconds t) {
    constexpr Seconds               edge     = kSquareDrive + kSquareTurn;
    const Seconds                   u        = cycle(t, 4.0 * edge);
    const int                       side     = std::min(3, static_cast<int>(u / edge));
    const Seconds                   s        = u - side * edge;
    constexpr std::array<double, 4> corner_x = {-1.0, 1.0, 1.0, -1.0};
    constexpr std::array<double, 4> corner_y = {-1.0, -1.0, 1.0, 1.0};
    const auto                      at       = static_cast<std::size_t>(side);
    const auto                      next     = static_cast<std::size_t>((side + 1) % 4);
    SimPose                         pose;
    const Radians                   heading = degrees(90.0 * side);
    if (s < kSquareDrive) {
        const double f = s / kSquareDrive;
        pose.x = kFieldCenter + kSquareHalf * (corner_x[at] + f * (corner_x[next] - corner_x[at]));
        pose.y = kFieldCenter + kSquareHalf * (corner_y[at] + f * (corner_y[next] - corner_y[at]));
        pose.yaw = heading;
    } else {
        pose.x   = kFieldCenter + kSquareHalf * corner_x[next];
        pose.y   = kFieldCenter + kSquareHalf * corner_y[next];
        pose.yaw = heading + degrees(90.0) * (s - kSquareDrive) / kSquareTurn;
    }
    return pose;
}

// Out and back on one axis: 0 -> peak -> 0 over kSweep.
double sweep(Seconds s, double peak) { return peak * std::sin(kPi * s / kSweep); }

SimPose attitude(Seconds t) {
    const Seconds u = cycle(t, kAttitudeCycle);
    SimPose       pose;
    pose.x        = kFieldCenter;
    pose.y        = kFieldCenter;
    pose.attitude = true;
    if (u < kSweep) {
        pose.yaw = sweep(u, kYawSweep);
    } else if (u < 2.0 * kSweep) {
        pose.roll = sweep(u - kSweep, kTiltSweep);
    } else if (u < 3.0 * kSweep) {
        pose.pitch = sweep(u - 2.0 * kSweep, kTiltSweep);
    }
    return pose;
}

uint32_t mix(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}

protocol::SourceMillis publisherMillis(Seconds since_start) {
    const auto ms = static_cast<uint64_t>(std::llround(kStartMs + since_start * 1000.0));
    return static_cast<protocol::SourceMillis>(ms & 0xFFFFFFFFu); // wraps as the Brain's does
}

} // namespace

SimPose simMotion(SimMode mode, Seconds t) {
    switch (mode) {
    case SimMode::kFigure8:
    case SimMode::kRestart:
        return figure8(t);
    case SimMode::kCircle:
    case SimMode::kDropout:
        return circle(t);
    case SimMode::kSquare:
        return square(t);
    case SimMode::kAttitude:
        return attitude(t);
    }
    return figure8(t);
}

protocol::SessionId simSession(uint32_t seed, uint32_t run) {
    const uint32_t id = mix(seed * 0x9E3779B9u + run * 0x85EBCA6Bu + 0x5A3C19E2u);
    return id != 0 ? id : 1u;
}

Seconds simTime(const SimConfig& config, Tick k) {
    return static_cast<Seconds>(k) / config.rate_hz;
}

std::optional<protocol::PoseRecord> simRecord(const SimConfig& config, Tick k) {
    const Seconds t = simTime(config, k);
    if (config.mode == SimMode::kDropout && cycle(t, kDropCycle) >= kDropFrom) {
        return std::nullopt;
    }
    protocol::PoseRecord record;
    Seconds              motion_t = t;
    if (config.mode == SimMode::kRestart) {
        const Tick    per_run = static_cast<Tick>(std::llround(kSessionSpan * config.rate_hz));
        const Tick    run     = per_run > 0 ? k / per_run : 0;
        const Tick    first   = run * per_run;
        const Seconds in_run  = simTime(config, k - first);
        record.session        = simSession(config.seed, static_cast<uint32_t>(run));
        record.epoch          = in_run >= kEpochAt ? 1 : 0;
        record.seq            = static_cast<protocol::Sequence>(k - first);
        record.t_ms           = publisherMillis(in_run);
        motion_t              = in_run + record.epoch * kEpochJump;
    } else {
        record.session = simSession(config.seed, 0);
        record.seq     = static_cast<protocol::Sequence>(k);
        record.t_ms    = publisherMillis(t);
    }
    const SimPose pose = simMotion(config.mode, motion_t);
    record.x_mm        = protocol::toMillimeters(pose.x);
    record.y_mm        = protocol::toMillimeters(pose.y);
    record.yaw_cdeg    = protocol::toCentidegrees(pose.yaw);
    if (pose.attitude) {
        record.roll_cdeg  = protocol::toCentidegrees(pose.roll);
        record.pitch_cdeg = protocol::toCentidegrees(pose.pitch);
        record.flags      = protocol::kFullAttitude;
    }
    return record;
}

sources::Bytes simBytes(const SimConfig& config, Tick k) {
    sources::Bytes bytes;
    if (k % kConsoleEvery == kConsoleEvery / 2) {
        bytes.assign(kConsoleLine.begin(), kConsoleLine.end());
    }
    const std::optional<protocol::PoseRecord> record = simRecord(config, k);
    if (!record) {
        return bytes;
    }
    char              line[protocol::kMaxLineLength + 1];
    const std::size_t length = protocol::encodeLine(*record, line, sizeof(line));
    bytes.insert(bytes.end(), line, line + length);
    return bytes;
}

std::optional<SimMode> simModeOf(std::string_view name) {
    for (const SimMode mode : {SimMode::kFigure8, SimMode::kCircle, SimMode::kSquare,
                               SimMode::kAttitude, SimMode::kRestart, SimMode::kDropout}) {
        if (name == simModeName(mode)) {
            return mode;
        }
    }
    return std::nullopt;
}

const char* simModeName(SimMode mode) {
    switch (mode) {
    case SimMode::kFigure8:
        return "figure8";
    case SimMode::kCircle:
        return "circle";
    case SimMode::kSquare:
        return "square";
    case SimMode::kAttitude:
        return "attitude";
    case SimMode::kRestart:
        return "restart";
    case SimMode::kDropout:
        return "dropout";
    }
    return "figure8";
}

} // namespace spectagatr::receiver::sim
