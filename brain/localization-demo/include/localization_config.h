// localization_config.h
// Localization and viewer settings. Motor ports and controls are in drive_config.h.
// Rebuild and upload after a change.
//
// Values come from the archived robot profile, brain/robot/gatr2_robot.h at
// commit 10c568e.
//   UNVERIFIED: taken from the archive, not measured again for this demo.
//   MEASURED:   from an archived measurement, redo it when the robot changes.
//
// Robot frame: +x forward, +y left, origin at the center of the driven-wheel
// rectangle. Field frame (spectaGATR): origin at the bottom left corner of the
// field diagram, +x right, +y up, heading counterclockwise from +x.

#pragma once
#include <cstdint>

#include "encoder_tracker.h"
#include "odometry.h"

namespace config
{

constexpr double kInch = 0.0254;

// Controller/radio diagnostic: simulated pose only, no sensor reads or drive.
// Set true and upload to a separate Brain slot; false restores normal localization.
constexpr bool kControllerViewerTest = true;

// VEX IMU, flat and right side up. Robot still while it calibrates.
constexpr uint8_t kImuPort = 20;

// Pose shown at start, before any movement. Put the robot here first.
constexpr double kInitialXInches        = 7.0;
constexpr double kInitialYInches        = 24.0;
constexpr double kInitialHeadingDegrees = 0.0; // 0 faces +x (right on the diagram)

inline odometry::Pose initialPose() {
    odometry::Pose p;
    p.x       = kInitialXInches * kInch;
    p.y       = kInitialYInches * kInch;
    p.heading = odometry::degreesToRadians(kInitialHeadingDegrees);
    return p;
}

// Tracking wheels.
//   diameter: about 48 mm. UNVERIFIED.
//   counts_per_rev: 4000 = x4 decode (every A/B edge) of the AS5047P default
//     1000 pulses per rev, encoder on the wheel axle. UNVERIFIED: see scale.
//   scale: MEASURED by 23 inch straight pushes in the archive, which read
//     forward 0.278 m and sideways 0.297 m with scale 1. The ~2x shortfall says
//     counts_per_rev or diameter is wrong; until that is found, scale carries
//     it. If you fix counts_per_rev or diameter, set scale back to 1.0 and
//     measure again.
//   direction: -1, both encoders count down for forward / leftward pushes
//     (archive: reversed). Check with the README sign test.
//   offset: forward wheel 0.1115 m right of center (right tread centerline),
//     sideways wheel 0.011 m ahead of center. UNVERIFIED.
inline odometry::WheelSetup wheels() {
    odometry::WheelSetup w;

    w.forward.channel        = 1; // Pico GP2/GP3, J3
    w.forward.diameter       = 0.048;
    w.forward.counts_per_rev = 4000;
    w.forward.direction      = -1;
    w.forward.scale          = (23.0 * kInch) / (0.445 - 0.167);
    w.forward.offset         = -0.1115; // left of center, so negative is right

    w.sideways.channel        = 0; // Pico GP0/GP1, J2
    w.sideways.diameter       = 0.048;
    w.sideways.counts_per_rev = 4000;
    w.sideways.direction      = -1;
    w.sideways.scale          = (23.0 * kInch) / (0.740 - 0.443);
    w.sideways.offset         = 0.011; // forward of center

    return w;
}

// Encoder link. A gap longer than kStaleMs, a Pico restart, or a step over
// kMaxStepCounts rebases odometry instead of integrating.
inline localization::TrackerLimits trackerLimits() {
    localization::TrackerLimits l;
    l.stale_ms        = 250;
    l.max_step_counts = 20000; // about 0.75 m of raw wheel travel
    return l;
}

// spectaGATR pose lines per second (1..50).
constexpr int kVizRateHz = 10;

} // namespace config
