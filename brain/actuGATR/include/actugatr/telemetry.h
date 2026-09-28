// telemetry.h
// TELEMETRY motion and wheels groups from a drive task snapshot, in wire
// units: mm, mm/s, centidegrees, centidegrees/s and rpm x10, rounded and
// saturated to each field. Display and recording only; nothing here feeds
// control.
//
// motion: command id, state, reason, plan mode, segment and count, the
// field destination (zero when the command has none resolved), the chassis
// command the drive applied (after desaturation), cross track, distance and
// heading errors, drive fault. wheels: motor velocity targets per wheel
// group, positive driving forward.

#pragma once
#include "actugatr/drive_requests.h"
#include "translaGATR/frames.h"

namespace actugatr
{

// Motion and wheels groups with their flags; stamp_ms and the attitude
// group are left for the caller.
translagatr::BrainTelemetry telemetryOf(const DriveSnapshot& snapshot);

} // namespace actugatr
