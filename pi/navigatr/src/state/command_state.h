// command_state.h
// Latest intent from the brain, meters and radians. Defaults are usable with
// no brain link, so the Pi runs standalone on a bench. Command Collection
// carries the previous state forward when no update arrives; no update is
// not an empty command. Pose initialization is edge triggered: init_sequence
// increments once per new init command and localization remembers which
// sequence it last applied.
//
// object_wire_id is the brain's wire-protocol object id; only the
// brain-facing command and publisher implementations interpret it.

#pragma once
#include <cstdint>

#include "math/transforms.h"

namespace navigatr
{

struct CommandState {
    bool stream_on = true;

    uint64_t init_sequence = 0;   // 0 = never commanded
    Pose2D   init_pose;
    uint8_t  mode = 0;

    bool    object_requested = false;
    uint8_t object_wire_id   = 0;
};

} // namespace navigatr
