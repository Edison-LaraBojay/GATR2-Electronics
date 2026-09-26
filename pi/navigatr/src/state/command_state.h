// command_state.h
// Latest intent from the brain, meters and radians. Defaults are usable with
// no brain link, so the Pi runs standalone on a bench. Command Collection
// carries the previous state forward when no update arrives; no update is
// not an empty command. Pose initialization is edge triggered: init_sequence
// increments once per new init command and localization remembers which
// (session, sequence) it last applied.
//
// session is the brain application session the Pi opened; 0 means none.
// object_wire_id is the brain's landmark wire id; only the brain-facing
// command and publisher implementations interpret it and the reply context.

#pragma once
#include <cstdint>

#include "math/transforms.h"
#include "resources/serial_link.h"

namespace navigatr
{

// The reply owed for the request processed this cycle, wire values. The
// brain_link commands slot clears it on every run and the brain_link
// publisher answers it in the same cycle. result is final except SET_POSE
// Pending (Ok once localization applied placement_sequence) and SELECT Ok
// (refined against the landmark mapping).
struct BrainReplyContext {
    bool     pending            = false;
    uint8_t  op                 = 0;
    uint32_t session            = 0;   // reply header session
    uint16_t request_id         = 0;
    uint32_t pi_instance        = 0;
    uint8_t  result             = 0;
    uint32_t nonce              = 0;   // HELLO echo
    uint64_t placement_sequence = 0;   // SET_POSE: init_sequence it was recorded under
    uint8_t  landmark_id        = 0;   // SELECT echo
    uint8_t  select_flags       = 0;
    TransmitWindow window;
};

struct CommandState {
    uint32_t session = 0;   // current brain session, 0 = none

    uint64_t init_sequence = 0;   // 0 = never commanded
    uint32_t init_session  = 0;   // session that sent init_pose
    Pose2D   init_pose;

    // Target selection is a command edge like init: object_sequence
    // increments once per newly accepted select or release and once per new
    // session, so a retransmitted command never re-activates and a
    // genuinely new command always does.
    bool     object_requested = false;
    uint8_t  object_wire_id   = 0;
    uint64_t object_sequence  = 0;   // 0 = never commanded

    BrainReplyContext reply;   // this cycle only
};

} // namespace navigatr
