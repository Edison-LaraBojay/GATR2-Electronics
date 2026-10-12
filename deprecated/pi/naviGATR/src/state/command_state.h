// command_state.h
// Latest intent from the brain, meters and radians. Defaults are usable with
// no brain link, so the Pi runs standalone on a bench. Command Collection
// carries the previous state forward when no update arrives; no update is
// not an empty command. Pose initialization is edge triggered: init_sequence
// increments once per new init command and localization remembers which
// (session, sequence) it last applied.
//
// session is the brain application session the Pi opened; 0 means none.
// The reply context and the profile status hold wire codes; only the
// brain-facing command and publisher implementations interpret them.

#pragma once
#include <array>
#include <cstdint>

#include "translaGATR/frames.h"
#include "core/time.h"
#include "math/transforms.h"
#include "resources/serial_link.h"

namespace navigatr
{

// The reply owed for the request processed this cycle, wire values. The
// brain_link commands slot clears it on every run and the brain_link
// publisher answers it in the same cycle. result is final except SET_POSE
// Pending (Ok once localization applied placement_sequence), READ_DOC Ok
// (the publisher owns the documents) and CONTROL Ok (calibration added).
struct BrainReplyContext {
    bool     pending            = false;
    uint8_t  op                 = 0;
    uint32_t session            = 0;   // reply header session
    uint16_t request_id         = 0;
    uint32_t pi_instance        = 0;
    uint8_t  result             = 0;
    uint32_t nonce              = 0;   // HELLO echo
    uint64_t placement_sequence = 0;   // SET_POSE: init_sequence it was recorded under

    uint32_t profile_id     = 0;   // PROFILE_WRITE, PROFILE_APPLY
    uint16_t received       = 0;   // PROFILE_WRITE: contiguous bytes staged
    uint8_t  profile_state  = 0;   // PROFILE_APPLY
    uint8_t  profile_reason = 0;
    uint8_t  profile_detail = 0;

    uint8_t  doc_kind    = 0;   // READ_DOC request
    uint32_t doc_id      = 0;
    uint16_t doc_offset  = 0;
    uint8_t  doc_max_len = 0;

    uint8_t action         = 0;   // CONTROL echo
    uint8_t control_detail = 0;   // CONTROL: translagatr::ControlDetail

    uint8_t wheel_count = 0;   // READ_WHEELS Ok
    std::array<translagatr::WheelReading, translagatr::kWheelReadingsMax> wheels{};

    TransmitWindow window;
};

// Robot profile as the state block reports it. applied_id is the running
// profile; the reported id may be newer (applying, or rejected while
// applied_id keeps running).
struct ProfileStatus {
    uint8_t  state      = 0;   // translagatr::ProfileState
    uint8_t  reason     = 0;   // translagatr::ProfileReason
    uint8_t  detail     = 0;
    uint32_t id         = 0;
    uint32_t applied_id = 0;   // 0 = none
};

// The Brain's latest PATH_REPORT, for inspection only. Field frame.
struct PathReport {
    struct Point {
        double x_m = 0.0;
        double y_m = 0.0;
    };

    uint32_t      session    = 0;
    uint32_t      command_id = 0;
    uint8_t       mode       = 0;   // translagatr::PathMode, 0 = no path
    uint8_t       count      = 0;
    std::array<Point, translagatr::kPathReportMaxPoints> points{};
    MonotonicTime received;
};

struct CommandState {
    uint32_t session = 0;   // current brain session, 0 = none

    // Brain link liveness, set by the brain_link commands slot every cycle.
    uint32_t      pi_instance = 0;       // this Pi on the link, 0 = no brain link
    bool          link_open   = false;   // the transport read without closing
    MonotonicTime last_request;          // host time of the newest decoded request

    uint64_t init_sequence = 0;   // 0 = never commanded
    uint32_t init_session  = 0;   // session that sent init_pose
    Pose2D   init_pose;

    // Target selection for target resolution, a command edge like init:
    // object_sequence increments once per new select or release, so a
    // repeated command never re-activates. Brain link v4 has no selection
    // op; only in-process callers set it.
    bool     object_requested = false;
    uint8_t  object_wire_id   = 0;
    uint64_t object_sequence  = 0;   // 0 = never commanded

    ProfileStatus profile;
    PathReport    path;

    BrainReplyContext reply;   // this cycle only
};

} // namespace navigatr
