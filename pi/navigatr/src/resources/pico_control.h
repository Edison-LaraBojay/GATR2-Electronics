// pico_control.h
// Typed contract for the Pico acquisition link beyond sensor samples: its
// identity, IMU state, and the Pi -> Pico command channel. The pico_telemetry
// resource implements it; the commands and publishing slots use it.
//
// Thread safety: every call may come from any thread. Frames are read and
// written only by the owning resource on the estimation worker.
//
// Commands: request ids start at a random base per Pi process. A command is
// resent every 200 ms, under the same request id, until a status frame of
// its boot names it; completion and failure come only from those reports.
// A Pico reboot fails every unsettled command of the old boot (PicoLink).

#pragma once
#include <cstdint>

#include "common/frames.h"
#include "core/time.h"

namespace navigatr
{

struct PicoLinkState {
    bool     frames_fresh = false; // a sensor frame within 250 ms, serial link open
    bool     identity     = false; // firmware sends v2 frames (boot and epochs known)
    uint16_t boot_id      = 0;
    uint8_t  acq_epoch    = 0;
    uint8_t  imu_epoch    = 0;
    uint64_t reboots      = 0; // since the Pi started: boot_id, frame version or clock regression
    uint64_t restarts     = 0; // acq_epoch changes seen
    uint64_t imu_restarts = 0; // imu_epoch changes seen

    bool              status_known = false; // a status frame of the current boot arrived
    gatr2::PicoStatus status;
    MonotonicTime     last_frame;  // host clock
    MonotonicTime     last_status; // host clock
};

enum class PicoRequestState : uint8_t {
    kUnknown,   // no such handle
    kSending,   // resent until the Pico reports this request
    kRunning,   // the Pico reports it running
    kCompleted, // the Pico reports it completed
    kFailed,    // detail: PicoLink, ImuAbsent, PicoRefused or TimedOut
};

struct PicoRequestStatus {
    PicoRequestState state  = PicoRequestState::kUnknown;
    uint8_t          detail = gatr2::kControlDetailNone; // ControlDetail on failure
};

class PicoControl
{
public:
    virtual ~PicoControl() = default;

    virtual PicoLinkState link() const = 0;

    // Starts one command (a PicoOp and its body byte) for the current boot.
    // Returns a handle, or 0 when no v2 identity is known, the link is down,
    // the op is unknown or timeout_s is not positive. The command keeps its
    // request id through every resend, so a lost acknowledgement never runs
    // it twice. Bounded by timeout (clamped to an hour): Failed with TimedOut
    // once the Pico has reported it, else with PicoLink.
    virtual uint32_t submit(uint8_t op, uint8_t arg, MonotonicTime now, double timeout_s) = 0;

    virtual PicoRequestStatus request(uint32_t handle) const = 0;
};

} // namespace navigatr
