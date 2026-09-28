// pico_control.h
// Typed contract for the Pico acquisition link beyond sensor samples: its
// identity, IMU state, and the Pi -> Pico command channel. The pico_telemetry
// resource implements it; the commands and publishing slots use it.
//
// Thread safety: link() and request() may be called from any thread; submit()
// from the estimation worker only.

#pragma once
#include <cstdint>

#include "common/frames.h"
#include "core/time.h"

namespace navigatr
{

struct PicoLinkState {
    bool     frames_fresh = false; // sensor frames within the freshness window
    bool     identity     = false; // firmware sends v2 frames (boot and epochs known)
    uint16_t boot_id      = 0;
    uint8_t  acq_epoch    = 0;
    uint8_t  imu_epoch    = 0;
    uint64_t reboots      = 0; // boot_id changes seen since the Pi started
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
    kFailed,    // see detail
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
    // Returns a handle, or 0 when no v2 identity is known or the link is down.
    // The command keeps its request id through every resend, so a lost
    // acknowledgement never runs it twice. Bounded by timeout.
    virtual uint32_t submit(uint8_t op, uint8_t arg, MonotonicTime now, double timeout_s) = 0;

    virtual PicoRequestStatus request(uint32_t handle) const = 0;
};

} // namespace navigatr
