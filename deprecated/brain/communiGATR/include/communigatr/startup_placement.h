// startup_placement.h
// When a program may place the robot at its configured starting pose on its
// own: once per program start, only after the link, the robot profile and the
// sensors are ready, and never again after that. A reconnect, a Pi restart or
// a lost placement later on needs an explicit placement from the operator;
// the robot has moved since the start and the starting pose no longer holds.

#pragma once
#include <cstdint>

#include "investigatr/geometry.h"

namespace communigatr
{

using investigatr::Seconds;

enum class StartupPolicy : uint8_t {
    kAlways,     // place at program start even if the Pi keeps a placement
    kIfUnplaced, // keep a placement the Pi already has, e.g. after a Brain restart
    kNever,      // operator places explicitly
};

struct StartupInputs {
    bool connected     = false;
    bool profile_ready = false; // applied, or no profile configured
    bool sensors_ready = false; // encoders and IMU fresh, IMU not calibrating
    bool calibrating   = false; // Pi calibration still running
    bool localized     = false; // Pi reports a placement
};

enum class StartupState : uint8_t {
    kWaiting,   // for readiness, within the wait
    kSubmit,    // place now
    kPlaced,    // submitted by this helper
    kKept,      // the Pi already had a placement (kIfUnplaced)
    kExpired,   // the wait passed first; place explicitly
    kDisabled,  // kNever
};

const char* toString(StartupState state);

class StartupPlacement {
public:
    StartupPlacement(StartupPolicy policy, Seconds wait);

    // Call every loop from program start. kSubmit: submit the starting pose
    // now, then call submitted() once the link accepts it. It stays kSubmit
    // until then, or until the wait passes.
    StartupState update(const StartupInputs& inputs, Seconds now);
    void         submitted() { state_ = StartupState::kPlaced; }

    StartupState state() const { return state_; }

private:
    StartupPolicy policy_;
    Seconds       wait_;
    bool          started_ = false;
    Seconds       start_   = 0;
    StartupState  state_   = StartupState::kWaiting;
};

} // namespace communigatr
