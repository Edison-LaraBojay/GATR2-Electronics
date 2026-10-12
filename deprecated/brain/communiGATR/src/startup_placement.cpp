// startup_placement.cpp

#include "communigatr/startup_placement.h"

namespace communigatr
{

const char* toString(StartupState state) {
    switch (state) {
    case StartupState::kWaiting: return "waiting for link, profile and sensors";
    case StartupState::kSubmit: return "placing";
    case StartupState::kPlaced: return "placed at start";
    case StartupState::kKept: return "kept the Pi placement";
    case StartupState::kExpired: return "startup wait over: place explicitly";
    case StartupState::kDisabled: return "place explicitly";
    }
    return "?";
}

StartupPlacement::StartupPlacement(StartupPolicy policy, Seconds wait)
    : policy_(policy), wait_(wait),
      state_(policy == StartupPolicy::kNever ? StartupState::kDisabled : StartupState::kWaiting) {}

StartupState StartupPlacement::update(const StartupInputs& in, Seconds now) {
    if (state_ != StartupState::kWaiting && state_ != StartupState::kSubmit) {
        return state_;
    }
    if (!started_) {
        started_ = true;
        start_   = now;
    }
    const bool ready = in.connected && in.profile_ready && in.sensors_ready && !in.calibrating;
    if (now - start_ > wait_) {
        state_ = StartupState::kExpired;
        return state_;
    }
    if (ready) {
        if (policy_ == StartupPolicy::kIfUnplaced && in.localized) {
            state_ = StartupState::kKept;
            return state_;
        }
        state_ = StartupState::kSubmit;
        return state_;
    }
    state_ = StartupState::kWaiting;
    return state_;
}

} // namespace communigatr
