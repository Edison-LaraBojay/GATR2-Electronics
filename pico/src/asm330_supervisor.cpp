// asm330_supervisor.cpp

#include "asm330_supervisor.h"

#include "frames.h"

namespace asm330
{

void Supervisor::start(uint32_t now_us) {
    state_    = State::Wait;
    since_us_ = now_us;
    hold_us_  = 0;
}

void Supervisor::restart(uint32_t now_us) {
    retry_.restart();
    start(now_us);
}

void Supervisor::stop() { state_ = State::Off; }

Action Supervisor::step(uint32_t now_us) {
    switch (state_) {
    case State::Off:
        return Action::None;

    case State::Wait:
        if (now_us - since_us_ < hold_us_) {
            return Action::None;
        }
        retry_.attempt();
        ++epoch_;
        state_    = State::Probe;
        since_us_ = now_us;
        return Action::Probe;

    case State::Probe:
        return Action::Probe;

    case State::Reset:
        if (now_us - since_us_ > kResetUs) {
            fail(now_us, gatr2::kPicoImuReasonBoot);
            return Action::None;
        }
        if (now_us - last_poll_us_ < kPollUs) {
            return Action::None;
        }
        last_poll_us_ = now_us;
        return Action::PollReset;

    case State::Configure:
        return Action::Configure;

    case State::Run:
        if (now_us - since_us_ >= imu::kStableUs) {
            retry_.stable();
        }
        if (now_us - last_check_us_ < kCheckUs) {
            return Action::None;
        }
        last_check_us_ = now_us;
        return Action::Check;
    }
    return Action::None;
}

void Supervisor::probed(uint32_t now_us, bool who_am_i_ok) {
    if (state_ != State::Probe) {
        return;
    }
    if (!who_am_i_ok) {
        fail(now_us, gatr2::kPicoImuReasonNoResponse);
        return;
    }
    state_        = State::Reset;
    since_us_     = now_us;
    last_poll_us_ = now_us;
}

void Supervisor::resetPolled(uint32_t now_us, bool cleared) {
    if (state_ != State::Reset) {
        return;
    }
    if (cleared) {
        state_ = State::Configure;
    } else if (now_us - since_us_ > kResetUs) {
        fail(now_us, gatr2::kPicoImuReasonBoot);
    }
}

void Supervisor::configured(uint32_t now_us, bool ok) {
    if (state_ != State::Configure) {
        return;
    }
    if (!ok) {
        fail(now_us, gatr2::kPicoImuReasonFeatures);
        return;
    }
    state_          = State::Run;
    since_us_       = now_us;
    last_check_us_  = now_us;
    check_failures_ = 0;
}

void Supervisor::checked(uint32_t now_us, uint8_t reason) {
    if (state_ != State::Run) {
        return;
    }
    if (reason == gatr2::kPicoImuReasonNone) {
        check_failures_ = 0;
        return;
    }
    if (++check_failures_ >= kCheckFailures) {
        fail(now_us, reason);
    }
}

void Supervisor::fail(uint32_t now_us, uint8_t reason) {
    hold_us_  = retry_.failed(reason);
    state_    = State::Wait;
    since_us_ = now_us;
}

} // namespace asm330
