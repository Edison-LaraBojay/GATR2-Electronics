// bno08x_supervisor.cpp

#include "bno08x_supervisor.h"

#include "frames.h"

namespace bno08x
{

namespace
{

bool elapsed(uint32_t now_us, uint32_t since_us, uint32_t us) { return now_us - since_us > us; }

} // namespace

void Supervisor::start(uint32_t now_us) {
    ++epoch_;
    release(now_us);
}

Action Supervisor::restart(uint32_t now_us) {
    retry_.restart();
    ++epoch_;
    state_    = State::Reset;
    since_us_ = now_us;
    hold_us_  = kRestartHoldUs;
    return Action::HoldReset;
}

Action Supervisor::stop() {
    state_ = State::Off;
    return Action::HoldReset;
}

Action Supervisor::step(uint32_t now_us, const Events& ev) {
    switch (state_) {
    case State::Off:
        return Action::None;

    case State::Reset:
        if (now_us - since_us_ >= hold_us_) {
            release(now_us);
            return Action::ReleaseReset;
        }
        return Action::None;

    case State::Boot:
        if (ev.open_failed) {
            return fail(now_us, translagatr::kPicoImuReasonNoResponse);
        }
        if (ev.reset) {
            return enable(now_us);
        }
        if (elapsed(now_us, since_us_, kBootUs)) {
            return fail(now_us, translagatr::kPicoImuReasonBoot);
        }
        return Action::None;

    case State::WaitAck:
        if (ev.reset) {
            return enable(now_us);
        }
        if (ev.enable_failed) {
            return fail(now_us, translagatr::kPicoImuReasonFeatures);
        }
        if (ev.ack) {
            state_          = State::Run;
            since_us_       = now_us;
            last_report_us_ = now_us;
            return Action::None;
        }
        if (elapsed(now_us, since_us_, kAckUs)) {
            return fail(now_us, translagatr::kPicoImuReasonFeatures);
        }
        return Action::None;

    case State::Run:
        if (ev.reset) {
            return enable(now_us);
        }
        if (ev.report) {
            last_report_us_ = now_us;
        } else if (elapsed(now_us, last_report_us_, kStaleUs)) {
            return fail(now_us, translagatr::kPicoImuReasonStream);
        }
        if (now_us - since_us_ >= imu::kStableUs) {
            retry_.stable();
        }
        return Action::None;
    }
    return Action::None;
}

void Supervisor::release(uint32_t now_us) {
    retry_.attempt();
    state_    = State::Boot;
    since_us_ = now_us;
}

Action Supervisor::fail(uint32_t now_us, uint8_t reason) {
    ++epoch_;
    hold_us_  = retry_.failed(reason);
    state_    = State::Reset;
    since_us_ = now_us;
    return Action::HoldReset;
}

Action Supervisor::enable(uint32_t now_us) {
    // A hub reset after boot restarts the report stream.
    if (state_ == State::WaitAck || state_ == State::Run) {
        ++epoch_;
    }
    state_    = State::WaitAck;
    since_us_ = now_us;
    return Action::Enable;
}

} // namespace bno08x
