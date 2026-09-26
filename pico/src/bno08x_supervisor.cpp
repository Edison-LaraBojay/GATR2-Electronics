// bno08x_supervisor.cpp

#include "bno08x_supervisor.h"

namespace bno08x
{

namespace
{

bool elapsed(uint32_t now_us, uint32_t since_us, uint32_t us) { return now_us - since_us > us; }

} // namespace

void Supervisor::start(uint32_t now_us) {
    state_    = State::Boot;
    since_us_ = now_us;
}

Action Supervisor::step(uint32_t now_us, const Events& ev) {
    switch (state_) {
    case State::Reset:
        if (now_us - since_us_ >= hold_us_) {
            start(now_us);
            return Action::ReleaseReset;
        }
        return Action::None;

    case State::Boot:
        if (ev.reset) {
            return enable(now_us);
        }
        if (elapsed(now_us, since_us_, kBootUs)) {
            return fail(now_us);
        }
        return Action::None;

    case State::WaitAck:
        if (ev.reset) {
            return enable(now_us);
        }
        if (ev.enable_failed) {
            return fail(now_us);
        }
        if (ev.ack) {
            state_          = State::Run;
            since_us_       = now_us;
            last_report_us_ = now_us;
            return Action::None;
        }
        if (elapsed(now_us, since_us_, kAckUs)) {
            return fail(now_us);
        }
        return Action::None;

    case State::Run:
        if (ev.reset) {
            return enable(now_us);
        }
        if (ev.report) {
            last_report_us_ = now_us;
        } else if (elapsed(now_us, last_report_us_, kStaleUs)) {
            return fail(now_us);
        }
        if (now_us - since_us_ >= kStableUs) {
            backoff_us_ = kBackoffMinUs;
        }
        return Action::None;
    }
    return Action::None;
}

Action Supervisor::fail(uint32_t now_us) {
    state_      = State::Reset;
    since_us_   = now_us;
    hold_us_    = backoff_us_;
    backoff_us_ = backoff_us_ >= kBackoffMaxUs / 2 ? kBackoffMaxUs : backoff_us_ * 2;
    return Action::HoldReset;
}

Action Supervisor::enable(uint32_t now_us) {
    state_    = State::WaitAck;
    since_us_ = now_us;
    return Action::Enable;
}

} // namespace bno08x
