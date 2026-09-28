// simulated_state.cpp

#include "sim/simulated_state.h"

namespace actugatr
{

void SimulatedState::setRobot(const Pose& pose, Seconds now) {
    samples_.push_back(Sample{now, pose});
    while (samples_.size() > 256) {
        samples_.pop_front();
    }
}

void SimulatedState::setField(const investigatr::Field& field) {
    field_            = field;
    field_.generation = ++generation_;
    have_field_       = true;
}

void SimulatedState::clearField() {
    have_field_ = false;
}

investigatr::RobotState SimulatedState::robot(Seconds now) {
    investigatr::RobotState out;
    out.connected = connected_;
    out.link_age  = link_age_;
    out.status    = connected_ ? status_ : RobotStatus::kNoLink;
    const Sample* pick = nullptr;
    for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
        if (now - it->at >= latency_ - 1e-12) {
            pick = &*it;
            break;
        }
    }
    if (pick == nullptr) {
        if (out.status == RobotStatus::kValid) {
            out.status = RobotStatus::kNoPose;
        }
        return out;
    }
    if (out.status == RobotStatus::kValid) {
        out.pose  = pick->pose;
        out.age   = now - pick->at;
        out.frame = frame_;
    }
    return out;
}

bool SimulatedState::field(investigatr::Field& out) {
    if (!have_field_) {
        return false;
    }
    if (out.generation != field_.generation) {
        out = field_;
        ++copies_;
    }
    return true;
}

} // namespace actugatr
