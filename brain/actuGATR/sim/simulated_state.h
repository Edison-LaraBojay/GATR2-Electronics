// simulated_state.h
// Host StateSource fed with truth: pose samples with latency, status, link
// state, frame, and a settable complete field.

#pragma once
#include <deque>

#include "investigatr/state_source.h"

namespace actugatr
{

using investigatr::FrameGeneration;
using investigatr::Pose;
using investigatr::RobotStatus;
using investigatr::Seconds;

class SimulatedState : public investigatr::StateSource {
public:
    // Truth at time now. robot() reports the newest sample at least latency
    // old, aged from its sample time.
    void setRobot(const Pose& pose, Seconds now);
    void setLatency(Seconds latency) { latency_ = latency; }
    void setStatus(RobotStatus status) { status_ = status; }
    void setConnected(bool connected) { connected_ = connected; }
    void setLinkAge(Seconds age) { link_age_ = age; }
    void setFrame(FrameGeneration frame) { frame_ = frame; }

    // Publishes a new complete field generation.
    void setField(const investigatr::Field& field);
    void clearField();

    investigatr::RobotState robot(Seconds now) override;
    bool                    field(investigatr::Field& out) override;

    int fieldCopies() const { return copies_; }

private:
    struct Sample {
        Seconds at = 0;
        Pose    pose;
    };

    std::deque<Sample> samples_;
    Seconds            latency_   = 0;
    RobotStatus        status_    = RobotStatus::kValid;
    bool               connected_ = true;
    Seconds            link_age_  = 0;
    FrameGeneration    frame_     = 1;

    investigatr::Field field_;
    bool               have_field_ = false;
    uint32_t           generation_ = 0;
    int                copies_     = 0;
};

} // namespace actugatr
