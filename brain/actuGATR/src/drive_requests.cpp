// drive_requests.cpp

#include "actugatr/drive_requests.h"

namespace actugatr
{

CommandId DriveRequests::goTo(PlanMode mode, const Pose& destination,
                              const Reference& relative_to, const MoveOptions& options) {
    last_id_ = last_id_ == UINT32_MAX ? 1 : last_id_ + 1;
    DriveRequest r;
    r.kind        = DriveRequest::Kind::kGoTo;
    r.id          = last_id_;
    r.mode        = mode;
    r.destination = destination;
    r.relative_to = relative_to;
    r.options     = options;
    pending_      = r;
    return last_id_;
}

void DriveRequests::manual(const ManualDemand& demand, Seconds now) {
    DriveRequest r;
    r.kind   = DriveRequest::Kind::kManual;
    r.demand = demand;
    r.at     = now;
    pending_ = r;
}

void DriveRequests::stop() {
    pending_      = DriveRequest{};
    pending_.kind = DriveRequest::Kind::kStop;
}

DriveRequest DriveRequests::take() {
    const DriveRequest r = pending_;
    pending_             = DriveRequest{};
    return r;
}

DriveSnapshot DriveRequests::snapshot() const {
    DriveSnapshot s = published_;
    if (pending_.kind == DriveRequest::Kind::kGoTo) {
        s.motion            = MotionStatus{};
        s.motion.command_id = pending_.id;
        s.motion.state      = MotionState::kWaiting;
        s.motion.mode       = pending_.mode;
        s.mode              = DriveMode::kNavigate;
    }
    return s;
}

bool moving(const DriveSnapshot& s) {
    return s.motion.command_id != 0 && s.motion.state != MotionState::kIdle &&
           !isTerminal(s.motion.state);
}

bool sendManual(const ManualDemand& d, bool requested_this_cycle, const DriveSnapshot& s) {
    const bool sticks = d.forward != 0 || d.strafe != 0 || d.turn != 0;
    return sticks || (!requested_this_cycle && !moving(s));
}

void apply(const DriveRequest& request, DriveOwner& owner) {
    switch (request.kind) {
    case DriveRequest::Kind::kNone: break;
    case DriveRequest::Kind::kGoTo:
        owner.motionControl().setNextId(request.id);
        if (request.mode == PlanMode::kAvoiding) {
            owner.goToAvoiding(request.destination, request.relative_to, request.options);
        } else {
            owner.goToDirect(request.destination, request.relative_to, request.options);
        }
        break;
    case DriveRequest::Kind::kManual: owner.manual(request.demand, request.at); break;
    case DriveRequest::Kind::kStop: owner.cancel(); break;
    }
}

} // namespace actugatr
