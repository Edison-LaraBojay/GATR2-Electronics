// straight_planner.cpp

#include "sim/straight_planner.h"

#include <cmath>

namespace actugatr
{
namespace
{

using investigatr::PathSegment;
using investigatr::Pose;
using investigatr::SegmentKind;
using investigatr::wrapAngle;

void addTurn(investigatr::Path& path, const Pose& at, double heading) {
    const double change = wrapAngle(heading - at.heading);
    if (std::fabs(change) < 1e-3) {
        return;
    }
    PathSegment s;
    s.kind           = SegmentKind::kTurn;
    s.start          = at;
    s.end            = at;
    s.end.heading    = wrapAngle(heading);
    s.turn_direction = change >= 0 ? 1 : -1;
    path.segments.push_back(s);
}

} // namespace

investigatr::PlanResult StraightPlanner::plan(const investigatr::PlanRequest& request) const {
    ++plans_;
    last_ = request;
    investigatr::PlanResult out;
    out.status   = forced_;
    out.blocking = blocking_;
    if (forced_ != investigatr::PlanStatus::kOk) {
        return out;
    }
    if (request.mode == investigatr::PlanMode::kAvoiding && request.field == nullptr) {
        out.status = investigatr::PlanStatus::kNoField;
        return out;
    }
    out.path.mode     = request.mode;
    const Pose& a     = request.start;
    const Pose& b     = request.goal;
    const double dist = std::hypot(b.x - a.x, b.y - a.y);
    if (request.model.holonomic) {
        if (dist > 1e-3) {
            PathSegment s;
            s.kind  = SegmentKind::kTranslate;
            s.start = a;
            s.end   = b;
            out.path.segments.push_back(s);
        } else {
            addTurn(out.path, a, b.heading);
        }
        return out;
    }
    Pose at = a;
    if (dist > 1e-3) {
        const double travel  = std::atan2(b.y - a.y, b.x - a.x);
        const bool   reverse = request.allow_reverse && request.model.reverse &&
                             std::fabs(wrapAngle(travel - a.heading)) > 0.5 * investigatr::kPi;
        const double facing = reverse ? wrapAngle(travel + investigatr::kPi) : travel;
        addTurn(out.path, at, facing);
        at.heading = facing;
        PathSegment s;
        s.kind    = SegmentKind::kTranslate;
        s.start   = at;
        s.end     = Pose{b.x, b.y, facing};
        s.reverse = reverse;
        out.path.segments.push_back(s);
        at = s.end;
    }
    addTurn(out.path, at, b.heading);
    return out;
}

bool StraightPlanner::clear(const investigatr::Path&, const investigatr::Field&,
                            const investigatr::MotionModel&) const {
    ++clears_;
    return clear_;
}

} // namespace actugatr
