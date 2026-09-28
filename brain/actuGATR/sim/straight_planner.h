// straight_planner.h
// Host test planner: direct paths shaped like the geometric planner's (turn,
// translate, turn for non-holonomic; one translate with a heading change for
// holonomic), no obstacle checks. A forced status and clear() answer let
// tests drive Motion through failures and replans.

#pragma once

#include "investigatr/planner.h"

namespace actugatr
{

class StraightPlanner : public investigatr::PathPlanner {
public:
    investigatr::PlanResult plan(const investigatr::PlanRequest& request) const override;
    bool clear(const investigatr::Path& path, const investigatr::Field& field,
               const investigatr::MotionModel& model) const override;

    void force(investigatr::PlanStatus status, investigatr::ObjectId blocking = 0) {
        forced_   = status;
        blocking_ = blocking;
    }
    void setClear(bool clear) { clear_ = clear; }

    int plans() const { return plans_; }
    int clears() const { return clears_; }
    const investigatr::PlanRequest& lastRequest() const { return last_; }

private:
    investigatr::PlanStatus          forced_   = investigatr::PlanStatus::kOk;
    investigatr::ObjectId            blocking_ = 0;
    bool                             clear_    = true;
    mutable int                      plans_    = 0;
    mutable int                      clears_   = 0;
    mutable investigatr::PlanRequest last_;
};

} // namespace actugatr
