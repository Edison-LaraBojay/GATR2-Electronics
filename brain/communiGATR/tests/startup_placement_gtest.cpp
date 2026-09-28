// startup_placement_gtest.cpp
// Program-start placement happens once, only when ready, and never after.

#include "communigatr/startup_placement.h"

#include <gtest/gtest.h>

using namespace communigatr;

namespace
{

StartupInputs ready(bool localized = false) {
    StartupInputs in;
    in.connected     = true;
    in.profile_ready = true;
    in.sensors_ready = true;
    in.calibrating   = false;
    in.localized     = localized;
    return in;
}

} // namespace

TEST(StartupPlacement, WaitsForEveryConditionThenPlacesOnce) {
    StartupPlacement p(StartupPolicy::kAlways, 10.0);
    StartupInputs    in = ready();
    in.sensors_ready    = false;
    EXPECT_EQ(p.update(in, 0.0), StartupState::kWaiting);
    in             = ready();
    in.calibrating = true;
    EXPECT_EQ(p.update(in, 1.0), StartupState::kWaiting);
    in.calibrating   = false;
    in.profile_ready = false;
    EXPECT_EQ(p.update(in, 2.0), StartupState::kWaiting);
    EXPECT_EQ(p.update(ready(), 3.0), StartupState::kSubmit);
    // Until the caller reports the submission, it stays due.
    EXPECT_EQ(p.update(ready(), 3.1), StartupState::kSubmit);
    p.submitted();
    EXPECT_EQ(p.update(ready(), 3.2), StartupState::kPlaced);
    // Losing and regaining the link never places again.
    StartupInputs lost = ready();
    lost.connected     = false;
    EXPECT_EQ(p.update(lost, 4.0), StartupState::kPlaced);
    EXPECT_EQ(p.update(ready(), 5.0), StartupState::kPlaced);
}

TEST(StartupPlacement, ReadinessLostBeforeSubmittingWaitsAgain) {
    StartupPlacement p(StartupPolicy::kAlways, 10.0);
    EXPECT_EQ(p.update(ready(), 0.0), StartupState::kSubmit);
    StartupInputs lost = ready();
    lost.connected     = false;
    EXPECT_EQ(p.update(lost, 0.1), StartupState::kWaiting);
    EXPECT_EQ(p.update(ready(), 0.2), StartupState::kSubmit);
}

TEST(StartupPlacement, ExpiresInsteadOfPlacingLate) {
    StartupPlacement p(StartupPolicy::kAlways, 10.0);
    StartupInputs    in = ready();
    in.connected        = false;
    EXPECT_EQ(p.update(in, 0.0), StartupState::kWaiting);
    EXPECT_EQ(p.update(in, 10.5), StartupState::kExpired);
    EXPECT_EQ(p.update(ready(), 11.0), StartupState::kExpired);
}

TEST(StartupPlacement, IfUnplacedKeepsAnExistingPlacement) {
    StartupPlacement keep(StartupPolicy::kIfUnplaced, 10.0);
    EXPECT_EQ(keep.update(ready(true), 0.0), StartupState::kKept);
    StartupPlacement place(StartupPolicy::kIfUnplaced, 10.0);
    EXPECT_EQ(place.update(ready(false), 0.0), StartupState::kSubmit);
    StartupPlacement always(StartupPolicy::kAlways, 10.0);
    EXPECT_EQ(always.update(ready(true), 0.0), StartupState::kSubmit);
}

TEST(StartupPlacement, NeverIsDisabled) {
    StartupPlacement p(StartupPolicy::kNever, 10.0);
    EXPECT_EQ(p.update(ready(), 0.0), StartupState::kDisabled);
}
