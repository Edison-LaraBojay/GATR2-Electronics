// link_events_gtest.cpp
// Recovery history from successive link snapshots.

#include "communigatr/link_events.h"

#include <gtest/gtest.h>

using namespace communigatr;

namespace
{

LinkSnapshot up(uint32_t session = 1, uint32_t pi = 7) {
    LinkSnapshot s;
    s.connected   = true;
    s.session     = session;
    s.pi_instance = pi;
    s.profile     = ProfileSync::kApplied;
    s.state_valid = true;
    s.localized   = true;
    s.health      = translagatr::kHealthPicoLink;
    s.calibration = translagatr::kCalibrationNone;
    return s;
}

} // namespace

TEST(LinkEvents, FirstConnectionIsNotARestore) {
    LinkEvents e;
    e.update(up(), 0.0);
    for (std::size_t i = 0; i < e.size(); ++i) {
        EXPECT_NE(e.at(i).event, LinkEvent::kLinkRestored);
        EXPECT_NE(e.at(i).event, LinkEvent::kLinkLost);
    }
}

TEST(LinkEvents, OutageIsTimed) {
    LinkEvents e;
    e.update(up(), 0.0);
    LinkSnapshot down = up();
    down.connected    = false;
    e.update(down, 1.0);
    ASSERT_GE(e.size(), 1u);
    EXPECT_EQ(e.at(0).event, LinkEvent::kLinkLost);
    e.update(up(), 3.5);
    EXPECT_EQ(e.at(0).event, LinkEvent::kLinkRestored);
    EXPECT_DOUBLE_EQ(e.at(0).duration, 2.5);
    EXPECT_DOUBLE_EQ(e.at(0).at, 3.5);
}

TEST(LinkEvents, SessionAndPiRestartAreDistinguished) {
    LinkEvents e;
    e.update(up(1, 7), 0.0);
    e.update(up(2, 7), 1.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kNewSession);
    e.update(up(3, 9), 2.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kPiRestarted);
}

TEST(LinkEvents, PlacementProfileAndOdometry) {
    LinkEvents   e;
    LinkSnapshot s = up();
    s.localized    = false;
    s.profile      = ProfileSync::kApplying;
    e.update(s, 0.0);
    s.profile = ProfileSync::kApplied;
    e.update(s, 1.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kProfileApplied);
    s.localized = true;
    e.update(s, 2.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kPlaced);
    s.localized      = false;
    s.odometry_epoch = 1;
    e.update(s, 3.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kOdometryReset);
    EXPECT_EQ(e.at(1).event, LinkEvent::kPlacementLost);
    s.profile = ProfileSync::kRejected;
    e.update(s, 4.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kProfileRejected);
}

TEST(LinkEvents, PicoAndImuHealth) {
    LinkEvents   e;
    LinkSnapshot s = up();
    e.update(s, 0.0);
    s.health = 0;
    e.update(s, 1.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kPicoLost);
    s.health = translagatr::kHealthPicoLink | translagatr::kHealthImuFailed;
    e.update(s, 2.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kImuFailed);
    EXPECT_EQ(e.at(1).event, LinkEvent::kPicoRestored);
    s.health = translagatr::kHealthPicoLink;
    e.update(s, 3.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kImuReady);
}

TEST(LinkEvents, CalibrationProgressWithoutRepeatsForWaiting) {
    LinkEvents   e;
    LinkSnapshot s = up();
    e.update(s, 0.0);
    s.calibration = translagatr::kCalibrationRunning;
    e.update(s, 1.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kCalibrating);
    const uint32_t before = e.total();
    s.calibration         = translagatr::kCalibrationWaitingStill;
    e.update(s, 1.5);
    s.calibration = translagatr::kCalibrationRunning;
    e.update(s, 2.0);
    EXPECT_EQ(e.total(), before);
    s.calibration = translagatr::kCalibrationDone;
    e.update(s, 4.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kCalibrated);
    s.calibration = translagatr::kCalibrationFailed;
    e.update(s, 5.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kCalibrationFailed);
}

TEST(LinkEvents, StateFieldsIgnoredWithoutAStateReply) {
    LinkEvents   e;
    LinkSnapshot s = up();
    e.update(s, 0.0);
    const uint32_t before = e.total();
    LinkSnapshot   no_state = s;
    no_state.state_valid    = false;
    no_state.localized      = false;
    no_state.health         = 0;
    e.update(no_state, 1.0);
    EXPECT_EQ(e.total(), before);
}

TEST(LinkEvents, RingKeepsTheNewest) {
    LinkEvents   e;
    LinkSnapshot s = up();
    e.update(s, 0.0);
    const uint32_t before = e.total();
    for (int i = 0; i < 40; ++i) {
        s.connected = (i % 2) == 1;
        e.update(s, 1.0 + i);
    }
    EXPECT_EQ(e.size(), LinkEvents::kCapacity);
    EXPECT_EQ(e.total(), before + 40u);
    EXPECT_DOUBLE_EQ(e.at(0).at, 40.0);
}

TEST(LinkEvents, SessionGapHidesNothing) {
    LinkEvents e;
    e.update(up(1, 7), 0.0);

    // Client between sessions: no session, no state sample.
    LinkSnapshot gap;
    gap.pi_instance = 0;
    e.update(gap, 1.0);
    const uint32_t before = e.total();

    // Same Pi, new session: only the session is new.
    e.update(up(2, 7), 2.0);
    bool session = false;
    for (std::size_t i = 0; i < e.total() - before; ++i) {
        session = session || e.at(i).event == LinkEvent::kNewSession;
    }
    EXPECT_TRUE(session);

    // Pi restart through a gap: the new Pi starts unplaced.
    e.update(gap, 3.0);
    LinkSnapshot restarted = up(3, 9);
    restarted.localized    = false;
    const uint32_t mark    = e.total();
    e.update(restarted, 4.0);
    bool pi = false, lost = false, new_session = false;
    for (std::size_t i = 0; i < e.total() - mark; ++i) {
        pi          = pi || e.at(i).event == LinkEvent::kPiRestarted;
        lost        = lost || e.at(i).event == LinkEvent::kPlacementLost;
        new_session = new_session || e.at(i).event == LinkEvent::kNewSession;
    }
    EXPECT_TRUE(pi);
    EXPECT_TRUE(lost);
    EXPECT_FALSE(new_session);
}

TEST(LinkEvents, CalibratingEntersThroughAnyWaitingState) {
    LinkEvents   e;
    LinkSnapshot s = up();
    s.calibration  = translagatr::kCalibrationDone;
    e.update(s, 0.0);
    s.calibration = translagatr::kCalibrationWaitingData;
    e.update(s, 1.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kCalibrating);
    s.calibration = translagatr::kCalibrationDone;
    e.update(s, 2.0);
    s.calibration = translagatr::kCalibrationWaitingStill;
    e.update(s, 3.0);
    EXPECT_EQ(e.at(0).event, LinkEvent::kCalibrating);

    // The first state already calibrating counts as the start.
    LinkEvents   f;
    LinkSnapshot first = up();
    first.calibration  = translagatr::kCalibrationWaitingStill;
    f.update(first, 0.0);
    ASSERT_GE(f.size(), 1u);
    EXPECT_EQ(f.at(0).event, LinkEvent::kCalibrating);
}
