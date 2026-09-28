// asm330_supervisor_gtest.cpp

#include <gtest/gtest.h>

#include "asm330_supervisor.h"
#include "frames.h"

using namespace asm330;

namespace
{

// Probe, reset and configure from a due attempt at t. Returns the time Run
// was entered.
uint32_t bringUp(Supervisor& s, uint32_t t) {
    EXPECT_EQ(s.step(t), Action::Probe);
    s.probed(t, true);
    EXPECT_EQ(s.state(), State::Reset);
    t += kPollUs;
    EXPECT_EQ(s.step(t), Action::PollReset);
    s.resetPolled(t, true);
    EXPECT_EQ(s.step(t), Action::Configure);
    s.configured(t, true);
    EXPECT_TRUE(s.running());
    return t;
}

// A due attempt at t whose WHO_AM_I is wrong. Returns the wait it set.
uint32_t failProbe(Supervisor& s, uint32_t t) {
    EXPECT_EQ(s.step(t), Action::Probe);
    s.probed(t, false);
    EXPECT_EQ(s.state(), State::Wait);
    return s.holdUs();
}

} // namespace

TEST(Asm330Supervisor, BringsUpWithoutBlocking) {
    Supervisor s;
    s.start(1000);
    EXPECT_EQ(s.state(), State::Wait);
    bringUp(s, 1000);
    EXPECT_EQ(s.epoch(), 1);
    EXPECT_EQ(s.retry().attempts(), 1);
}

TEST(Asm330Supervisor, WrongWhoAmIIsRetriedNotPermanent) {
    Supervisor s;
    uint32_t   t = 0;
    s.start(t);

    const uint32_t waits[] = {500000, 1000000, 2000000, 4000000, 30000000, 30000000};
    for (uint32_t wait : waits) {
        EXPECT_EQ(failProbe(s, t), wait);
        EXPECT_EQ(s.step(t + wait - 1), Action::None);
        t += wait;
    }
    EXPECT_TRUE(s.retry().exhausted());
    EXPECT_EQ(s.retry().reason(), translagatr::kPicoImuReasonNoResponse);

    // The chip appears: the next slow attempt brings it up.
    t = bringUp(s, t);
    EXPECT_EQ(s.retry().attempts(), 7);
    EXPECT_EQ(s.epoch(), 7);
}

TEST(Asm330Supervisor, ResetPollIsRateLimitedAndBounded) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.step(0), Action::Probe);
    s.probed(0, true);
    EXPECT_EQ(s.step(kPollUs - 1), Action::None);
    EXPECT_EQ(s.step(kPollUs), Action::PollReset);
    s.resetPolled(kPollUs, false);
    EXPECT_EQ(s.step(kPollUs + 1), Action::None);

    uint32_t t = kPollUs;
    while (t <= kResetUs) {
        t += kPollUs;
        const Action a = s.step(t);
        if (a == Action::PollReset) {
            s.resetPolled(t, false);
        }
    }
    EXPECT_EQ(s.state(), State::Wait);
    EXPECT_EQ(s.holdUs(), imu::kBackoffMinUs);
    EXPECT_EQ(s.retry().reason(), translagatr::kPicoImuReasonBoot);
}

TEST(Asm330Supervisor, ConfigureReadbackMismatchFails) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.step(0), Action::Probe);
    s.probed(0, true);
    EXPECT_EQ(s.step(kPollUs), Action::PollReset);
    s.resetPolled(kPollUs, true);
    EXPECT_EQ(s.step(kPollUs), Action::Configure);
    s.configured(kPollUs, false);
    EXPECT_EQ(s.state(), State::Wait);
    EXPECT_EQ(s.retry().reason(), translagatr::kPicoImuReasonFeatures);
}

TEST(Asm330Supervisor, HealthCheckFailureRestartsTheDevice) {
    Supervisor s;
    s.start(0);
    uint32_t t = bringUp(s, 0);

    EXPECT_EQ(s.step(t + kCheckUs - 1), Action::None);
    t += kCheckUs;
    EXPECT_EQ(s.step(t), Action::Check);
    s.checked(t, translagatr::kPicoImuReasonNone);
    EXPECT_TRUE(s.running());

    // Past kStableUs: failures are the loss of a stable device.
    t += imu::kStableUs;
    EXPECT_EQ(s.step(t), Action::Check);
    s.checked(t, translagatr::kPicoImuReasonStream);
    EXPECT_TRUE(s.running());
    t += kCheckUs;
    EXPECT_EQ(s.step(t), Action::Check);
    s.checked(t, translagatr::kPicoImuReasonStream);
    EXPECT_EQ(s.state(), State::Wait);
    EXPECT_EQ(s.holdUs(), imu::kBackoffMinUs);
    EXPECT_EQ(s.retry().reason(), translagatr::kPicoImuReasonStream);

    const uint8_t epoch = s.epoch();
    bringUp(s, t + imu::kBackoffMinUs);
    EXPECT_EQ(s.epoch(), static_cast<uint8_t>(epoch + 1));
}

TEST(Asm330Supervisor, OneBadCheckIsTolerated) {
    Supervisor s;
    s.start(0);
    uint32_t t = bringUp(s, 0);
    for (int i = 0; i < 6; ++i) {
        t += kCheckUs;
        EXPECT_EQ(s.step(t), Action::Check);
        s.checked(t, i % 2 == 0 ? translagatr::kPicoImuReasonNoResponse : translagatr::kPicoImuReasonNone);
        EXPECT_TRUE(s.running()) << i;
    }
    EXPECT_EQ(s.epoch(), 1);
}

TEST(Asm330Supervisor, RestartAndStop) {
    Supervisor s;
    s.start(0);
    uint32_t t = 0;
    for (int i = 0; i < imu::kQuickAttempts; ++i) {
        t += failProbe(s, t);
    }
    ASSERT_TRUE(s.retry().exhausted());

    s.stop();
    EXPECT_EQ(s.state(), State::Off);
    EXPECT_EQ(s.step(t + 100000000u), Action::None);

    s.restart(t);
    EXPECT_FALSE(s.retry().exhausted());
    EXPECT_EQ(s.retry().attempts(), 0);
    bringUp(s, t);
    EXPECT_EQ(s.retry().attempts(), 1);
}

TEST(Asm330Supervisor, LateResultsAreIgnored) {
    Supervisor s;
    s.start(0);
    bringUp(s, 0);
    s.probed(10, false);
    s.resetPolled(10, false);
    s.configured(10, false);
    EXPECT_TRUE(s.running());
}
