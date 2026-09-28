// bno08x_supervisor_gtest.cpp

#include <gtest/gtest.h>

#include "bno08x_supervisor.h"
#include "frames.h"
#include "gyro_window.h"

using namespace bno08x;

namespace
{

Events resetEvent() {
    Events e;
    e.reset = true;
    return e;
}

Events ackEvent() {
    Events e;
    e.ack = true;
    return e;
}

Events reportEvent() {
    Events e;
    e.report = true;
    return e;
}

Events openFailedEvent() {
    Events e;
    e.open_failed = true;
    return e;
}

// Boot, enable and ack from a Boot state entered at t. Returns the time Run
// was entered.
uint32_t bootToRun(Supervisor& s, uint32_t t) {
    EXPECT_EQ(s.state(), State::Boot);
    EXPECT_EQ(s.step(t + 100000, resetEvent()), Action::Enable);
    EXPECT_EQ(s.step(t + 101000, ackEvent()), Action::None);
    EXPECT_EQ(s.state(), State::Run);
    return t + 101000;
}

// Held since t for hold_us: nothing until the hold ends, then release.
void expectHold(Supervisor& s, uint32_t t, uint32_t hold_us) {
    EXPECT_EQ(s.state(), State::Reset);
    EXPECT_EQ(s.holdUs(), hold_us);
    EXPECT_EQ(s.step(t + hold_us - 1, resetEvent()), Action::None);
    EXPECT_EQ(s.step(t + hold_us, Events{}), Action::ReleaseReset);
    EXPECT_EQ(s.state(), State::Boot);
}

// Steady reports from t for at least imu::kStableUs. Returns the last time.
uint32_t runStable(Supervisor& s, uint32_t t) {
    const uint32_t run = t;
    while (t - run < imu::kStableUs) {
        t += 5000;
        EXPECT_EQ(s.step(t, reportEvent()), Action::None);
    }
    return t;
}

} // namespace

TEST(Bno08xSupervisor, AbsentImuRetriesFiveTimesQuicklyThenSlowly) {
    Supervisor s;
    uint32_t   t = 0xFFF00000u; // crosses the 32-bit wrap
    s.start(t);

    const uint32_t holds[] = {500000, 1000000, 2000000, 4000000, 30000000, 30000000, 30000000};
    for (uint32_t hold : holds) {
        EXPECT_EQ(s.step(t + kBootUs, Events{}), Action::None);
        t += kBootUs + 1;
        EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
        expectHold(s, t, hold);
        t += hold;
    }
    EXPECT_TRUE(s.retry().exhausted());
    EXPECT_EQ(s.retry().attempts(), 8);
    EXPECT_EQ(s.retry().reason(), gatr2::kPicoImuReasonBoot);
}

TEST(Bno08xSupervisor, BootEnableAckRun) {
    Supervisor s;
    s.start(1000);
    EXPECT_EQ(s.state(), State::Boot);
    EXPECT_EQ(s.step(2000, Events{}), Action::None);
    EXPECT_EQ(s.state(), State::Boot);

    EXPECT_EQ(s.step(95000, resetEvent()), Action::Enable);
    EXPECT_EQ(s.state(), State::WaitAck);
    EXPECT_EQ(s.step(96000, Events{}), Action::None);
    EXPECT_EQ(s.state(), State::WaitAck);

    EXPECT_EQ(s.step(97000, ackEvent()), Action::None);
    EXPECT_EQ(s.state(), State::Run);
    EXPECT_EQ(s.step(98000, reportEvent()), Action::None);
    EXPECT_EQ(s.state(), State::Run);
    EXPECT_EQ(s.epoch(), 1);
    EXPECT_EQ(s.retry().attempts(), 1);
}

TEST(Bno08xSupervisor, AckTimeoutFails) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.step(1000, resetEvent()), Action::Enable);
    EXPECT_EQ(s.step(1000 + kAckUs, reportEvent()), Action::None);
    EXPECT_EQ(s.step(1000 + kAckUs + 1, Events{}), Action::HoldReset);
    expectHold(s, 1000 + kAckUs + 1, imu::kBackoffMinUs);
    EXPECT_EQ(s.retry().reason(), gatr2::kPicoImuReasonFeatures);
}

TEST(Bno08xSupervisor, EnableErrorFails) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.step(1000, resetEvent()), Action::Enable);
    Events e;
    e.enable_failed = true;
    EXPECT_EQ(s.step(2000, e), Action::HoldReset);
    EXPECT_EQ(s.state(), State::Reset);
}

TEST(Bno08xSupervisor, FailedOpenIsRetriedLikeAnyAttempt) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.step(1000, openFailedEvent()), Action::HoldReset);
    EXPECT_EQ(s.retry().reason(), gatr2::kPicoImuReasonNoResponse);
    expectHold(s, 1000, imu::kBackoffMinUs);

    // The driver opens again on this release; now it succeeds.
    const uint32_t run = bootToRun(s, 1000 + imu::kBackoffMinUs);
    EXPECT_EQ(s.retry().attempts(), 2);
    runStable(s, run);
    EXPECT_FALSE(s.retry().exhausted());
}

TEST(Bno08xSupervisor, HubResetWhileRunningReenablesAndDropsSamples) {
    Supervisor s;
    s.start(0);
    uint32_t      t     = bootToRun(s, 0);
    const uint8_t epoch = s.epoch();

    GyroWindow w;
    w.add(t + 1000, 50);
    t += 2000;
    const Action a = s.step(t, resetEvent());
    EXPECT_EQ(a, Action::Enable);
    EXPECT_EQ(s.state(), State::WaitAck);
    EXPECT_EQ(s.epoch(), static_cast<uint8_t>(epoch + 1));
    ASSERT_TRUE(dropsSamples(a));
    w.clear();
    int32_t mdps = 0;
    EXPECT_FALSE(w.take(t, mdps));

    EXPECT_EQ(s.step(t + 1000, ackEvent()), Action::None);
    EXPECT_EQ(s.state(), State::Run);
}

TEST(Bno08xSupervisor, OnlyResetAndEnableDropSamples) {
    EXPECT_TRUE(dropsSamples(Action::HoldReset));
    EXPECT_TRUE(dropsSamples(Action::Enable));
    EXPECT_FALSE(dropsSamples(Action::None));
    EXPECT_FALSE(dropsSamples(Action::ReleaseReset));
}

TEST(Bno08xSupervisor, StaleReportsFail) {
    Supervisor s;
    s.start(0);
    uint32_t t = bootToRun(s, 0);

    t += 50000;
    EXPECT_EQ(s.step(t, reportEvent()), Action::None);
    EXPECT_EQ(s.step(t + kStaleUs, Events{}), Action::None);
    EXPECT_EQ(s.step(t + kStaleUs + 1, Events{}), Action::HoldReset);
    expectHold(s, t + kStaleUs + 1, imu::kBackoffMinUs);
    EXPECT_EQ(s.retry().reason(), gatr2::kPicoImuReasonStream);
}

TEST(Bno08xSupervisor, StableRunEndsTheEpisode) {
    Supervisor s;
    uint32_t   t = 0;
    s.start(t);

    // Two boot failures: the next hold would be 2 s.
    t += kBootUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 500000);
    t += 500000 + kBootUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 1000000);
    t += 1000000;

    // The third attempt boots and runs past kStableUs with steady reports.
    t = runStable(s, bootToRun(s, t));
    EXPECT_EQ(s.retry().attempts(), 3);
    EXPECT_EQ(s.retry().reason(), gatr2::kPicoImuReasonNone);

    // Losing the stable device starts a new episode with the shortest hold.
    t += kStaleUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, imu::kBackoffMinUs);
    EXPECT_EQ(s.retry().attempts(), 1);
}

TEST(Bno08xSupervisor, ShortRunCountsAsAFailedAttempt) {
    Supervisor s;
    uint32_t   t = kBootUs + 1;
    s.start(0);
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 500000);
    t += 500000;

    t = bootToRun(s, t);
    t += 500000;
    EXPECT_EQ(s.step(t, reportEvent()), Action::None);

    t += kStaleUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 1000000);
}

TEST(Bno08xSupervisor, RestartPulsesResetAndStartsANewEpisode) {
    Supervisor s;
    uint32_t   t = 0;
    s.start(t);
    for (int i = 0; i < imu::kQuickAttempts; ++i) {
        t += kBootUs + 1;
        EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
        t += s.holdUs();
        EXPECT_EQ(s.step(t, Events{}), Action::ReleaseReset);
    }
    EXPECT_TRUE(s.retry().exhausted());
    const uint8_t epoch = s.epoch();

    t += 1000;
    EXPECT_EQ(s.restart(t), Action::HoldReset);
    EXPECT_EQ(s.epoch(), static_cast<uint8_t>(epoch + 1));
    EXPECT_FALSE(s.retry().exhausted());
    EXPECT_EQ(s.retry().attempts(), 0);
    expectHold(s, t, kRestartHoldUs);
    EXPECT_EQ(s.retry().attempts(), 1);
    bootToRun(s, t + kRestartHoldUs);
}

TEST(Bno08xSupervisor, StopHoldsResetUntilRestart) {
    Supervisor s;
    s.start(0);
    uint32_t t = bootToRun(s, 0);

    EXPECT_EQ(s.stop(), Action::HoldReset);
    EXPECT_EQ(s.state(), State::Off);
    for (int i = 0; i < 10; ++i) {
        t += 10000000;
        EXPECT_EQ(s.step(t, resetEvent()), Action::None);
    }
    EXPECT_EQ(s.state(), State::Off);

    EXPECT_EQ(s.restart(t), Action::HoldReset);
    expectHold(s, t, kRestartHoldUs);
}

TEST(Bno08xSupervisor, EpochCountsEveryReinitialization) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.epoch(), 1);

    // A failure starts a reinitialization; the release does not add another.
    uint32_t t = kBootUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    EXPECT_EQ(s.epoch(), 2);
    t += s.holdUs();
    EXPECT_EQ(s.step(t, Events{}), Action::ReleaseReset);
    EXPECT_EQ(s.epoch(), 2);

    // The boot's own reset event continues the same initialization.
    t = bootToRun(s, t);
    EXPECT_EQ(s.epoch(), 2);
}
