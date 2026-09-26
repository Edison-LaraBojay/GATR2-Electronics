// bno08x_supervisor_gtest.cpp

#include <gtest/gtest.h>

#include "bno08x_supervisor.h"
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

// Boot, enable, and ack from start(t). Returns the time Run was entered.
uint32_t bringUp(Supervisor& s, uint32_t t) {
    s.start(t);
    EXPECT_EQ(s.step(t + 100000, resetEvent()), Action::Enable);
    EXPECT_EQ(s.step(t + 101000, ackEvent()), Action::None);
    EXPECT_EQ(s.state(), State::Run);
    return t + 101000;
}

// Held since t for hold_us: nothing until the hold ends, then release.
void expectHold(Supervisor& s, uint32_t t, uint32_t hold_us) {
    EXPECT_EQ(s.state(), State::Reset);
    EXPECT_EQ(s.step(t + hold_us - 1, resetEvent()), Action::None);
    EXPECT_EQ(s.step(t + hold_us, Events{}), Action::ReleaseReset);
    EXPECT_EQ(s.state(), State::Boot);
}

} // namespace

TEST(Bno08xSupervisor, AbsentImuRetriesWithGrowingCappedBackoff) {
    Supervisor s;
    uint32_t   t = 0xFFF00000u; // crosses the 32-bit wrap
    s.start(t);

    const uint32_t holds[] = {500000, 1000000, 2000000, 4000000, 8000000, 8000000, 8000000};
    for (uint32_t hold : holds) {
        EXPECT_EQ(s.step(t + kBootUs, Events{}), Action::None);
        t += kBootUs + 1;
        EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
        expectHold(s, t, hold);
        t += hold;
    }
    EXPECT_EQ(s.backoffUs(), kBackoffMaxUs);
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
}

TEST(Bno08xSupervisor, AckTimeoutFails) {
    Supervisor s;
    s.start(0);
    EXPECT_EQ(s.step(1000, resetEvent()), Action::Enable);
    EXPECT_EQ(s.step(1000 + kAckUs, reportEvent()), Action::None);
    EXPECT_EQ(s.step(1000 + kAckUs + 1, Events{}), Action::HoldReset);
    expectHold(s, 1000 + kAckUs + 1, kBackoffMinUs);
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

TEST(Bno08xSupervisor, HubResetWhileRunningReenablesAndDropsSamples) {
    Supervisor s;
    uint32_t   t = bringUp(s, 0);

    GyroWindow w;
    w.add(t + 1000, 50);
    t += 2000;
    const Action a = s.step(t, resetEvent());
    EXPECT_EQ(a, Action::Enable);
    EXPECT_EQ(s.state(), State::WaitAck);
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
    uint32_t   t = bringUp(s, 0);

    t += 50000;
    EXPECT_EQ(s.step(t, reportEvent()), Action::None);
    EXPECT_EQ(s.step(t + kStaleUs, Events{}), Action::None);
    EXPECT_EQ(s.step(t + kStaleUs + 1, Events{}), Action::HoldReset);
    expectHold(s, t + kStaleUs + 1, kBackoffMinUs);
}

TEST(Bno08xSupervisor, BackoffResetsAfterStableRun) {
    Supervisor s;
    uint32_t   t = 0;
    s.start(t);

    // Two boot failures, next hold 2 s.
    t += kBootUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 500000);
    t += 500000 + kBootUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 1000000);
    t += 1000000;
    EXPECT_EQ(s.backoffUs(), 2000000u);

    // Boot succeeds and runs past kStableUs with steady reports.
    const uint32_t run = bringUp(s, t);
    for (t = run; t - run < kStableUs;) {
        t += 5000;
        EXPECT_EQ(s.step(t, reportEvent()), Action::None);
    }
    EXPECT_EQ(s.backoffUs(), kBackoffMinUs);

    t += kStaleUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, kBackoffMinUs);
}

TEST(Bno08xSupervisor, ShortRunKeepsBackoff) {
    Supervisor s;
    uint32_t   t = kBootUs + 1;
    s.start(0);
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 500000);
    t += 500000;

    t = bringUp(s, t);
    t += 500000;
    EXPECT_EQ(s.step(t, reportEvent()), Action::None);
    EXPECT_EQ(s.backoffUs(), 1000000u);

    t += kStaleUs + 1;
    EXPECT_EQ(s.step(t, Events{}), Action::HoldReset);
    expectHold(s, t, 1000000);
}
