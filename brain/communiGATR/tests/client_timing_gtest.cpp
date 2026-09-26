// client_timing_gtest.cpp
// Bus ownership on the fake half-duplex bus: the Brain never transmits while
// a Pi reply can still start, and the default timeout meets the budget
// T >= A_req + W + A_rep + R + L from docs/interfaces.md.

#include "communigatr/client.h"

#include <gtest/gtest.h>

#include "sim/link_rig.h"

using namespace communigatr;

namespace
{

constexpr Seconds kLimit      = 3.0;
constexpr Seconds kPiWindow   = 0.040; // Navigatr brain_link reply window default
constexpr Seconds kByte       = 10.0 / 115200;
constexpr Seconds kMaxRequest = 26 * kByte; // SET_POSE
constexpr Seconds kMaxReply   = 59 * kByte; // GET_STATE Ok
constexpr Seconds kPiRelease  = 0.0002;     // post guard plus transmitter empty poll
constexpr Seconds kEps        = 1e-9;

void openSession(LinkRig& rig) {
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
}

} // namespace

TEST(ClientTiming, DefaultTimeoutMeetsBudget) {
    const ClientConfig config;
    const Seconds      latency_allowed =
        config.response_timeout - (kMaxRequest + kPiWindow + kMaxReply + kPiRelease);
    EXPECT_GE(latency_allowed, 0.012);
    EXPECT_GE(config.request_gap, 0.005);
    EXPECT_GT(config.link_timeout, config.state_period + config.response_timeout);
}

// A reply anywhere in the Pi's window, with 10 ms of V5 latency, arrives
// before the timeout: no retries, no collisions.
TEST(ClientTiming, ReplyAnywhereInWindowIsAccepted) {
    for (Seconds delay : {0.0, 0.005, 0.015, 0.025, 0.035, kPiWindow}) {
        FakeBusConfig bus;
        bus.reply_delay      = delay;
        bus.reply_window     = kPiWindow;
        bus.brain_tx_latency = 0.005;
        bus.brain_rx_latency = 0.005;
        LinkRig rig({}, bus);
        rig.pi.setLandmark(3, FakeLandmark{});
        openSession(rig);

        const PlacementTicket ticket = rig.client().submitPlacement(10, 20, 30);
        rig.client().selectLandmark(3);
        rig.run(1.0);

        SCOPED_TRACE(delay);
        EXPECT_EQ(rig.client().placementResult(ticket), PlacementResult::kApplied);
        EXPECT_EQ(rig.client().selection(), SelectionState::kActive);
        EXPECT_EQ(rig.client().stats().timeouts, 0u);
        EXPECT_EQ(rig.client().stats().resends, 0u);
        EXPECT_EQ(rig.client().stats().uncorrelated, 0u);
        EXPECT_EQ(rig.bus.silentReplies(), 0);
        EXPECT_FALSE(rig.bus.collision());
    }
}

// With the Pi silent, every next Brain frame starts after the latest moment a
// reply to the previous one could have ended.
TEST(ClientTiming, RetryStartsAfterLatestPossibleReply) {
    FakeBusConfig bus;
    bus.brain_tx_latency = 0.005;
    LinkRig rig({}, bus);
    openSession(rig);
    rig.bus.setPiPresent(false);
    const std::size_t first = rig.bus.log().size();
    rig.client().submitPlacement(1, 2, 3);
    rig.run(1.0);

    const auto& log = rig.bus.log();
    ASSERT_GT(log.size(), first + 5);
    for (std::size_t i = first + 1; i < log.size(); ++i) {
        ASSERT_TRUE(log[i].brain && log[i - 1].brain);
        EXPECT_GE(log[i].start, log[i - 1].end + kPiWindow + kMaxReply + kPiRelease);
    }
}

TEST(ClientTiming, GapAfterReplyAndPollPeriod) {
    LinkRig rig;
    openSession(rig);
    rig.run(1.0);
    const ClientConfig& config = rig.client().config();
    const auto&         log    = rig.bus.log();

    Seconds last_poll = -1;
    for (std::size_t i = 0; i < log.size(); ++i) {
        const Transmission& t = log[i];
        if (!t.brain) {
            continue;
        }
        if (i > 0 && !log[i - 1].brain) {
            const Seconds reply_readable = log[i - 1].end - rig.bus.config().release;
            EXPECT_GE(t.start, reply_readable + config.request_gap - kEps);
        }
        if (t.bytes[5] == gatr2::kOpGetState) {
            if (last_poll >= 0) {
                EXPECT_GE(t.start - last_poll, config.state_period - kEps);
            }
            last_poll = t.start;
        }
    }
    EXPECT_FALSE(rig.bus.collision());
}

// The collision check has teeth: a timeout shorter than the Pi's reply delay
// lets the Brain retry into a late reply.
TEST(ClientTiming, ShortTimeoutCollides) {
    ClientConfig config;
    config.response_timeout = 0.015;
    config.request_gap      = 0;
    FakeBusConfig bus;
    bus.reply_delay = 0.0135;
    LinkRig rig(config, bus);
    rig.run(0.5);
    EXPECT_TRUE(rig.bus.collision());
}
