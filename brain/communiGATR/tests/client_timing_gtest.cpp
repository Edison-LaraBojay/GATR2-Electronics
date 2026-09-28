// client_timing_gtest.cpp
// Bus ownership on the fake half-duplex bus with v4 frames up to 128 bytes:
// the Brain never transmits while a Pi reply can still start, every
// per-request timeout meets T >= A_req + W + A_rep + R + L from
// docs/interfaces.md, and state polling stays responsive while documents
// move.

#include "communigatr/client.h"

#include <algorithm>
#include <gtest/gtest.h>

#include "sim/link_rig.h"

using namespace communigatr;

namespace
{

constexpr Seconds kLimit     = 5.0;
constexpr Seconds kPiWindow  = 0.040; // Navigatr brain_link reply window default
constexpr Seconds kByte      = 10.0 / 115200;
constexpr Seconds kPiRelease = 0.0002; // post guard plus transmitter empty poll
constexpr Seconds kEps       = 1e-9;

uint8_t opOf(const Transmission& t) { return t.bytes.size() > 5 ? t.bytes[5] : 0; }

// Largest reply frame for a request op.
int maxReplyFrame(uint8_t op) {
    return gatr2::brainReplyMaxLen(op, gatr2::kResultOk) + gatr2::kLinkEnvelopeLen;
}

// Largest request frame per op.
int maxRequestFrame(uint8_t op) {
    return gatr2::brainRequestMaxLen(op) + gatr2::kLinkEnvelopeLen;
}

const uint8_t kOps[] = {gatr2::kOpHello,        gatr2::kOpSetPose,  gatr2::kOpGetState,
                        gatr2::kOpProfileWrite, gatr2::kOpProfileApply, gatr2::kOpReadDoc,
                        gatr2::kOpControl,      gatr2::kOpPathReport};

ClientConfig withProfile() {
    RobotProfile p;
    p.topology = LocalizationTopology::kThreeWheel;
    p.wheels   = {{0, 0.024, 2048, 0.1, 0.15, 0.0, false},
                  {1, 0.024, 2048, 0.1, -0.15, 0.0, true},
                  {2, 0.024, 2048, -0.12, 0.0, investigatr::kPi / 2, false}};
    p.imu_source = ImuSource::kPico;
    p.footprint  = {0.2, 0.2, 0.2, 0.2};
    p.cameras    = {{0, 0.1, 0.0, 0.3, 0.0, 0.0, 0.0}, {1, -0.1, 0.0, 0.3, 0.0, 0.0, 3.1},
                    {2, 0.0, 0.1, 0.3, 0.0, 0.0, 1.5}, {3, 0.0, -0.1, 0.3, 0.0, 0.0, -1.5}};
    ClientConfig config;
    config.profile = makeProfileDocument(p);
    return config;
}

} // namespace

TEST(ClientTiming, EveryOpTimeoutMeetsBudget) {
    const ClientConfig config;
    FakePi             pi;
    FakeBus            bus(pi);
    Client             client(bus.brainPort(), [] { return 1u; }, config);
    for (uint8_t op : kOps) {
        const int     request = maxRequestFrame(op);
        const int     reply   = maxReplyFrame(op);
        const Seconds timeout = client.responseTimeout(op, static_cast<uint16_t>(request));
        const Seconds latency = timeout - (request * kByte + kPiWindow + reply * kByte + kPiRelease);
        SCOPED_TRACE(int(op));
        EXPECT_GE(latency, 0.0124);
        EXPECT_GE(timeout, config.response_timeout);
    }
    // The v3 pair needs no allowance; 128 byte frames do.
    EXPECT_DOUBLE_EQ(client.responseTimeout(gatr2::kOpSetPose, 26), config.response_timeout);
    EXPECT_NEAR(client.responseTimeout(gatr2::kOpReadDoc, 22),
                config.response_timeout + (22 + 128 - 85) * kByte, 1e-12);
    EXPECT_NEAR(client.responseTimeout(gatr2::kOpProfileWrite, 128),
                config.response_timeout + (128 + 25 - 85) * kByte, 1e-12);
    EXPECT_EQ(maxReplyFrame(gatr2::kOpReadDoc), 128);
    EXPECT_EQ(maxRequestFrame(gatr2::kOpProfileWrite), 128);
    EXPECT_GE(config.request_gap, 0.005);
    EXPECT_GT(config.link_timeout, config.state_period + config.response_timeout);
}

// A reply anywhere in the Pi's window, with 10 ms of V5 latency, arrives
// before the timeout for every op, including 128 byte profile writes and
// document chunks: no retries, no collisions.
TEST(ClientTiming, ReplyAnywhereInWindowIsAccepted) {
    for (Seconds delay : {0.0, 0.015, 0.030, kPiWindow}) {
        FakeBusConfig bus;
        bus.reply_delay      = delay;
        bus.reply_window     = kPiWindow;
        bus.brain_tx_latency = 0.005;
        bus.brain_rx_latency = 0.005;
        LinkRig rig(withProfile(), bus);
        rig.pi.setProfileMode(true);
        rig.pi.setField(makeFakeField(12));
        ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
        ASSERT_TRUE(
            rig.runUntil([&] { return rig.client().field().generation != 0; }, 2 * kLimit));

        const PlacementTicket ticket = rig.client().submitPlacement(10, 20, 30);
        const ControlTicket   recal  = rig.client().recalibrate();
        const gatr2::PathPoint points[13] = {};
        rig.client().reportPath(1, gatr2::kPathAvoiding, points, 13);
        rig.run(1.0);

        SCOPED_TRACE(delay);
        EXPECT_EQ(rig.client().placementResult(ticket), PlacementResult::kApplied);
        EXPECT_EQ(rig.client().controlStatus(recal).state, ControlResult::kOk);
        EXPECT_TRUE(rig.pi.path().have);
        EXPECT_EQ(rig.client().stats().timeouts, 0u);
        EXPECT_EQ(rig.client().stats().resends, 0u);
        EXPECT_EQ(rig.client().stats().uncorrelated, 0u);
        EXPECT_EQ(rig.bus.silentReplies(), 0);
        EXPECT_FALSE(rig.bus.collision());
        EXPECT_GE(rig.client().stats().doc_chunks, 7u);
    }
}

// With the Pi silent, every next Brain frame starts after the latest moment a
// reply to the previous one could have ended, whatever its size.
TEST(ClientTiming, RetryStartsAfterLatestPossibleReply) {
    FakeBusConfig bus;
    bus.brain_tx_latency = 0.005;
    LinkRig rig(withProfile(), bus);
    rig.pi.setProfileMode(true);
    rig.pi.setField(makeFakeField(60));
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().fieldSync().map_reading; }, kLimit));
    rig.bus.setPiPresent(false);
    const std::size_t first = rig.bus.log().size();
    rig.client().submitPlacement(1, 2, 3);
    rig.client().reportPath(1, gatr2::kPathDirect, nullptr, 0);
    rig.run(1.5);

    const auto& log = rig.bus.log();
    ASSERT_GT(log.size(), first + 5);
    bool read_doc = false;
    for (std::size_t i = first + 1; i < log.size(); ++i) {
        ASSERT_TRUE(log[i].brain && log[i - 1].brain);
        read_doc      = read_doc || opOf(log[i - 1]) == gatr2::kOpReadDoc;
        const int max = maxReplyFrame(opOf(log[i - 1]));
        EXPECT_GE(log[i].start, log[i - 1].end + kPiWindow + max * kByte + kPiRelease)
            << "after op " << int(opOf(log[i - 1]));
    }
    EXPECT_TRUE(read_doc);
}

TEST(ClientTiming, GapAfterReplyPollPeriodAndTransfersOnlyWhenPollNotDue) {
    LinkRig rig(withProfile());
    rig.pi.setProfileMode(true);
    rig.pi.setField(makeFakeField(128));
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().generation != 0; }, kLimit));
    rig.run(1.0);
    const ClientConfig& config = rig.client().config();
    const auto&         log    = rig.bus.log();

    Seconds last_poll = -1;
    int     transfers = 0;
    for (std::size_t i = 0; i < log.size(); ++i) {
        const Transmission& t = log[i];
        if (!t.brain) {
            continue;
        }
        if (i > 0 && !log[i - 1].brain) {
            const Seconds reply_readable = log[i - 1].end - rig.bus.config().release;
            EXPECT_GE(t.start, reply_readable + config.request_gap - kEps);
        }
        const uint8_t op = opOf(t);
        if (op == gatr2::kOpGetState) {
            if (last_poll >= 0) {
                EXPECT_GE(t.start - last_poll, config.state_period - kEps);
            }
            last_poll = t.start;
        } else if (op == gatr2::kOpReadDoc || op == gatr2::kOpPathReport) {
            ++transfers;
            ASSERT_GE(last_poll, 0.0);
            EXPECT_LT(t.start, last_poll + config.state_period) << "transfer with the poll due";
        }
    }
    EXPECT_GT(transfers, 50);
    EXPECT_FALSE(rig.bus.collision());
}

// Pose polling while a 128 object map and its replaced estimates move: polls
// are never further apart than one period plus one 128 byte exchange.
TEST(ClientTiming, StatePollingStaysResponsiveDuringTransfers) {
    LinkRig rig(withProfile());
    rig.pi.setProfileMode(true);
    rig.pi.setField(makeFakeField(128));
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    Seconds       last      = -1;
    Seconds       worst     = 0;
    int           polls     = 0;
    std::size_t   seen      = rig.bus.log().size();
    Seconds       next_pub  = rig.now();
    const Seconds end       = rig.now() + 4.0;
    while (rig.now() < end) {
        rig.step();
        if (rig.now() >= next_pub) {
            rig.pi.publishNominalEstimate();
            next_pub = rig.now() + 0.4;
        }
        const auto& log = rig.bus.log();
        for (; seen < log.size(); ++seen) {
            if (log[seen].brain && opOf(log[seen]) == gatr2::kOpGetState) {
                if (last >= 0) {
                    worst = std::max(worst, log[seen].start - last);
                }
                last = log[seen].start;
                ++polls;
            }
        }
    }
    EXPECT_NE(rig.client().field().generation, 0u);
    EXPECT_GE(rig.client().stats().estimates, 2u);
    EXPECT_GT(rig.client().stats().doc_chunks, 60u);
    EXPECT_GT(polls, 100);
    // 20 ms period plus one 128 byte READ_DOC exchange and the request gap.
    EXPECT_LE(worst, 0.020 + 0.018 + 0.005);
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
