// client_telemetry_gtest.cpp
// TELEMETRY: the fake Pi's rules, and the client on both transports:
// the target bit carried, latest wins, one send per period on a grid, on a
// fast link never the turn of a due state poll, on a slow link late but never
// starved, a lost report never resent, and an older or refusing Pi stops it
// for the session without dropping the session.

#include "communigatr/client.h"

#include <algorithm>
#include <gtest/gtest.h>
#include <vector>

#include "sim/link_rig.h"

using namespace communigatr;

namespace
{

constexpr Seconds kLimit = 3.0;

translagatr::BrainTelemetry sample(int i) {
    translagatr::BrainTelemetry t;
    t.flags = translagatr::kTelemetryAttitude | translagatr::kTelemetryMotion |
              translagatr::kTelemetryWheels | translagatr::kTelemetryTarget;
    t.stamp_ms            = 1000u + static_cast<uint32_t>(i);
    t.roll_cdeg           = static_cast<int16_t>(-150 + i);
    t.pitch_cdeg          = static_cast<int16_t>(220 - i);
    t.command_id          = 7u + static_cast<uint32_t>(i);
    t.motion_state        = 2;
    t.motion_reason       = 20;
    t.plan_mode           = 1;
    t.segment             = 3;
    t.segment_count       = 5;
    t.target_x_mm         = 1200 + i;
    t.target_y_mm         = -1800;
    t.target_heading_cdeg = -9000;
    t.cmd_vx_mm_s         = 640;
    t.cmd_vy_mm_s         = -12;
    t.cmd_omega_cdeg_s    = 5730;
    t.cross_track_mm      = -31;
    t.distance_error_mm   = 812;
    t.heading_error_cdeg  = 1500;
    t.drive_fault         = 0;
    t.wheel_count         = 2;
    t.wheel_rpm_x10[0]    = 4521;
    t.wheel_rpm_x10[1]    = -3310;
    return t;
}

void expectSame(const translagatr::BrainTelemetry& a, const translagatr::BrainTelemetry& b) {
    EXPECT_EQ(a.flags, b.flags);
    EXPECT_EQ(a.stamp_ms, b.stamp_ms);
    EXPECT_EQ(a.roll_cdeg, b.roll_cdeg);
    EXPECT_EQ(a.pitch_cdeg, b.pitch_cdeg);
    EXPECT_EQ(a.command_id, b.command_id);
    EXPECT_EQ(a.motion_state, b.motion_state);
    EXPECT_EQ(a.motion_reason, b.motion_reason);
    EXPECT_EQ(a.plan_mode, b.plan_mode);
    EXPECT_EQ(a.segment, b.segment);
    EXPECT_EQ(a.segment_count, b.segment_count);
    EXPECT_EQ(a.target_x_mm, b.target_x_mm);
    EXPECT_EQ(a.target_y_mm, b.target_y_mm);
    EXPECT_EQ(a.target_heading_cdeg, b.target_heading_cdeg);
    EXPECT_EQ(a.cmd_vx_mm_s, b.cmd_vx_mm_s);
    EXPECT_EQ(a.cmd_vy_mm_s, b.cmd_vy_mm_s);
    EXPECT_EQ(a.cmd_omega_cdeg_s, b.cmd_omega_cdeg_s);
    EXPECT_EQ(a.cross_track_mm, b.cross_track_mm);
    EXPECT_EQ(a.distance_error_mm, b.distance_error_mm);
    EXPECT_EQ(a.heading_error_cdeg, b.heading_error_cdeg);
    EXPECT_EQ(a.drive_fault, b.drive_fault);
    EXPECT_EQ(a.wheel_count, b.wheel_count);
    for (int i = 0; i < translagatr::kTelemetryWheelsMax; ++i) {
        EXPECT_EQ(a.wheel_rpm_x10[i], b.wheel_rpm_x10[i]);
    }
}

translagatr::BrainRequest hello(uint16_t id, uint32_t nonce) {
    translagatr::BrainRequest q;
    q.op         = translagatr::kOpHello;
    q.request_id = id;
    q.nonce      = nonce;
    return q;
}

translagatr::BrainRequest telemetry(uint32_t session, uint16_t id, int i) {
    translagatr::BrainRequest q;
    q.op         = translagatr::kOpTelemetry;
    q.session    = session;
    q.request_id = id;
    q.telemetry  = sample(i);
    return q;
}

std::size_t saw(const FakePi& pi, uint8_t op) {
    std::size_t n = 0;
    for (const translagatr::BrainRequest& r : pi.requests()) {
        n += r.op == op ? 1 : 0;
    }
    return n;
}

uint8_t opOf(const Transmission& t) { return t.bytes.size() > 5 ? t.bytes[5] : 0; }

std::string name(const testing::TestParamInfo<RigTransport>& info) {
    return info.param == RigTransport::kUsb ? "Usb" : "Rs485";
}

class Telemetry : public testing::TestWithParam<RigTransport> {
protected:
    Telemetry() : rig(ClientConfig{}, FakeBusConfig{}, GetParam()) {}

    void open() { ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit)); }

    // GET_STATE requests the Pi decodes in duration, with a report every
    // report_every seconds (0: none).
    std::size_t pollsWith(Seconds duration, Seconds report_every) {
        const std::size_t before = saw(rig.pi, translagatr::kOpGetState);
        const Seconds     end    = rig.now() + duration;
        Seconds           next   = rig.now();
        int               i      = 0;
        while (rig.now() < end) {
            if (report_every > 0 && rig.now() >= next) {
                rig.client().reportTelemetry(sample(i++));
                next += report_every;
            }
            rig.step();
        }
        return saw(rig.pi, translagatr::kOpGetState) - before;
    }

    LinkRig rig;
};

} // namespace

// ---------------------------------------------------------------------------
// Fake Pi
// ---------------------------------------------------------------------------

TEST(FakePiTelemetry, KeptAnsweredOkAndNeverTouchesTheRobot) {
    FakePi         pi;
    const uint32_t s = pi.answer(hello(1, 5)).session;
    pi.robot().x_mm  = 250;
    translagatr::BrainReply r = pi.answer(telemetry(s, 2, 1));
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(pi.telemetryKept(), 1);
    expectSame(pi.telemetry(), sample(1));
    EXPECT_EQ(pi.robot().x_mm, 250);
    EXPECT_EQ(pi.placementsApplied(), 0);

    // A repeat is answered again and kept once; the same id with another
    // body is refused.
    EXPECT_EQ(pi.answer(telemetry(s, 2, 1)).result, translagatr::kResultOk);
    EXPECT_EQ(pi.telemetryKept(), 1);
    EXPECT_EQ(pi.answer(telemetry(s, 2, 2)).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(pi.answer(telemetry(0x55, 3, 2)).result, translagatr::kResultUnknownSession);

    // Header only on the wire.
    std::vector<uint8_t> frame(translagatr::kMaxFrameLen);
    r = pi.answer(telemetry(s, 4, 3));
    frame.resize(translagatr::encodeBrainReply(r, frame.data(), translagatr::kMaxFrameLen));
    EXPECT_EQ(frame.size(), std::size_t{translagatr::kBrainReplyHeaderLen + translagatr::kLinkEnvelopeLen});
    expectSame(pi.telemetry(), sample(3));

    pi.restart(0x77);
    EXPECT_EQ(pi.telemetry().flags, 0);
}

TEST(FakePiTelemetry, OldPiAndRefusingPi) {
    FakePi old;
    old.setUnsupportedOp(translagatr::kOpTelemetry);
    const uint32_t s = old.answer(hello(1, 5)).session;
    EXPECT_EQ(old.answer(telemetry(s, 2, 1)).result, translagatr::kResultUnsupportedOp);
    EXPECT_EQ(old.telemetryKept(), 0);
    // Nothing recorded: the next request id is still new.
    translagatr::BrainRequest state;
    state.op         = translagatr::kOpGetState;
    state.session    = s;
    state.request_id = 2;
    EXPECT_EQ(old.answer(state).result, translagatr::kResultOk);

    FakePi refusing;
    refusing.setTelemetryResult(translagatr::kResultInvalidArgument);
    const uint32_t r = refusing.answer(hello(1, 5)).session;
    EXPECT_EQ(refusing.answer(telemetry(r, 2, 1)).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(refusing.telemetryKept(), 0);
}

// ---------------------------------------------------------------------------
// Client, both transports
// ---------------------------------------------------------------------------

TEST_P(Telemetry, ArrivesIntactAndLatestWins) {
    Client& client = rig.client();
    EXPECT_FALSE(client.reportTelemetry(sample(1))); // no session
    EXPECT_EQ(client.stats().telemetry_dropped, 1u);
    open();

    EXPECT_TRUE(client.reportTelemetry(sample(2)));
    EXPECT_TRUE(client.reportTelemetry(sample(3)));
    EXPECT_EQ(client.stats().telemetry_replaced, 1u);
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 1; }, kLimit));
    expectSame(rig.pi.telemetry(), sample(3));
    rig.run(0.5);
    EXPECT_EQ(saw(rig.pi, translagatr::kOpTelemetry), 1u);
    EXPECT_EQ(client.stats().telemetry_reports, 1u);
    EXPECT_EQ(client.stats().unexpected, 0u);
    EXPECT_EQ(client.stats().timeouts, 0u);
    EXPECT_FALSE(client.telemetryUnsupported());
}

// kTelemetryTarget reaches the Pi as sent: set with a target at the field
// origin, clear with no destination.
TEST_P(Telemetry, TheTargetBitArrivesAsSent) {
    Client& client = rig.client();
    open();

    translagatr::BrainTelemetry t; // target_* all 0
    t.flags        = translagatr::kTelemetryMotion | translagatr::kTelemetryTarget;
    t.stamp_ms     = 500;
    t.command_id   = 11;
    t.motion_state = 4; // completed
    EXPECT_TRUE(client.reportTelemetry(t));
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 1; }, kLimit));
    expectSame(rig.pi.telemetry(), t);

    t.flags        = translagatr::kTelemetryMotion;
    t.stamp_ms     = 600;
    t.command_id   = 12;
    t.motion_state = 1; // waiting
    EXPECT_TRUE(client.reportTelemetry(t));
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 2; }, kLimit));
    expectSame(rig.pi.telemetry(), t);
    EXPECT_EQ(client.stats().telemetry_dropped, 0u);
}

// Reports every step go out once per period; one report per period, as the
// programs send, is never replaced by drift.
TEST_P(Telemetry, OnePerPeriodOnAGrid) {
    open();
    const ClientStats& stats = rig.client().stats();
    pollsWith(3.0, 0.001);
    EXPECT_GE(stats.telemetry_reports, 29u);
    EXPECT_LE(stats.telemetry_reports, 31u);
    EXPECT_GT(stats.telemetry_replaced, 2500u);

    const uint32_t sent     = stats.telemetry_reports;
    const uint32_t replaced = stats.telemetry_replaced;
    rig.run(0.3);
    pollsWith(5.0, 0.1);
    EXPECT_GE(stats.telemetry_reports - sent, 49u);
    EXPECT_LE(stats.telemetry_reports - sent, 51u);
    EXPECT_EQ(stats.telemetry_replaced, replaced);
    EXPECT_EQ(stats.timeouts, 0u);
}

// Telemetry at its fastest costs at most 15% of the state polls; on these
// fakes about 6 to 10% (one TELEMETRY exchange per period).
TEST_P(Telemetry, StatePollsKeepTheirRate) {
    open();
    const std::size_t without = pollsWith(3.0, 0);
    const std::size_t with    = pollsWith(3.0, 0.001);
    EXPECT_GT(without, 120u);
    EXPECT_GE(with * 100, without * 85) << with << " polls vs " << without;
    EXPECT_GE(rig.client().stats().telemetry_reports, 29u);
}

TEST_P(Telemetry, OldPiStopsItForTheSessionAndKeepsTheSession) {
    rig.pi.setUnsupportedOp(translagatr::kOpTelemetry);
    open();
    Client&        client  = rig.client();
    const uint32_t session = client.session();
    EXPECT_TRUE(client.reportTelemetry(sample(1)));
    ASSERT_TRUE(rig.runUntil([&] { return client.telemetryUnsupported(); }, kLimit));

    EXPECT_EQ(client.session(), session);
    EXPECT_TRUE(client.ready());
    EXPECT_TRUE(client.connected(rig.now()));
    EXPECT_EQ(client.error(), LinkError::kNone);
    EXPECT_EQ(client.stats().session_losses, 0u);
    EXPECT_EQ(client.stats().telemetry_refused, 1u);
    EXPECT_FALSE(client.reportTelemetry(sample(2)));

    const std::size_t polls = saw(rig.pi, translagatr::kOpGetState);
    rig.run(1.0);
    EXPECT_EQ(saw(rig.pi, translagatr::kOpTelemetry), 1u);
    EXPECT_GT(saw(rig.pi, translagatr::kOpGetState), polls + 40);
    EXPECT_EQ(client.session(), session);
    EXPECT_EQ(client.stats().timeouts, 0u);

    // A new session tries once more.
    rig.pi.restart(0x0C0FFEE0);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.ready() && client.session() != session; }, kLimit));
    EXPECT_FALSE(client.telemetryUnsupported());
    EXPECT_TRUE(client.reportTelemetry(sample(3)));
    ASSERT_TRUE(rig.runUntil([&] { return client.telemetryUnsupported(); }, kLimit));
    EXPECT_EQ(saw(rig.pi, translagatr::kOpTelemetry), 2u);
    EXPECT_EQ(client.stats().telemetry_refused, 2u);
    EXPECT_TRUE(client.ready());
}

TEST_P(Telemetry, InvalidArgumentAlsoStopsItForTheSession) {
    rig.pi.setTelemetryResult(translagatr::kResultInvalidArgument);
    open();
    Client&        client  = rig.client();
    const uint32_t session = client.session();
    EXPECT_TRUE(client.reportTelemetry(sample(1)));
    ASSERT_TRUE(rig.runUntil([&] { return client.telemetryUnsupported(); }, kLimit));
    EXPECT_EQ(client.session(), session);
    EXPECT_EQ(client.stats().session_losses, 0u);
    EXPECT_EQ(rig.pi.telemetryKept(), 0);
    EXPECT_FALSE(client.reportTelemetry(sample(2)));
    rig.run(0.5);
    EXPECT_EQ(saw(rig.pi, translagatr::kOpTelemetry), 1u);
    EXPECT_TRUE(client.ready());
}

// A report of one session never goes out in the next.
TEST_P(Telemetry, PiRestartDropsTheUnsentReport) {
    open();
    Client&        client  = rig.client();
    const uint32_t session = client.session();
    rig.pi.restart(0x0D15EA5E);
    EXPECT_TRUE(client.reportTelemetry(sample(1)));
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.ready() && client.session() != session; }, kLimit));
    const uint32_t fresh = client.session();
    rig.run(0.5);
    for (const translagatr::BrainRequest& r : rig.pi.requests()) {
        if (r.op == translagatr::kOpTelemetry) {
            EXPECT_NE(r.session, fresh);
        }
    }
    EXPECT_EQ(rig.pi.telemetryKept(), 0);
    EXPECT_TRUE(client.reportTelemetry(sample(2)));
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 1; }, kLimit));
    expectSame(rig.pi.telemetry(), sample(2));
}

// A report held through an outage would reach the Pi looking fresh: it is
// dropped two periods after it was made.
TEST_P(Telemetry, AReportHeldThroughAnOutageIsDropped) {
    open();
    Client& client = rig.client();
    auto    cut    = [&](bool off) {
        if (GetParam() == RigTransport::kUsb) {
            rig.usb.setPlugged(!off);
        } else {
            rig.bus.setPiPresent(!off);
        }
    };
    cut(true);
    rig.run(0.5);
    const std::size_t before = saw(rig.pi, translagatr::kOpTelemetry);
    EXPECT_TRUE(client.reportTelemetry(sample(1)));
    rig.run(1.0);
    cut(false);
    ASSERT_TRUE(rig.runUntil([&] { return client.connected(rig.now()); }, kLimit));
    rig.run(0.3);
    EXPECT_EQ(saw(rig.pi, translagatr::kOpTelemetry), before);
    EXPECT_EQ(rig.pi.telemetryKept(), 0);
    EXPECT_EQ(client.stats().telemetry_dropped, 1u);

    EXPECT_TRUE(client.reportTelemetry(sample(2)));
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 1; }, kLimit));
    expectSame(rig.pi.telemetry(), sample(2));
}

INSTANTIATE_TEST_SUITE_P(Transports, Telemetry,
                         testing::Values(RigTransport::kRs485, RigTransport::kUsb), name);

// ---------------------------------------------------------------------------
// Client on slow links, both transports: every exchange outlasts the poll's
// idle window, so the state poll is due in every slot after a state reply
// ---------------------------------------------------------------------------

namespace
{

FakeUsbConfig slowUsb() {
    FakeUsbConfig c;
    c.turnaround = 0.011;
    return c;
}

FakeBusConfig slowBus() {
    FakeBusConfig c;
    c.reply_delay = 0.008;
    return c;
}

struct SlowRun {
    std::vector<Seconds> sends;            // TELEMETRY send times
    Seconds              start        = 0;
    Seconds              end          = 0;
    std::size_t          polls        = 0; // GET_STATE the Pi decoded
    std::size_t          paths        = 0; // PATH_REPORT the Pi decoded
    int                  most_between = 0; // other requests between two GET_STATE

    // Longest wait for a send, from the start and to the end included.
    Seconds worstGap() const {
        Seconds worst = sends.empty() ? end - start : sends.front() - start;
        for (std::size_t k = 1; k < sends.size(); ++k) {
            worst = std::max(worst, sends[k] - sends[k - 1]);
        }
        return sends.empty() ? worst : std::max(worst, end - sends.back());
    }
    Seconds bestGap() const {
        Seconds best = 1e9;
        for (std::size_t k = 1; k < sends.size(); ++k) {
            best = std::min(best, sends[k] - sends[k - 1]);
        }
        return best;
    }
};

class SlowTelemetry : public testing::TestWithParam<RigTransport> {
protected:
    SlowTelemetry() : rig(ClientConfig{}, slowBus(), GetParam(), slowUsb()) {}

    void open() { ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit)); }

    // A report every 100 ms as the programs send, or none; with paths, a path
    // report every step keeps a transfer waiting.
    SlowRun runFor(Seconds duration, bool report, bool paths) {
        Client&           client = rig.client();
        const std::size_t from   = rig.pi.requests().size();
        SlowRun           run;
        run.start      = rig.now();
        run.end        = run.start + duration;
        Seconds  next  = rig.now();
        int      i     = 0;
        uint32_t sent  = client.stats().telemetry_reports;
        const translagatr::PathPoint points[3] = {{0, 0}, {500, 0}, {500, 500}};
        while (rig.now() < run.end) {
            if (report && rig.now() >= next) {
                client.reportTelemetry(sample(i++));
                next += 0.1;
            }
            if (paths) {
                client.reportPath(9, translagatr::kPathDirect, points, 3);
            }
            rig.step();
            if (client.stats().telemetry_reports != sent) {
                sent = client.stats().telemetry_reports;
                run.sends.push_back(rig.now());
            }
        }
        const std::vector<translagatr::BrainRequest>& requests = rig.pi.requests();
        int between = 0;
        for (std::size_t k = from; k < requests.size(); ++k) {
            if (requests[k].op == translagatr::kOpGetState) {
                ++run.polls;
                between = 0;
            } else {
                run.most_between = std::max(run.most_between, ++between);
            }
            run.paths += requests[k].op == translagatr::kOpPathReport ? 1 : 0;
        }
        return run;
    }

    LinkRig rig;
};

} // namespace

// The on-time slot never comes here, and before the overdue rule no report
// was ever sent. Half a period late a report takes the due poll's turn: at
// most one exchange between two polls, no wait over the Pi's 250 ms
// attitude staleness limit, never closer than half a period.
TEST_P(SlowTelemetry, LateButNeverStarved) {
    open();
    const ClientStats& stats = rig.client().stats();
    const SlowRun      run   = runFor(5.0, true, false);
    EXPECT_GE(run.sends.size(), 30u);
    EXPECT_LE(run.sends.size(), 51u);
    EXPECT_LE(run.worstGap(), 0.25);
    EXPECT_GE(run.bestGap(), 0.05 - 0.001);
    EXPECT_LE(run.most_between, 1);
    EXPECT_GT(stats.telemetry_overdue, 0u);
    EXPECT_LE(stats.telemetry_overdue, stats.telemetry_reports);
    EXPECT_EQ(stats.telemetry_dropped, 0u);
    EXPECT_EQ(stats.timeouts, 0u);
    EXPECT_EQ(stats.unexpected, 0u);
    // The last one may still be on its way.
    EXPECT_GE(rig.pi.telemetryKept() + 1, static_cast<int>(run.sends.size()));
    EXPECT_LE(rig.pi.telemetryKept(), static_cast<int>(run.sends.size()));
}

// One exchange per 1.5 periods at most: the polls keep most of their rate.
TEST_P(SlowTelemetry, StatePollsKeepMostOfTheirRate) {
    open();
    const std::size_t without = runFor(3.0, false, false).polls;
    const std::size_t with    = runFor(3.0, true, false).polls;
    EXPECT_GT(without, 100u);
    EXPECT_GE(with * 100, without * 80) << with << " polls vs " << without;
}

// A waiting transfer that yielded its four polls goes first; telemetry takes
// another slot. Still one exchange at most between two polls, and neither
// starves.
TEST_P(SlowTelemetry, SharesTheLinkWithAWaitingTransfer) {
    open();
    const SlowRun run = runFor(5.0, true, true);
    EXPECT_LE(run.most_between, 1);
    EXPECT_GE(run.sends.size(), 25u);
    EXPECT_LE(run.worstGap(), 0.25);
    EXPECT_GE(run.paths, 25u);
    EXPECT_GE(run.polls, 4 * run.paths);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

INSTANTIATE_TEST_SUITE_P(Transports, SlowTelemetry,
                         testing::Values(RigTransport::kRs485, RigTransport::kUsb), name);

// ---------------------------------------------------------------------------
// Client on the RS-485 bus: timing from the bus log
// ---------------------------------------------------------------------------

struct BusRun {
    Seconds  worst_gap  = 0; // between GET_STATE starts
    int      polls      = 0;
    int      telemetry  = 0;
    Seconds  worst_telemetry_gap = 0; // between TELEMETRY starts
    int      most_between = 0;        // TELEMETRY between two GET_STATE starts
    uint32_t overdue    = 0;          // ClientStats::telemetry_overdue
    bool     due        = false;      // a TELEMETRY started with the state poll due
    bool     first_slot = true;       // every TELEMETRY in the first slot after a state reply
    bool     collision  = false;
};

// Three seconds on the bus after the session opens, with a report every step
// or none.
BusRun busRun(bool report, const FakeBusConfig& bus = {}) {
    LinkRig rig(ClientConfig{}, bus);
    EXPECT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    const ClientConfig& config = rig.client().config();
    const Seconds       start  = rig.now();
    const Seconds       end    = start + 3.0;
    int                 i      = 0;
    while (rig.now() < end) {
        if (report) {
            rig.client().reportTelemetry(sample(i++));
        }
        rig.step();
    }
    BusRun      run;
    Seconds     last_poll      = -1;
    Seconds     last_telemetry = -1;
    int         between        = 0;
    const auto& log            = rig.bus.log();
    for (std::size_t k = 0; k < log.size(); ++k) {
        const Transmission& t = log[k];
        if (!t.brain) {
            continue;
        }
        if (opOf(t) == translagatr::kOpGetState) {
            if (last_poll >= 0 && t.start > start) {
                run.worst_gap = std::max(run.worst_gap, t.start - last_poll);
                ++run.polls;
            }
            last_poll = t.start;
            between   = 0;
        } else if (opOf(t) == translagatr::kOpTelemetry) {
            ++run.telemetry;
            if (last_telemetry >= 0) {
                run.worst_telemetry_gap = std::max(run.worst_telemetry_gap, t.start - last_telemetry);
            }
            last_telemetry   = t.start;
            run.most_between = std::max(run.most_between, ++between);
            run.due = run.due || last_poll < 0 || t.start >= last_poll + config.state_period;
            run.first_slot = run.first_slot && k >= 2 && !log[k - 1].brain &&
                             log[k - 2].brain && opOf(log[k - 2]) == translagatr::kOpGetState &&
                             t.start - log[k - 1].end <= config.request_gap + 0.002;
        }
    }
    run.overdue   = rig.client().stats().telemetry_overdue;
    run.collision = rig.bus.collision();
    return run;
}

// One TELEMETRY exchange on the bus: request, reply delay, reply, release.
Seconds telemetryExchange(const FakeBusConfig& bus) {
    const Seconds byte = 10.0 / bus.baud;
    return (translagatr::kBrainRequestHeaderLen + translagatr::kTelemetryBodyLen +
            translagatr::kLinkEnvelopeLen) *
               byte +
           bus.reply_delay +
           (translagatr::kBrainReplyHeaderLen + translagatr::kLinkEnvelopeLen) * byte +
           bus.release;
}

// On the default bus every TELEMETRY starts in the first slot after a state
// reply, with the poll not due; it pushes the next poll back by at most one
// TELEMETRY exchange and the request gap.
TEST(TelemetryBus, OnTheDefaultBusNeverTakesTheTurnOfADuePoll) {
    const FakeBusConfig bus;
    const ClientConfig  config;
    const BusRun        without = busRun(false, bus);
    const BusRun        with    = busRun(true, bus);
    EXPECT_EQ(without.telemetry, 0);
    EXPECT_GE(with.telemetry, 29);
    EXPECT_LE(with.telemetry, 31);
    EXPECT_FALSE(with.due);
    EXPECT_EQ(with.overdue, 0u);
    EXPECT_TRUE(with.first_slot);
    EXPECT_FALSE(with.collision);
    EXPECT_LE(without.worst_gap, config.state_period + 0.002);
    EXPECT_LE(with.worst_gap, without.worst_gap + telemetryExchange(bus) + config.request_gap + 0.001);
    EXPECT_GE(with.polls * 100, without.polls * 85);
}

// A reply 8 ms after the request leaves no idle window: the poll is due in
// every slot. Each TELEMETRY still starts in the first slot after a state
// reply, one at most between two polls, each pushing the next poll back by
// at most one exchange and the gap (plus a 1 ms rig step for its reply).
TEST(TelemetryBus, OnASlowBusTakesTheFirstSlotHalfAPeriodLate) {
    const FakeBusConfig bus = slowBus();
    const ClientConfig  config;
    const BusRun        without = busRun(false, bus);
    const BusRun        with    = busRun(true, bus);
    EXPECT_EQ(without.telemetry, 0);
    EXPECT_GE(with.telemetry, 17);
    EXPECT_LE(with.telemetry, 31);
    EXPECT_GT(with.overdue, 0u);
    EXPECT_LE(with.worst_telemetry_gap, 0.25);
    EXPECT_EQ(with.most_between, 1);
    EXPECT_TRUE(with.first_slot);
    EXPECT_FALSE(with.collision);
    EXPECT_LE(with.worst_gap, without.worst_gap + telemetryExchange(bus) + config.request_gap + 0.002);
    EXPECT_GE(with.polls * 100, without.polls * 80);
}

TEST(TelemetryBus, ALostReportIsNotResent) {
    LinkRig rig;
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    auto onWire = [&] {
        std::size_t n = 0;
        for (const Transmission& t : rig.bus.log()) {
            n += t.brain && opOf(t) == translagatr::kOpTelemetry ? 1 : 0;
        }
        return n;
    };
    EXPECT_TRUE(rig.client().reportTelemetry(sample(1)));
    ASSERT_TRUE(rig.runUntil([&] { return onWire() == 1; }, kLimit));
    rig.bus.fault(BusFault::kDropRequest); // the frame on the wire has not reached the Pi
    rig.run(0.5);
    EXPECT_EQ(onWire(), 1u);
    EXPECT_EQ(rig.pi.telemetryKept(), 0);
    EXPECT_EQ(rig.client().stats().timeouts, 1u);
    EXPECT_TRUE(rig.client().ready());

    EXPECT_TRUE(rig.client().reportTelemetry(sample(2)));
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 1; }, kLimit));
    expectSame(rig.pi.telemetry(), sample(2));
}

TEST(TelemetryClient, BodiesTheCodecCannotCarryAreRefused) {
    LinkRig rig;
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    Client& client = rig.client();

    translagatr::BrainTelemetry t = sample(1);
    t.wheel_count                 = translagatr::kTelemetryWheelsMax + 1;
    EXPECT_FALSE(client.reportTelemetry(t));
    t.flags = translagatr::kTelemetryAttitude; // no wheels group: the count is ignored
    EXPECT_TRUE(client.reportTelemetry(t));
    t       = sample(1);
    t.flags = static_cast<uint8_t>(t.flags | 0x80u);
    EXPECT_FALSE(client.reportTelemetry(t));
    t.flags = static_cast<uint8_t>(sample(1).flags | (1u << 4)); // first unassigned bit
    EXPECT_FALSE(client.reportTelemetry(t));
    EXPECT_EQ(client.stats().telemetry_dropped, 3u);

    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.telemetryKept() == 1; }, kLimit));
    EXPECT_EQ(rig.pi.telemetry().flags, translagatr::kTelemetryAttitude);
    EXPECT_EQ(rig.pi.telemetry().wheel_count, 0); // zero on the wire without the group
}
