// client_gtest.cpp
// Client against the fake Pi on the fake bus: sessions, correlation,
// retries, placement acknowledgement, calibration control, bench IMU
// samples, path reports, restarts and faults.

#include "communigatr/client.h"

#include <deque>
#include <gtest/gtest.h>
#include <vector>

#include "sim/link_rig.h"

using namespace communigatr;

namespace
{

constexpr Seconds kLimit = 3.0;

// Brain frames on the wire carrying this op (payload byte 1, frame offset 5).
std::vector<std::vector<uint8_t>> sent(const FakeBus& bus, uint8_t op) {
    std::vector<std::vector<uint8_t>> frames;
    for (const Transmission& t : bus.log()) {
        if (t.brain && t.bytes.size() > 5 && t.bytes[5] == op) {
            frames.push_back(t.bytes);
        }
    }
    return frames;
}

translagatr::BrainRequest decoded(const std::vector<uint8_t>& frame) {
    translagatr::BrainRequest request;
    EXPECT_TRUE(
        translagatr::decodeBrainRequest(frame.data(), static_cast<uint16_t>(frame.size()), request));
    return request;
}

std::vector<uint8_t> encode(const translagatr::BrainReply& reply) {
    std::vector<uint8_t> frame(translagatr::kMaxFrameLen);
    frame.resize(translagatr::encodeBrainReply(reply, frame.data(), static_cast<uint16_t>(frame.size())));
    return frame;
}

void openSession(LinkRig& rig) {
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
}

// Runs until one more frame with this op is on the wire; faults queued right
// after apply to it, since it has not reached the Pi yet.
void runUntilSent(LinkRig& rig, uint8_t op) {
    const std::size_t before = sent(rig.bus, op).size();
    ASSERT_TRUE(rig.runUntil([&] { return sent(rig.bus, op).size() > before; }, kLimit));
}

// Port with scripted reads, for byte timing the bus cannot express.
class ScriptedPort : public BytePort {
public:
    int read(uint8_t* buf, int max) override {
        if (reads.empty()) {
            return 0;
        }
        std::vector<uint8_t> chunk = reads.front();
        reads.pop_front();
        const int n = static_cast<int>(chunk.size()) < max ? static_cast<int>(chunk.size()) : max;
        for (int i = 0; i < n; ++i) {
            buf[i] = chunk[static_cast<std::size_t>(i)];
        }
        return n;
    }

    bool write(const uint8_t* data, int len) override {
        writes.emplace_back(data, data + len);
        return true;
    }

    std::deque<std::vector<uint8_t>>  reads;
    std::vector<std::vector<uint8_t>> writes;
};

// The fake Pi answers inside write(), with no bus timing.
class LoopbackPort : public BytePort {
public:
    explicit LoopbackPort(FakePi& pi) : pi_(pi) {}

    int read(uint8_t* buf, int max) override {
        int n = 0;
        while (n < max && !rx_.empty()) {
            buf[n++] = rx_.front();
            rx_.pop_front();
        }
        return n;
    }

    bool write(const uint8_t* data, int len) override {
        for (const auto& reply : pi_.receive(data, static_cast<std::size_t>(len))) {
            rx_.insert(rx_.end(), reply.begin(), reply.end());
        }
        return true;
    }

private:
    FakePi&             pi_;
    std::deque<uint8_t> rx_;
};

// Opens a session on a scripted port: HELLO, then one state poll.
void openScripted(Client& client, ScriptedPort& port, FakePi& pi, Seconds& now) {
    client.poll(now);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    now += 0.001;
    client.poll(now);
    now += 0.006;
    client.poll(now);
    ASSERT_EQ(decoded(port.writes.back()).op, translagatr::kOpGetState);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    now += 0.001;
    client.poll(now);
    ASSERT_TRUE(client.ready());
}

} // namespace

// ---------------------------------------------------------------------------
// Session and readiness
// ---------------------------------------------------------------------------

TEST(Client, ReadyOnlyAfterCorrelatedState) {
    FakeBusConfig bus;
    bus.reply_delay = 0.010;
    LinkRig rig({}, bus);
    Client& client = rig.client();
    EXPECT_FALSE(client.ready());
    EXPECT_FALSE(client.state().valid);

    ASSERT_TRUE(rig.runUntil([&] { return client.session() != 0; }, kLimit));
    EXPECT_EQ(client.session(), rig.pi.session());
    EXPECT_EQ(client.piInstance(), rig.pi.piInstance());
    EXPECT_FALSE(client.ready());
    EXPECT_FALSE(client.connected(rig.now()));

    // A forged state reply for another session is dropped.
    runUntilSent(rig, translagatr::kOpGetState);
    const translagatr::BrainRequest request = rig.bus.brainRequests().back();
    translagatr::BrainReply         forged;
    forged.op          = translagatr::kOpGetState;
    forged.session     = request.session + 1;
    forged.request_id  = request.request_id;
    forged.pi_instance = rig.pi.piInstance();
    forged.state.x_mm  = 999;
    rig.bus.sendToBrain(encode(forged), rig.now());
    rig.step();
    rig.step();
    EXPECT_FALSE(client.ready());
    EXPECT_EQ(client.stats().uncorrelated, 1u);

    openSession(rig);
    EXPECT_TRUE(client.connected(rig.now()));
    EXPECT_TRUE(client.state().valid);
    EXPECT_NE(client.state().state.x_mm, 999);
    EXPECT_EQ(client.state().session, client.session());
    EXPECT_GT(client.state().round_trip, 0.0);
    EXPECT_EQ(client.stats().sessions, 1u);
    EXPECT_EQ(rig.pi.sessionsOpened(), 1);
}

TEST(Client, PollsStateWithFreshIdsAndNoImuBlockByDefault) {
    LinkRig rig;
    openSession(rig);
    rig.run(0.5);
    const auto polls = sent(rig.bus, translagatr::kOpGetState);
    ASSERT_GE(polls.size(), 15u);
    for (std::size_t i = 1; i < polls.size(); ++i) {
        EXPECT_NE(decoded(polls[i]).request_id, decoded(polls[i - 1]).request_id);
    }
    for (const auto& poll : polls) {
        EXPECT_EQ(decoded(poll).imu_flags, 0);
        EXPECT_EQ(poll.size(), 23u); // 8 header + 9 IMU block + 6 envelope
    }
    EXPECT_EQ(rig.client().stats().resends, 0u);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
    EXPECT_EQ(rig.pi.imuSamples(), 0);
}

// ---------------------------------------------------------------------------
// Bench IMU block
// ---------------------------------------------------------------------------

TEST(ClientBenchImu, EveryStatePollCarriesTheSampleAsGiven) {
    BenchImuSample sample{true, 1234, -450123};
    ClientConfig   config;
    config.bench_imu = [&] { return sample; };
    LinkRig rig(config);
    openSession(rig);
    rig.run(0.08);
    const auto polls = sent(rig.bus, translagatr::kOpGetState);
    ASSERT_GE(polls.size(), 2u);
    for (const auto& bytes : polls) {
        const auto request = decoded(bytes);
        EXPECT_EQ(request.imu_flags, translagatr::kBenchImuValid);
        EXPECT_EQ(request.imu_stamp_ms, 1234u);
        EXPECT_EQ(request.imu_rotation_mdeg, -450123);
    }
    EXPECT_EQ(rig.pi.imuSamples(), static_cast<int>(polls.size()));
    sample = {false, 1234, -450123};
    runUntilSent(rig, translagatr::kOpGetState);
    EXPECT_EQ(rig.bus.brainRequests().back().imu_flags, 0);
    EXPECT_EQ(rig.bus.brainRequests().back().imu_stamp_ms, 1234u);
}

TEST(ClientBenchImu, PendingPlacementInterleavesStatePollBeforeRetry) {
    ClientConfig config;
    config.pending_retry = 0;
    config.bench_imu     = [] { return BenchImuSample{true, 1234, 90000}; };
    LinkRig rig(config);
    openSession(rig);
    rig.pi.setApplyDelay(5);
    const auto ticket = rig.client().submitPlacement(610, -457, -9000);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    bool placement_seen      = false;
    bool imu_since_placement = false;
    for (const auto& request : rig.bus.brainRequests()) {
        if (request.op == translagatr::kOpGetState) {
            imu_since_placement = true;
        } else if (request.op == translagatr::kOpSetPose) {
            if (placement_seen) {
                EXPECT_TRUE(imu_since_placement);
            }
            placement_seen      = true;
            imu_since_placement = false;
        }
    }
    const auto placements = sent(rig.bus, translagatr::kOpSetPose);
    ASSERT_GE(placements.size(), 2u);
    for (const auto& bytes : placements) {
        EXPECT_EQ(bytes, placements.front());
    }
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
}

// Review finding B: a SET_POSE the Pi may never have seen is resent before
// any newer request id, or the Pi would answer the resend Stale.
TEST(ClientBenchImu, TimedOutPlacementIsResentBeforeAnyStatePoll) {
    ClientConfig config;
    config.bench_imu = [] { return BenchImuSample{true, 1234, 90000}; };
    LinkRig rig(config);
    openSession(rig);
    const auto ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, translagatr::kOpSetPose);
    rig.bus.fault(BusFault::kDropRequest);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto requests = rig.bus.brainRequests();
    std::size_t first   = 0;
    while (requests[first].op != translagatr::kOpSetPose) {
        ++first;
    }
    ASSERT_LT(first + 1, requests.size());
    EXPECT_EQ(requests[first + 1].op, translagatr::kOpSetPose);
    EXPECT_EQ(requests[first + 1].request_id, requests[first].request_id);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    EXPECT_EQ(rig.client().placementStatus(ticket).result, translagatr::kResultOk);
}

TEST(ClientBenchImu, DroppedPlacementReplyResendsIdenticalPlacementNext) {
    ClientConfig config;
    config.bench_imu = [] { return BenchImuSample{true, 1234, 90000}; };
    LinkRig rig(config);
    openSession(rig);
    const auto ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, translagatr::kOpSetPose);
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, translagatr::kOpSetPose);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
}

// ---------------------------------------------------------------------------
// Faults on the bus
// ---------------------------------------------------------------------------

TEST(Client, FragmentedReplyAccepted) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, translagatr::kOpGetState);
    const uint32_t replies = rig.client().stats().replies;
    rig.bus.fault(BusFault::kFragment, 0.004);
    rig.run(0.05);
    EXPECT_GT(rig.client().stats().replies, replies);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

TEST(Client, TruncatedThenValidReplyAccepted) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, translagatr::kOpGetState);
    const uint32_t replies = rig.client().stats().replies;
    rig.bus.fault(BusFault::kTruncate, 0.002);
    rig.run(0.05);
    EXPECT_GT(rig.client().stats().replies, replies);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

TEST(Client, CorruptReplyTimesOutAndNextPollHasNewId) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, translagatr::kOpGetState);
    const uint16_t lost = rig.bus.brainRequests().back().request_id;
    rig.bus.fault(BusFault::kCorrupt);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().timeouts == 1; }, kLimit));
    runUntilSent(rig, translagatr::kOpGetState);
    EXPECT_NE(rig.bus.brainRequests().back().request_id, lost);
    EXPECT_EQ(rig.client().stats().resends, 0u);
}

TEST(Client, DroppedStateReplyNextPollHasNewId) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, translagatr::kOpGetState);
    const uint16_t lost = rig.bus.brainRequests().back().request_id;
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().timeouts == 1; }, kLimit));
    runUntilSent(rig, translagatr::kOpGetState);
    EXPECT_NE(rig.bus.brainRequests().back().request_id, lost);
    EXPECT_EQ(rig.client().stats().resends, 0u);
    EXPECT_TRUE(rig.client().ready());
}

TEST(Client, DuplicateReplyDoesNotApplyTwice) {
    LinkRig rig;
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, translagatr::kOpSetPose);
    rig.bus.fault(BusFault::kDuplicate);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    rig.run(0.1);
    EXPECT_GE(rig.client().stats().uncorrelated, 1u);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    EXPECT_EQ(sent(rig.bus, translagatr::kOpSetPose).size(), 1u);
}

TEST(Client, DroppedPlacementReplyResendsSameBytes) {
    LinkRig rig;
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, translagatr::kOpSetPose);
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, translagatr::kOpSetPose);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    EXPECT_EQ(rig.client().stats().resends, 1u);
}

TEST(Client, DroppedPlacementRequestResendsSameBytes) {
    LinkRig rig;
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(1, 2, 3);
    runUntilSent(rig, translagatr::kOpSetPose);
    rig.bus.fault(BusFault::kDropRequest);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, translagatr::kOpSetPose);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
}

TEST(Client, UncorrelatedRepliesChangeNothing) {
    FakeBusConfig bus;
    bus.reply_delay = 0.010;
    LinkRig rig({}, bus);
    openSession(rig);
    Client&        client  = rig.client();
    const uint32_t session = client.session();
    runUntilSent(rig, translagatr::kOpGetState);
    const translagatr::BrainRequest request = rig.bus.brainRequests().back();

    translagatr::BrainReply stale;
    stale.op                   = translagatr::kOpGetState;
    stale.session              = session;
    stale.request_id           = static_cast<uint16_t>(request.request_id - 1); // old id
    stale.pi_instance          = rig.pi.piInstance() + 1; // would mean a restart
    stale.state.x_mm           = 111;
    std::vector<uint8_t> bytes = encode(stale);

    stale.request_id                 = request.request_id;
    stale.session                    = session ^ 0x1u; // old session
    const std::vector<uint8_t> other = encode(stale);
    bytes.insert(bytes.end(), other.begin(), other.end());

    stale.op                            = translagatr::kOpSetPose; // wrong op
    stale.session                       = session;
    const std::vector<uint8_t> wrong_op = encode(stale);
    bytes.insert(bytes.end(), wrong_op.begin(), wrong_op.end());

    rig.bus.sendToBrain(bytes, rig.now());
    rig.run(0.05);
    EXPECT_EQ(client.stats().uncorrelated, 3u);
    EXPECT_EQ(client.stats().pi_restarts, 0u);
    EXPECT_EQ(client.stats().session_losses, 0u);
    EXPECT_EQ(client.session(), session);
    EXPECT_NE(client.state().state.x_mm, 111);
}

TEST(Client, BytesBeforeSendAreDrained) {
    ScriptedPort port;
    FakePi       pi;
    Client       client(port, [] { return 0x1234u; });

    // The receive read finds nothing; a perfect HELLO reply arrives before
    // the send and is drained with the rest.
    translagatr::BrainReply early;
    early.op                         = translagatr::kOpHello;
    early.session                    = 0x5E55;
    early.request_id                 = 1;
    early.pi_instance                = pi.piInstance();
    early.nonce                      = 0x1234;
    const std::vector<uint8_t> frame = encode(early);
    port.reads                       = {{}, frame};
    client.poll(0.0);
    ASSERT_EQ(port.writes.size(), 1u);
    EXPECT_EQ(client.stats().drained_bytes, frame.size());
    client.poll(0.001);
    EXPECT_EQ(client.session(), 0u);

    // The Pi's answer to the request that was actually sent opens it.
    for (const auto& reply : pi.receive(port.writes[0].data(), port.writes[0].size())) {
        port.reads.push_back(reply);
    }
    client.poll(0.002);
    EXPECT_EQ(client.session(), pi.session());
}

// ---------------------------------------------------------------------------
// Session changes
// ---------------------------------------------------------------------------

TEST(Client, PiRestartOpensNewSessionAndClearsState) {
    LinkRig rig;
    openSession(rig);
    Client&        client = rig.client();
    const uint32_t old    = client.session();

    rig.pi.restart(0xFEED0001);
    ASSERT_TRUE(rig.runUntil([&] { return client.stats().pi_restarts == 1; }, kLimit));
    EXPECT_FALSE(client.ready());
    EXPECT_FALSE(client.connected(rig.now()));
    EXPECT_FALSE(client.state().valid);
    EXPECT_EQ(client.session(), 0u);

    openSession(rig);
    EXPECT_NE(client.session(), old);
    EXPECT_EQ(client.session(), rig.pi.session());
    EXPECT_EQ(client.piInstance(), 0xFEED0001u);
    EXPECT_EQ(client.state().pi_instance, 0xFEED0001u);
    EXPECT_EQ(client.stats().sessions, 2u);
    EXPECT_EQ(client.stats().session_losses, 1u);
}

TEST(Client, UnknownSessionOpensNewSession) {
    LinkRig rig;
    openSession(rig);
    Client&        client = rig.client();
    const uint32_t old    = client.session();

    // Another HELLO reaches the Pi; our session is no longer its session.
    translagatr::BrainRequest hello;
    hello.op         = translagatr::kOpHello;
    hello.request_id = 7;
    hello.nonce      = 0xABCDEF;
    ASSERT_EQ(rig.pi.answer(hello).result, translagatr::kResultOk);

    ASSERT_TRUE(rig.runUntil([&] { return client.stats().session_losses == 1; }, kLimit));
    EXPECT_EQ(client.stats().pi_restarts, 0u);
    EXPECT_FALSE(client.state().valid);
    openSession(rig);
    EXPECT_NE(client.session(), old);
    EXPECT_EQ(client.session(), rig.pi.session());
}

TEST(Client, InFlightPlacementLostWithSessionIsNeverResent) {
    LinkRig rig;
    rig.pi.setApplyDelay(-1);
    openSession(rig);
    Client&               client = rig.client();
    const PlacementTicket ticket = client.submitPlacement(500, 500, 0);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementStatus(ticket).result == translagatr::kResultPending; }, kLimit));

    rig.pi.restart(0x77);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(ticket) == PlacementResult::kSessionLost; }, kLimit));
    EXPECT_FALSE(client.placementPending());
    const std::size_t before = sent(rig.bus, translagatr::kOpSetPose).size();

    openSession(rig);
    rig.run(0.5);
    EXPECT_EQ(sent(rig.bus, translagatr::kOpSetPose).size(), before);
    for (const translagatr::BrainRequest& r : rig.pi.requests()) {
        EXPECT_FALSE(r.op == translagatr::kOpSetPose && r.session == rig.pi.session());
    }
    EXPECT_EQ(rig.pi.placementsApplied(), 0);
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kSessionLost);
}

TEST(Client, StaleHelloGetsNewNonce) {
    LinkRig rig;
    rig.nonces = {0xAAAA0001};
    openSession(rig);
    const uint32_t    old    = rig.client().session();
    const std::size_t before = sent(rig.bus, translagatr::kOpHello).size();

    // The rebooted Brain draws the same nonce first.
    rig.rebootBrain();
    rig.nonces = {0xAAAA0001, 0xAAAA0002};
    openSession(rig);
    EXPECT_EQ(rig.client().stats().stale_hellos, 1u);
    EXPECT_NE(rig.client().session(), old);

    const auto hellos = sent(rig.bus, translagatr::kOpHello);
    ASSERT_EQ(hellos.size(), before + 2);
    EXPECT_EQ(decoded(hellos[before]).nonce, 0xAAAA0001u);
    EXPECT_EQ(decoded(hellos[before + 1]).nonce, 0xAAAA0002u);
    EXPECT_NE(decoded(hellos[before]).request_id, decoded(hellos[before + 1]).request_id);
}

TEST(Client, RepeatedNonceIsReplaced) {
    LinkRig rig;
    rig.nonces = {0x42, 0x42};
    openSession(rig);
    rig.rebootBrain();
    rig.nonces = {0x42, 0x42, 0x42};
    openSession(rig);
    const auto hellos = sent(rig.bus, translagatr::kOpHello);
    ASSERT_EQ(hellos.size(), 3u);
    EXPECT_EQ(decoded(hellos[1]).nonce, 0x42u);
    EXPECT_NE(decoded(hellos[2]).nonce, 0x42u);
    EXPECT_NE(decoded(hellos[2]).nonce, 0u);
}

TEST(Client, BrainRebootIgnoresOldSessionReplies) {
    FakeBusConfig bus;
    bus.reply_delay = 0.010;
    LinkRig rig({}, bus);
    rig.nonces = {0xA0A0A0A0};
    openSession(rig);
    const uint32_t session_a = rig.client().session();

    translagatr::BrainReply old_hello;
    old_hello.op          = translagatr::kOpHello;
    old_hello.session     = session_a;
    old_hello.request_id  = 1;
    old_hello.pi_instance = rig.pi.piInstance();
    old_hello.nonce       = 0xA0A0A0A0;

    translagatr::BrainReply old_state;
    old_state.op          = translagatr::kOpGetState;
    old_state.session     = session_a;
    old_state.request_id  = 2;
    old_state.pi_instance = rig.pi.piInstance();
    old_state.state.x_mm  = 4242;

    // Session B: request ids restart at 1; A's replies arrive late.
    rig.rebootBrain();
    Client& client = rig.client();
    runUntilSent(rig, translagatr::kOpHello);
    EXPECT_EQ(rig.bus.brainRequests().back().request_id, 1);
    rig.bus.sendToBrain(encode(old_hello), rig.now());
    rig.step();
    rig.step();
    EXPECT_EQ(client.session(), 0u);

    ASSERT_TRUE(rig.runUntil([&] { return client.session() != 0; }, kLimit));
    EXPECT_NE(client.session(), session_a);
    runUntilSent(rig, translagatr::kOpGetState);
    EXPECT_EQ(rig.bus.brainRequests().back().request_id, 2);
    rig.bus.sendToBrain(encode(old_state), rig.now());
    openSession(rig);
    EXPECT_NE(client.state().state.x_mm, 4242);
    EXPECT_EQ(client.stats().uncorrelated, 2u);

    // A delayed session A request changes nothing on the Pi.
    translagatr::BrainRequest old_pose;
    old_pose.op         = translagatr::kOpSetPose;
    old_pose.session    = session_a;
    old_pose.request_id = 9;
    old_pose.x_mm       = 77;
    EXPECT_EQ(rig.pi.answer(old_pose).result, translagatr::kResultUnknownSession);
    EXPECT_EQ(rig.pi.placementsApplied(), 0);
}

// ---------------------------------------------------------------------------
// Placement
// ---------------------------------------------------------------------------

TEST(Client, PlacementAppliedOnlyAfterStateShowsAnchor) {
    LinkRig rig;
    openSession(rig);
    Client&        client = rig.client();
    const uint32_t anchor = client.state().state.anchor_revision;

    const PlacementTicket ticket = client.submitPlacement(100, 200, 9000);
    ASSERT_NE(ticket, 0u);
    EXPECT_TRUE(client.placementPending());
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementStatus(ticket).anchor_revision == anchor + 1; }, kLimit));

    // Ok recorded; the cached state still predates the placement.
    EXPECT_EQ(client.placementStatus(ticket).result, translagatr::kResultOk);
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kPending);
    EXPECT_TRUE(client.placementPending());
    EXPECT_EQ(client.state().state.anchor_revision, anchor);

    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    EXPECT_FALSE(client.placementPending());
    EXPECT_EQ(client.state().state.anchor_revision, anchor + 1);
    EXPECT_EQ(client.state().state.x_mm, 100);
    EXPECT_EQ(client.state().state.heading_cdeg, 9000);
    EXPECT_NE(client.state().state.robot_flags & translagatr::kRobotAnchorCommand, 0);
}

TEST(Client, PendingPlacementResendsSameBytesUntilApplied) {
    LinkRig rig;
    rig.pi.setApplyDelay(3);
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(-300, 50, -17999);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, translagatr::kOpSetPose);
    ASSERT_GE(frames.size(), 2u);
    for (const auto& frame : frames) {
        EXPECT_EQ(frame, frames[0]);
    }
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

TEST(Client, PlacementNeverAppliedTimesOut) {
    LinkRig rig;
    rig.pi.setApplyDelay(-1);
    openSession(rig);
    Client&               client = rig.client();
    const Seconds         start  = rig.now();
    const PlacementTicket ticket = client.submitPlacement(1, 1, 1);
    ASSERT_TRUE(rig.runUntil([&] { return !client.placementPending(); }, kLimit));
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kTimedOut);
    EXPECT_EQ(client.placementStatus(ticket).result, translagatr::kResultPending);
    EXPECT_LE(rig.now() - start, client.config().placement_deadline + 0.1);

    const std::size_t count = sent(rig.bus, translagatr::kOpSetPose).size();
    EXPECT_LE(count, static_cast<std::size_t>(client.config().placement_attempts));
    rig.run(0.5);
    EXPECT_EQ(sent(rig.bus, translagatr::kOpSetPose).size(), count);
    EXPECT_TRUE(client.connected(rig.now()));
}

TEST(Client, SilentPiPlacementTimesOutWithBackToBackResends) {
    LinkRig rig;
    openSession(rig);
    rig.bus.setPiPresent(false);
    const PlacementTicket ticket = rig.client().submitPlacement(1, 1, 1);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().placementPending(); }, kLimit));
    EXPECT_EQ(rig.client().placementResult(ticket), PlacementResult::kTimedOut);
    const auto frames = sent(rig.bus, translagatr::kOpSetPose);
    ASSERT_GE(frames.size(), 2u);
    for (const auto& frame : frames) {
        EXPECT_EQ(frame, frames[0]);
    }
    // Nothing between the resends while the outcome is unknown.
    const auto requests = rig.bus.brainRequests();
    bool       inside   = false;
    std::size_t seen    = 0;
    for (const auto& r : requests) {
        if (r.op == translagatr::kOpSetPose) {
            inside = ++seen < frames.size();
        } else {
            EXPECT_FALSE(inside) << "op " << int(r.op);
        }
    }
}

TEST(Client, PlacementRefusedWithoutSession) {
    LinkRig               rig;
    Client&               client = rig.client();
    EXPECT_EQ(client.submitPlacement(1, 2, 3), 0u);
    EXPECT_FALSE(client.placementPending());
    openSession(rig);
    const PlacementTicket first = client.submitPlacement(1, 2, 3);
    EXPECT_NE(first, 0u);
    EXPECT_EQ(client.submitPlacement(4, 5, 6), 0u); // one at a time
    EXPECT_EQ(client.placementResult(0), PlacementResult::kNone);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(first) == PlacementResult::kApplied; }, kLimit));
    EXPECT_EQ(rig.pi.robot().x_mm, 1);

    const PlacementTicket second = client.submitPlacement(4, 5, 6);
    EXPECT_EQ(second, first + 1);
    EXPECT_EQ(client.placementResult(first), PlacementResult::kNone);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(second) == PlacementResult::kApplied; }, kLimit));
    EXPECT_EQ(rig.pi.placementsApplied(), 2);
}

TEST(Client, PlacementAckWithoutStateExpiresAndAllowsNewPlacement) {
    ClientConfig config;
    config.placement_deadline = 0.08;
    config.placement_attempts = 1;
    LinkRig rig(config);
    openSession(rig);
    Client&               client = rig.client();
    const PlacementTicket first  = client.submitPlacement(100, 200, 300);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementStatus(first).anchor_revision != 0; }, kLimit));
    ASSERT_EQ(client.placementResult(first), PlacementResult::kPending);

    // SET_POSE Ok arrived, but every confirming GET_STATE is lost.
    rig.bus.setPiPresent(false);
    ASSERT_TRUE(rig.runUntil([&] { return !client.placementPending(); }, 0.1));
    EXPECT_EQ(client.placementResult(first), PlacementResult::kTimedOut);
    EXPECT_EQ(client.placementStatus(first).result, translagatr::kResultOk);

    // A fresh ticket gets its own deadline. One successful send is allowed
    // to wait for its confirming state even when the send limit is one.
    rig.bus.setPiPresent(true);
    const PlacementTicket second = client.submitPlacement(400, 500, 600);
    ASSERT_NE(second, 0u);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(second) == PlacementResult::kApplied; }, kLimit));
    EXPECT_EQ(client.state().state.x_mm, 400);
    EXPECT_EQ(rig.pi.placementsApplied(), 2);
    EXPECT_FALSE(rig.bus.collision());
}

TEST(Client, LateConfirmingStateDoesNotReviveExpiredPlacement) {
    ClientConfig config;
    config.placement_deadline = 0.08;
    LinkRig rig(config);
    openSession(rig);
    Client&               client = rig.client();
    const PlacementTicket ticket = client.submitPlacement(100, 200, 300);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementStatus(ticket).anchor_revision != 0; }, kLimit));
    runUntilSent(rig, translagatr::kOpGetState);

    // Receive the otherwise valid confirmation only after its deadline.
    rig.step(0.1);
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kTimedOut);
    EXPECT_FALSE(client.placementPending());
    EXPECT_EQ(client.state().state.x_mm, 100);
    rig.run(0.1);
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kTimedOut);
}

TEST(Client, LatePlacementAckCannotAcknowledgeQueuedReplacement) {
    ClientConfig config;
    config.placement_deadline = 0.020;
    ScriptedPort port;
    FakePi       pi;
    Client       client(port, [] { return 7u; }, config);
    Seconds      now = 0;
    openScripted(client, port, pi, now);

    const PlacementTicket first = client.submitPlacement(100, 200, 300);
    now += 0.006;
    client.poll(now);
    ASSERT_EQ(decoded(port.writes.back()).op, translagatr::kOpSetPose);
    const auto old_reply = encode(pi.answer(decoded(port.writes.back())));
    now += 0.021;
    client.poll(now);
    ASSERT_EQ(client.placementResult(first), PlacementResult::kTimedOut);
    const PlacementTicket second = client.submitPlacement(400, 500, 600);
    ASSERT_NE(second, 0u);

    // The first request still owns the bus response window. Its late Ok
    // must release that window without acknowledging the unsent second one.
    const std::size_t writes = port.writes.size();
    port.reads.push_back(old_reply);
    now += 0.001;
    client.poll(now);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kPending);
    EXPECT_EQ(client.placementStatus(second).anchor_revision, 0u);
    ASSERT_EQ(port.writes.size(), writes);

    now += 0.006;
    client.poll(now);
    ASSERT_EQ(port.writes.size(), writes + 1);
    ASSERT_EQ(decoded(port.writes.back()).x_mm, 400);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    now += 0.001;
    client.poll(now);
    now += 0.006;
    client.poll(now);
    ASSERT_EQ(decoded(port.writes.back()).op, translagatr::kOpGetState);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    now += 0.001;
    client.poll(now);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kApplied);
    EXPECT_EQ(client.state().state.x_mm, 400);
}

TEST(Client, ExpiredRequestTimeoutDoesNotExpireQueuedReplacement) {
    ClientConfig config;
    config.placement_deadline = 0.020;
    ScriptedPort port;
    FakePi       pi;
    Client       client(port, [] { return 7u; }, config);
    Seconds      now = 0;
    openScripted(client, port, pi, now);

    const PlacementTicket first = client.submitPlacement(100, 200, 300);
    now += 0.006;
    client.poll(now);
    const Seconds sent_at = now;
    now += 0.021;
    client.poll(now);
    ASSERT_EQ(client.placementResult(first), PlacementResult::kTimedOut);
    const PlacementTicket second = client.submitPlacement(400, 500, 600);
    ASSERT_NE(second, 0u);

    // Keep the original response window open until its transport timeout;
    // that timeout belongs to the old ticket, not the queued replacement.
    const std::size_t writes = port.writes.size();
    client.poll(sent_at + 0.059);
    ASSERT_EQ(port.writes.size(), writes);
    client.poll(sent_at + 0.061);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kPending);
    ASSERT_EQ(port.writes.size(), writes);
    client.poll(sent_at + 0.067);
    ASSERT_EQ(port.writes.size(), writes + 1);
    ASSERT_EQ(decoded(port.writes.back()).x_mm, 400);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(sent_at + 0.068);
    client.poll(sent_at + 0.074);
    ASSERT_EQ(decoded(port.writes.back()).op, translagatr::kOpGetState);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(sent_at + 0.075);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kApplied);
}

// ---------------------------------------------------------------------------
// Calibration control
// ---------------------------------------------------------------------------

TEST(ClientControl, RecalibrateAndReinitializeTickets) {
    LinkRig rig;
    Client& client = rig.client();
    EXPECT_EQ(client.recalibrate(), 0u); // no session
    openSession(rig);
    const ControlTicket recal = client.recalibrate();
    ASSERT_NE(recal, 0u);
    EXPECT_EQ(client.reinitialize(), 0u); // one at a time
    EXPECT_EQ(client.controlStatus(recal).state, ControlResult::kPending);
    EXPECT_EQ(client.controlStatus(recal).action, translagatr::kControlRecalibrate);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(recal).state, ControlResult::kOk);
    EXPECT_EQ(client.controlStatus(recal).calibration, translagatr::kCalibrationRunning);

    rig.pi.setMoving(true);
    const ControlTicket moving = client.reinitialize();
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(moving).state, ControlResult::kNotStationary);
    EXPECT_EQ(client.controlStatus(moving).result, translagatr::kResultNotStationary);
    EXPECT_EQ(client.controlStatus(recal).state, ControlResult::kNone); // not the latest
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);
}

TEST(ClientControl, DroppedRequestIsResentFirstWithSameBytesAndExecutesOnce) {
    LinkRig rig;
    openSession(rig);
    Client&             client = rig.client();
    const ControlTicket ticket = client.reinitialize();
    runUntilSent(rig, translagatr::kOpControl);
    rig.bus.fault(BusFault::kDropRequest);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kOk);
    const auto frames = sent(rig.bus, translagatr::kOpControl);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
    const auto requests = rig.bus.brainRequests();
    for (std::size_t i = 0; i + 1 < requests.size(); ++i) {
        if (requests[i].op == translagatr::kOpControl) {
            EXPECT_EQ(requests[i + 1].op, translagatr::kOpControl);
            break;
        }
    }
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);

    // Dropped reply: the resend is a duplicate, answered from the record.
    const ControlTicket again = client.recalibrate();
    runUntilSent(rig, translagatr::kOpControl);
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(again).state, ControlResult::kOk);
    EXPECT_EQ(rig.pi.controlsExecuted(), 2);
}

// A placement outranks a control, but not a control's resend after a
// timeout: its newer request id would make the resend Stale.
TEST(ClientControl, TimedOutControlIsResentBeforeAWaitingPlacement) {
    LinkRig rig;
    openSession(rig);
    Client&             client = rig.client();
    const ControlTicket ticket = client.reinitialize();
    runUntilSent(rig, translagatr::kOpControl);
    rig.bus.fault(BusFault::kDropRequest);
    const PlacementTicket placement = client.submitPlacement(10, 20, 30);
    ASSERT_NE(placement, 0u);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kOk);
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(placement) == PlacementResult::kApplied; }, kLimit));
}

TEST(ClientControl, SilentPiTimesOutAndSessionLossSettles) {
    LinkRig rig;
    openSession(rig);
    Client& client = rig.client();
    rig.bus.setPiPresent(false);
    const ControlTicket ticket = client.recalibrate();
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kTimedOut);
    EXPECT_LE(sent(rig.bus, translagatr::kOpControl).size(),
              static_cast<std::size_t>(client.config().control_attempts));

    rig.bus.setPiPresent(true);
    rig.pi.setApplyDelay(-1);
    ASSERT_TRUE(rig.runUntil([&] { return client.connected(rig.now()); }, kLimit));
    rig.pi.setMoving(false);
    const ControlTicket lost = client.recalibrate();
    rig.pi.restart(0x4242);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(lost).state, ControlResult::kSessionLost);
}

TEST(ClientControl, PendingIsAskedAgainWithTheSameBytesUntilItCompletes) {
    ClientConfig config;
    config.control_retry = 0.05;
    LinkRig rig(config);
    rig.pi.setControlPendingRequests(12);
    openSession(rig);
    Client&             client = rig.client();
    const ControlTicket ticket = client.reinitImu();
    ASSERT_NE(ticket, 0u);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.controlStatus(ticket).result == translagatr::kResultPending; }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kPending);
    EXPECT_EQ(client.reinitialize(), 0u);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kOk);
    EXPECT_EQ(client.controlStatus(ticket).action, translagatr::kControlReinitImu);
    EXPECT_EQ(client.controlStatus(ticket).calibration, translagatr::kCalibrationRunning);
    const auto frames = sent(rig.bus, translagatr::kOpControl);
    ASSERT_GE(frames.size(), 2u);
    for (const auto& frame : frames) {
        EXPECT_EQ(frame, frames.front());
    }
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);
    // State polls keep flowing while the Pi works.
    EXPECT_GT(sent(rig.bus, translagatr::kOpGetState).size(), frames.size());
}

TEST(ClientControl, FailedCarriesItsDetail) {
    LinkRig rig;
    rig.pi.setControlPendingRequests(3);
    rig.pi.setControlFailure(translagatr::kControlDetailImuAbsent);
    openSession(rig);
    Client&             client = rig.client();
    const ControlTicket ticket = client.reinitImu();
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kFailed);
    EXPECT_EQ(client.controlStatus(ticket).result, translagatr::kResultFailed);
    EXPECT_EQ(client.controlStatus(ticket).detail, translagatr::kControlDetailImuAbsent);
}

TEST(ClientControl, PendingControlGivesUpAfterControlWait) {
    ClientConfig config;
    config.control_wait = 0.5;
    LinkRig rig(config);
    rig.pi.setControlPendingRequests(1000000);
    openSession(rig);
    Client&             client = rig.client();
    const Seconds       start  = rig.now();
    const ControlTicket ticket = client.restartAcquisition();
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kTimedOut);
    EXPECT_EQ(client.controlStatus(ticket).result, translagatr::kResultPending);
    EXPECT_NEAR(rig.now() - start, 0.5, 0.05);
}

// Once the Pi answered it holds the record: more lost replies than
// control_attempts only cost time, and state polls go between the resends.
TEST(ClientControl, AnsweredControlOutlastsLostReplies) {
    LinkRig rig;
    rig.pi.setControlPendingRequests(60);
    openSession(rig);
    Client&             client = rig.client();
    const ControlTicket ticket = client.reinitImu();
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.controlStatus(ticket).result == translagatr::kResultPending; }, kLimit));
    const std::size_t controls = sent(rig.bus, translagatr::kOpControl).size();
    const std::size_t polls    = sent(rig.bus, translagatr::kOpGetState).size();

    rig.bus.setPiPresent(false);
    rig.run(2.0);
    EXPECT_TRUE(client.controlPending());
    const std::size_t resends = sent(rig.bus, translagatr::kOpControl).size() - controls;
    EXPECT_GT(resends, static_cast<std::size_t>(client.config().control_attempts));
    EXPECT_GT(sent(rig.bus, translagatr::kOpGetState).size() - polls, resends);

    rig.bus.setPiPresent(true);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kOk);
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);
    const auto frames = sent(rig.bus, translagatr::kOpControl);
    for (const auto& frame : frames) {
        EXPECT_EQ(frame, frames.front());
    }
}

// A stalled writer (USB transmit task blocked, one-slot queue full) fails
// writes at once: an answered control survives them.
TEST(ClientControl, AnsweredControlOutlastsFailedWrites) {
    LinkRig rig;
    rig.pi.setControlPendingRequests(40);
    openSession(rig);
    Client&             client = rig.client();
    const ControlTicket ticket = client.reinitImu();
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.controlStatus(ticket).result == translagatr::kResultPending; }, kLimit));
    const uint32_t errors = client.stats().write_errors;
    rig.bus.failWrites(4 * client.config().control_attempts);
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.stats().write_errors, errors + 4 * client.config().control_attempts);
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kOk);
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);
}

// ---------------------------------------------------------------------------
// Wheel readings
// ---------------------------------------------------------------------------

TEST(ClientWheels, RequestedReadingsArriveOnceWithTheirAges) {
    LinkRig rig;
    Client& client = rig.client();
    EXPECT_EQ(client.requestWheels(), 0u); // no session
    translagatr::WheelReading forward;
    forward.port      = 0;
    forward.flags     = translagatr::kWheelFresh | translagatr::kWheelValid;
    forward.counts    = 12345;
    forward.travel_um = 456789;
    forward.age_ms    = 7;
    translagatr::WheelReading sideways = forward;
    sideways.port                = 1;
    sideways.counts              = -2000;
    sideways.travel_um           = -73631;
    sideways.discontinuity       = 3;
    rig.pi.setWheels({forward, sideways});
    openSession(rig);

    const WheelTicket ticket = client.requestWheels();
    ASSERT_NE(ticket, 0u);
    EXPECT_EQ(client.requestWheels(), 0u); // one read at a time
    EXPECT_EQ(client.wheelStatus(ticket).state, WheelResult::kPending);
    ASSERT_TRUE(rig.runUntil([&] { return !client.wheelsPending(); }, kLimit));
    const WheelStatus s = client.wheelStatus(ticket);
    EXPECT_EQ(s.state, WheelResult::kOk);
    EXPECT_EQ(s.result, translagatr::kResultOk);
    EXPECT_EQ(s.readings.sequence, 1u);
    ASSERT_EQ(s.readings.count, 2);
    EXPECT_EQ(s.readings.wheels[1].counts, -2000);
    EXPECT_EQ(client.wheelStatus(ticket + 1).state, WheelResult::kNone);
    EXPECT_EQ(client.wheelStatus(0).state, WheelResult::kNone);
    const WheelReadings& r = client.wheelReadings();
    EXPECT_EQ(r.sequence, 1u);
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_FALSE(r.busy);
    ASSERT_EQ(r.count, 2);
    EXPECT_EQ(r.wheels[0].counts, 12345);
    EXPECT_EQ(r.wheels[0].travel_um, 456789);
    EXPECT_EQ(r.wheels[0].age_ms, 7);
    EXPECT_EQ(r.wheels[1].port, 1);
    EXPECT_EQ(r.wheels[1].travel_um, -73631);
    EXPECT_EQ(r.wheels[1].discontinuity, 3);
    EXPECT_GT(r.round_trip, 0.0);
    EXPECT_LE(r.received_at, rig.now());
    rig.run(0.3);
    EXPECT_EQ(client.wheelReadings().sequence, 1u);
    EXPECT_EQ(sent(rig.bus, translagatr::kOpReadWheels).size(), 1u);
}

// Every read settles: a refusal, a lost send and a lost session are each
// visible on the ticket, and none of them looks like new readings.
TEST(ClientWheels, RefusalTimeoutAndSessionLossSettleTheTicket) {
    LinkRig rig;
    rig.pi.setProfileMode(true); // no profile applied: NotReady
    openSession(rig);
    Client& client = rig.client();

    WheelTicket ticket = client.requestWheels();
    ASSERT_TRUE(rig.runUntil([&] { return !client.wheelsPending(); }, kLimit));
    EXPECT_EQ(client.wheelStatus(ticket).state, WheelResult::kRejected);
    EXPECT_EQ(client.wheelStatus(ticket).result, translagatr::kResultNotReady);
    EXPECT_EQ(client.wheelReadings().sequence, 0u);
    EXPECT_EQ(client.wheelReadings().result, translagatr::kResultNotReady);

    // Lost request: one send, then kTimedOut; never resent.
    ticket = client.requestWheels();
    ASSERT_NE(ticket, 0u);
    runUntilSent(rig, translagatr::kOpReadWheels);
    rig.bus.fault(BusFault::kDropRequest);
    ASSERT_TRUE(rig.runUntil([&] { return !client.wheelsPending(); }, kLimit));
    EXPECT_EQ(client.wheelStatus(ticket).state, WheelResult::kTimedOut);
    rig.run(0.3);
    EXPECT_EQ(sent(rig.bus, translagatr::kOpReadWheels).size(), 2u);

    // A failed write is a lost send too. Ten failures in a row span more
    // than a poll period, so the read is among them.
    ticket = client.requestWheels();
    const uint32_t errors = client.stats().write_errors;
    rig.bus.failWrites(10);
    ASSERT_TRUE(rig.runUntil([&] { return !client.wheelsPending(); }, kLimit));
    EXPECT_EQ(client.wheelStatus(ticket).state, WheelResult::kTimedOut);
    ASSERT_TRUE(
        rig.runUntil([&] { return client.stats().write_errors == errors + 10; }, kLimit));

    // Pi restart with a read queued or in flight.
    ticket = client.requestWheels();
    rig.pi.restart(0x4242);
    ASSERT_TRUE(rig.runUntil([&] { return !client.wheelsPending(); }, kLimit));
    EXPECT_EQ(client.wheelStatus(ticket).state, WheelResult::kSessionLost);
    EXPECT_EQ(client.requestWheels(), 0u); // no session yet
    ASSERT_TRUE(rig.runUntil([&] { return client.ready(); }, kLimit));
    EXPECT_NE(client.requestWheels(), 0u);
    EXPECT_EQ(client.wheelReadings().sequence, 0u);
}

// ---------------------------------------------------------------------------
// Path report
// ---------------------------------------------------------------------------

TEST(ClientPath, LatestWinsOneAttemptAndThinsToThirteen) {
    LinkRig rig;
    Client& client = rig.client();
    const translagatr::PathPoint one[1] = {{1, 2}};
    EXPECT_FALSE(client.reportPath(1, translagatr::kPathDirect, one, 1)); // no session
    openSession(rig);

    std::vector<translagatr::PathPoint> many;
    for (int i = 0; i < 40; ++i) {
        many.push_back({i * 100, -i});
    }
    EXPECT_TRUE(client.reportPath(7, translagatr::kPathDirect, one, 1));
    EXPECT_TRUE(client.reportPath(8, translagatr::kPathAvoiding, many.data(), many.size()));
    EXPECT_EQ(client.stats().paths_dropped, 2u); // no session, then replaced
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.path().have; }, kLimit));
    const FakePath& path = rig.pi.path();
    EXPECT_EQ(path.command_id, 8u);
    EXPECT_EQ(path.mode, translagatr::kPathAvoiding);
    ASSERT_EQ(path.points.size(), 13u);
    EXPECT_EQ(path.points.front().x_mm, 0);
    EXPECT_EQ(path.points.back().x_mm, 3900);
    for (std::size_t i = 1; i < path.points.size(); ++i) {
        EXPECT_GT(path.points[i].x_mm, path.points[i - 1].x_mm);
    }
    EXPECT_EQ(sent(rig.bus, translagatr::kOpPathReport).size(), 1u);

    // Clearing, and a lost report is not resent.
    EXPECT_TRUE(client.reportPath(8, translagatr::kPathNone, nullptr, 0));
    runUntilSent(rig, translagatr::kOpPathReport);
    rig.bus.fault(BusFault::kDropRequest);
    rig.run(0.3);
    EXPECT_EQ(sent(rig.bus, translagatr::kOpPathReport).size(), 2u);
    EXPECT_TRUE(rig.pi.path().have);
    EXPECT_FALSE(client.reportPath(8, 3, nullptr, 0));
}

// ---------------------------------------------------------------------------
// Request ids, versions, link loss
// ---------------------------------------------------------------------------

TEST(Client, RequestIdWrapsToOne) {
    ClientConfig config;
    config.request_gap  = 0;
    config.state_period = 0;
    FakePi       pi;
    LoopbackPort port(pi);
    Client       client(port, [] { return 7u; }, config);
    Seconds      now = 0;
    while (pi.requests().size() < 65540 && now < 100.0) {
        now += 0.001;
        client.poll(now);
    }

    const auto& requests = pi.requests();
    bool        wrapped  = false;
    for (std::size_t i = 0; i < requests.size(); ++i) {
        ASSERT_NE(requests[i].request_id, 0);
        if (i > 0 && requests[i - 1].request_id == 0xFFFF) {
            EXPECT_EQ(requests[i].request_id, 1);
            wrapped = true;
        }
    }
    EXPECT_TRUE(wrapped);
    EXPECT_EQ(client.stats().timeouts, 0u);
    EXPECT_EQ(client.stats().unexpected, 0u);
    EXPECT_TRUE(client.connected(now));
}

TEST(Client, UnsupportedVersionIsTerminalWithSlowHello) {
    LinkRig rig;
    rig.pi.setVersion(3);
    rig.run(5.0);
    Client& client = rig.client();
    EXPECT_EQ(client.error(), LinkError::kUnsupportedVersion);
    EXPECT_EQ(client.peerVersion(), 3);
    EXPECT_FALSE(client.ready());
    EXPECT_EQ(client.session(), 0u);
    const std::size_t hellos = sent(rig.bus, translagatr::kOpHello).size();
    EXPECT_GE(hellos, 4u);
    EXPECT_LE(hellos, 6u);
    EXPECT_EQ(client.stats().timeouts, 0u);
}

TEST(Client, UnsupportedOpIsTerminal) {
    LinkRig rig;
    rig.pi.setUnsupportedOp(translagatr::kOpGetState);
    rig.run(3.0);
    Client& client = rig.client();
    EXPECT_EQ(client.error(), LinkError::kUnsupportedOp);
    EXPECT_FALSE(client.ready());
    EXPECT_LE(sent(rig.bus, translagatr::kOpHello).size(), 4u);
    EXPECT_LE(sent(rig.bus, translagatr::kOpGetState).size(), 4u);

    // A compatible Pi clears the error.
    rig.pi.setUnsupportedOp(0);
    openSession(rig);
    EXPECT_EQ(client.error(), LinkError::kNone);
}

TEST(Client, LinkLossDisconnectsAndRecovers) {
    LinkRig rig;
    openSession(rig);
    Client&        client  = rig.client();
    const uint32_t session = client.session();

    rig.bus.setPiPresent(false);
    const Seconds lost = rig.now();
    ASSERT_TRUE(rig.runUntil([&] { return !client.connected(rig.now()); }, kLimit));
    EXPECT_NEAR(rig.now() - lost, client.config().link_timeout, 0.03);
    EXPECT_GT(client.linkAge(rig.now()), client.config().link_timeout);
    EXPECT_TRUE(client.ready());
    EXPECT_GT(client.stats().timeouts, 0u);

    rig.bus.setPiPresent(true);
    ASSERT_TRUE(rig.runUntil([&] { return client.connected(rig.now()); }, kLimit));
    EXPECT_EQ(client.session(), session);
    EXPECT_EQ(client.stats().sessions, 1u);
    EXPECT_LT(client.linkAge(rig.now()), 0.03);
}

TEST(Client, WriteFailuresAreCountedAndRetried) {
    LinkRig rig;
    rig.bus.failWrites(2);
    openSession(rig);
    EXPECT_EQ(rig.client().stats().write_errors, 2u);
}
