// client_gtest.cpp
// Client against the fake Pi on the fake bus: sessions, correlation,
// retries, placement acknowledgement, selection, restarts and faults.

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

gatr2::BrainRequest decoded(const std::vector<uint8_t>& frame) {
    gatr2::BrainRequest request;
    EXPECT_TRUE(
        gatr2::decodeBrainRequest(frame.data(), static_cast<uint16_t>(frame.size()), request));
    return request;
}

std::vector<uint8_t> encode(const gatr2::BrainReply& reply) {
    std::vector<uint8_t> frame(gatr2::kMaxFrameLen);
    frame.resize(gatr2::encodeBrainReply(reply, frame.data(), static_cast<uint16_t>(frame.size())));
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
    runUntilSent(rig, gatr2::kOpGetState);
    const gatr2::BrainRequest request = rig.bus.brainRequests().back();
    gatr2::BrainReply         forged;
    forged.op          = gatr2::kOpGetState;
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

TEST(Client, PollsStateWithFreshIds) {
    LinkRig rig;
    openSession(rig);
    rig.run(0.5);
    const auto polls = sent(rig.bus, gatr2::kOpGetState);
    ASSERT_GE(polls.size(), 15u);
    for (std::size_t i = 1; i < polls.size(); ++i) {
        EXPECT_NE(decoded(polls[i]).request_id, decoded(polls[i - 1]).request_id);
    }
    EXPECT_EQ(rig.client().stats().resends, 0u);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

TEST(ClientBenchImu, OptionalCallbackReplacesOnlyStatePollAndPreservesSampleStamp) {
    BenchImuSample sample{true, 1234, -450123};
    ClientConfig config;
    config.bench_imu = [&] { return sample; };
    LinkRig rig(config);
    openSession(rig);
    rig.run(0.08);
    EXPECT_TRUE(sent(rig.bus, gatr2::kOpGetState).empty());
    const auto polls = sent(rig.bus, gatr2::kOpGetStateWithImu);
    ASSERT_GE(polls.size(), 2u);
    for (const auto& bytes : polls) {
        const auto request = decoded(bytes);
        EXPECT_EQ(request.imu_flags, gatr2::kBenchImuValid);
        EXPECT_EQ(request.imu_stamp_ms, 1234u);
        EXPECT_EQ(request.imu_rotation_mdeg, -450123);
    }
    sample = {false, 1234, -450123};
    runUntilSent(rig, gatr2::kOpGetStateWithImu);
    EXPECT_EQ(rig.bus.brainRequests().back().imu_flags, 0);
    EXPECT_EQ(rig.bus.brainRequests().back().imu_stamp_ms, 1234u);
}

TEST(ClientBenchImu, PendingPlacementAlwaysInterleavesImuPollBeforeRetry) {
    ClientConfig config;
    config.pending_retry = 0;
    config.bench_imu = [] { return BenchImuSample{true, 1234, 90000}; };
    LinkRig rig(config);
    openSession(rig);
    rig.pi.setApplyDelay(5);
    const auto ticket = rig.client().submitPlacement(610, -457, -9000);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    bool placement_seen = false;
    bool imu_since_placement = false;
    for (const auto& request : rig.bus.brainRequests()) {
        if (request.op == gatr2::kOpGetStateWithImu) {
            imu_since_placement = true;
        } else if (request.op == gatr2::kOpSetPose) {
            if (placement_seen) {
                EXPECT_TRUE(imu_since_placement);
            }
            placement_seen = true;
            imu_since_placement = false;
        }
    }
    const auto placements = sent(rig.bus, gatr2::kOpSetPose);
    ASSERT_GE(placements.size(), 2u);
    for (const auto& bytes : placements) EXPECT_EQ(bytes, placements.front());
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
}

TEST(ClientBenchImu, DroppedPlacementReplyPollsImuThenResendsIdenticalPlacement) {
    ClientConfig config;
    config.bench_imu = [] { return BenchImuSample{true, 1234, 90000}; };
    LinkRig rig(config);
    openSession(rig);
    const auto ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, gatr2::kOpSetPose);
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, gatr2::kOpSetPose);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
    bool after_placement = false;
    bool saw_imu = false;
    for (const auto& request : rig.bus.brainRequests()) {
        if (request.op == gatr2::kOpSetPose) {
            if (after_placement) {
                EXPECT_TRUE(saw_imu);
            }
            after_placement = true;
        } else if (after_placement && request.op == gatr2::kOpGetStateWithImu) {
            saw_imu = true;
        }
    }
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
}

TEST(ClientBenchImu, UnsupportedPeerReportsErrorRatherThanPretendingImuWasSent) {
    ClientConfig config;
    config.bench_imu = [] { return BenchImuSample{true, 1234, 0}; };
    LinkRig rig(config);
    rig.pi.setUnsupportedOp(gatr2::kOpGetStateWithImu);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().error() == LinkError::kUnsupportedOp; }, kLimit));
    EXPECT_FALSE(rig.client().ready());
    EXPECT_TRUE(sent(rig.bus, gatr2::kOpGetState).empty());
}

// ---------------------------------------------------------------------------
// Faults on the bus
// ---------------------------------------------------------------------------

TEST(Client, FragmentedReplyAccepted) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, gatr2::kOpGetState);
    const uint32_t replies = rig.client().stats().replies;
    rig.bus.fault(BusFault::kFragment, 0.004);
    rig.run(0.05);
    EXPECT_GT(rig.client().stats().replies, replies);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

TEST(Client, TruncatedThenValidReplyAccepted) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, gatr2::kOpGetState);
    const uint32_t replies = rig.client().stats().replies;
    rig.bus.fault(BusFault::kTruncate, 0.002);
    rig.run(0.05);
    EXPECT_GT(rig.client().stats().replies, replies);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
}

TEST(Client, CorruptReplyTimesOutAndNextPollHasNewId) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, gatr2::kOpGetState);
    const uint16_t lost = rig.bus.brainRequests().back().request_id;
    rig.bus.fault(BusFault::kCorrupt);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().timeouts == 1; }, kLimit));
    runUntilSent(rig, gatr2::kOpGetState);
    EXPECT_NE(rig.bus.brainRequests().back().request_id, lost);
    EXPECT_EQ(rig.client().stats().resends, 0u);
}

TEST(Client, DroppedStateReplyNextPollHasNewId) {
    LinkRig rig;
    openSession(rig);
    runUntilSent(rig, gatr2::kOpGetState);
    const uint16_t lost = rig.bus.brainRequests().back().request_id;
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().timeouts == 1; }, kLimit));
    runUntilSent(rig, gatr2::kOpGetState);
    EXPECT_NE(rig.bus.brainRequests().back().request_id, lost);
    EXPECT_EQ(rig.client().stats().resends, 0u);
    EXPECT_TRUE(rig.client().ready());
}

TEST(Client, DuplicateReplyDoesNotApplyTwice) {
    LinkRig rig;
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, gatr2::kOpSetPose);
    rig.bus.fault(BusFault::kDuplicate);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    rig.run(0.1);
    EXPECT_GE(rig.client().stats().uncorrelated, 1u);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    EXPECT_EQ(sent(rig.bus, gatr2::kOpSetPose).size(), 1u);
}

TEST(Client, DroppedPlacementReplyResendsSameBytes) {
    LinkRig rig;
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(610, -457, -9000);
    runUntilSent(rig, gatr2::kOpSetPose);
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, gatr2::kOpSetPose);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    EXPECT_EQ(rig.client().stats().resends, 1u);
}

TEST(Client, DroppedPlacementRequestResendsSameBytes) {
    LinkRig rig;
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(1, 2, 3);
    runUntilSent(rig, gatr2::kOpSetPose);
    rig.bus.fault(BusFault::kDropRequest);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, gatr2::kOpSetPose);
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
    runUntilSent(rig, gatr2::kOpGetState);
    const gatr2::BrainRequest request = rig.bus.brainRequests().back();

    gatr2::BrainReply stale;
    stale.op                   = gatr2::kOpGetState;
    stale.session              = session;
    stale.request_id           = static_cast<uint16_t>(request.request_id - 1); // old id
    stale.pi_instance          = rig.pi.piInstance() + 1; // would mean a restart
    stale.state.x_mm           = 111;
    std::vector<uint8_t> bytes = encode(stale);

    stale.request_id                 = request.request_id;
    stale.session                    = session ^ 0x1u; // old session
    const std::vector<uint8_t> other = encode(stale);
    bytes.insert(bytes.end(), other.begin(), other.end());

    stale.op                            = gatr2::kOpSetPose; // wrong op
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
    gatr2::BrainReply early;
    early.op                         = gatr2::kOpHello;
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
    gatr2::BrainRequest hello;
    hello.op         = gatr2::kOpHello;
    hello.request_id = 7;
    hello.nonce      = 0xABCDEF;
    ASSERT_EQ(rig.pi.answer(hello).result, gatr2::kResultOk);

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
        [&] { return client.placementStatus(ticket).result == gatr2::kResultPending; }, kLimit));

    rig.pi.restart(0x77);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(ticket) == PlacementResult::kSessionLost; }, kLimit));
    EXPECT_FALSE(client.placementPending());
    const std::size_t before = sent(rig.bus, gatr2::kOpSetPose).size();

    openSession(rig);
    rig.run(0.5);
    EXPECT_EQ(sent(rig.bus, gatr2::kOpSetPose).size(), before);
    for (const gatr2::BrainRequest& r : rig.pi.requests()) {
        EXPECT_FALSE(r.op == gatr2::kOpSetPose && r.session == rig.pi.session());
    }
    EXPECT_EQ(rig.pi.placementsApplied(), 0);
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kSessionLost);
}

TEST(Client, StaleHelloGetsNewNonce) {
    LinkRig rig;
    rig.nonces = {0xAAAA0001};
    openSession(rig);
    const uint32_t    old    = rig.client().session();
    const std::size_t before = sent(rig.bus, gatr2::kOpHello).size();

    // The rebooted Brain draws the same nonce first.
    rig.rebootBrain();
    rig.nonces = {0xAAAA0001, 0xAAAA0002};
    openSession(rig);
    EXPECT_EQ(rig.client().stats().stale_hellos, 1u);
    EXPECT_NE(rig.client().session(), old);

    const auto hellos = sent(rig.bus, gatr2::kOpHello);
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
    const auto hellos = sent(rig.bus, gatr2::kOpHello);
    ASSERT_EQ(hellos.size(), 3u);
    EXPECT_EQ(decoded(hellos[1]).nonce, 0x42u);
    EXPECT_NE(decoded(hellos[2]).nonce, 0x42u);
    EXPECT_NE(decoded(hellos[2]).nonce, 0u);
}

TEST(Client, BrainRebootIgnoresOldSession) {
    FakeBusConfig bus;
    bus.reply_delay = 0.010;
    LinkRig      rig({}, bus);
    FakeLandmark landmark;
    rig.pi.setLandmark(3, landmark);
    rig.nonces = {0xA0A0A0A0};

    // Session A selects a landmark.
    openSession(rig);
    rig.client().selectLandmark(3);
    ASSERT_TRUE(
        rig.runUntil([&] { return rig.client().selection() == SelectionState::kActive; }, kLimit));
    const uint32_t session_a = rig.client().session();

    gatr2::BrainReply old_hello;
    old_hello.op          = gatr2::kOpHello;
    old_hello.session     = session_a;
    old_hello.request_id  = 1;
    old_hello.pi_instance = rig.pi.piInstance();
    old_hello.nonce       = 0xA0A0A0A0;

    gatr2::BrainReply old_state;
    old_state.op          = gatr2::kOpGetState;
    old_state.session     = session_a;
    old_state.request_id  = 2;
    old_state.pi_instance = rig.pi.piInstance();
    old_state.state.x_mm  = 4242;

    // Session B: request ids restart at 1; A's replies arrive late.
    rig.rebootBrain();
    Client& client = rig.client();
    EXPECT_EQ(client.selection(), SelectionState::kNotRequested);
    runUntilSent(rig, gatr2::kOpHello);
    EXPECT_EQ(rig.bus.brainRequests().back().request_id, 1);
    rig.bus.sendToBrain(encode(old_hello), rig.now());
    rig.step();
    rig.step();
    EXPECT_EQ(client.session(), 0u);

    ASSERT_TRUE(rig.runUntil([&] { return client.session() != 0; }, kLimit));
    EXPECT_NE(client.session(), session_a);
    EXPECT_FALSE(rig.pi.landmarkRequested());
    runUntilSent(rig, gatr2::kOpGetState);
    EXPECT_EQ(rig.bus.brainRequests().back().request_id, 2);
    rig.bus.sendToBrain(encode(old_state), rig.now());
    openSession(rig);
    EXPECT_NE(client.state().state.x_mm, 4242);
    EXPECT_EQ(client.stats().uncorrelated, 2u);

    // A delayed session A request changes nothing on the Pi.
    gatr2::BrainRequest old_select;
    old_select.op           = gatr2::kOpSelectLandmark;
    old_select.session      = session_a;
    old_select.request_id   = 9;
    old_select.landmark_id  = 3;
    old_select.select_flags = gatr2::kSelectFlagSelected;
    EXPECT_EQ(rig.pi.answer(old_select).result, gatr2::kResultUnknownSession);
    EXPECT_FALSE(rig.pi.landmarkRequested());
    EXPECT_EQ(client.selection(), SelectionState::kNotRequested);
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
    EXPECT_EQ(client.placementStatus(ticket).result, gatr2::kResultOk);
    EXPECT_EQ(client.placementResult(ticket), PlacementResult::kPending);
    EXPECT_TRUE(client.placementPending());
    EXPECT_EQ(client.state().state.anchor_revision, anchor);

    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    EXPECT_FALSE(client.placementPending());
    EXPECT_EQ(client.state().state.anchor_revision, anchor + 1);
    EXPECT_EQ(client.state().state.x_mm, 100);
    EXPECT_EQ(client.state().state.heading_cdeg, 9000);
    EXPECT_NE(client.state().state.robot_flags & gatr2::kRobotAnchorCommand, 0);
}

TEST(Client, PendingPlacementResendsSameBytesUntilApplied) {
    LinkRig rig;
    rig.pi.setApplyDelay(3);
    openSession(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(-300, 50, -17999);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    const auto frames = sent(rig.bus, gatr2::kOpSetPose);
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
    EXPECT_EQ(client.placementStatus(ticket).result, gatr2::kResultPending);
    EXPECT_LE(rig.now() - start, client.config().placement_deadline + 0.1);

    const std::size_t count = sent(rig.bus, gatr2::kOpSetPose).size();
    EXPECT_LE(count, static_cast<std::size_t>(client.config().placement_attempts));
    rig.run(0.5);
    EXPECT_EQ(sent(rig.bus, gatr2::kOpSetPose).size(), count);
    EXPECT_TRUE(client.connected(rig.now()));
}

TEST(Client, SilentPiPlacementTimesOut) {
    LinkRig rig;
    openSession(rig);
    rig.bus.setPiPresent(false);
    const PlacementTicket ticket = rig.client().submitPlacement(1, 1, 1);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().placementPending(); }, kLimit));
    EXPECT_EQ(rig.client().placementResult(ticket), PlacementResult::kTimedOut);
    const auto frames = sent(rig.bus, gatr2::kOpSetPose);
    EXPECT_EQ(frames.size(), static_cast<std::size_t>(rig.client().config().placement_attempts));
    for (const auto& frame : frames) {
        EXPECT_EQ(frame, frames[0]);
    }
}

TEST(Client, PlacementAckWithoutStateExpiresAndAllowsNewPlacement) {
    ClientConfig config;
    config.placement_deadline = 0.08;
    config.placement_attempts = 1;
    LinkRig rig(config);
    openSession(rig);
    Client& client = rig.client();
    const PlacementTicket first = client.submitPlacement(100, 200, 300);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementStatus(first).anchor_revision != 0; }, kLimit));
    ASSERT_EQ(client.placementResult(first), PlacementResult::kPending);

    // SET_POSE Ok arrived, but every confirming GET_STATE is lost.
    rig.bus.setPiPresent(false);
    ASSERT_TRUE(rig.runUntil([&] { return !client.placementPending(); }, 0.1));
    EXPECT_EQ(client.placementResult(first), PlacementResult::kTimedOut);
    EXPECT_EQ(client.placementStatus(first).result, gatr2::kResultOk);

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
    Client& client = rig.client();
    const PlacementTicket ticket = client.submitPlacement(100, 200, 300);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementStatus(ticket).anchor_revision != 0; }, kLimit));
    runUntilSent(rig, gatr2::kOpGetState);

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
    FakePi pi;
    Client client(port, [] { return 7u; }, config);
    client.poll(0.0);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(0.001);

    const PlacementTicket first = client.submitPlacement(100, 200, 300);
    client.poll(0.006);
    const auto old_reply = encode(pi.answer(decoded(port.writes.back())));
    client.poll(0.027);
    ASSERT_EQ(client.placementResult(first), PlacementResult::kTimedOut);
    const PlacementTicket second = client.submitPlacement(400, 500, 600);
    ASSERT_NE(second, 0u);

    // The first request still owns the bus response window. Its late Ok
    // must release that window without acknowledging the unsent second one.
    port.reads.push_back(old_reply);
    client.poll(0.028);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kPending);
    EXPECT_EQ(client.placementStatus(second).anchor_revision, 0u);
    ASSERT_EQ(port.writes.size(), 2u);

    client.poll(0.034);
    ASSERT_EQ(port.writes.size(), 3u);
    ASSERT_EQ(decoded(port.writes.back()).x_mm, 400);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(0.035);
    client.poll(0.041);
    ASSERT_EQ(decoded(port.writes.back()).op, gatr2::kOpGetState);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(0.042);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kApplied);
    EXPECT_EQ(client.state().state.x_mm, 400);
}

TEST(Client, ExpiredRequestTimeoutDoesNotExpireQueuedReplacement) {
    ClientConfig config;
    config.placement_deadline = 0.020;
    ScriptedPort port;
    FakePi pi;
    Client client(port, [] { return 7u; }, config);
    client.poll(0.0);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(0.001);

    const PlacementTicket first = client.submitPlacement(100, 200, 300);
    client.poll(0.006);
    client.poll(0.027);
    ASSERT_EQ(client.placementResult(first), PlacementResult::kTimedOut);
    const PlacementTicket second = client.submitPlacement(400, 500, 600);
    ASSERT_NE(second, 0u);

    // Keep the original response window open until its transport timeout;
    // that timeout belongs to the old ticket, not the queued replacement.
    client.poll(0.040);
    ASSERT_EQ(port.writes.size(), 2u);
    client.poll(0.067);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kPending);
    ASSERT_EQ(port.writes.size(), 2u);
    client.poll(0.073);
    ASSERT_EQ(port.writes.size(), 3u);
    ASSERT_EQ(decoded(port.writes.back()).x_mm, 400);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(0.074);
    client.poll(0.080);
    ASSERT_EQ(decoded(port.writes.back()).op, gatr2::kOpGetState);
    port.reads.push_back(encode(pi.answer(decoded(port.writes.back()))));
    client.poll(0.081);
    EXPECT_EQ(client.placementResult(second), PlacementResult::kApplied);
}

TEST(Client, OnePlacementAtATime) {
    LinkRig               rig;
    Client&               client = rig.client();
    const PlacementTicket first  = client.submitPlacement(1, 2, 3);
    EXPECT_NE(first, 0u);
    EXPECT_EQ(client.submitPlacement(4, 5, 6), 0u);
    EXPECT_EQ(client.placementResult(0), PlacementResult::kNone);

    // Submitted before any session: sent once one opens.
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
    EXPECT_TRUE(client.connected(now));
}

TEST(Client, UnsupportedVersionIsTerminalWithSlowHello) {
    LinkRig rig;
    rig.pi.setVersion(4);
    rig.run(5.0);
    Client& client = rig.client();
    EXPECT_EQ(client.error(), LinkError::kUnsupportedVersion);
    EXPECT_EQ(client.peerVersion(), 4);
    EXPECT_FALSE(client.ready());
    EXPECT_EQ(client.session(), 0u);
    const std::size_t hellos = sent(rig.bus, gatr2::kOpHello).size();
    EXPECT_GE(hellos, 4u);
    EXPECT_LE(hellos, 6u);
    EXPECT_EQ(client.stats().timeouts, 0u);
}

TEST(Client, UnsupportedOpIsTerminal) {
    LinkRig rig;
    rig.pi.setUnsupportedOp(gatr2::kOpGetState);
    rig.run(3.0);
    Client& client = rig.client();
    EXPECT_EQ(client.error(), LinkError::kUnsupportedOp);
    EXPECT_FALSE(client.ready());
    EXPECT_LE(sent(rig.bus, gatr2::kOpHello).size(), 4u);
    EXPECT_LE(sent(rig.bus, gatr2::kOpGetState).size(), 4u);

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

// ---------------------------------------------------------------------------
// Landmark selection
// ---------------------------------------------------------------------------

TEST(Client, SelectionStatuses) {
    LinkRig      rig;
    FakeLandmark landmark;
    landmark.x_mm         = 2000;
    landmark.y_mm         = 1000;
    landmark.heading_cdeg = -4500;
    landmark.age_ms       = 120;
    rig.pi.setLandmark(3, landmark);
    openSession(rig);
    Client& client = rig.client();
    EXPECT_EQ(client.selection(), SelectionState::kNotRequested);

    client.selectLandmark(3);
    EXPECT_EQ(client.selection(), SelectionState::kPending);
    ASSERT_TRUE(
        rig.runUntil([&] { return client.selection() == SelectionState::kActive; }, kLimit));
    EXPECT_EQ(client.state().state.landmark_id, 3);
    EXPECT_EQ(client.state().state.landmark_source, gatr2::kLandmarkSourceObserved);
    EXPECT_EQ(client.state().state.lm_x_mm, 2000);
    EXPECT_EQ(client.state().state.landmark_age_ms, 120);

    // Unknown id: settled, not resent.
    client.selectLandmark(9);
    ASSERT_TRUE(rig.runUntil([&] { return client.selection() == SelectionState::kUnknownLandmark; },
                             kLimit));
    const std::size_t selects = sent(rig.bus, gatr2::kOpSelectLandmark).size();
    rig.run(1.0);
    EXPECT_EQ(sent(rig.bus, gatr2::kOpSelectLandmark).size(), selects);
    EXPECT_EQ(client.selection(), SelectionState::kUnknownLandmark);

    // Release.
    client.selectLandmark(0);
    EXPECT_EQ(client.selection(), SelectionState::kNotRequested);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.pi.landmarkRequested(); }, kLimit));
    EXPECT_EQ(decoded(sent(rig.bus, gatr2::kOpSelectLandmark).back()).select_flags, 0);

    // World estimation noop.
    rig.pi.setWorldEstimationNoop(true);
    client.selectLandmark(3);
    ASSERT_TRUE(
        rig.runUntil([&] { return client.selection() == SelectionState::kUnsupported; }, kLimit));
}

TEST(Client, SelectionRetriesSameBytes) {
    LinkRig rig;
    rig.pi.setLandmark(3, FakeLandmark{});
    openSession(rig);
    rig.client().selectLandmark(3);
    runUntilSent(rig, gatr2::kOpSelectLandmark);
    rig.bus.fault(BusFault::kDropReply);
    ASSERT_TRUE(
        rig.runUntil([&] { return rig.client().selection() == SelectionState::kActive; }, kLimit));
    const auto frames = sent(rig.bus, gatr2::kOpSelectLandmark);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], frames[1]);
}

TEST(Client, SilentSelectBacksOffThenRetries) {
    LinkRig rig;
    rig.pi.setLandmark(3, FakeLandmark{});
    openSession(rig);
    Client& client = rig.client();
    rig.bus.setPiPresent(false);
    client.selectLandmark(3);
    rig.run(0.6);
    const std::size_t first = sent(rig.bus, gatr2::kOpSelectLandmark).size();
    EXPECT_EQ(first, static_cast<std::size_t>(client.config().select_attempts));

    rig.bus.setPiPresent(true);
    ASSERT_TRUE(
        rig.runUntil([&] { return client.selection() == SelectionState::kActive; }, kLimit));
    const auto frames = sent(rig.bus, gatr2::kOpSelectLandmark);
    EXPECT_NE(decoded(frames.back()).request_id, decoded(frames.front()).request_id);
}

TEST(Client, SessionChangeClearsSelectionAndSelectsAgain) {
    LinkRig rig;
    rig.pi.setLandmark(3, FakeLandmark{});
    openSession(rig);
    Client& client = rig.client();
    client.selectLandmark(3);
    ASSERT_TRUE(
        rig.runUntil([&] { return client.selection() == SelectionState::kActive; }, kLimit));

    rig.pi.restart(0x99);
    ASSERT_TRUE(rig.runUntil([&] { return client.stats().pi_restarts == 1; }, kLimit));
    EXPECT_EQ(client.selection(), SelectionState::kPending);
    EXPECT_FALSE(rig.pi.landmarkRequested());
    ASSERT_TRUE(
        rig.runUntil([&] { return client.selection() == SelectionState::kActive; }, kLimit));
    EXPECT_TRUE(rig.pi.landmarkRequested());
    EXPECT_EQ(rig.pi.landmarkWireId(), 3);
}
