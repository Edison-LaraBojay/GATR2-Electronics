// profile_sync_gtest.cpp
// Robot profile upload and apply against the fake Pi: chunked writes,
// resume, Pending, idempotent retry across Brain restarts, continuity loss on
// a changed profile, settled rejections, local rejection, and the placement
// and state source gates that depend on it.

#include "communigatr/client.h"

#include <gtest/gtest.h>
#include <vector>

#include "communigatr/link_driver.h"
#include "sim/link_rig.h"

using namespace communigatr;
using investigatr::kPi;
using investigatr::RobotStatus;

namespace
{

constexpr Seconds kLimit = 3.0;

// Three wheels, Pico IMU and four cameras: the largest document, two chunks.
RobotProfile largeProfile() {
    RobotProfile p;
    p.topology   = LocalizationTopology::kThreeWheel;
    p.wheels     = {{0, 0.024, 2048, 0.05, 0.15, 0.0, false},
                    {1, 0.024, 2048, 0.05, -0.15, 0.0, true},
                    {2, 0.024, 2048, -0.12, 0.0, kPi / 2, false}};
    p.imu_source = ImuSource::kPico;
    p.footprint  = {0.2, 0.2, 0.2, 0.2};
    p.cameras    = {{0, 0.1, 0.0, 0.3, 0, 0, 0}, {1, -0.1, 0.0, 0.3, 0, 0, 3.1},
                    {2, 0.0, 0.1, 0.3, 0, 0, 1.5}, {3, 0.0, -0.1, 0.3, 0, 0, -1.5}};
    return p;
}

// Encoder port 0 forward, port 1 sideways, VEX IMU on Smart Port 1.
RobotProfile benchProfile() {
    RobotProfile p;
    p.topology       = LocalizationTopology::kTwoWheelImu;
    p.wheels         = {{0, 0.024, 2048, 0.0, 0.10, 0.0, false},
                        {1, 0.024, 2048, -0.08, 0.0, kPi / 2, false}};
    p.imu_source     = ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = {0.2, 0.2, 0.2, 0.2};
    return p;
}

ClientConfig configFor(const RobotProfile& profile) {
    ClientConfig config;
    config.profile = makeProfileDocument(profile);
    return config;
}

std::vector<gatr2::BrainRequest> requests(const FakeBus& bus, uint8_t op) {
    std::vector<gatr2::BrainRequest> out;
    for (const gatr2::BrainRequest& r : bus.brainRequests()) {
        if (r.op == op) {
            out.push_back(r);
        }
    }
    return out;
}

void applied(LinkRig& rig) {
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
}

} // namespace

TEST(ProfileSync, UploadsInChunksThenAppliesThroughPending) {
    LinkRig rig(configFor(largeProfile()));
    rig.pi.setProfileMode(true);
    rig.pi.setProfileApplyDelay(4);
    Client&               client = rig.client();
    const ProfileStatus&  status = client.profile();
    const ProfileDocument doc    = makeProfileDocument(largeProfile());
    EXPECT_EQ(doc.len, gatr2::kProfileMaxLen);
    EXPECT_EQ(status.state, ProfileSync::kWaiting);
    EXPECT_EQ(status.id, profileId(doc));

    ASSERT_TRUE(rig.runUntil([&] { return client.ready(); }, kLimit));
    EXPECT_EQ(client.submitPlacement(1, 2, 3), 0u); // not applied yet
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kNoProfile);
    applied(rig);
    EXPECT_EQ(rig.pi.appliedProfile(), status.id);
    EXPECT_EQ(status.received, doc.len);
    EXPECT_EQ(status.reason, gatr2::kProfileReasonNone);

    const auto        writes = requests(rig.bus, gatr2::kOpProfileWrite);
    const std::size_t chunks = (doc.len + gatr2::kProfileChunkMax - 1) / gatr2::kProfileChunkMax;
    ASSERT_EQ(writes.size(), chunks);
    ASSERT_GE(chunks, 2u);
    uint16_t at = 0;
    for (const auto& w : writes) {
        EXPECT_EQ(w.offset, at);
        EXPECT_EQ(w.profile_id, status.id);
        EXPECT_EQ(w.total_len, doc.len);
        at = static_cast<uint16_t>(at + w.data_len);
    }
    EXPECT_EQ(writes.front().data_len, gatr2::kProfileChunkMax);
    EXPECT_EQ(at, doc.len);
    const auto applies = requests(rig.bus, gatr2::kOpProfileApply);
    EXPECT_GE(applies.size(), 2u); // Pending, then Ok
    EXPECT_EQ(rig.pi.profilesApplied(), 1);

    // Unplaced after the new profile; placement now allowed.
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kUnplaced);
    const PlacementTicket ticket = client.submitPlacement(100, 200, 0);
    ASSERT_NE(ticket, 0u);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kValid);
    EXPECT_FALSE(rig.bus.collision());
}

TEST(ProfileSync, LostChunkRepliesAndRequestsResumeWithoutGaps) {
    LinkRig rig(configFor(largeProfile()));
    rig.pi.setProfileMode(true);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    ASSERT_TRUE(rig.runUntil(
        [&] { return !requests(rig.bus, gatr2::kOpProfileWrite).empty(); }, kLimit));
    rig.bus.fault(BusFault::kDropReply); // first chunk taken, answer lost
    ASSERT_TRUE(rig.runUntil(
        [&] { return requests(rig.bus, gatr2::kOpProfileWrite).size() == 2; }, kLimit));
    rig.bus.fault(BusFault::kDropRequest); // this chunk never arrives
    applied(rig);
    EXPECT_EQ(rig.client().stats().timeouts, 2u);
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
    // Every write started at or below what the Pi held; no gap was ever sent.
    for (const auto& w : requests(rig.bus, gatr2::kOpProfileWrite)) {
        EXPECT_EQ(w.offset % gatr2::kProfileChunkMax, 0);
    }
}

TEST(ProfileSync, BrainRestartWithSameProfileIsIdempotent) {
    LinkRig rig(configFor(benchProfile()));
    rig.pi.setProfileMode(true);
    applied(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(500, -250, 9000);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
        kLimit));
    const uint32_t    epoch  = rig.pi.robot().odometry_epoch;
    const std::size_t writes = requests(rig.bus, gatr2::kOpProfileWrite).size();

    rig.rebootBrain();
    applied(rig);
    // The Pi already runs it: nothing uploaded, nothing reset, still placed.
    EXPECT_EQ(requests(rig.bus, gatr2::kOpProfileWrite).size(), writes);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch);
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kValid);
}

TEST(ProfileSync, ChangedProfileLosesContinuityAndNeedsPlacement) {
    LinkRig rig(configFor(benchProfile()));
    rig.pi.setProfileMode(true);
    applied(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(500, -250, 9000);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
        kLimit));
    const uint32_t epoch = rig.pi.robot().odometry_epoch;

    RobotProfile tuned            = benchProfile();
    tuned.wheels[1].travel_scale = 1.013; // a new measured correction
    rig.rebootBrain(configFor(tuned));
    applied(rig);
    EXPECT_EQ(rig.pi.appliedProfile(), profileId(makeProfileDocument(tuned)));
    EXPECT_EQ(rig.pi.profilesApplied(), 2);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch + 1);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().state().state.odometry_epoch == epoch + 1; }, kLimit));
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kUnplaced);
}

TEST(ProfileSync, RejectionIsSettledWithoutRetryStorm) {
    LinkRig rig(configFor(benchProfile()));
    rig.pi.setProfileMode(true);
    rig.pi.setProfileRejection(gatr2::kProfileReasonEncoderPort, 1);
    Client& client = rig.client();
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.profile().state == ProfileSync::kRejected; }, kLimit));
    EXPECT_EQ(client.profile().result, gatr2::kResultProfileRejected);
    EXPECT_EQ(client.profile().reason, gatr2::kProfileReasonEncoderPort);
    EXPECT_EQ(client.profile().detail, 1);
    rig.run(2.0);
    EXPECT_EQ(requests(rig.bus, gatr2::kOpProfileApply).size(), 1u);
    EXPECT_EQ(client.submitPlacement(1, 1, 1), 0u);
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kNoProfile);

    // Explicit resubmit: one more attempt; the Pi remembers the refusal.
    client.resubmitProfile();
    EXPECT_EQ(client.profile().state, ProfileSync::kWriting);
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.profile().state == ProfileSync::kRejected; }, kLimit));
    rig.run(1.0);
    EXPECT_EQ(requests(rig.bus, gatr2::kOpProfileApply).size(), 2u);

    // A new session tries once more.
    rig.rebootBrain();
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().state == ProfileSync::kRejected; }, kLimit));
    rig.run(1.0);
    EXPECT_EQ(requests(rig.bus, gatr2::kOpProfileApply).size(), 3u);
}

TEST(ProfileSync, PiWithoutBrainProfileSupportRejectsClearly) {
    LinkRig rig(configFor(benchProfile()));
    rig.pi.setProfileMode(false); // XML configured localization
    Client& client = rig.client();
    ASSERT_TRUE(rig.runUntil(
        [&] { return client.profile().state == ProfileSync::kRejected; }, kLimit));
    EXPECT_EQ(client.profile().reason, gatr2::kProfileReasonNotAccepted);
    EXPECT_STREQ(profileReasonName(client.profile().reason), "not accepted by this Pi");
    EXPECT_EQ(client.submitPlacement(1, 1, 1), 0u);
}

TEST(ProfileSync, LocallyInvalidProfileIsNeverSent) {
    RobotProfile bad    = benchProfile();
    bad.wheels[1].angle = 0.0; // parallel to wheel 0
    LinkRig rig(configFor(bad));
    rig.pi.setProfileMode(true);
    Client& client = rig.client();
    EXPECT_EQ(client.profile().state, ProfileSync::kInvalid);
    EXPECT_EQ(client.profile().reason, gatr2::kProfileReasonObservability);
    ASSERT_TRUE(rig.runUntil([&] { return client.ready(); }, kLimit));
    rig.run(1.0);
    EXPECT_TRUE(requests(rig.bus, gatr2::kOpProfileWrite).empty());
    EXPECT_TRUE(requests(rig.bus, gatr2::kOpProfileApply).empty());
    EXPECT_EQ(client.submitPlacement(1, 1, 1), 0u);
    client.resubmitProfile(); // no effect on a local rejection
    EXPECT_EQ(client.profile().state, ProfileSync::kInvalid);
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kNoProfile);

    // Bytes that do not decode are refused the same way.
    ClientConfig raw;
    raw.profile.len = 30;
    FakePi  pi;
    FakeBus bus(pi);
    Client  other(bus.brainPort(), [] { return 1u; }, raw);
    EXPECT_EQ(other.profile().state, ProfileSync::kInvalid);
    EXPECT_EQ(other.profile().reason, gatr2::kProfileReasonFormat);
}

TEST(ProfileSync, PiRestartReappliesAndRequiresPlacement) {
    LinkRig rig(configFor(benchProfile()));
    rig.pi.setProfileMode(true);
    applied(rig);
    const PlacementTicket ticket = rig.client().submitPlacement(500, -250, 9000);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
        kLimit));

    rig.pi.restart(0x5151);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    EXPECT_EQ(rig.client().profile().state, ProfileSync::kWaiting);
    applied(rig);
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kUnplaced);
}

TEST(ProfileSync, InterruptedUploadCompletesInTheNextSession) {
    LinkRig rig(configFor(largeProfile()));
    rig.pi.setProfileMode(true);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().received == gatr2::kProfileChunkMax; }, kLimit));

    // Another HELLO takes the session between the chunks. Staging survives.
    gatr2::BrainRequest hello;
    hello.op         = gatr2::kOpHello;
    hello.request_id = 1;
    hello.nonce      = 0xFEEDF00D;
    ASSERT_EQ(rig.pi.answer(hello).result, gatr2::kResultOk);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().session_losses == 1; }, kLimit));
    const std::size_t before = requests(rig.bus, gatr2::kOpProfileWrite).size();
    applied(rig);

    // The new session starts at offset 0; the Pi reports the held bytes and
    // only the rest is sent.
    const auto writes = requests(rig.bus, gatr2::kOpProfileWrite);
    ASSERT_GE(writes.size(), before + 2);
    EXPECT_EQ(writes[before].offset, 0);
    EXPECT_EQ(writes[before + 1].offset, gatr2::kProfileChunkMax);
    for (std::size_t i = before + 1; i < writes.size(); ++i) {
        EXPECT_GT(writes[i].offset, writes[i - 1].offset);
    }
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
}

TEST(ProfileSync, ReplacedStagingIsRewritten) {
    LinkRig rig(configFor(benchProfile()));
    rig.pi.setProfileMode(true);
    rig.pi.setProfileApplyDelay(40);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().state == ProfileSync::kApplying; }, kLimit));

    // Something else staged another document on the Pi.
    const ProfileDocument other = makeProfileDocument(largeProfile());
    rig.pi.replaceStaging(profileId(other), other.len);
    const std::size_t before = requests(rig.bus, gatr2::kOpProfileWrite).size();

    applied(rig);
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
    EXPECT_GT(requests(rig.bus, gatr2::kOpProfileWrite).size(), before); // rewritten
}

TEST(ProfileSync, NoProfileConfiguredSendsNothing) {
    LinkRig rig;
    Client& client = rig.client();
    EXPECT_EQ(client.profile().state, ProfileSync::kNone);
    EXPECT_FALSE(client.profileConfigured());
    ASSERT_TRUE(rig.runUntil([&] { return client.ready(); }, kLimit));
    rig.run(0.5);
    EXPECT_TRUE(requests(rig.bus, gatr2::kOpProfileWrite).empty());
    EXPECT_TRUE(requests(rig.bus, gatr2::kOpProfileApply).empty());
    EXPECT_NE(client.submitPlacement(1, 1, 1), 0u);
}
