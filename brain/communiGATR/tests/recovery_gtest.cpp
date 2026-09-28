// recovery_gtest.cpp
// Restart, reconnect and reinitialize scenarios over both transports: the
// RS-485 half-duplex bus and the USB console with NG1 lines. Brain and Pi
// restarts, cable pulls, stale replies, Pico restarts, explicit
// reinitialize and recalibrate, and no resumption of interrupted commands.

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "communigatr/link_driver.h"
#include "communigatr/usb_line.h"
#include "sim/link_rig.h"

using namespace communigatr;
using investigatr::kPi;
using investigatr::Pose;
using investigatr::RobotStatus;

namespace
{

constexpr Seconds kLimit = 5.0;

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

ClientConfig benchConfig() {
    ClientConfig config;
    config.profile   = makeProfileDocument(benchProfile());
    config.bench_imu = [] { return BenchImuSample{true, 1000, 0}; };
    return config;
}

std::string name(const testing::TestParamInfo<RigTransport>& info) {
    return info.param == RigTransport::kUsb ? "Usb" : "Rs485";
}

class Recovery : public testing::TestWithParam<RigTransport> {
protected:
    Recovery() : rig(benchConfig(), FakeBusConfig{}, GetParam()) {
        rig.pi.setProfileMode(true);
        rig.pi.setField(makeFakeField(12));
    }

    // Profile applied, placed, field published.
    void ready(const Pose& pose = Pose{0.5, 0.4, 0.0}) {
        ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
        place(pose);
        ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().generation != 0; }, kLimit));
    }

    void place(const Pose& pose) {
        const PlacementTicket ticket = rig.driver().place(pose);
        ASSERT_NE(ticket, 0u);
        ASSERT_TRUE(rig.runUntil(
            [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
            kLimit));
    }

    RobotStatus status() { return rig.driver().robot(rig.now()).status; }

    // Link cut in both directions, as the chosen transport loses it.
    void cut(bool cut_off) {
        if (GetParam() == RigTransport::kUsb) {
            rig.usb.setPlugged(!cut_off);
        } else {
            rig.bus.setPiPresent(!cut_off);
        }
    }

    // Raw frame to the Brain, on the chosen transport.
    void inject(const gatr2::BrainReply& reply) {
        std::vector<uint8_t> frame(gatr2::kMaxFrameLen);
        frame.resize(gatr2::encodeBrainReply(reply, frame.data(), gatr2::kMaxFrameLen));
        if (GetParam() == RigTransport::kUsb) {
            char              line[kUsbLineMax];
            const std::size_t n = encodeUsbLine(frame.data(), frame.size(), line, sizeof(line));
            rig.usb.textToBrain(std::string(line, n));
        } else {
            rig.bus.sendToBrain(frame, rig.now());
        }
    }

    std::size_t piSaw(uint8_t op) const {
        std::size_t n = 0;
        for (const gatr2::BrainRequest& r : rig.pi.requests()) {
            n += r.op == op ? 1 : 0;
        }
        return n;
    }

    LinkRig rig;
};

} // namespace

TEST_P(Recovery, BrainRestartKeepsProfilePlacementAndMap) {
    ready();
    const uint32_t epoch    = rig.pi.robot().odometry_epoch;
    const uint32_t anchor   = rig.pi.robot().anchor_revision;
    const uint32_t sessions = rig.pi.sessionsOpened();
    const int      writes   = rig.pi.profileWrites();
    const Pose     pose     = rig.driver().robot(rig.now()).pose;

    rig.rebootBrain();
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kValid; }, kLimit));
    EXPECT_EQ(rig.pi.sessionsOpened(), static_cast<int>(sessions) + 1);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch);
    EXPECT_EQ(rig.pi.robot().anchor_revision, anchor); // no placement needed
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    EXPECT_EQ(rig.pi.profileWrites(), writes); // the state shows it applied: nothing uploaded
    EXPECT_EQ(rig.driver().robot(rig.now()).pose.x, pose.x);
    EXPECT_EQ(rig.driver().robot(rig.now()).pose.y, pose.y);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().generation != 0; }, kLimit));
}

// The Pi still applying the same profile when the Brain comes back: the new
// Brain uploads and applies it again, and the Pi swaps it in once.
TEST_P(Recovery, BrainRestartWhileThePiAppliesTheSameProfileAppliesItOnce) {
    rig.pi.setProfileApplyDelay(40);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().result == gatr2::kResultPending; }, kLimit));
    rig.rebootBrain();
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    place(Pose{0.3, 0.3, 0.0});
    const uint32_t epoch = rig.pi.robot().odometry_epoch;
    rig.rebootBrain();
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kValid; }, kLimit));
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch);
}

TEST_P(Recovery, PiRestartInTheMiddleOfTheProfileUpload) {
    RobotProfile large = benchProfile();
    for (uint8_t slot = 0; slot < gatr2::kProfileMaxCameras; ++slot) {
        CameraMount c;
        c.slot = slot;
        c.z    = 0.3;
        large.cameras.push_back(c);
    }
    ClientConfig config = benchConfig();
    config.profile      = makeProfileDocument(large);
    ASSERT_GT(config.profile.len, gatr2::kProfileChunkMax);
    rig.rebootBrain(config);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.pi.stagingReceived() == gatr2::kProfileChunkMax; }, kLimit));
    rig.pi.restart(0x0D15EA5E); // staging gone with the process
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    EXPECT_EQ(rig.client().stats().pi_restarts, 1u);
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    rig.run(0.2);
    EXPECT_EQ(status(), RobotStatus::kUnplaced);
}

TEST_P(Recovery, PlacementIsRefusedUntilTheProfileApplies) {
    rig.pi.setProfileApplyDelay(60);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    for (int i = 0; i < 10 && !rig.client().profileApplied(); ++i) {
        EXPECT_EQ(rig.driver().place(Pose{0.5, 0.5, 0.0}), 0u);
        EXPECT_EQ(rig.client().submitPlacement(500, 500, 0), 0u);
        rig.run(0.02);
    }
    EXPECT_EQ(piSaw(gatr2::kOpSetPose), 0u);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    place(Pose{0.5, 0.5, 0.0});
    EXPECT_EQ(status(), RobotStatus::kValid);
}

// Pending replies lost while the Pico works: the same request id is asked
// again and the Pi answers from its record; the control runs once and its
// final result arrives.
TEST_P(Recovery, ControlPendingThroughLostRepliesRunsOnce) {
    ready();
    for (const uint8_t failure : {gatr2::kControlDetailNone, gatr2::kControlDetailImuAbsent}) {
        SCOPED_TRACE(int(failure));
        const int executed = rig.pi.controlsExecuted();
        rig.pi.setControlFailure(failure);
        rig.pi.setControlPendingRequests(40);
        const ControlTicket ticket = rig.client().reinitImu();
        ASSERT_NE(ticket, 0u);
        ASSERT_TRUE(rig.runUntil(
            [&] { return rig.client().controlStatus(ticket).result == gatr2::kResultPending; },
            kLimit));
        cut(true);
        rig.run(0.15);
        cut(false);
        ASSERT_TRUE(rig.runUntil([&] { return !rig.client().controlPending(); }, kLimit));
        const ControlStatus s = rig.client().controlStatus(ticket);
        if (failure == gatr2::kControlDetailNone) {
            EXPECT_EQ(s.state, ControlResult::kOk);
        } else {
            EXPECT_EQ(s.state, ControlResult::kFailed);
            EXPECT_EQ(s.detail, failure);
        }
        EXPECT_EQ(rig.pi.controlsExecuted(), executed + 1);
    }
    rig.pi.setControlFailure(gatr2::kControlDetailNone);
}

TEST_P(Recovery, PiRestartReappliesProfileAndAsksForPlacement) {
    ready();
    rig.pi.restart(0x0D15EA5E);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    rig.run(0.2);
    EXPECT_EQ(status(), RobotStatus::kUnplaced); // never back to the start pose by itself
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().field().pi_instance == 0x0D15EA5Eu; }, kLimit));
    EXPECT_EQ(rig.client().stats().maps, 1u); // cached map reused
    place(Pose{1.0, 1.0, kPi});
    EXPECT_EQ(status(), RobotStatus::kValid);
}

TEST_P(Recovery, LinkInterruptionResumesTheSameSessionAndPose) {
    ready();
    const uint32_t                     session = rig.client().session();
    const investigatr::FrameGeneration frame   = rig.driver().robot(rig.now()).frame;
    cut(true);
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kNoLink; }, kLimit));
    rig.run(1.0);
    cut(false);
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kValid; }, kLimit));
    EXPECT_EQ(rig.client().session(), session);
    EXPECT_EQ(rig.client().stats().sessions, 1u);
    EXPECT_EQ(rig.driver().robot(rig.now()).frame, frame);
    EXPECT_EQ(rig.pi.placementsApplied(), 1);
    if (GetParam() == RigTransport::kUsb) {
        EXPECT_GE(rig.usb.unansweredAfterReopen(), 1);
    }
}

TEST_P(Recovery, TransfersInterruptedByTheLinkComplete) {
    rig.pi.setField(makeFakeField(80));
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().fieldSync().map_received >= 4 * gatr2::kDocChunkMax; },
        kLimit));
    cut(true);
    rig.run(0.6);
    EXPECT_EQ(rig.client().field().generation, 0u);
    cut(false);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().generation != 0; }, 3 * kLimit));
    EXPECT_EQ(rig.client().field().map, rig.pi.mapDocument());
    EXPECT_EQ(rig.client().stats().maps, 1u);
}

TEST_P(Recovery, RepliesFromBeforeTheCutAreIgnored) {
    ready();
    const StateSample before = rig.client().state();
    gatr2::BrainReply old;
    old.op          = gatr2::kOpGetState;
    old.session     = rig.client().session();
    old.request_id  = 3; // long answered
    old.pi_instance = rig.pi.piInstance();
    old.state.x_mm  = 9999;
    inject(old);
    rig.run(0.2);
    EXPECT_GE(rig.client().stats().uncorrelated, 1u);
    EXPECT_NE(rig.client().state().state.x_mm, 9999);
    EXPECT_EQ(rig.client().state().state.x_mm, before.state.x_mm);
}

TEST_P(Recovery, PicoRestartChangesTheFrameNotThePlacement) {
    ready();
    const investigatr::FrameGeneration frame = rig.driver().robot(rig.now()).frame;
    rig.pi.robot().odometry_epoch += 1;
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.driver().robot(rig.now()).frame != frame; }, kLimit));
    EXPECT_EQ(status(), RobotStatus::kValid);
}

TEST_P(Recovery, ExplicitReinitializeNeedsStillnessThenPlacement) {
    ready();
    Client&        client = rig.client();
    const uint32_t epoch  = rig.pi.robot().odometry_epoch;

    rig.pi.setMoving(true);
    ControlTicket ticket = client.reinitialize();
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kNotStationary);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch);
    EXPECT_EQ(status(), RobotStatus::kValid);

    rig.pi.setMoving(false);
    rig.pi.setCalibrationRequests(20);
    ticket = client.reinitialize();
    ASSERT_TRUE(rig.runUntil([&] { return !client.controlPending(); }, kLimit));
    EXPECT_EQ(client.controlStatus(ticket).state, ControlResult::kOk);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch + 1);
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kCalibrating; }, kLimit));
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kUnplaced; }, kLimit));
    place(Pose{0.2, 0.2, 0.0});
    EXPECT_EQ(status(), RobotStatus::kValid);
}

TEST_P(Recovery, ImuReinitIsFollowedUntilThePiReportsTheOutcome) {
    ready();
    rig.pi.setControlPendingRequests(25);
    const ControlTicket ticket = rig.client().reinitImu();
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().controlPending(); }, kLimit));
    EXPECT_EQ(rig.client().controlStatus(ticket).state, ControlResult::kOk);
    EXPECT_EQ(rig.pi.controlsExecuted(), 1);
    EXPECT_GT(piSaw(gatr2::kOpControl), 1u); // asked again, executed once
}

TEST_P(Recovery, InterruptedCommandsAreNeverResumed) {
    ready();
    rig.pi.setApplyDelay(-1);
    rig.pi.setControlPendingRequests(1000000);
    const ControlTicket control = rig.client().reinitImu();
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().controlStatus(control).result == gatr2::kResultPending; },
        kLimit));
    const PlacementTicket placement = rig.driver().place(Pose{1.0, 1.0, 0.0});
    ASSERT_NE(placement, 0u);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementStatus(placement).result == gatr2::kResultPending; },
        kLimit));
    rig.pi.setControlPendingRequests(0);
    rig.pi.restart(0x600D0001);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().controlPending(); }, kLimit));
    EXPECT_EQ(rig.client().controlStatus(control).state, ControlResult::kSessionLost);
    EXPECT_EQ(rig.client().placementResult(placement), PlacementResult::kSessionLost);
    const std::size_t controls   = piSaw(gatr2::kOpControl);
    const std::size_t placements = piSaw(gatr2::kOpSetPose);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    rig.run(1.0);
    EXPECT_EQ(piSaw(gatr2::kOpControl), controls);
    EXPECT_EQ(piSaw(gatr2::kOpSetPose), placements);
    EXPECT_EQ(status(), RobotStatus::kUnplaced);
}

TEST_P(Recovery, ProfileExchangeInterruptedByTheLinkCompletes) {
    rig.pi.setProfileApplyDelay(10);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().state == ProfileSync::kApplying; }, kLimit));
    cut(true);
    rig.run(0.5);
    EXPECT_FALSE(rig.client().profileApplied());
    cut(false);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
}

INSTANTIATE_TEST_SUITE_P(Transports, Recovery,
                         testing::Values(RigTransport::kRs485, RigTransport::kUsb), name);

// ---------------------------------------------------------------------------
// USB console sharing
// ---------------------------------------------------------------------------

TEST(RecoveryUsb, ConsoleTextOnTheLineIsIgnored) {
    LinkRig rig(ClientConfig{}, FakeBusConfig{}, RigTransport::kUsb);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    const uint32_t replies = rig.client().stats().replies;
    for (int i = 0; i < 20; ++i) {
        rig.usb.textToBrain("PROS: task started\n");
        rig.usb.textToBrain("NG1:0g12\n"); // lowercase digit: dropped
        rig.usb.textToPi("printf from the Brain\n");
        rig.run(0.02);
    }
    EXPECT_GT(rig.client().stats().replies, replies + 10);
    EXPECT_EQ(rig.client().stats().timeouts, 0u);
    EXPECT_GE(rig.usb.brainLines().ignored, 20u);
    EXPECT_GE(rig.usb.brainLines().dropped, 20u);
    EXPECT_GE(rig.usb.piLines().ignored, 20u);
    EXPECT_TRUE(rig.client().connected(rig.now()));
}

TEST(RecoveryUsb, DiagnosticPrefixBeforeTheMarkerIsTolerated) {
    LinkRig rig(ClientConfig{}, FakeBusConfig{}, RigTransport::kUsb);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    // A reply line with kernel text glued in front still decodes.
    gatr2::BrainReply fake;
    fake.op          = gatr2::kOpGetState;
    fake.session     = rig.client().session();
    fake.request_id  = 1;
    fake.pi_instance = rig.pi.piInstance();
    std::vector<uint8_t> frame(gatr2::kMaxFrameLen);
    frame.resize(gatr2::encodeBrainReply(fake, frame.data(), gatr2::kMaxFrameLen));
    char              line[kUsbLineMax];
    const std::size_t n = encodeUsbLine(frame.data(), frame.size(), line, sizeof(line));
    rig.usb.textToBrain("[kernel] note NG1:" + std::string(line + kUsbMarkerLen, n - kUsbMarkerLen));
    rig.run(0.1);
    EXPECT_GE(rig.usb.brainLines().frames, 1u);
    EXPECT_GE(rig.client().stats().uncorrelated, 1u); // decoded, then rejected as old
}
