// profile_change_gtest.cpp
// Runtime robot profile change, as applying a wheel calibration does: the
// new document is uploaded and applied as a new profile, the Pi starts a new
// odometry epoch, and a placement is needed again, never made on its own.
// Same document again, documents refused by the Brain check, and a change
// in the middle of an upload. Both transports.

#include <gtest/gtest.h>
#include <string>

#include "communigatr/link_driver.h"
#include "communigatr/readiness.h"
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
    p.wheels         = {{0, 0.024, 2048, 0.0, 0.10, 0.0}, {1, 0.024, 2048, -0.08, 0.0, kPi / 2}};
    p.imu_source     = ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = {0.2, 0.2, 0.2, 0.2};
    return p;
}

// Two PROFILE_WRITE chunks: four camera mounts.
RobotProfile largeProfile() {
    RobotProfile p = benchProfile();
    for (uint8_t slot = 0; slot < translagatr::kProfileMaxCameras; ++slot) {
        CameraMount c;
        c.slot = slot;
        c.x    = 0.05 * slot;
        c.z    = 0.3;
        p.cameras.push_back(c);
    }
    return p;
}

std::string name(const testing::TestParamInfo<RigTransport>& info) {
    return info.param == RigTransport::kUsb ? "Usb" : "Rs485";
}

class ProfileChange : public testing::TestWithParam<RigTransport> {
protected:
    ProfileChange() : rig(config(), FakeBusConfig{}, GetParam()) {
        rig.pi.setProfileMode(true);
        rig.pi.robot().health = translagatr::kHealthEncodersFresh | translagatr::kHealthGyroFresh;
    }

    static ClientConfig config() {
        ClientConfig c;
        c.bench_imu = [] { return BenchImuSample{true, 1000, 0}; };
        return c;
    }

    void applied() {
        ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    }

    void place(const Pose& pose) {
        const PlacementTicket ticket = rig.driver().place(pose);
        ASSERT_NE(ticket, 0u);
        ASSERT_TRUE(rig.runUntil(
            [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
            kLimit));
    }

    RobotStatus status() { return rig.driver().robot(rig.now()).status; }

    LinkRig rig;
};

} // namespace

TEST_P(ProfileChange, NewTravelScaleIsAppliedAsANewProfileAndNeedsPlacement) {
    ASSERT_TRUE(rig.driver().setProfile(benchProfile()));
    applied();
    place(Pose{0.6, 0.4, kPi / 2});
    const uint32_t old_id = rig.client().profile().id;
    const uint32_t epoch  = rig.pi.robot().odometry_epoch;
    ASSERT_EQ(status(), RobotStatus::kValid);

    RobotProfile scaled           = rig.driver().profile();
    scaled.wheels[1].travel_scale = 1.0125;
    ASSERT_TRUE(rig.driver().setProfile(scaled));
    EXPECT_NE(rig.client().profile().id, old_id);
    EXPECT_EQ(rig.client().profile().state, ProfileSync::kWaiting);
    EXPECT_DOUBLE_EQ(rig.driver().profile().wheels[1].travel_scale, 1.0125);
    EXPECT_EQ(status(), RobotStatus::kNoProfile); // nothing runs on the old frame meanwhile

    applied();
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
    EXPECT_EQ(rig.pi.profilesApplied(), 2);
    ASSERT_TRUE(rig.runUntil([&] { return status() == RobotStatus::kUnplaced; }, kLimit));
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch + 1);
    EXPECT_EQ(rig.driver().readiness(rig.now()).state, Readiness::kNeedsPlacement);
    rig.run(0.5);
    EXPECT_EQ(rig.pi.placementsApplied(), 1); // never placed again by itself

    place(Pose{0.6, 0.4, kPi / 2}); // the operator's explicit placement at the last pose
    EXPECT_EQ(status(), RobotStatus::kValid);
}

TEST_P(ProfileChange, SameProfileAgainChangesNothing) {
    ASSERT_TRUE(rig.driver().setProfile(benchProfile()));
    applied();
    place(Pose{0.5, 0.5, 0.0});
    const int      writes = rig.pi.profileWrites();
    const uint32_t epoch  = rig.pi.robot().odometry_epoch;

    ASSERT_TRUE(rig.driver().setProfile(benchProfile()));
    EXPECT_EQ(rig.client().profile().state, ProfileSync::kApplied);
    rig.run(0.5);
    EXPECT_EQ(rig.pi.profileWrites(), writes);
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    EXPECT_EQ(rig.pi.robot().odometry_epoch, epoch);
    EXPECT_EQ(status(), RobotStatus::kValid);
}

TEST_P(ProfileChange, ProfileRefusedByTheBrainCheckLeavesTheRunningOne) {
    ASSERT_TRUE(rig.driver().setProfile(benchProfile()));
    applied();
    const uint32_t id     = rig.client().profile().id;
    const int      writes = rig.pi.profileWrites();

    RobotProfile bad           = benchProfile();
    bad.wheels[0].travel_scale = 1.5; // outside 0.9..1.1
    EXPECT_FALSE(rig.driver().setProfile(bad));
    EXPECT_EQ(rig.client().profile().id, id);
    EXPECT_EQ(rig.client().profile().state, ProfileSync::kApplied);
    EXPECT_DOUBLE_EQ(rig.driver().profile().wheels[0].travel_scale, 1.0);
    EXPECT_FALSE(rig.client().setProfile(ProfileDocument{})); // empty
    rig.run(0.3);
    EXPECT_EQ(rig.pi.profileWrites(), writes);
}

TEST_P(ProfileChange, InvalidFirstProfileShowsItsReasonUntilReplaced) {
    EXPECT_FALSE(rig.client().setProfile(ProfileDocument{}));
    EXPECT_FALSE(rig.client().profileConfigured());
    RobotProfile bad           = benchProfile();
    bad.wheels[1].encoder_port = 0; // used twice
    EXPECT_FALSE(rig.driver().setProfile(bad));
    EXPECT_TRUE(rig.driver().hasProfile());
    EXPECT_EQ(rig.client().profile().state, ProfileSync::kInvalid);
    EXPECT_EQ(rig.client().profile().reason, translagatr::kProfileReasonEncoderPort);
    EXPECT_EQ(rig.client().profile().detail, 1);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
    rig.run(0.3);
    EXPECT_EQ(rig.pi.profileWrites(), 0);
    EXPECT_EQ(rig.driver().readiness(rig.now()).state, Readiness::kProfileRejected);
    EXPECT_EQ(rig.client().submitPlacement(0, 0, 0), 0u);

    ASSERT_TRUE(rig.driver().setProfile(benchProfile()));
    applied();
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
}

TEST_P(ProfileChange, ChangeInTheMiddleOfAnUploadEndsWithTheNewDocument) {
    ASSERT_TRUE(rig.driver().setProfile(largeProfile()));
    ASSERT_GT(rig.client().config().profile.len, translagatr::kProfileChunkMax);
    // First chunk staged on the Pi, its reply still on the way.
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.pi.stagingReceived() == translagatr::kProfileChunkMax; }, kLimit));
    RobotProfile changed           = largeProfile();
    changed.wheels[0].travel_scale = 0.995;
    ASSERT_TRUE(rig.driver().setProfile(changed));
    applied();
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
    EXPECT_EQ(rig.pi.profilesApplied(), 1);
    EXPECT_EQ(rig.client().stats().unexpected, 0u);
    EXPECT_EQ(rig.client().profile().reason, translagatr::kProfileReasonNone);
}

TEST_P(ProfileChange, ChangeWhileThePiAppliesTheOldOneEndsWithTheNewOne) {
    rig.pi.setProfileApplyDelay(30);
    ASSERT_TRUE(rig.driver().setProfile(benchProfile()));
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().result == translagatr::kResultPending; }, kLimit));
    RobotProfile changed           = benchProfile();
    changed.wheels[1].travel_scale = 1.02;
    ASSERT_TRUE(rig.driver().setProfile(changed));
    applied();
    rig.run(0.5);
    EXPECT_TRUE(rig.client().profileApplied());
    EXPECT_EQ(rig.pi.appliedProfile(), rig.client().profile().id);
}

// The APPLY Ok can arrive while the newest state still predates the Pi's
// swap. The profile counts as applied only with a state that shows it, so
// the old profile's frame and placement never pass as valid or ready.
TEST_P(ProfileChange, NewProfileNeverShowsTheOldPlacement) {
    for (int delay = 0; delay <= 5; ++delay) {
        SCOPED_TRACE(delay);
        LinkRig r(config(), FakeBusConfig{}, GetParam());
        r.pi.setProfileMode(true);
        r.pi.robot().health = translagatr::kHealthEncodersFresh | translagatr::kHealthGyroFresh;
        ASSERT_TRUE(r.driver().setProfile(benchProfile()));
        ASSERT_TRUE(r.runUntil([&] { return r.client().profileApplied(); }, kLimit));
        const PlacementTicket ticket = r.driver().place(Pose{0.5, 0.4, 0.0});
        ASSERT_TRUE(r.runUntil(
            [&] { return r.client().placementResult(ticket) == PlacementResult::kApplied; },
            kLimit));
        ASSERT_EQ(r.driver().readiness(r.now()).state, Readiness::kReady);
        const uint32_t epoch = r.pi.robot().odometry_epoch;

        r.pi.setProfileApplyDelay(delay);
        RobotProfile scaled           = benchProfile();
        scaled.wheels[1].travel_scale = 1.0125;
        ASSERT_TRUE(r.driver().setProfile(scaled));
        int  valid = 0, ready = 0, placed_after = 0, other_profile = 0;
        bool applied = false;
        for (int i = 0; i < 1500; ++i) {
            r.step();
            valid += r.driver().robot(r.now()).status == RobotStatus::kValid ? 1 : 0;
            const LinkReadiness rd = r.driver().readiness(r.now());
            ready += rd.state == Readiness::kReady ? 1 : 0;
            if (r.client().profileApplied()) {
                applied = true;
                placed_after += rd.localized ? 1 : 0;
                other_profile += r.client().state().state.profile_id != r.client().profile().id;
            }
        }
        EXPECT_TRUE(applied);
        EXPECT_EQ(valid, 0);
        EXPECT_EQ(ready, 0);
        EXPECT_EQ(placed_after, 0);
        EXPECT_EQ(other_profile, 0);
        EXPECT_EQ(r.pi.robot().odometry_epoch, epoch + 1);
        EXPECT_EQ(r.pi.profilesApplied(), 2);
    }
}

// A Brain restart with an edited profile while the Pi still runs the old
// one, placed: the old placement never shows under the edited profile.
TEST_P(ProfileChange, BrainRestartWithAnEditedProfileNeverShowsTheOldPlacement) {
    for (int delay = 0; delay <= 5; ++delay) {
        SCOPED_TRACE(delay);
        LinkRig r(config(), FakeBusConfig{}, GetParam());
        r.pi.setProfileMode(true);
        r.pi.robot().health = translagatr::kHealthEncodersFresh | translagatr::kHealthGyroFresh;
        ASSERT_TRUE(r.driver().setProfile(benchProfile()));
        ASSERT_TRUE(r.runUntil([&] { return r.client().profileApplied(); }, kLimit));
        const PlacementTicket ticket = r.driver().place(Pose{0.5, 0.4, 0.0});
        ASSERT_TRUE(r.runUntil(
            [&] { return r.client().placementResult(ticket) == PlacementResult::kApplied; },
            kLimit));

        r.pi.setProfileApplyDelay(delay);
        RobotProfile edited     = benchProfile();
        edited.wheels[0].radius = 0.025;
        ClientConfig c          = config();
        c.profile               = makeProfileDocument(edited);
        r.rebootBrain(c);
        int  valid = 0, placed = 0;
        bool applied = false;
        for (int i = 0; i < 1500; ++i) {
            r.step();
            valid += r.driver().robot(r.now()).status == RobotStatus::kValid ? 1 : 0;
            const LinkReadiness rd = r.driver().readiness(r.now());
            if (r.client().profileApplied()) {
                applied = true;
                placed += rd.localized || rd.state == Readiness::kReady ? 1 : 0;
            }
        }
        EXPECT_TRUE(applied);
        EXPECT_EQ(valid, 0);
        EXPECT_EQ(placed, 0);
        EXPECT_EQ(r.pi.appliedProfile(), r.client().profile().id);
        EXPECT_EQ(r.pi.robot().robot_flags & translagatr::kRobotLocalized, 0);
    }
}

INSTANTIATE_TEST_SUITE_P(Transports, ProfileChange,
                         testing::Values(RigTransport::kRs485, RigTransport::kUsb), name);
