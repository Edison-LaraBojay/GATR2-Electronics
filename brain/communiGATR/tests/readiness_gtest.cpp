// readiness_gtest.cpp
// Readiness summary over a real client and the v4 fake Pi: the order of
// conditions, the profile IMU source deciding which health bits count, Pi
// calibration states, placement, and reconnecting after a lost link.

#include <gtest/gtest.h>
#include <set>
#include <string>

#include "communigatr/readiness.h"
#include "sim/link_rig.h"

using namespace communigatr;
using investigatr::kPi;
using investigatr::Pose;

namespace
{

constexpr Seconds kLimit = 5.0;

constexpr uint8_t kSensorsFresh = gatr2::kHealthEncodersFresh | gatr2::kHealthGyroFresh;

RobotProfile profileWith(LocalizationTopology topology, ImuSource imu) {
    RobotProfile p;
    p.topology   = topology;
    p.imu_source = imu;
    p.footprint  = {0.2, 0.2, 0.2, 0.2};
    if (topology == LocalizationTopology::kThreeWheel) {
        p.wheels = {{0, 0.024, 2048, 0.0, 0.12, 0.0},
                    {1, 0.024, 2048, 0.0, -0.12, 0.0},
                    {2, 0.024, 2048, -0.08, 0.0, kPi / 2}};
    } else {
        p.wheels = {{0, 0.024, 2048, 0.0, 0.10, 0.0}, {1, 0.024, 2048, -0.08, 0.0, kPi / 2}};
    }
    if (imu == ImuSource::kBrainVex) {
        p.vex_smart_port = 1;
    }
    return p;
}

class ReadinessSummary : public testing::Test {
protected:
    void start(const ClientConfig& config, bool profile_mode = true) {
        rig.rebootBrain(config);
        rig.pi.setProfileMode(profile_mode);
    }

    Readiness now() { return rig.driver().readiness(rig.now()).state; }

    bool reach(Readiness wanted) {
        return rig.runUntil([&] { return now() == wanted; }, kLimit);
    }

    void place() {
        const PlacementTicket ticket = rig.driver().place(Pose{0.5, 0.5, 0.0});
        ASSERT_NE(ticket, 0u);
        ASSERT_TRUE(rig.runUntil(
            [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
            kLimit));
    }

    LinkRig rig;
    bool    brain_calibrating = false;
};

} // namespace

TEST(ReadinessNames, EveryStateHasItsOwnName) {
    std::set<std::string> names;
    for (int i = 0; i <= static_cast<int>(Readiness::kReady); ++i) {
        names.insert(toString(static_cast<Readiness>(i)));
    }
    EXPECT_EQ(names.size(), static_cast<std::size_t>(Readiness::kReady) + 1);
    EXPECT_EQ(names.count("?"), 0u);
    EXPECT_STREQ(toString(Readiness::kReady), "ready");
    for (uint8_t c = gatr2::kCalibrationNone; c <= gatr2::kCalibrationFailed; ++c) {
        EXPECT_STRNE(calibrationName(c), "?");
    }
    EXPECT_STREQ(calibrationName(99), "?");
}

TEST(ReadinessNames, HealthBitsDecodeOneByOne) {
    EXPECT_FALSE(decodeHealth(0).encoders_fresh);
    const HealthBits all = decodeHealth(0xFF);
    EXPECT_TRUE(all.encoders_fresh && all.imu_fresh && all.vision_alive && all.bias_calibrated &&
                all.pico_link && all.imu_initializing && all.imu_failed && all.stationary);
    const struct {
        uint8_t bit;
        bool HealthBits::*field;
    } bits[] = {
        {gatr2::kHealthEncodersFresh, &HealthBits::encoders_fresh},
        {gatr2::kHealthGyroFresh, &HealthBits::imu_fresh},
        {gatr2::kHealthVisionAlive, &HealthBits::vision_alive},
        {gatr2::kHealthBiasCalibrated, &HealthBits::bias_calibrated},
        {gatr2::kHealthPicoLink, &HealthBits::pico_link},
        {gatr2::kHealthImuInitializing, &HealthBits::imu_initializing},
        {gatr2::kHealthImuFailed, &HealthBits::imu_failed},
        {gatr2::kHealthStationary, &HealthBits::stationary},
    };
    for (const auto& b : bits) {
        const HealthBits h   = decodeHealth(b.bit);
        int              set = 0;
        for (const auto& other : bits) {
            set += (h.*(other.field)) ? 1 : 0;
        }
        EXPECT_TRUE(h.*(b.field)) << int(b.bit);
        EXPECT_EQ(set, 1) << int(b.bit);
    }
}

TEST_F(ReadinessSummary, ConnectingUntilTheFirstStateThenReconnectingAfterALoss) {
    ClientConfig config;
    config.profile = makeProfileDocument(
        profileWith(LocalizationTopology::kTwoWheelImu, ImuSource::kBrainVex));
    config.bench_imu = [] { return BenchImuSample{true, 1000, 0}; };
    start(config);
    rig.bus.setPiPresent(false);
    rig.run(0.5);
    EXPECT_EQ(now(), Readiness::kConnecting);

    rig.pi.robot().health = kSensorsFresh;
    rig.bus.setPiPresent(true);
    // A session opening for the first time is still connecting.
    ASSERT_TRUE(rig.runUntil(
        [&] {
            EXPECT_NE(now(), Readiness::kReconnecting);
            return rig.client().ready();
        },
        kLimit));
    EXPECT_EQ(rig.client().stats().sessions, 1u);
    ASSERT_TRUE(reach(Readiness::kNeedsPlacement));
    place();
    EXPECT_EQ(now(), Readiness::kReady);

    rig.bus.setPiPresent(false);
    ASSERT_TRUE(reach(Readiness::kReconnecting));
    rig.bus.setPiPresent(true);
    ASSERT_TRUE(reach(Readiness::kReady)); // same session and anchor

    rig.pi.restart(0x0D15EA5E);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    EXPECT_EQ(now(), Readiness::kReconnecting);
    ASSERT_TRUE(reach(Readiness::kNeedsPlacement)); // Pi restart loses the anchor
}

TEST_F(ReadinessSummary, ProfilePendingThenRejectedByThePi) {
    ClientConfig config;
    config.profile = makeProfileDocument(
        profileWith(LocalizationTopology::kTwoWheelImu, ImuSource::kPico));
    start(config);
    rig.pi.setProfileRejection(gatr2::kProfileReasonEncoderPort, 1);
    rig.pi.setProfileApplyDelay(0);
    ASSERT_TRUE(reach(Readiness::kProfilePending));
    ASSERT_TRUE(reach(Readiness::kProfileRejected));
    EXPECT_EQ(rig.client().profile().reason, gatr2::kProfileReasonEncoderPort);
}

TEST_F(ReadinessSummary, ProfileRefusedByTheBrainCheckIsRejectedAtOnce) {
    RobotProfile bad = profileWith(LocalizationTopology::kTwoWheelImu, ImuSource::kBrainVex);
    bad.wheels[1].angle = 0.0; // parallel wheels cannot resolve sideways travel
    ClientConfig config;
    config.profile = makeProfileDocument(bad);
    start(config);
    ASSERT_TRUE(reach(Readiness::kProfileRejected));
    EXPECT_EQ(rig.client().profile().state, ProfileSync::kInvalid);
    EXPECT_EQ(rig.pi.profileWrites(), 0);
}

TEST_F(ReadinessSummary, VexProfileIgnoresThePicoImuAndWaitsForTheBrainImu) {
    ClientConfig config;
    config.profile = makeProfileDocument(
        profileWith(LocalizationTopology::kTwoWheelImu, ImuSource::kBrainVex));
    config.bench_imu = [this] {
        BenchImuSample s;
        s.valid       = !brain_calibrating;
        s.calibrating = brain_calibrating;
        s.stamp_ms    = 1000;
        return s;
    };
    start(config);

    // No encoders yet.
    rig.pi.robot().health = gatr2::kHealthGyroFresh;
    ASSERT_TRUE(reach(Readiness::kSensorsUnavailable));

    // The VEX IMU calibrating on the Brain: the Pi sees no fresh IMU.
    brain_calibrating     = true;
    rig.pi.robot().health = gatr2::kHealthEncodersFresh;
    ASSERT_TRUE(reach(Readiness::kSensorsInitializing));
    EXPECT_TRUE(rig.driver().readiness(rig.now()).brain_imu_calibrating);

    // Calibrated, but still no fresh IMU at the Pi.
    brain_calibrating = false;
    ASSERT_TRUE(reach(Readiness::kSensorsUnavailable));

    // A broken external IMU on the Pico does not matter to this profile.
    rig.pi.robot().health = kSensorsFresh | gatr2::kHealthImuFailed | gatr2::kHealthImuInitializing;
    ASSERT_TRUE(reach(Readiness::kNeedsPlacement));
    const LinkReadiness r = rig.driver().readiness(rig.now());
    EXPECT_EQ(r.imu, ImuUse::kBrainVex);
    EXPECT_TRUE(r.health.imu_failed);
    place();
    EXPECT_EQ(now(), Readiness::kReady);
}

TEST_F(ReadinessSummary, PicoProfileFollowsImuHealthAndCalibration) {
    ClientConfig config;
    config.profile = makeProfileDocument(
        profileWith(LocalizationTopology::kTwoWheelImu, ImuSource::kPico));
    start(config);
    rig.pi.robot().health = gatr2::kHealthEncodersFresh | gatr2::kHealthImuInitializing;
    ASSERT_TRUE(reach(Readiness::kSensorsInitializing));
    rig.pi.robot().health = gatr2::kHealthEncodersFresh | gatr2::kHealthImuFailed;
    ASSERT_TRUE(reach(Readiness::kSensorsUnavailable));
    rig.pi.robot().health = kSensorsFresh | gatr2::kHealthImuFailed; // failure outranks freshness
    rig.run(0.1);
    EXPECT_EQ(now(), Readiness::kSensorsUnavailable);
    rig.pi.robot().health = gatr2::kHealthEncodersFresh; // IMU not fresh
    rig.run(0.1);
    EXPECT_EQ(now(), Readiness::kSensorsUnavailable);

    rig.pi.robot().health = kSensorsFresh;
    const struct {
        uint8_t                calibration;
        Readiness expected;
    } steps[] = {
        {gatr2::kCalibrationRunning, Readiness::kCalibrating},
        {gatr2::kCalibrationWaitingStill, Readiness::kWaitingStill},
        {gatr2::kCalibrationWaitingData, Readiness::kCalibrating},
        {gatr2::kCalibrationFailed, Readiness::kCalibrationFailed},
        {gatr2::kCalibrationDone, Readiness::kNeedsPlacement},
    };
    for (const auto& s : steps) {
        rig.pi.setCalibrationRequests(0); // hold the scripted state
        rig.pi.robot().calibration = s.calibration;
        EXPECT_TRUE(reach(s.expected)) << calibrationName(s.calibration);
        EXPECT_EQ(rig.driver().readiness(rig.now()).calibration, s.calibration);
    }
    place();
    EXPECT_EQ(now(), Readiness::kReady);
}

TEST_F(ReadinessSummary, ThreeWheelsWithoutImuNeedNoImuHealth) {
    ClientConfig config;
    config.profile =
        makeProfileDocument(profileWith(LocalizationTopology::kThreeWheel, ImuSource::kNone));
    start(config);
    rig.pi.robot().health = gatr2::kHealthEncodersFresh | gatr2::kHealthImuFailed;
    ASSERT_TRUE(reach(Readiness::kNeedsPlacement));
    EXPECT_EQ(rig.driver().readiness(rig.now()).imu, ImuUse::kNone);
}

TEST_F(ReadinessSummary, PlacementInFlightAndFirstPoseAreNotReady) {
    ClientConfig config;
    config.profile = makeProfileDocument(
        profileWith(LocalizationTopology::kTwoWheelImu, ImuSource::kBrainVex));
    config.bench_imu = [] { return BenchImuSample{true, 1000, 0}; };
    start(config);
    rig.pi.robot().health = kSensorsFresh;
    ASSERT_TRUE(reach(Readiness::kNeedsPlacement));
    place();
    ASSERT_EQ(now(), Readiness::kReady);

    // A new placement in flight replaces the anchor: not ready meanwhile.
    rig.pi.setApplyDelay(-1);
    ASSERT_NE(rig.driver().place(Pose{0.7, 0.5, 0.0}), 0u);
    rig.run(0.2);
    EXPECT_TRUE(rig.client().placementPending());
    EXPECT_EQ(now(), Readiness::kNeedsPlacement);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().placementPending(); }, kLimit));

    // Placed but the estimator has no pose yet.
    rig.pi.robot().robot_flags = gatr2::kRobotLocalized | gatr2::kRobotAnchorCommand;
    ASSERT_TRUE(reach(Readiness::kSensorsInitializing));
    rig.pi.robot().robot_flags |= gatr2::kRobotPoseValid | gatr2::kRobotAgeKnown;
    ASSERT_TRUE(reach(Readiness::kReady));
}

TEST_F(ReadinessSummary, WithoutABrainProfileOnlyEncodersAndPlacementCount) {
    start(ClientConfig{}, false);
    rig.pi.robot().health      = gatr2::kHealthEncodersFresh;
    rig.pi.robot().robot_flags = gatr2::kRobotPoseValid | gatr2::kRobotAgeKnown |
                                 gatr2::kRobotLocalized | gatr2::kRobotAnchorConfigured;
    ASSERT_TRUE(reach(Readiness::kNeedsPlacement));
    EXPECT_EQ(rig.driver().readiness(rig.now()).imu, ImuUse::kUnknown);
    // The Pi's configured anchor counts only when accepted.
    EXPECT_EQ(readinessOf(rig.client(), rig.now(), true).state, Readiness::kReady);
}
