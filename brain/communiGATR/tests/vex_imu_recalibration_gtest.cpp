// vex_imu_recalibration_gtest.cpp
// Gated VEX IMU recalibration: the VEX calibration starts only after the Pi's
// stationary check answers Ok, never while the robot moves, never on a
// refusal or a lost answer; the IMU's own failures and time limit. Unit
// steps, then the real Client against the fake Pi on both transports.

#include "communigatr/vex_imu_recalibration.h"

#include <gtest/gtest.h>
#include <string>

#include "communigatr/robot_profile.h"
#include "sim/link_rig.h"

using namespace communigatr;
using investigatr::kPi;

namespace
{

constexpr Seconds kLimit = 5.0;

ControlStatus answer(ControlTicket ticket, ControlResult state,
                     uint8_t result = gatr2::kResultOk) {
    ControlStatus s;
    s.ticket = ticket;
    s.action = gatr2::kControlRecalibrate;
    s.state  = state;
    s.result = result;
    return s;
}

BenchImuSample imu(bool valid, bool calibrating) {
    BenchImuSample s;
    s.valid       = valid;
    s.calibrating = calibrating;
    return s;
}

// VEX IMU stand-in: recalibrate() starts a calibration of `duration`.
struct FakeVexImu {
    Seconds duration = 2.0;
    Seconds until    = -1.0;
    int     resets   = 0;
    bool    refuse   = false;

    bool recalibrate(Seconds now) {
        if (refuse) {
            return false;
        }
        ++resets;
        until = now + duration;
        return true;
    }

    BenchImuSample sample(Seconds now) const {
        return now < until ? imu(false, true) : imu(true, false);
    }
};

} // namespace

// ---------------------------------------------------------------------------
// Steps
// ---------------------------------------------------------------------------

TEST(VexImuRecalibration, StartsOnlyAfterThePiOkAndEndsWithAValidSample) {
    VexImuRecalibration r;
    EXPECT_EQ(r.state(), VexRecalibrationState::kIdle);
    EXPECT_FALSE(r.begin(0)); // the link refused the CONTROL
    ASSERT_TRUE(r.begin(7));
    EXPECT_TRUE(r.active());
    EXPECT_EQ(r.ticket(), 7u);
    EXPECT_FALSE(r.begin(8)); // one run at a time

    EXPECT_EQ(r.update(answer(7, ControlResult::kPending), imu(true, false), 0.0),
              VexRecalibrationState::kChecking);
    r.started(true, 0.0); // ignored before the Pi's Ok
    EXPECT_EQ(r.state(), VexRecalibrationState::kChecking);
    EXPECT_EQ(r.update(answer(6, ControlResult::kOk), imu(true, false), 0.1),
              VexRecalibrationState::kChecking); // another ticket's answer
    EXPECT_EQ(r.update(answer(7, ControlResult::kOk), imu(true, false), 0.1),
              VexRecalibrationState::kStart);
    EXPECT_EQ(r.update(answer(7, ControlResult::kOk), imu(true, false), 0.2),
              VexRecalibrationState::kStart); // waits for started()

    r.started(true, 0.3);
    EXPECT_EQ(r.state(), VexRecalibrationState::kCalibrating);
    EXPECT_EQ(r.update(answer(7, ControlResult::kOk), imu(false, true), 0.4),
              VexRecalibrationState::kCalibrating);
    EXPECT_EQ(r.update(answer(7, ControlResult::kOk), imu(false, true), 2.2),
              VexRecalibrationState::kCalibrating);
    EXPECT_EQ(r.update(answer(7, ControlResult::kOk), imu(true, false), 2.4),
              VexRecalibrationState::kDone);
    EXPECT_FALSE(r.active());
    EXPECT_EQ(r.check().state, ControlResult::kOk);
    EXPECT_TRUE(r.begin(9)); // a new run
    EXPECT_EQ(r.check().state, ControlResult::kPending);
}

TEST(VexImuRecalibration, MovementRefusalAndNoAnswerStartNothing) {
    struct Case {
        ControlResult         state;
        uint8_t               result;
        VexRecalibrationState end;
    };
    const Case cases[] = {
        {ControlResult::kNotStationary, gatr2::kResultNotStationary,
         VexRecalibrationState::kMoving},
        {ControlResult::kNotReady, gatr2::kResultNotReady, VexRecalibrationState::kRefused},
        {ControlResult::kRejected, gatr2::kResultInvalidArgument, VexRecalibrationState::kRefused},
        {ControlResult::kFailed, gatr2::kResultFailed, VexRecalibrationState::kRefused},
        {ControlResult::kTimedOut, gatr2::kResultOk, VexRecalibrationState::kRefused},
        {ControlResult::kSessionLost, gatr2::kResultOk, VexRecalibrationState::kRefused},
        {ControlResult::kNone, gatr2::kResultOk, VexRecalibrationState::kRefused}, // replaced
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(int(c.state));
        VexImuRecalibration r;
        ASSERT_TRUE(r.begin(3));
        EXPECT_EQ(r.update(answer(3, c.state, c.result), imu(true, false), 0.1), c.end);
        EXPECT_FALSE(r.active());
        EXPECT_EQ(r.check().result, c.result);
        r.started(true, 0.2); // nothing to start
        EXPECT_EQ(r.state(), c.end);
    }
}

TEST(VexImuRecalibration, ImuThatDoesNotStartOrFinishFails) {
    VexImuRecalibration refused;
    ASSERT_TRUE(refused.begin(1));
    refused.update(answer(1, ControlResult::kOk), imu(true, false), 0.0);
    refused.started(false, 0.0); // PROS refused the reset
    EXPECT_EQ(refused.state(), VexRecalibrationState::kImuFailed);

    // Started, but the IMU never reports calibrating.
    VexImuRecalibration silent;
    ASSERT_TRUE(silent.begin(1));
    silent.update(answer(1, ControlResult::kOk), imu(true, false), 0.0);
    silent.started(true, 0.0);
    EXPECT_EQ(silent.update(answer(1, ControlResult::kOk), imu(true, false), 0.5),
              VexRecalibrationState::kCalibrating);
    EXPECT_EQ(silent.update(answer(1, ControlResult::kOk), imu(true, false), 1.1),
              VexRecalibrationState::kImuFailed);

    // Calibrating past the limit.
    VexImuRecalibration stuck(3.0);
    ASSERT_TRUE(stuck.begin(1));
    stuck.update(answer(1, ControlResult::kOk), imu(true, false), 0.0);
    stuck.started(true, 0.0);
    EXPECT_EQ(stuck.update(answer(1, ControlResult::kOk), imu(false, true), 2.9),
              VexRecalibrationState::kCalibrating);
    EXPECT_EQ(stuck.update(answer(1, ControlResult::kOk), imu(false, true), 3.1),
              VexRecalibrationState::kImuFailed);

    // Done calibrating, but the sample is invalid (IMU error or unplugged).
    VexImuRecalibration broken;
    ASSERT_TRUE(broken.begin(1));
    broken.update(answer(1, ControlResult::kOk), imu(true, false), 0.0);
    broken.started(true, 0.0);
    broken.update(answer(1, ControlResult::kOk), imu(false, true), 0.1);
    EXPECT_EQ(broken.update(answer(1, ControlResult::kOk), imu(false, false), 2.0),
              VexRecalibrationState::kImuFailed);
    EXPECT_TRUE(broken.begin(2)); // retry allowed
}

TEST(VexImuRecalibration, NamesAreDistinct) {
    const VexRecalibrationState all[] = {
        VexRecalibrationState::kIdle,        VexRecalibrationState::kChecking,
        VexRecalibrationState::kStart,       VexRecalibrationState::kCalibrating,
        VexRecalibrationState::kDone,        VexRecalibrationState::kMoving,
        VexRecalibrationState::kRefused,     VexRecalibrationState::kImuFailed};
    for (VexRecalibrationState a : all) {
        EXPECT_STRNE(toString(a), "?");
        for (VexRecalibrationState b : all) {
            if (a != b) {
                EXPECT_STRNE(toString(a), toString(b));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Against the fake Pi
// ---------------------------------------------------------------------------

namespace
{

RobotProfile vexProfile() {
    RobotProfile p;
    p.topology       = LocalizationTopology::kTwoWheelImu;
    p.wheels         = {{0, 0.024, 2048, 0.0, 0.10, 0.0}, {1, 0.024, 2048, -0.08, 0.0, kPi / 2}};
    p.imu_source     = ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = {0.2, 0.2, 0.2, 0.2};
    return p;
}

std::string name(const testing::TestParamInfo<RigTransport>& info) {
    return info.param == RigTransport::kUsb ? "Usb" : "Rs485";
}

class VexRecalibrationLink : public testing::TestWithParam<RigTransport> {
protected:
    VexRecalibrationLink() : rig(config(), FakeBusConfig{}, GetParam()) {
        rig.pi.setProfileMode(true);
    }

    ClientConfig config() {
        ClientConfig c;
        c.profile   = makeProfileDocument(vexProfile());
        c.bench_imu = [this] { return vex.sample(rig.now()); };
        return c;
    }

    // One loop of a program: the link's answer and the IMU sample in, the
    // IMU started when the helper says so.
    VexRecalibrationState loop(VexImuRecalibration& r) {
        const VexRecalibrationState s =
            r.update(rig.client().controlStatus(r.ticket()), vex.sample(rig.now()), rig.now());
        if (s == VexRecalibrationState::kStart) {
            r.started(vex.recalibrate(rig.now()), rig.now());
        }
        return r.state();
    }

    // Runs the program loop every 20 ms until the run ends.
    VexRecalibrationState finish(VexImuRecalibration& r) {
        for (int i = 0; i < 500 && r.active(); ++i) {
            rig.run(0.02);
            loop(r);
        }
        return r.state();
    }

    FakeVexImu vex;
    LinkRig    rig;
};

} // namespace

TEST_P(VexRecalibrationLink, MovingRobotNeverStartsTheVexCalibration) {
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    rig.pi.setMoving(true);
    VexImuRecalibration r;
    ASSERT_TRUE(r.begin(rig.client().recalibrate()));
    EXPECT_EQ(finish(r), VexRecalibrationState::kMoving);
    EXPECT_EQ(vex.resets, 0);

    rig.pi.setMoving(false);
    ASSERT_TRUE(r.begin(rig.client().recalibrate()));
    EXPECT_EQ(finish(r), VexRecalibrationState::kDone);
    EXPECT_EQ(vex.resets, 1);
    EXPECT_EQ(rig.client().controlStatus(r.ticket()).state, ControlResult::kOk);
}

TEST_P(VexRecalibrationLink, NoProfileOrNoAnswerStartsNothing) {
    rig.pi.setProfileRejection(gatr2::kProfileReasonEncoderPort); // never applied: NotReady
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().profile().state == ProfileSync::kRejected; }, kLimit));
    VexImuRecalibration r;
    ASSERT_TRUE(r.begin(rig.client().recalibrate()));
    EXPECT_EQ(finish(r), VexRecalibrationState::kRefused);
    EXPECT_EQ(r.check().state, ControlResult::kNotReady);
    EXPECT_EQ(vex.resets, 0);

    // The Pi gone before it answers.
    rig.bus.setPiPresent(false);
    rig.usb.setPlugged(false);
    ASSERT_TRUE(r.begin(rig.client().recalibrate()));
    EXPECT_EQ(finish(r), VexRecalibrationState::kRefused);
    EXPECT_EQ(r.check().state, ControlResult::kTimedOut);
    EXPECT_EQ(vex.resets, 0);
}

INSTANTIATE_TEST_SUITE_P(Transports, VexRecalibrationLink,
                         testing::Values(RigTransport::kRs485, RigTransport::kUsb), name);
