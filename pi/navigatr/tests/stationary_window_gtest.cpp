// stationary_window_gtest.cpp
// The stationary window every IMU bias path shares, and gyro bias
// calibration on top of it: only new samples count, gaps, staleness and
// identity changes restart it waiting for data, movement restarts it
// waiting for stillness, it qualifies by elapsed sample time, calibration is
// bounded and later windows move the bias in bounded steps.

#include <gtest/gtest.h>

#include <cmath>

#include "impl/localization/stationary_window.h"
#include "math/angles.h"

using namespace navigatr;

namespace
{

using Phase = StationaryWindow::Phase;
using Kind  = StationaryWindow::Kind;

StillnessConfig config(int64_t window_ms = 500, long samples = 20) {
    StillnessConfig c;
    c.window_ms   = window_ms;
    c.min_samples = samples;
    return c;   // 100 ms gaps, 1 mm per wheel, 1 deg/s about the mean, 5 deg/s
}

// One wheel and one gyro sampled together every tick_ms, the gyro with an
// accumulator.
struct Rig {
    StationaryWindow w;
    size_t           wheel = 0;
    size_t           gyro  = 0;
    uint64_t         seq   = 0;
    int64_t          t     = 1000;
    double           accum = 0.0;
    double           rate  = 0.0;   // previous gyro rate, for the accumulator

    explicit Rig(const StillnessConfig& c = config()) {
        w.configure(c);
        wheel = w.addSource(Kind::kWheel, "wheel 0");
        gyro  = w.addSource(Kind::kGyro, "gyro");
    }

    static StillSample sample(uint64_t seq, int64_t t, double value) {
        StillSample s;
        s.sequence = seq;
        s.at       = deviceTime(t);
        s.received = hostTime(t);
        s.value    = value;
        return s;
    }

    // Both sources at t + tick_ms; the gyro's accumulator integrates its rate.
    void tick(double travel_m, double rate_rad_s, int64_t tick_ms = 20,
              uint64_t discontinuity = 0) {
        const int64_t prev = t;
        t += tick_ms;
        ++seq;
        accum += 0.5 * (rate + rate_rad_s) * (t - prev) / 1000.0;
        rate = rate_rad_s;
        StillSample e = sample(seq, t, travel_m);
        e.discontinuity = discontinuity;
        w.add(wheel, e);
        StillSample g       = sample(seq, t, rate_rad_s);
        g.has_accumulated   = true;
        g.accumulated       = accum;
        w.add(gyro, g);
        w.poll(hostTime(t));
    }

    void still(int ticks, double rate_rad_s = 0.0, double travel_m = 0.0) {
        for (int i = 0; i < ticks; ++i) {
            tick(travel_m, rate_rad_s);
        }
    }
};

} // namespace

TEST(StationaryWindow, QualifiesByElapsedSampleTimeNotBySampleCount) {
    Rig r;
    // 40 samples 5 ms apart: twice the count, a fraction of the window
    for (int i = 0; i < 40; ++i) {
        r.tick(0.0, 0.0, 5);
    }
    EXPECT_EQ(r.w.windows(), 0u);
    EXPECT_FALSE(r.w.stationary());
    EXPECT_EQ(r.w.phase(), Phase::kCollecting);

    Rig slow;
    slow.still(1);             // the first sample of the window
    slow.still(20);            // 21 samples, 400 ms
    EXPECT_EQ(slow.w.windows(), 0u);
    EXPECT_EQ(slow.w.progressMs(), 400);
    slow.still(5);             // 500 ms
    EXPECT_EQ(slow.w.windows(), 1u);
    EXPECT_TRUE(slow.w.stationary());

    // too few samples inside the time: not qualified either
    Rig sparse(config(500, 40));
    sparse.still(30);
    EXPECT_EQ(sparse.w.windows(), 0u);
    sparse.still(11);
    EXPECT_EQ(sparse.w.windows(), 1u);
}

TEST(StationaryWindow, RetainedRecordsAreNeverNewEvidence) {
    Rig r;
    r.still(1);
    const StillSample same = Rig::sample(r.seq, r.t, 0.0);
    for (int i = 0; i < 100; ++i) {
        EXPECT_FALSE(r.w.add(r.wheel, same));
    }
    // the gyro alone moves on; the wheel repeats its record
    for (int i = 0; i < 40; ++i) {
        r.t += 20;
        StillSample g = Rig::sample(++r.seq, r.t, 0.0);
        EXPECT_TRUE(r.w.add(r.gyro, g));
        EXPECT_FALSE(r.w.add(r.wheel, same));
    }
    EXPECT_EQ(r.w.windows(), 0u);
    // the same sequence under a new record epoch is new, and a restart
    StillSample next = same;
    next.epoch       = 1;
    EXPECT_TRUE(r.w.add(r.wheel, next));
    EXPECT_EQ(r.w.phase(), Phase::kWaitingData);
}

TEST(StationaryWindow, GapsStalenessAndIdentityChangesWaitForData) {
    Rig r;
    r.still(20);
    r.tick(0.0, 0.0, 150);   // a gap longer than max_gap_ms between samples
    EXPECT_EQ(r.w.phase(), Phase::kCollecting);   // both sources begin again
    EXPECT_NE(r.w.reason().find("gap"), std::string::npos);
    EXPECT_EQ(r.w.progressMs(), 0);

    r.still(26);
    ASSERT_EQ(r.w.windows(), 1u);
    ASSERT_TRUE(r.w.stationary());
    // silence: the host clock moves on without samples
    r.w.poll(hostTime(r.t + 101));
    EXPECT_FALSE(r.w.stationary());
    EXPECT_EQ(r.w.phase(), Phase::kWaitingData);
    EXPECT_NE(r.w.reason().find("stale"), std::string::npos);

    r.still(26);
    ASSERT_TRUE(r.w.stationary());
    r.tick(0.0, 0.0, 20, 1);   // the encoder rebased: a discontinuity
    EXPECT_FALSE(r.w.stationary());
    EXPECT_NE(r.w.reason().find("restarted"), std::string::npos);
    const uint64_t windows = r.w.windows();
    r.still(20);
    EXPECT_EQ(r.w.windows(), windows);   // the whole window again, from the rebase
}

TEST(StationaryWindow, OutOfOrderSamplesAndAccumulatorDropsRestart) {
    Rig r;
    r.still(10);
    StillSample back = Rig::sample(++r.seq, r.t - 5, 0.0);
    r.w.add(r.wheel, back);
    EXPECT_NE(r.w.reason().find("out of order"), std::string::npos);

    Rig a;
    a.still(10);
    StillSample g       = Rig::sample(++a.seq, a.t + 20, 0.0);
    g.has_accumulated   = true;
    g.accumulated       = a.accum;
    g.accumulated_epoch = 1;   // the producer dropped an interval
    a.w.add(a.gyro, g);
    EXPECT_EQ(a.w.phase(), Phase::kWaitingData);
    EXPECT_NE(a.w.reason().find("accumulator"), std::string::npos);
}

TEST(StationaryWindow, MovingStoppingMoving) {
    Rig r;
    r.still(30);
    ASSERT_TRUE(r.w.stationary());

    double travel = 0.0;
    for (int i = 0; i < 10; ++i) {   // 10 mm per tick
        travel += 0.01;
        r.tick(travel, 0.0);
        EXPECT_FALSE(r.w.stationary());
        EXPECT_EQ(r.w.phase(), Phase::kWaitingStill);
    }
    EXPECT_NE(r.w.reason().find("moved"), std::string::npos);

    // stopped: waiting for stillness a quarter window, then collecting
    const uint64_t windows = r.w.windows();
    r.still(3, 0.0, travel);
    EXPECT_EQ(r.w.phase(), Phase::kWaitingStill);
    r.still(4, 0.0, travel);
    EXPECT_EQ(r.w.phase(), Phase::kCollecting);
    r.still(19, 0.0, travel);
    EXPECT_EQ(r.w.windows(), windows + 1);
    EXPECT_TRUE(r.w.stationary());

    // creeping under the travel limit stays still; one more millimeter does not
    r.still(10, 0.0, travel + 0.0009);
    EXPECT_TRUE(r.w.stationary());
    r.tick(travel + 0.0021, 0.0);
    EXPECT_FALSE(r.w.stationary());
}

TEST(StationaryWindow, GyroMagnitudeAndVariationAreMovement) {
    Rig bias;   // a constant 3 deg/s bias is still
    bias.still(30, degToRad(3.0));
    EXPECT_TRUE(bias.w.stationary());

    Rig fast;   // above 5 deg/s is turning, however steady
    fast.still(30, degToRad(6.0));
    EXPECT_FALSE(fast.w.stationary());
    EXPECT_EQ(fast.w.phase(), Phase::kWaitingStill);
    EXPECT_NE(fast.w.reason().find("turning"), std::string::npos);

    Rig swing;   // alternating 0 and 2.5 deg/s: 1.25 deg/s from the mean
    for (int i = 0; i < 30; ++i) {
        swing.tick(0.0, degToRad(i % 2 == 0 ? 0.0 : 2.5));
    }
    EXPECT_FALSE(swing.w.stationary());
    EXPECT_NE(swing.w.reason().find("varies"), std::string::npos);

    Rig noise;   // alternating 0 and 1.5 deg/s: 0.75 from the mean
    for (int i = 0; i < 30; ++i) {
        noise.tick(0.0, degToRad(i % 2 == 0 ? 0.0 : 1.5));
    }
    EXPECT_TRUE(noise.w.stationary());
}

TEST(StationaryWindow, GyroRateIsTheAccumulatedAngleOverElapsedTime) {
    Rig r;
    // rates that alternate 0.01 and 0.03 rad/s: the accumulator integrates
    // the trapezoid, 0.02 rad/s over the window
    for (int i = 0; i < 26; ++i) {
        r.tick(0.0, i % 2 == 0 ? 0.01 : 0.03);
    }
    StationaryWindow::Qualified q;
    ASSERT_TRUE(r.w.takeQualified(q));
    EXPECT_TRUE(q.gyro);
    EXPECT_NEAR(q.elapsed_s, 0.5, 1e-12);
    EXPECT_NEAR(q.gyro_rate, 0.02, 1e-4);
    EXPECT_FALSE(r.w.takeQualified(q));   // once

    // the producer integrated packets the samples never showed: its
    // accumulator wins over the rates
    StationaryWindow w;
    w.configure(config());
    const size_t gyro = w.addSource(Kind::kGyro, "gyro");
    for (int i = 0; i <= 25; ++i) {
        StillSample g     = Rig::sample(static_cast<uint64_t>(i + 1), 20 * i, 0.01);
        g.has_accumulated = true;
        g.accumulated     = 0.03 * 0.02 * i;   // 0.03 rad/s
        w.add(gyro, g);
    }
    ASSERT_TRUE(w.takeQualified(q));
    EXPECT_NEAR(q.gyro_rate, 0.03, 1e-12);
}

TEST(StationaryWindow, VexRotationIsReducedEvidence) {
    StationaryWindow w;
    w.configure(config());
    const size_t wheel    = w.addSource(Kind::kWheel, "wheel");
    const size_t rotation = w.addSource(Kind::kRotation, "rotation");
    int64_t      t        = 0;
    double       angle    = 0.0;
    const auto   tick     = [&](double dangle) {
        t += 20;
        angle += dangle;
        w.add(wheel, Rig::sample(static_cast<uint64_t>(t), t, 0.0));
        w.add(rotation, Rig::sample(static_cast<uint64_t>(t), t, angle));
        w.poll(hostTime(t));
    };
    for (int i = 0; i < 30; ++i) {
        tick(degToRad(0.01));   // drift of 0.5 deg/s
    }
    EXPECT_TRUE(w.stationary());
    for (int i = 0; i < 5; ++i) {
        tick(degToRad(2.0));   // a turn
    }
    EXPECT_FALSE(w.stationary());
    EXPECT_NE(w.reason().find("turning"), std::string::npos);
}

TEST(GyroBiasCalibration, FirstQualifiedWindowGivesTheBias) {
    GyroBiasCalibration c;
    c.configure(GyroBiasCalibration::Config{});
    EXPECT_FALSE(c.calibrated());
    EXPECT_EQ(c.attempts(), 1u);
    EXPECT_EQ(c.state(Phase::kCollecting), BiasCalibration::kRunning);
    EXPECT_EQ(c.state(Phase::kWaitingStill), BiasCalibration::kWaitingStill);
    EXPECT_EQ(c.state(Phase::kWaitingData), BiasCalibration::kWaitingData);
    c.poll(hostTime(0));
    c.qualified(0.0123, hostTime(2000));
    EXPECT_TRUE(c.calibrated());
    EXPECT_DOUBLE_EQ(c.bias(), 0.0123);
    EXPECT_EQ(c.state(Phase::kWaitingStill), BiasCalibration::kDone);   // moving later
}

TEST(GyroBiasCalibration, NoQualifiedWindowWithinTheBoundFailsUntilANewStart) {
    GyroBiasCalibration c;
    c.configure(GyroBiasCalibration::Config{});
    c.poll(hostTime(1000));
    c.poll(hostTime(61000));
    EXPECT_FALSE(c.failed());
    c.poll(hostTime(61001));
    EXPECT_TRUE(c.failed());
    EXPECT_EQ(c.state(Phase::kCollecting), BiasCalibration::kFailed);
    EXPECT_NE(c.note().find("60 s"), std::string::npos);
    c.qualified(0.01, hostTime(62000));   // too late: no silent success
    EXPECT_FALSE(c.calibrated());

    c.start("recalibrate");   // a retry is a new, bounded attempt
    EXPECT_FALSE(c.failed());
    EXPECT_EQ(c.attempts(), 2u);
    c.poll(hostTime(70000));
    c.qualified(0.01, hostTime(72000));
    EXPECT_TRUE(c.calibrated());
}

TEST(GyroBiasCalibration, LaterWindowsMoveTheBiasInBoundedSteps) {
    GyroBiasCalibration c;
    c.configure(GyroBiasCalibration::Config{});
    c.qualified(0.010, hostTime(0));
    c.qualified(0.011, hostTime(2000));   // 0.2 of the difference
    EXPECT_NEAR(c.bias(), 0.0102, 1e-12);
    c.qualified(1.0, hostTime(4000));     // at most 0.2 deg/s per window
    EXPECT_NEAR(c.bias(), 0.0102 + degToRad(0.2), 1e-12);
    c.qualified(-1.0, hostTime(6000));
    EXPECT_NEAR(c.bias(), 0.0102, 1e-12);
    EXPECT_EQ(c.steps(), 3u);
}

TEST(GyroBiasCalibration, ARestartInvalidatesTheBias) {
    GyroBiasCalibration c;
    c.configure(GyroBiasCalibration::Config{});
    c.qualified(0.01, hostTime(0));
    ASSERT_TRUE(c.calibrated());
    c.start("IMU source restarted");
    EXPECT_FALSE(c.calibrated());
    EXPECT_EQ(c.state(Phase::kWaitingData), BiasCalibration::kWaitingData);
    EXPECT_EQ(c.note(), "IMU source restarted");
    c.qualified(0.02, hostTime(3000));
    EXPECT_DOUBLE_EQ(c.bias(), 0.02);   // measured again, not stepped from the old one
}

TEST(GyroBiasCalibration, OffMeansZeroBiasAndNothingToWaitFor) {
    GyroBiasCalibration::Config off;
    off.enabled = false;
    GyroBiasCalibration c;
    c.configure(off);
    EXPECT_TRUE(c.calibrated());
    EXPECT_EQ(c.bias(), 0.0);
    c.start("recalibrate");
    c.qualified(0.5, hostTime(0));
    EXPECT_TRUE(c.calibrated());
    EXPECT_EQ(c.bias(), 0.0);
    EXPECT_EQ(c.attempts(), 0u);
}

TEST(StationaryWindow, StatusReportsWindowAndCalibration) {
    Rig                 r;
    GyroBiasCalibration c;
    c.configure(GyroBiasCalibration::Config{});
    StillnessStatus s = stillnessOf(r.w, &c);
    EXPECT_TRUE(s.monitored);
    EXPECT_EQ(s.calibration, BiasCalibration::kWaitingData);
    EXPECT_EQ(s.window_ms, 500);
    r.still(26);
    StationaryWindow::Qualified q;
    ASSERT_TRUE(r.w.takeQualified(q));
    c.qualified(q.gyro_rate, hostTime(r.t));
    s = stillnessOf(r.w, &c);
    EXPECT_TRUE(s.stationary);
    EXPECT_EQ(s.calibration, BiasCalibration::kDone);
    EXPECT_TRUE(s.has_bias);
    EXPECT_EQ(s.windows, 1u);
    const StillnessStatus none = stillnessOf(r.w, nullptr);
    EXPECT_EQ(none.calibration, BiasCalibration::kNone);
}
