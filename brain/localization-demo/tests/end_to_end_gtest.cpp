// end_to_end_gtest.cpp
// The whole data path without the devices: Pico record bytes, cut and
// corrupted on the UART, through the Pi relay forwarder, as lines into the
// Brain scanner, then applyRecord with an IMU reading.

#include "encoder_record.h"
#include "forwarder.h"
#include "localization.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace localization
{
namespace
{

using encoder_record::Record;
using odometry::kPi;

// 1000 counts per meter, forward wheel 0.1 m right, sideways 0.05 m ahead,
// both counting down for positive travel, like the robot.
odometry::WheelSetup robotLikeWheels() {
    odometry::WheelSetup w;
    w.forward.channel         = 1;
    w.forward.diameter        = 1.0 / kPi;
    w.forward.counts_per_rev  = 1000;
    w.forward.direction       = -1;
    w.forward.offset          = -0.1;
    w.sideways.channel        = 0;
    w.sideways.diameter       = 1.0 / kPi;
    w.sideways.counts_per_rev = 1000;
    w.sideways.direction      = -1;
    w.sideways.offset         = 0.05;
    return w;
}

// Pico side: cumulative counts for a robot that moves along its body axes.
struct FakePico {
    uint16_t run     = 0x0A0A;
    uint16_t seq     = 0;
    double   forward = 0; // meters of forward wheel travel
    double   left    = 0;

    std::vector<uint8_t> record() {
        Record r;
        r.run_id   = run;
        r.seq      = seq++;
        r.channel0 = static_cast<int32_t>(std::lround(-left * 1000));
        r.channel1 = static_cast<int32_t>(std::lround(-forward * 1000));
        const encoder_record::RecordBytes b = encoder_record::encode(r);
        return {b.begin(), b.end()};
    }
};

struct Brain {
    explicit Brain(const odometry::Pose& start) : state(start, TrackerLimits{}) {}

    encoder_record::LineScanner scanner;
    State                       state;
    double                      rotation_degrees = 0;

    void receive(const std::string& text, uint32_t now_ms) {
        for (char c : text) {
            if (scanner.push(c)) {
                ImuReading imu;
                imu.valid            = true;
                imu.rotation_degrees = rotation_degrees;
                applyRecord(state, robotLikeWheels(), scanner.record(), now_ms, imu);
            }
        }
    }
};

// Pi side: UART bytes in uneven chunks, USB writes in uneven chunks.
struct Link {
    relay::Forwarder forwarder;
    std::mt19937     rng{42};

    std::string carry(const std::vector<uint8_t>& uart) {
        std::size_t at = 0;
        std::string usb;
        while (at < uart.size()) {
            const std::size_t n = std::min<std::size_t>(1 + rng() % 9, uart.size() - at);
            forwarder.receive(uart.data() + at, n);
            at += n;
            // The Brain takes a few bytes at a time.
            const relay::Pending p = forwarder.pending();
            const std::size_t    w = std::min<std::size_t>(p.size, 1 + rng() % 20);
            usb.append(p.data, w);
            forwarder.wrote(w);
        }
        for (relay::Pending p = forwarder.pending(); p.size > 0; p = forwarder.pending()) {
            usb.append(p.data, p.size);
            forwarder.wrote(p.size);
        }
        return usb;
    }
};

TEST(EndToEnd, ForwardSidewaysAndSpinThroughTheRelay) {
    FakePico pico;
    Link     link;
    Brain    brain(odometry::Pose{1.0, 1.0, 0.0});
    uint32_t now = 0;

    const auto tick = [&](std::vector<uint8_t> bytes) {
        brain.receive(link.carry(bytes), now);
        now += 20;
    };

    tick(pico.record()); // baseline
    for (int i = 0; i < 50; ++i) {
        pico.forward += 0.02; // 1 m forward
        tick(pico.record());
    }
    EXPECT_NEAR(brain.state.pose.x, 2.0, 1e-3);
    EXPECT_NEAR(brain.state.pose.y, 1.0, 1e-3);

    for (int i = 0; i < 25; ++i) {
        pico.left += 0.02; // 0.5 m left
        tick(pico.record());
    }
    EXPECT_NEAR(brain.state.pose.x, 2.0, 1e-3);
    EXPECT_NEAR(brain.state.pose.y, 1.5, 1e-3);

    // Spin a full turn counterclockwise in place: wheels roll only from the offsets.
    for (int i = 1; i <= 72; ++i) {
        const double turn = odometry::degreesToRadians(5.0);
        pico.forward += 0.1 * turn;
        pico.left += 0.05 * turn;
        brain.rotation_degrees -= 5.0;
        tick(pico.record());
    }
    EXPECT_NEAR(brain.state.pose.x, 2.0, 5e-3);
    EXPECT_NEAR(brain.state.pose.y, 1.5, 5e-3);
    EXPECT_NEAR(brain.state.pose.heading, 0.0, 1e-9);
    EXPECT_FALSE(brain.state.health.uncertain());
}

TEST(EndToEnd, CorruptionRestartAndGapDoNotMoveThePose) {
    FakePico pico;
    Link     link;
    Brain    brain(odometry::Pose{});
    uint32_t now = 0;

    brain.receive(link.carry(pico.record()), now);
    now += 20;
    pico.forward = 0.3;
    std::vector<uint8_t> bytes = pico.record();
    bytes[9] ^= 0xFF; // corrupted on the UART: dropped
    brain.receive(link.carry(bytes), now);
    now += 20;
    EXPECT_NEAR(brain.state.pose.x, 0.0, 1e-9);

    brain.receive(link.carry(pico.record()), now); // next good record carries the travel
    now += 20;
    EXPECT_NEAR(brain.state.pose.x, 0.3, 1e-3);

    // Pico restarts: counts back to zero, new run id. No jump back.
    pico     = FakePico{};
    pico.run = 0x0B0B;
    brain.receive(link.carry(pico.record()), now);
    now += 20;
    EXPECT_NEAR(brain.state.pose.x, 0.3, 1e-3);
    pico.forward = 0.1;
    brain.receive(link.carry(pico.record()), now);
    now += 20;
    EXPECT_NEAR(brain.state.pose.x, 0.4, 1e-3);

    // Link gap of 2 s while the robot is pushed 1 m: lost, reported.
    now += 2000;
    pico.forward = 1.1;
    brain.receive(link.carry(pico.record()), now);
    EXPECT_NEAR(brain.state.pose.x, 0.4, 1e-3);
    EXPECT_EQ(brain.state.health.restarts, 1u);
    EXPECT_EQ(brain.state.health.gaps, 1u);
    EXPECT_TRUE(brain.state.health.uncertain());
}

} // namespace
} // namespace localization
