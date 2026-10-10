// publisher_gtest.cpp

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>

#include "spectagatr/publisher.h"

namespace
{

using namespace spectagatr;
using protocol::DecodeError;
using protocol::PoseRecord;

constexpr SessionId kSession = 0x5A3C19E2;
constexpr double    kNaN     = std::numeric_limits<double>::quiet_NaN();
constexpr double    kInf     = std::numeric_limits<double>::infinity();

std::atomic<Milliseconds> g_now{0};

Milliseconds fakeClock() { return g_now.load(); }

struct Pulled {
    bool        ok = false;
    std::string text;
    PoseRecord  record;
};

Pulled pull(Publisher& publisher, Milliseconds now) {
    char              buffer[kLineCapacity];
    Pulled            out;
    const std::size_t length = publisher.nextLine(now, buffer, sizeof(buffer));
    if (length == 0) {
        return out;
    }
    out.text.assign(buffer, length);
    EXPECT_EQ(out.text.back(), '\n');
    const protocol::DecodeResult result =
        protocol::decodeLine(std::string_view(buffer, length - 1));
    EXPECT_EQ(result.error, DecodeError::kNone) << out.text;
    out.ok     = result.error == DecodeError::kNone;
    out.record = result.record;
    return out;
}

class SpectaPublisherTest : public ::testing::Test {
  protected:
    void SetUp() override {
        g_now = 1000;
        publisher.begin(kSession);
    }

    // New pose, pulled one period later than the last.
    Pulled publishHeading(Radians heading) {
        publisher.updatePose(1.0, 1.0, heading);
        now += 1000;
        return pull(publisher, now);
    }

    Publisher    publisher{&fakeClock};
    Milliseconds now = 1000;
};

TEST_F(SpectaPublisherTest, HeadingOverloadRoundTrip) {
    g_now = 123456;
    publisher.updatePose(1.5, -0.25, kPi / 2);
    const Pulled line = pull(publisher, 123460);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.session, kSession);
    EXPECT_EQ(line.record.epoch, 0);
    EXPECT_EQ(line.record.seq, 0u);
    EXPECT_EQ(line.record.t_ms, 123456u);
    EXPECT_EQ(line.record.x_mm, 1500);
    EXPECT_EQ(line.record.y_mm, -250);
    EXPECT_EQ(line.record.yaw_cdeg, 9000);
    EXPECT_EQ(line.record.roll_cdeg, 0);
    EXPECT_EQ(line.record.pitch_cdeg, 0);
    EXPECT_EQ(line.record.flags, 0);
    EXPECT_DOUBLE_EQ(protocol::toMeters(line.record.x_mm), 1.5);
    EXPECT_NEAR(protocol::toRadians(line.record.yaw_cdeg), kPi / 2, 1e-12);
}

TEST_F(SpectaPublisherTest, AttitudeOverloadRoundTrip) {
    for (int i = 0; i < 3; ++i) {
        publisher.resetTrail();
    }
    publisher.begin(kSession, 42);
    g_now = 123456;
    publisher.updatePose(1.5, -0.25,
                         Attitude{degreesToRadians(-1.5), degreesToRadians(0.75), kPi / 2});
    const Pulled line = pull(publisher, 123456);
    ASSERT_TRUE(line.ok);
    // The PROTOCOL.md example.
    EXPECT_EQ(line.text.substr(0, 51), "SGP1 5A3C19E2 3 42 123456 1500 -250 9000 -150 75 1*");
    EXPECT_EQ(line.record.roll_cdeg, -150);
    EXPECT_EQ(line.record.pitch_cdeg, 75);
    EXPECT_EQ(line.record.yaw_cdeg, 9000);
    EXPECT_EQ(line.record.flags, protocol::kFullAttitude);
}

TEST_F(SpectaPublisherTest, HeadingAfterAttitudeZeroesRollAndPitch) {
    publisher.updatePose(0.5, 0.5, Attitude{0.2, -0.1, 1.0});
    Pulled line = pull(publisher, 1000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.flags, protocol::kFullAttitude);
    EXPECT_NE(line.record.roll_cdeg, 0);
    EXPECT_NE(line.record.pitch_cdeg, 0);

    publisher.updatePose(0.5, 0.5, 1.0);
    line = pull(publisher, 2000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.flags, 0);
    EXPECT_EQ(line.record.roll_cdeg, 0);
    EXPECT_EQ(line.record.pitch_cdeg, 0);
    EXPECT_EQ(line.record.yaw_cdeg, 5730); // 1 rad = 57.2958 deg

    // Also when the attitude update was never sent.
    publisher.updatePose(0.5, 0.5, Attitude{0.2, -0.1, 1.0});
    publisher.updatePose(0.5, 0.5, 1.0);
    line = pull(publisher, 3000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.flags, 0);
    EXPECT_EQ(line.record.roll_cdeg, 0);
    EXPECT_EQ(line.record.pitch_cdeg, 0);
}

TEST_F(SpectaPublisherTest, InchAndDegreeInputsThroughTheHelpers) {
    publisher.updatePose(inchesToMeters(24.0), inchesToMeters(-48.0), degreesToRadians(-30.0));
    const Pulled line = pull(publisher, 1000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.x_mm, 610);   // 609.6 mm
    EXPECT_EQ(line.record.y_mm, -1219); // -1219.2 mm
    EXPECT_EQ(line.record.yaw_cdeg, -3000);
}

TEST_F(SpectaPublisherTest, HeadingWrapsIntoHalfOpenTurn) {
    EXPECT_EQ(publishHeading(3 * kPi / 2).record.yaw_cdeg, -9000);
    EXPECT_EQ(publishHeading(-3 * kPi / 2).record.yaw_cdeg, 9000);
    EXPECT_EQ(publishHeading(kPi).record.yaw_cdeg, 18000);
    EXPECT_EQ(publishHeading(-kPi).record.yaw_cdeg, 18000);
    EXPECT_EQ(publishHeading(2 * kPi + degreesToRadians(1.0)).record.yaw_cdeg, 100);
    EXPECT_EQ(publishHeading(-20 * kPi - degreesToRadians(0.5)).record.yaw_cdeg, -50);
    EXPECT_EQ(publishHeading(degreesToRadians(359.99)).record.yaw_cdeg, -1);
}

TEST_F(SpectaPublisherTest, NonFiniteRejectedKeepsPreviousPose) {
    publisher.updatePose(1.0, 2.0, 0.5);
    ASSERT_TRUE(pull(publisher, 1000).ok);

    publisher.updatePose(kNaN, 2.0, 0.5);
    publisher.updatePose(1.0, kInf, 0.5);
    publisher.updatePose(1.0, 2.0, kNaN);
    publisher.updatePose(1.0, 2.0, Attitude{kNaN, 0, 0});
    publisher.updatePose(1.0, 2.0, Attitude{0, -kInf, 0});
    publisher.updatePose(1.0, 2.0, Attitude{0, 0, kInf});
    EXPECT_EQ(publisher.stats().rejected, 6u);
    EXPECT_EQ(publisher.stats().updates, 1u);
    EXPECT_FALSE(pull(publisher, 5000).ok); // nothing new

    // A pending valid pose survives a later invalid one.
    publisher.updatePose(1.1, 2.2, 0.25);
    publisher.updatePose(kNaN, kNaN, kNaN);
    const Pulled line = pull(publisher, 6000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.x_mm, 1100);
    EXPECT_EQ(line.record.y_mm, 2200);
    EXPECT_EQ(line.record.yaw_cdeg, 1432); // 0.25 rad
    EXPECT_EQ(publisher.stats().rejected, 7u);
}

TEST_F(SpectaPublisherTest, OutOfRangeRejected) {
    publisher.updatePose(2000.0, -2000.0, 0.0);
    Pulled line = pull(publisher, 1000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.x_mm, 2000000);
    EXPECT_EQ(line.record.y_mm, -2000000);

    publisher.updatePose(2000.001, 0.0, 0.0);
    publisher.updatePose(0.0, -2500.0, 0.0);
    publisher.updatePose(1e300, 0.0, 0.0);
    publisher.updatePose(0.0, -1e300, Attitude{});
    EXPECT_EQ(publisher.stats().rejected, 4u);
    EXPECT_FALSE(pull(publisher, 5000).ok);
}

TEST_F(SpectaPublisherTest, RateCapTenHz) {
    int          lines = 0;
    Milliseconds last  = 0;
    for (Milliseconds t = 0; t < 1000; ++t) {
        g_now = t;
        publisher.updatePose(1.0, 1.0, 0.0);
        if (pull(publisher, t).ok) {
            if (lines > 0) {
                EXPECT_EQ(t - last, 100u);
            }
            last = t;
            ++lines;
        }
    }
    EXPECT_EQ(lines, 10);
    EXPECT_EQ(publisher.stats().lines, 10u);
    EXPECT_EQ(publisher.stats().overwritten, 891u); // 9 x 99; the last 99 are not sent yet
}

TEST_F(SpectaPublisherTest, RateCapFiftyHzWithSlowerPolling) {
    publisher.setRate(50);
    int lines = 0;
    // Updates every 2 ms, polls every 5 ms: 50 lines in one second.
    for (Milliseconds t = 0; t < 1000; ++t) {
        if (t % 2 == 0) {
            publisher.updatePose(1.0, 1.0, 0.0);
        }
        if (t % 5 == 0 && pull(publisher, t).ok) {
            ++lines;
        }
    }
    EXPECT_EQ(lines, 50);
}

TEST_F(SpectaPublisherTest, SlowPosesGoOutWhenTheyArrive) {
    // 4 Hz poses under a 10 Hz cap: every pose, no extra lines.
    int lines = 0;
    for (Milliseconds t = 0; t < 1000; ++t) {
        if (t % 250 == 0) {
            publisher.updatePose(1.0, 1.0, 0.0);
        }
        if (pull(publisher, t).ok) {
            EXPECT_EQ(t % 250, 0u);
            ++lines;
        }
    }
    EXPECT_EQ(lines, 4);
}

TEST_F(SpectaPublisherTest, RateClamped) {
    EXPECT_EQ(publisher.period(), 100u);
    EXPECT_DOUBLE_EQ(publisher.rate(), 10.0);
    publisher.setRate(0);
    EXPECT_EQ(publisher.period(), 1000u);
    publisher.setRate(-5);
    EXPECT_EQ(publisher.period(), 1000u);
    publisher.setRate(1000);
    EXPECT_EQ(publisher.period(), 20u);
    EXPECT_DOUBLE_EQ(publisher.rate(), 50.0);
    publisher.setRate(kNaN);
    EXPECT_EQ(publisher.period(), 100u);
    publisher.setRate(30);
    EXPECT_EQ(publisher.period(), 33u);
    Publisher slow(&fakeClock, 2);
    EXPECT_EQ(slow.period(), 500u);
}

TEST_F(SpectaPublisherTest, NoNewPoseNoLine) {
    publisher.updatePose(1.0, 1.0, 0.0);
    EXPECT_TRUE(pull(publisher, 0).ok);
    EXPECT_FALSE(pull(publisher, 500).ok);
    EXPECT_FALSE(pull(publisher, 1000).ok);
    EXPECT_FALSE(pull(publisher, 5000).ok);
    publisher.updatePose(1.0, 1.0, 0.0); // same values, new pose
    EXPECT_TRUE(pull(publisher, 5000).ok);
    EXPECT_EQ(publisher.stats().lines, 2u);
}

TEST_F(SpectaPublisherTest, NothingBeforeTheFirstPose) {
    EXPECT_FALSE(pull(publisher, 0).ok);
    EXPECT_FALSE(pull(publisher, 1000).ok);
}

TEST_F(SpectaPublisherTest, SeqIncrementsAndWraps) {
    publisher.begin(kSession, 0xFFFFFFFEu);
    const Sequence expected[] = {0xFFFFFFFEu, 0xFFFFFFFFu, 0u, 1u, 2u};
    for (Sequence seq : expected) {
        const Pulled line = publishHeading(0.0);
        ASSERT_TRUE(line.ok);
        EXPECT_EQ(line.record.seq, seq);
    }
}

TEST_F(SpectaPublisherTest, ResetTrailBumpsEpochForLaterPoses) {
    EXPECT_EQ(publishHeading(0.0).record.epoch, 0);

    // A pose supplied before the reset keeps its epoch.
    publisher.updatePose(1.0, 1.0, 0.0);
    publisher.resetTrail();
    Pulled line = pull(publisher, now += 1000);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.epoch, 0);

    EXPECT_EQ(publishHeading(0.0).record.epoch, 1);
    publisher.resetTrail();
    publisher.resetTrail();
    EXPECT_EQ(publishHeading(0.0).record.epoch, 3);
}

TEST_F(SpectaPublisherTest, EpochWraps) {
    for (int i = 0; i < 65535; ++i) {
        publisher.resetTrail();
    }
    EXPECT_EQ(publishHeading(0.0).record.epoch, 65535);
    publisher.resetTrail();
    EXPECT_EQ(publishHeading(0.0).record.epoch, 0);
}

TEST_F(SpectaPublisherTest, TimestampIsTheClockAtUpdateAndWraps) {
    g_now = 500;
    publisher.updatePose(1.0, 1.0, 0.0);
    g_now       = 700;
    Pulled line = pull(publisher, 700);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.t_ms, 500u);

    publisher.begin(kSession); // rate grid restarted near the wrap
    g_now = 0xFFFFFFF0u;
    publisher.updatePose(1.0, 1.0, 0.0);
    line = pull(publisher, 0xFFFFFFF0u);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.t_ms, 0xFFFFFFF0u);

    // The rate grid works across the clock wrap: next line due at 0x54.
    g_now = 0x10u;
    publisher.updatePose(1.0, 1.0, 0.0);
    EXPECT_FALSE(pull(publisher, 0x10u).ok);
    line = pull(publisher, 0x54u);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.t_ms, 0x10u);
}

TEST(SpectaPublisher, NoSessionNoLine) {
    Publisher publisher(&fakeClock);
    publisher.updatePose(1.0, 1.0, 0.0);
    EXPECT_EQ(publisher.session(), 0u);
    EXPECT_FALSE(pull(publisher, 0).ok);
    publisher.begin(kSession);
    EXPECT_EQ(publisher.session(), kSession);
    const Pulled line = pull(publisher, 0); // the pose from before begin
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.session, kSession);
}

TEST(SpectaPublisher, NullClockStampsZero) {
    Publisher publisher(nullptr);
    publisher.begin(kSession);
    publisher.updatePose(1.0, 1.0, 0.0);
    const Pulled line = pull(publisher, 0);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.record.t_ms, 0u);
}

TEST_F(SpectaPublisherTest, SmallBufferGivesNoLineAndKeepsThePose) {
    publisher.updatePose(1.0, 1.0, 0.0);
    char buffer[kLineCapacity];
    EXPECT_EQ(publisher.nextLine(0, buffer, kLineCapacity - 1), 0u);
    EXPECT_EQ(publisher.nextLine(0, nullptr, kLineCapacity), 0u);
    EXPECT_TRUE(pull(publisher, 0).ok);
}

TEST_F(SpectaPublisherTest, Stats) {
    for (int i = 0; i < 5; ++i) {
        publisher.updatePose(1.0, 1.0, 0.0);
    }
    ASSERT_TRUE(pull(publisher, 0).ok);
    publisher.updatePose(1.0, 1.0, 0.0);
    ASSERT_TRUE(pull(publisher, 100).ok);
    publisher.recordWrite(false);
    publisher.recordWrite(true);
    publisher.recordWrite(false);
    const PublisherStats s = publisher.stats();
    EXPECT_EQ(s.updates, 6u);
    EXPECT_EQ(s.lines, 2u);
    EXPECT_EQ(s.overwritten, 4u);
    EXPECT_EQ(s.rejected, 0u);
    EXPECT_EQ(s.write_errors, 2u);
}

TEST(SpectaPublisher, LongestLine) {
    Publisher publisher(+[]() -> Milliseconds { return 0xFFFFFFFFu; });
    publisher.begin(0xFFFFFFFFu, 0xFFFFFFFFu);
    for (int i = 0; i < 65535; ++i) {
        publisher.resetTrail();
    }
    const Radians most = degreesToRadians(-179.99);
    publisher.updatePose(-2000.0, -2000.0, Attitude{most, most, most});
    const Pulled line = pull(publisher, 0);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.text.size(), 88u) << line.text;
    EXPECT_LE(line.text.size(), protocol::kMaxLineLength);
}

TEST(SpectaPublisher, TypicalLine) {
    Publisher publisher(+[]() -> Milliseconds { return 123456u; });
    publisher.begin(kSession, 1234);
    publisher.updatePose(1.778, 1.778, degreesToRadians(90.0));
    const Pulled line = pull(publisher, 0);
    ASSERT_TRUE(line.ok);
    EXPECT_EQ(line.text.substr(0, 49), "SGP1 5A3C19E2 0 1234 123456 1778 1778 9000 0 0 0*");
    EXPECT_EQ(line.text.size(), 54u);
}

} // namespace
