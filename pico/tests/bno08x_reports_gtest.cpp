#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "bno08x_reports.h"

namespace {

using bno08x::Bno08xReports;

constexpr uint8_t kAccel = 0x01;
constexpr uint8_t kGyro = 0x07;
constexpr int16_t kOneGQ8 = 2511; // 9.8086 m/s^2 after Q8 quantization.

template <std::size_t N>
void putI16(std::array<uint8_t, N>& report, std::size_t offset, int16_t value) {
    const uint16_t bits = static_cast<uint16_t>(value);
    report[offset] = static_cast<uint8_t>(bits);
    report[offset + 1] = static_cast<uint8_t>(bits >> 8);
}

std::array<uint8_t, 10> accel(int16_t x, int16_t y, int16_t z) {
    std::array<uint8_t, 10> report{};
    report[0] = kAccel;
    putI16(report, 4, x);
    putI16(report, 6, y);
    putI16(report, 8, z);
    return report;
}

std::array<uint8_t, 16> gyro(int16_t x, int16_t y, int16_t z) {
    std::array<uint8_t, 16> report{};
    report[0] = kGyro;
    putI16(report, 4, x);
    putI16(report, 6, y);
    putI16(report, 8, z);
    // The existing telemetry contract deliberately leaves bias removal to Pi.
    // These fields must not be subtracted or mistaken for measured rates.
    putI16(report, 10, 12345);
    putI16(report, 12, -23456);
    putI16(report, 14, 30000);
    return report;
}

template <std::size_t N>
bool add(Bno08xReports& reports, const std::array<uint8_t, N>& report,
         uint32_t sample_us, uint32_t received_us) {
    return reports.add(report[0], report.data(), static_cast<uint16_t>(report.size()),
                       sample_us, received_us);
}

template <std::size_t N>
bool add(Bno08xReports& reports, const std::array<uint8_t, N>& report, uint32_t time_us) {
    return add(reports, report, time_us, time_us);
}

void acknowledgeBoth(Bno08xReports& reports) {
    reports.acknowledge(kAccel, 10000);
    reports.acknowledge(kGyro, 5000);
}

// Realistic 100 Hz accelerometer / 200 Hz gyro callback order, including
// telemetry cuts during calibration. Caller controls acknowledgments.
uint32_t settle(Bno08xReports& reports, const std::array<uint8_t, 10>& gravity,
                uint32_t start_us = 1000) {
    const auto stationary = gyro(0, 0, 0);
    uint32_t now = start_us;
    for (uint32_t i = 0; i <= 420; ++i) {
        now = start_us + i * 5000;
        EXPECT_TRUE(add(reports, stationary, now));
        if (i % 2 == 0) {
            EXPECT_TRUE(add(reports, gravity, now));
        }
        int32_t value = 7654321;
        const bool available = reports.take(now, value);
        if (i < 400) {
            EXPECT_FALSE(available);
            EXPECT_EQ(value, 7654321); // Unavailable is not a zero yaw sample.
        }
    }
    return now;
}

} // namespace

TEST(Bno08xReports, NeedsBothFeatureAcknowledgmentsAndRevocationStopsOutput) {
    Bno08xReports reports;
    EXPECT_FALSE(reports.acknowledged());
    reports.acknowledge(0x05, 5000);
    reports.acknowledge(kAccel, 0);
    EXPECT_FALSE(reports.acknowledged());
    reports.acknowledge(kGyro, 5000);
    EXPECT_FALSE(reports.acknowledged());

    const uint32_t now = settle(reports, accel(0, 0, kOneGQ8));
    int32_t value = 123;
    EXPECT_FALSE(reports.take(now, value));
    EXPECT_EQ(value, 123);

    reports.acknowledge(kAccel, 10000);
    EXPECT_TRUE(reports.acknowledged());
    ASSERT_TRUE(add(reports, gyro(0, 0, 512), now + 5000));
    ASSERT_TRUE(reports.take(now + 5000, value));
    EXPECT_EQ(value, 57296); // 1 rad/s, existing millidegrees/second wire unit.

    reports.acknowledge(kGyro, 0);
    EXPECT_FALSE(reports.acknowledged());
    ASSERT_TRUE(add(reports, gyro(0, 0, 512), now + 10000));
    value = 321;
    EXPECT_FALSE(reports.take(now + 10000, value));
    EXPECT_EQ(value, 321);
    reports.acknowledge(kGyro, 5000);
    EXPECT_FALSE(reports.take(now + 10000, value)); // Gated reports were dropped.
}

TEST(Bno08xReports, DecodesSignedXyzAndProjectsSidewaysMountWithoutUsingBiasFields) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    const uint32_t now = settle(reports, accel(0, -kOneGQ8, 0));
    ASSERT_TRUE(reports.aligned());
    ASSERT_TRUE(reports.healthy(now));

    // Sensor -Y is physical up. X and Z rotation must not contribute.
    ASSERT_TRUE(add(reports, gyro(700, -512, -900), now + 5000));
    int32_t value = 0;
    ASSERT_TRUE(reports.take(now + 5000, value));
    EXPECT_EQ(value, 57296);
    ASSERT_TRUE(add(reports, gyro(-700, 512, 900), now + 10000));
    ASSERT_TRUE(reports.take(now + 10000, value));
    EXPECT_EQ(value, -57296);
}

TEST(Bno08xReports, BatchesAfterAlignmentProduceOneMeanAndNeverRepeatIt) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    const uint32_t now = settle(reports, accel(0, 0, kOneGQ8));
    ASSERT_TRUE(reports.aligned());
    ASSERT_TRUE(add(reports, gyro(0, 0, 256), now + 5000, now + 20000));
    ASSERT_TRUE(add(reports, gyro(0, 0, 768), now + 10000, now + 20000));
    ASSERT_TRUE(add(reports, accel(0, 0, kOneGQ8), now + 10000, now + 20000));
    int32_t value = 0;
    ASSERT_TRUE(reports.take(now + 20000, value));
    EXPECT_EQ(value, 57296);
    value = 999;
    EXPECT_FALSE(reports.take(now + 25000, value));
    EXPECT_EQ(value, 999);
}

TEST(Bno08xReports, EitherMissingReportStreamPreventsAlignmentAndYaw) {
    Bno08xReports accel_only;
    Bno08xReports gyro_only;
    acknowledgeBoth(accel_only);
    acknowledgeBoth(gyro_only);
    const auto gravity = accel(0, 0, kOneGQ8);
    const auto stationary = gyro(0, 0, 0);
    for (uint32_t now = 1000; now <= 3001000; now += 10000) {
        ASSERT_TRUE(add(accel_only, gravity, now));
        ASSERT_TRUE(add(gyro_only, stationary, now));
    }
    EXPECT_FALSE(accel_only.aligned());
    EXPECT_FALSE(gyro_only.aligned());
    EXPECT_FALSE(accel_only.healthy(3001000));
    EXPECT_FALSE(gyro_only.healthy(3001000));
    int32_t value = 456;
    EXPECT_FALSE(accel_only.take(3001000, value));
    EXPECT_FALSE(gyro_only.take(3001000, value));
    EXPECT_EQ(value, 456);
}

TEST(Bno08xReports, RejectsMalformedTruncatedUnknownStaleAndFutureReports) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    const auto gravity = accel(0, 0, kOneGQ8);
    const auto rate = gyro(0, 0, 0);
    EXPECT_FALSE(reports.add(kAccel, gravity.data(), 9, 1000, 1000));
    EXPECT_FALSE(reports.add(kGyro, rate.data(), 10, 1000, 1000));
    EXPECT_FALSE(reports.add(kGyro, rate.data(), 15, 1000, 1000));
    EXPECT_FALSE(reports.add(kGyro, nullptr, 16, 1000, 1000));
    EXPECT_FALSE(reports.add(0x05, rate.data(), 16, 1000, 1000));
    auto wrong_id = rate;
    wrong_id[0] = kAccel;
    EXPECT_FALSE(reports.add(kGyro, wrong_id.data(), 16, 1000, 1000));
    EXPECT_FALSE(add(reports, rate, 1000, 101001));
    EXPECT_FALSE(add(reports, gravity, 1000, 101001));
    EXPECT_FALSE(add(reports, rate, 2001, 2000));
    EXPECT_FALSE(add(reports, gravity, 2001, 2000));
    EXPECT_FALSE(reports.healthy(2000));
    EXPECT_FALSE(reports.aligned());
}

TEST(Bno08xReports, DuplicateAndOlderTimestampsCannotRefreshStreamHealth) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    const uint32_t now = settle(reports, accel(0, 0, kOneGQ8));
    const auto gravity = accel(0, 0, kOneGQ8);
    const auto rate = gyro(0, 0, 0);
    ASSERT_TRUE(reports.healthy(now));
    EXPECT_FALSE(add(reports, gravity, now, now + 50000));
    EXPECT_FALSE(add(reports, rate, now, now + 50000));
    EXPECT_FALSE(add(reports, gravity, now - 5000, now + 50000));
    EXPECT_FALSE(add(reports, rate, now - 5000, now + 50000));
    EXPECT_FALSE(reports.healthy(now + 100001));
    int32_t value = 987;
    EXPECT_FALSE(reports.take(now + 100001, value));
    EXPECT_EQ(value, 987);
}

TEST(Bno08xReports, MissingAccelerationAfterAlignmentGatesAndDropsQueuedYaw) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    const uint32_t now = settle(reports, accel(0, 0, kOneGQ8));
    ASSERT_TRUE(add(reports, gyro(0, 0, 512), now + 90000));
    ASSERT_TRUE(add(reports, gyro(0, 0, 512), now + 105000));
    EXPECT_FALSE(reports.healthy(now + 105000));
    int32_t value = 753;
    EXPECT_FALSE(reports.take(now + 105000, value));
    EXPECT_EQ(value, 753);

    ASSERT_TRUE(add(reports, accel(0, 0, kOneGQ8), now + 105000));
    EXPECT_TRUE(reports.healthy(now + 105000));
    EXPECT_FALSE(reports.take(now + 105000, value));
    ASSERT_TRUE(add(reports, gyro(0, 0, 256), now + 110000));
    ASSERT_TRUE(reports.take(now + 110000, value));
    EXPECT_EQ(value, 28648);
}

TEST(Bno08xReports, ResetClearsAcknowledgmentsAlignmentAndBufferedYaw) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    uint32_t now = settle(reports, accel(0, 0, kOneGQ8));
    ASSERT_TRUE(add(reports, gyro(0, 0, 512), now + 5000));
    reports.reset();
    EXPECT_FALSE(reports.acknowledged());
    EXPECT_FALSE(reports.aligned());
    EXPECT_FALSE(reports.healthy(now + 5000));
    int32_t value = 159;
    EXPECT_FALSE(reports.take(now + 5000, value));
    EXPECT_EQ(value, 159);

    acknowledgeBoth(reports);
    now = settle(reports, accel(-kOneGQ8, 0, 0), now + 10000);
    ASSERT_TRUE(reports.aligned());
    ASSERT_TRUE(add(reports, gyro(-512, 800, -900), now + 5000));
    ASSERT_TRUE(reports.take(now + 5000, value));
    EXPECT_EQ(value, 57296); // New startup mounting axis, no old Z sample.
}

TEST(Bno08xReports, StartupAndProjectionWorkAcrossMicrosecondClockWrap) {
    Bno08xReports reports;
    acknowledgeBoth(reports);
    const uint32_t now = settle(reports, accel(0, 0, -kOneGQ8), 0xFFF00000u);
    ASSERT_TRUE(reports.aligned());
    ASSERT_TRUE(reports.healthy(now));
    ASSERT_TRUE(add(reports, gyro(500, -500, -512), now + 5000));
    int32_t value = 0;
    ASSERT_TRUE(reports.take(now + 5000, value));
    EXPECT_EQ(value, 57296);
    EXPECT_FALSE(reports.healthy(now + 105001));
}
