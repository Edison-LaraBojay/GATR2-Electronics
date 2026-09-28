#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "impl/resources/camera_capture.h"

using namespace navigatr;

TEST(CameraCapture, ConvertsBootTimeWithoutAssumingHostEpochOrSuspendOffset) {
    CaptureClockSample sample;
    sample.valid = true;
    // 24 hours since boot; the host clock only started ten seconds ago.
    sample.source_now_ns = INT64_C(86400000000000);
    sample.host_before = hostTime(10000);
    sample.host_after = hostTime(10002);
    const auto stamp = correlateCaptureTimestamp(sample.source_now_ns - 45000000, sample);
    ASSERT_TRUE(stamp.has_value());
    EXPECT_EQ(stamp->at.domain, ClockDomain::kHost);
    EXPECT_EQ(stamp->at.ms, 9956);
    EXPECT_EQ(stamp->uncertainty_ms, 3);

    // After a suspend, BOOTTIME advanced another hour while steady host
    // time advanced one second. A newly sampled pair still maps correctly.
    sample.source_now_ns += INT64_C(3601000000000);
    sample.host_before = hostTime(11000);
    sample.host_after = hostTime(11002);
    const auto resumed = correlateCaptureTimestamp(sample.source_now_ns - 45000000, sample);
    ASSERT_TRUE(resumed.has_value());
    EXPECT_EQ(resumed->at.ms, 10956);
}

TEST(CameraCapture, RejectsInvalidClockSamplesInsteadOfInventingExposure) {
    CaptureClockSample sample{INT64_C(1000000000), hostTime(100), hostTime(102), true};
    EXPECT_FALSE(correlateCaptureTimestamp(-1, sample));
    EXPECT_FALSE(correlateCaptureTimestamp(INT64_C(1000000001), sample));
    sample.host_after = deviceTime(102);
    EXPECT_FALSE(correlateCaptureTimestamp(INT64_C(900000000), sample));
    sample.host_after = hostTime(99);
    EXPECT_FALSE(correlateCaptureTimestamp(INT64_C(900000000), sample));
    sample.host_after = hostTime(102);
    sample.valid = false;
    EXPECT_FALSE(correlateCaptureTimestamp(INT64_C(900000000), sample));
}

TEST(CameraCapture, ExposureUsesMetadataAndCoversFullRollingReadout) {
    const CaptureTimestamp sensor{hostTime(1000), 2};
    const auto timing = estimateCaptureExposure(sensor, 30000, 33333, hostTime(1080));
    EXPECT_TRUE(timing.reliable);
    EXPECT_EQ(timing.at.ms, 1015);
    EXPECT_EQ(timing.uncertainty_ms, 37);
    // First-line exposure interpretation and readout-start interpretation,
    // at both ends of the unknown readout duration, fit the declared bound.
    for (int truth_ms : {1015, 1032, 985, 1002}) {
        EXPECT_LE(std::abs(timing.at.ms - truth_ms), timing.uncertainty_ms);
    }
    // A long actual exposure must widen the bound even if configured fps
    // or rounded frame-duration metadata claims something shorter.
    const auto slow = estimateCaptureExposure(sensor, 100000, 33333, hostTime(1150));
    EXPECT_EQ(slow.at.ms, 1050);
    EXPECT_GE(slow.uncertainty_ms, 100);
}

TEST(CameraCapture, MissingOrImpossibleMetadataIsExplicitlyUnreliable) {
    const auto missing = estimateCaptureExposure(std::nullopt, 10000, 33333, hostTime(2000));
    EXPECT_FALSE(missing.reliable);
    EXPECT_EQ(missing.at.ms, 2000);
    const auto future = estimateCaptureExposure(CaptureTimestamp{hostTime(2010), 2},
                                                 10000, 33333, hostTime(2000));
    EXPECT_FALSE(future.reliable);
    EXPECT_EQ(future.at.ms, 2000);
    EXPECT_FALSE(estimateCaptureExposure(CaptureTimestamp{hostTime(1950), 2},
                                         -1, 33333, hostTime(2000)).reliable);
    const auto unknown_exposure = estimateCaptureExposure(CaptureTimestamp{hostTime(1950), 2},
                                                           std::nullopt, 33333, hostTime(2000));
    EXPECT_TRUE(unknown_exposure.reliable);
    EXPECT_EQ(unknown_exposure.at.ms, 1950);
    EXPECT_GE(unknown_exposure.uncertainty_ms, 34);
}

TEST(CameraCapture, CopiesOnlyVisiblePixelsAndOwnsThemAfterDriverReuse) {
    std::vector<uint8_t> driver{1, 2, 3, 99, 99, 4, 5, 6, 99, 99};
    std::vector<uint8_t> frame;
    ASSERT_TRUE(copyY8Plane(driver.data(), driver.size(), 3, 2, 5, frame));
    EXPECT_EQ(frame, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6}));
    std::fill(driver.begin(), driver.end(), 0);
    EXPECT_EQ(frame, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6}));
}

TEST(CameraCapture, RejectsTruncatedCompletedPlanesAndInvalidStrides) {
    const std::vector<uint8_t> driver(10, 42);
    std::vector<uint8_t> frame{7};
    EXPECT_FALSE(copyY8Plane(driver.data(), 7, 3, 2, 5, frame));
    EXPECT_EQ(frame, (std::vector<uint8_t>{7}));
    EXPECT_FALSE(copyY8Plane(driver.data(), 10, 3, 2, 2, frame));
    EXPECT_FALSE(copyY8Plane(nullptr, 10, 3, 2, 5, frame));
    EXPECT_FALSE(copyY8Plane(driver.data(), 10, 0, 2, 5, frame));
    EXPECT_FALSE(copyY8Plane(driver.data(), 10, 3,
                           std::numeric_limits<std::size_t>::max(), 5, frame));
    // Padding after the last visible row need not be part of bytes-used.
    EXPECT_TRUE(copyY8Plane(driver.data(), 8, 3, 2, 5, frame));
}
