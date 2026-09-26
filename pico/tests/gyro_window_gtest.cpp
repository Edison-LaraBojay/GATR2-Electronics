// gyro_window_gtest.cpp

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gyro_window.h"

using namespace bno08x;

TEST(GyroWindow, MeanOverInterval) {
    GyroWindow w;
    w.add(1000, 100);
    w.add(6000, 102);
    w.add(11000, 98);
    w.add(16000, 104);

    int32_t mdps = 0;
    ASSERT_TRUE(w.take(20000, mdps));
    EXPECT_EQ(mdps, 11302); // 101 LSB
    EXPECT_EQ(w.pending(), 0);
}

TEST(GyroWindow, EmptyReturnsFalseAndNeverRepeats) {
    GyroWindow w;
    int32_t    mdps = 0;
    EXPECT_FALSE(w.take(20000, mdps));

    w.add(25000, 10);
    ASSERT_TRUE(w.take(40000, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(10, 1));
    EXPECT_FALSE(w.take(60000, mdps));
}

TEST(GyroWindow, ReportAfterCutWaitsForNextTake) {
    GyroWindow w;
    w.add(19000, 10);
    w.add(21000, 20);

    int32_t mdps = 0;
    ASSERT_TRUE(w.take(20000, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(10, 1));
    EXPECT_EQ(w.pending(), 1);
    ASSERT_TRUE(w.take(40000, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(20, 1));
}

TEST(GyroWindow, ReportAtCutBelongsToThatCut) {
    GyroWindow w;
    w.add(20000, 30);
    int32_t mdps = 0;
    ASSERT_TRUE(w.take(20000, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(30, 1));
}

TEST(GyroWindow, LateReportsDroppedAndCounted) {
    GyroWindow w;
    int32_t    mdps = 0;
    EXPECT_FALSE(w.take(20000, mdps));

    w.add(15000, 5);
    w.add(20000, 5);
    w.add(20001, 7);
    EXPECT_EQ(w.late(), 2u);
    EXPECT_EQ(w.pending(), 1);

    ASSERT_TRUE(w.take(40000, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(7, 1));
}

TEST(GyroWindow, IntervalAcrossWrap) {
    GyroWindow     w;
    int32_t        mdps = 0;
    const uint32_t cut  = 0xFFFFFF00u;
    EXPECT_FALSE(w.take(cut, mdps));

    w.add(0xFFFFFE00u, 99); // before the cut, late
    w.add(0xFFFFFFF0u, 10);
    w.add(0x00000010u, 20);
    EXPECT_EQ(w.late(), 1u);

    ASSERT_TRUE(w.take(0x00000100u, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(30, 2));

    w.add(0x00000080u, 1); // before the new cut, late
    EXPECT_EQ(w.late(), 2u);
}

TEST(GyroWindow, ClearDropsPendingKeepsCut) {
    GyroWindow w;
    int32_t    mdps = 0;
    EXPECT_FALSE(w.take(20000, mdps));

    w.add(25000, 10);
    w.clear();
    EXPECT_FALSE(w.take(40000, mdps));

    w.add(39000, 10);
    EXPECT_EQ(w.late(), 1u);
}

TEST(GyroWindow, FullWindowDropsAndCounts) {
    GyroWindow w;
    for (uint32_t i = 0; i < GyroWindow::kCapacity + 3; ++i) {
        w.add(1000 + i, 4);
    }
    EXPECT_EQ(w.overflowed(), 3u);
    EXPECT_EQ(w.pending(), GyroWindow::kCapacity);

    int32_t mdps = 0;
    ASSERT_TRUE(w.take(20000, mdps));
    EXPECT_EQ(mdps, q9MeanToMdps(4, 1));
}

TEST(GyroWindow, AtOrBeforeWraps) {
    EXPECT_TRUE(atOrBefore(5, 5));
    EXPECT_TRUE(atOrBefore(4, 5));
    EXPECT_FALSE(atOrBefore(6, 5));
    EXPECT_TRUE(atOrBefore(0xFFFFFFF0u, 0x10u));
    EXPECT_FALSE(atOrBefore(0x10u, 0xFFFFFFF0u));
}

TEST(Q9MeanToMdps, RoundingAndSign) {
    // 180000 / (512 * pi) = 111.90581936 mdeg/s per LSB.
    EXPECT_EQ(q9MeanToMdps(0, 3), 0);
    EXPECT_EQ(q9MeanToMdps(1, 1), 112);
    EXPECT_EQ(q9MeanToMdps(-1, 1), -112);
    EXPECT_EQ(q9MeanToMdps(6, 1), 671); // 671.43
    EXPECT_EQ(q9MeanToMdps(-6, 1), -671);
    EXPECT_EQ(q9MeanToMdps(1, 2), 56); // 55.95
    EXPECT_EQ(q9MeanToMdps(-1, 2), -56);
    EXPECT_EQ(q9MeanToMdps(512, 1), 57296); // 1 rad/s, 57295.78
    EXPECT_EQ(q9MeanToMdps(-512, 1), -57296);
    EXPECT_EQ(q9MeanToMdps(32767, 1), 3666818);
    EXPECT_EQ(q9MeanToMdps(-32768, 1), -3666930);
    EXPECT_EQ(q9MeanToMdps(-32768 * 32, 32), -3666930);
}

TEST(GyroWindow, KeepsFractionalProjectionUntilMeanIsConverted) {
    GyroWindow w;
    w.add(1000, 0.25f);
    w.add(6000, 0.5f);
    int32_t mdps = 0;
    ASSERT_TRUE(w.take(20000, mdps));
    EXPECT_EQ(mdps, 42);
}

TEST(GyroWindow, ProjectionCannotWrapAtTheSingleAxisLimit) {
    GyroWindow w;
    // Dotting three maximum readings with (1,1,1)/sqrt(3).
    const float projected = 32767.0f * std::sqrt(3.0f);
    w.add(1000, projected);
    int32_t mdps = 0;
    ASSERT_TRUE(w.take(20000, mdps));
    EXPECT_GT(mdps, 6000000);
    EXPECT_EQ(mdps, q9MeanToMdps(projected, 1));
    w.add(21000, -projected);
    ASSERT_TRUE(w.take(40000, mdps));
    EXPECT_EQ(mdps, -q9MeanToMdps(projected, 1));
}

TEST(GyroWindow, RejectsNonfiniteAndImpossibleRates) {
    GyroWindow w;
    w.add(1000, std::numeric_limits<float>::quiet_NaN());
    w.add(2000, std::numeric_limits<float>::infinity());
    w.add(3000, 100000.0f);
    int32_t mdps = 123;
    EXPECT_FALSE(w.take(20000, mdps));
    EXPECT_EQ(mdps, 123);
}
