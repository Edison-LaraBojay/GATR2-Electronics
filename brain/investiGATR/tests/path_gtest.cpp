// path_gtest.cpp

#include "investigatr/path.h"

#include <gtest/gtest.h>

using namespace investigatr;

TEST(Path, LengthCountsTranslationsOnly) {
    Path path;
    EXPECT_TRUE(path.empty());
    EXPECT_DOUBLE_EQ(path.length(), 0.0);

    PathSegment spin;
    spin.kind  = SegmentKind::kTurn;
    spin.start = Pose{0.0, 0.0, 0.0};
    spin.end   = Pose{0.0, 0.0, 1.0};

    PathSegment a;
    a.start = Pose{0.0, 0.0, 0.0};
    a.end   = Pose{3.0, 4.0, 0.0};
    PathSegment b;
    b.start = Pose{3.0, 4.0, 0.0};
    b.end   = Pose{3.0, 2.0, 0.0};

    path.segments = {spin, a, spin, b};
    EXPECT_FALSE(path.empty());
    EXPECT_DOUBLE_EQ(path.length(), 7.0);
}
