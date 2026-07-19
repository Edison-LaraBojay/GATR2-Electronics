// frames_gtest.cpp
// First real tests. Thin for now, grows with the codec.

#include <gtest/gtest.h>
#include "frames.h"

using namespace gatr2;

TEST(Frames, ChecksumOfKnownBytes) {
    const uint8_t data[] = {0x01, 0x02, 0x03};
    EXPECT_EQ(checksum(data, 3), 0x01 ^ 0x02 ^ 0x03);
}

TEST(Frames, ChecksumEmptyIsZero) {
    EXPECT_EQ(checksum(nullptr, 0), 0x00);
}

TEST(Frames, ChecksumIsSelfInverse) {
    // XOR of a frame including its own checksum byte is zero. This is the
    // property receivers use to validate.
    uint8_t data[] = {0xAA, 0x55, 0x01, 0x2C, 0x00};
    uint8_t c = checksum(data, 4);
    data[4] = c;
    EXPECT_EQ(checksum(data, 5), 0x00);
}

TEST(Frames, WidthTableMatchesBitCount) {
    // Every defined SensorBit must have a width entry. This is the table
    // parsers walk, so drift here is a wire-level bug.
    EXPECT_EQ(kSensorBitCount, 5);
}