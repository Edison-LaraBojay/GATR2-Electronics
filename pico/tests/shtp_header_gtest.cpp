// shtp_header_gtest.cpp

#include <gtest/gtest.h>

#include "shtp_header.h"

using namespace bno08x;

TEST(ShtpHeader, NullHeaderIsEmpty) {
    const uint8_t h[] = {0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(shtpRxLen(h), 0);
}

TEST(ShtpHeader, AllOnesRejected) {
    const uint8_t a[] = {0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t b[] = {0xFF, 0xFF, 0x02, 0x00};
    EXPECT_EQ(shtpRxLen(a), 0);
    EXPECT_EQ(shtpRxLen(b), 0);
}

TEST(ShtpHeader, ValidLengths) {
    const uint8_t header_only[] = {0x04, 0x00, 0x02, 0x00};
    const uint8_t gyro[]        = {0x19, 0x00, 0x03, 0x07};
    const uint8_t largest[]     = {0x00, 0x04, 0x00, 0x01};
    EXPECT_EQ(shtpRxLen(header_only), 4);
    EXPECT_EQ(shtpRxLen(gyro), 25);
    EXPECT_EQ(shtpRxLen(largest), 1024);
}

TEST(ShtpHeader, ContinuationBitMasked) {
    const uint8_t h[] = {0x19, 0x80, 0x03, 0x00};
    EXPECT_EQ(shtpRxLen(h), 25);
}

TEST(ShtpHeader, ShortLengthsRejected) {
    for (uint8_t len = 1; len < 4; ++len) {
        const uint8_t h[] = {len, 0x00, 0x02, 0x00};
        EXPECT_EQ(shtpRxLen(h), 0) << int(len);
    }
}

TEST(ShtpHeader, OverlongRejected) {
    const uint8_t a[] = {0x01, 0x04, 0x02, 0x00}; // 1025
    const uint8_t b[] = {0xFF, 0x7F, 0x02, 0x00}; // 32767
    EXPECT_EQ(shtpRxLen(a), 0);
    EXPECT_EQ(shtpRxLen(b), 0);
}

TEST(ShtpHeader, ChannelAboveFiveRejected) {
    const uint8_t five[] = {0x19, 0x00, 0x05, 0x00};
    EXPECT_EQ(shtpRxLen(five), 25);
    for (int chan : {6, 7, 8, 255}) {
        const uint8_t h[] = {0x19, 0x00, static_cast<uint8_t>(chan), 0x00};
        EXPECT_EQ(shtpRxLen(h), 0) << chan;
    }
}
