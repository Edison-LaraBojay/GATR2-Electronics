// pose_line_gtest.cpp
// Wire format v1: encode/decode round trips, CRC, ranges, malformed lines,
// and the marker scanner over fragmented, COBS-framed and garbage input.

#include "spectagatr/protocol/pose_line.h"
#include "spectagatr/protocol/units.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace spectagatr::protocol;

namespace
{

PoseRecord sample() {
    PoseRecord r;
    r.session    = 0x5A3C19E2;
    r.epoch      = 3;
    r.seq        = 42;
    r.t_ms       = 123456;
    r.x_mm       = 1500;
    r.y_mm       = -250;
    r.yaw_cdeg   = 9000;
    r.roll_cdeg  = -150;
    r.pitch_cdeg = 75;
    r.flags      = kFullAttitude;
    return r;
}

std::string encode(const PoseRecord& r) {
    char              buf[kMaxLineLength + 8];
    const std::size_t n = encodeLine(r, buf, sizeof(buf));
    return std::string(buf, n);
}

std::vector<std::string> scanAll(LineScanner& scanner, const std::string& bytes) {
    std::vector<std::string> lines;
    scanner.feed(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size(),
                 [&](std::string_view l) { lines.emplace_back(l); });
    return lines;
}

// PROS COBS packet: stream id then data, COBS encoded, 0x00 terminated.
std::string cobsPacket(const std::string& stream, const std::string& data) {
    const std::string raw = stream + data;
    std::string       out;
    std::size_t       code_at = 0;
    out.push_back('\x01');
    uint8_t code = 1;
    for (char c : raw) {
        if (c == '\0' || code == 0xFF) {
            out[code_at] = static_cast<char>(code);
            code_at      = out.size();
            out.push_back('\x01');
            code = 1;
            if (c == '\0') {
                continue;
            }
        }
        out.push_back(c);
        ++code;
    }
    out[code_at] = static_cast<char>(code);
    out.push_back('\0');
    return out;
}

} // namespace

TEST(PoseLine, KnownLineAndCrc) {
    // CRC-16/CCITT-FALSE check value.
    EXPECT_EQ(crc16("123456789"), 0x29B1);
    const std::string line = encode(sample());
    ASSERT_FALSE(line.empty());
    EXPECT_EQ(line.substr(0, line.find('*')), "SGP1 5A3C19E2 3 42 123456 1500 -250 9000 -150 75 1");
    EXPECT_EQ(line.back(), '\n');
    EXPECT_LE(line.size(), kMaxLineLength);
    EXPECT_EQ(line.find('\0'), std::string::npos);
}

TEST(PoseLine, RoundTripsExtremes) {
    PoseRecord r           = sample();
    r.session              = 0xFFFFFFFF;
    r.epoch                = 0xFFFF;
    r.seq                  = 0xFFFFFFFF;
    r.t_ms                 = 0xFFFFFFFF;
    r.x_mm                 = -kMaxCoordinateMm;
    r.y_mm                 = kMaxCoordinateMm;
    r.yaw_cdeg             = 18000;
    r.roll_cdeg            = -17999;
    r.pitch_cdeg           = 18000;
    r.flags                = 0xFF;
    const std::string line = encode(r);
    ASSERT_FALSE(line.empty());
    EXPECT_LE(line.size(), kMaxLineLength);
    const DecodeResult d = decodeLine(std::string_view(line).substr(0, line.size() - 1));
    ASSERT_EQ(d.error, DecodeError::kNone) << toString(d.error);
    EXPECT_EQ(d.record, r);
}

TEST(PoseLine, EncodeRefusesInvalidRecords) {
    char       buf[kMaxLineLength];
    PoseRecord r = sample();
    r.session    = 0;
    EXPECT_EQ(encodeLine(r, buf, sizeof(buf)), 0u);
    r          = sample();
    r.yaw_cdeg = -18000; // (-18000, 18000]
    EXPECT_EQ(encodeLine(r, buf, sizeof(buf)), 0u);
    r      = sample();
    r.x_mm = kMaxCoordinateMm + 1;
    EXPECT_EQ(encodeLine(r, buf, sizeof(buf)), 0u);
    EXPECT_EQ(encodeLine(sample(), buf, 20), 0u); // too small
}

TEST(PoseLine, DecodeClassifiesErrors) {
    std::string line = encode(sample());
    line.pop_back(); // '\n'
    EXPECT_EQ(decodeLine(line).error, DecodeError::kNone);
    EXPECT_EQ(decodeLine(line + "\r").error, DecodeError::kNone);

    std::string flipped = line;
    flipped[20]         = flipped[20] == '1' ? '2' : '1';
    EXPECT_EQ(decodeLine(flipped).error, DecodeError::kBadCrc);

    EXPECT_EQ(decodeLine("").error, DecodeError::kMalformed);
    EXPECT_EQ(decodeLine("SGP2 " + line.substr(5)).error, DecodeError::kMalformed);
    EXPECT_EQ(decodeLine(line.substr(0, line.size() - 1)).error, DecodeError::kMalformed);

    // Well formed but out of range: build the body by hand with a valid CRC.
    auto withCrc = [](const std::string& body) {
        char crc[8];
        std::snprintf(crc, sizeof(crc), "*%04X", static_cast<unsigned>(crc16(body)));
        return body + crc;
    };
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2 0 1 2 3 4 5 6 7 0")).error, DecodeError::kNone);
    EXPECT_EQ(decodeLine(withCrc("SGP1 00000000 0 1 2 3 4 5 6 7 0")).error,
              DecodeError::kOutOfRange); // session 0
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2 0 1 2 3 4 -18000 6 7 0")).error,
              DecodeError::kOutOfRange);
    EXPECT_EQ(decodeLine(withCrc("SGP1 5a3c19e2 0 1 2 3 4 5 6 7 0")).error,
              DecodeError::kMalformed); // lowercase hex
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2 0 1 2 3 4 5 6 7")).error,
              DecodeError::kMalformed); // 9 fields
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2  0 1 2 3 4 5 6 7 0")).error,
              DecodeError::kMalformed); // double space
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2 65536 1 2 3 4 5 6 7 0")).error,
              DecodeError::kMalformed); // epoch overflow
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2 0 1 2 +3 4 5 6 7 0")).error,
              DecodeError::kMalformed);
    EXPECT_EQ(decodeLine(withCrc("SGP1 5A3C19E2 0 1 2 99999999999 4 5 6 7 0")).error,
              DecodeError::kMalformed);
}

TEST(LineScanner, FindsLinesAmidConsoleTextAndAcrossChunks) {
    const std::string a     = encode(sample());
    PoseRecord        r2    = sample();
    r2.seq                  = 43;
    const std::string b     = encode(r2);
    const std::string input = "hello world\r\n" + a + "printf noise SGP" + b + "tail";
    LineScanner       scanner;
    // One byte at a time, then all at once: same result.
    std::vector<std::string> bytewise;
    for (char c : input) {
        const uint8_t byte = static_cast<uint8_t>(c);
        scanner.feed(&byte, 1, [&](std::string_view l) { bytewise.emplace_back(l); });
    }
    ASSERT_EQ(bytewise.size(), 2u);
    EXPECT_EQ(decodeLine(bytewise[0]).record, sample());
    EXPECT_EQ(decodeLine(bytewise[1]).record, r2);

    LineScanner all;
    EXPECT_EQ(scanAll(all, input), bytewise);
    EXPECT_EQ(all.stats().lines, 2u);
}

TEST(LineScanner, ReadsThroughProsCobsPackets) {
    const std::string line = encode(sample());
    const std::string wire = cobsPacket("sout", "booting\n") + cobsPacket("sout", line) +
                             cobsPacket("serr", "warning\n") + cobsPacket("sout", line);
    LineScanner       scanner;
    const auto        lines = scanAll(scanner, wire);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(decodeLine(lines[0]).error, DecodeError::kNone);
    EXPECT_EQ(decodeLine(lines[1]).record, sample());
}

TEST(LineScanner, DropsTruncatedOversizedAndGarbageLines) {
    const std::string good = encode(sample());
    LineScanner       scanner;
    // Cut by a COBS terminator, cut by a new marker, a control byte, too long.
    const std::string input = good.substr(0, 30) + std::string(1, '\0') + good.substr(0, 25) +
                              good + "SGP1 12\x01" + "34\n" + "SGP1 " +
                              std::string(kMaxLineLength, '7') + "\n" + good;
    const auto        lines = scanAll(scanner, input);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(decodeLine(lines[0]).record, sample());
    EXPECT_EQ(decodeLine(lines[1]).record, sample());
    EXPECT_EQ(scanner.stats().truncated, 2u);
    EXPECT_EQ(scanner.stats().garbage, 1u);
    EXPECT_EQ(scanner.stats().too_long, 1u);
}

TEST(LineScanner, ResetForgetsAPartialLine) {
    const std::string good = encode(sample());
    LineScanner       scanner;
    EXPECT_TRUE(scanAll(scanner, good.substr(0, 40)).empty());
    scanner.reset();
    EXPECT_TRUE(scanAll(scanner, good.substr(40)).empty());
    EXPECT_EQ(scanAll(scanner, good).size(), 1u);
}

TEST(Units, RoundingAndWrapping) {
    EXPECT_EQ(toMillimeters(1.2346), 1235);
    EXPECT_EQ(toMillimeters(-1.2346), -1235);
    EXPECT_EQ(toMillimeters(-0.0004), 0);
    EXPECT_EQ(toMillimeters(1e9), kMaxCoordinateMm + 1);
    EXPECT_EQ(toMillimeters(-1e9), -kMaxCoordinateMm - 1);
    EXPECT_EQ(toCentidegrees(kPi / 2), 9000);
    EXPECT_EQ(toCentidegrees(-kPi), 18000); // (-180, 180]
    EXPECT_EQ(toCentidegrees(kPi), 18000);
    EXPECT_EQ(toCentidegrees(3 * kPi / 2), -9000);
    EXPECT_EQ(toCentidegrees(-5 * kPi / 2), -9000);
    EXPECT_EQ(toCentidegrees(1e12), toCentidegrees(std::remainder(1e12, 2 * kPi)));
    EXPECT_NEAR(toRadians(toCentidegrees(0.123)), 0.123, kPi / 36000);
    EXPECT_DOUBLE_EQ(toMeters(-1500), -1.5);
}
