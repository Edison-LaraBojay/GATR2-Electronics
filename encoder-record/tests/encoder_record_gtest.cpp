// encoder_record_gtest.cpp

#include "encoder_record.h"

#include <gtest/gtest.h>

#include <algorithm>

#include <string>
#include <vector>

namespace encoder_record
{
namespace
{

Record sample(uint16_t seq = 7) {
    Record r;
    r.run_id   = 0xBEEF;
    r.seq      = seq;
    r.channel0 = -123456;
    r.channel1 = 2147483647;
    return r;
}

std::vector<Record> scanRaw(RecordScanner& scanner, const std::vector<uint8_t>& bytes) {
    std::vector<Record> out;
    for (uint8_t b : bytes) {
        if (scanner.push(b)) {
            out.push_back(scanner.record());
        }
    }
    return out;
}

std::vector<Record> scanText(LineScanner& scanner, const std::string& text) {
    std::vector<Record> out;
    for (char c : text) {
        if (scanner.push(c)) {
            out.push_back(scanner.record());
        }
    }
    return out;
}

std::vector<uint8_t> raw(const Record& r) {
    const RecordBytes b = encode(r);
    return {b.begin(), b.end()};
}

std::string text(const Record& r) {
    const Line line = encodeLine(encode(r));
    return {line.begin(), line.end()};
}

void append(std::vector<uint8_t>& to, const std::vector<uint8_t>& from) {
    to.insert(to.end(), from.begin(), from.end());
}

TEST(EncoderRecord, CrcMatchesCcittFalseCheckValue) {
    const char* check = "123456789";
    EXPECT_EQ(crc16(reinterpret_cast<const uint8_t*>(check), 9), 0x29B1);
}

TEST(EncoderRecord, LayoutIsLittleEndianAtFixedOffsets) {
    Record r;
    r.run_id   = 0x1234;
    r.seq      = 0xABCD;
    r.channel0 = 0x01020304;
    r.channel1 = -2; // 0xFFFFFFFE
    const RecordBytes b = encode(r);
    EXPECT_EQ(b[0], 0xE7);
    EXPECT_EQ(b[1], 0x5A);
    EXPECT_EQ(b[2], 0x34);
    EXPECT_EQ(b[3], 0x12);
    EXPECT_EQ(b[4], 0xCD);
    EXPECT_EQ(b[5], 0xAB);
    EXPECT_EQ(b[6], 0x04);
    EXPECT_EQ(b[9], 0x01);
    EXPECT_EQ(b[10], 0xFE);
    EXPECT_EQ(b[13], 0xFF);
    const uint16_t crc = crc16(b.data(), 14);
    EXPECT_EQ(b[14], crc & 0xFF);
    EXPECT_EQ(b[15], crc >> 8);
}

TEST(EncoderRecord, RoundTrip) {
    Record out;
    ASSERT_TRUE(decode(encode(sample()).data(), out));
    EXPECT_EQ(out.run_id, 0xBEEF);
    EXPECT_EQ(out.seq, 7);
    EXPECT_EQ(out.channel0, -123456);
    EXPECT_EQ(out.channel1, 2147483647);
}

TEST(EncoderRecord, DecodeRejectsEveryFlippedBit) {
    const RecordBytes good = encode(sample());
    for (std::size_t i = 0; i < kRecordSize; ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            RecordBytes bad = good;
            bad[i] ^= static_cast<uint8_t>(1 << bit);
            Record out;
            EXPECT_FALSE(decode(bad.data(), out)) << "byte " << i << " bit " << bit;
        }
    }
}

TEST(RecordScanner, StartsMidRecord) {
    std::vector<uint8_t> stream = raw(sample(1));
    stream.erase(stream.begin(), stream.begin() + 5);
    append(stream, raw(sample(2)));
    RecordScanner scanner;
    const auto got = scanRaw(scanner, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].seq, 2);
    EXPECT_EQ(scanner.stats().skipped, 11u);
}

TEST(RecordScanner, ManyRecordsOneReadAndPartialReads) {
    std::vector<uint8_t> stream;
    for (uint16_t s = 0; s < 5; ++s) {
        append(stream, raw(sample(s)));
    }
    RecordScanner scanner;
    std::vector<Record> got;
    // Odd sized reads cut records anywhere.
    for (std::size_t at = 0; at < stream.size(); at += 7) {
        const std::vector<uint8_t> part(stream.begin() + at,
                                        stream.begin() + std::min(at + 7, stream.size()));
        const auto some = scanRaw(scanner, part);
        got.insert(got.end(), some.begin(), some.end());
    }
    ASSERT_EQ(got.size(), 5u);
    for (uint16_t s = 0; s < 5; ++s) {
        EXPECT_EQ(got[s].seq, s);
    }
    EXPECT_EQ(scanner.stats().skipped, 0u);
}

TEST(RecordScanner, RecoversAfterCorruptionAndFakeMagic) {
    std::vector<uint8_t> stream = raw(sample(1));
    stream[8] ^= 0x40; // corrupt record 1
    append(stream, {0xE7, 0x5A, 0x00, 0xE7}); // garbage with a fake magic
    append(stream, raw(sample(2)));
    std::vector<uint8_t> cut = raw(sample(3));
    cut.resize(9); // record 3 loses its tail
    append(stream, cut);
    append(stream, raw(sample(4)));
    RecordScanner scanner;
    const auto got = scanRaw(scanner, stream);
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].seq, 2);
    EXPECT_EQ(got[1].seq, 4);
}

TEST(RecordScanner, ResetForgetsPartialRecord) {
    std::vector<uint8_t> first = raw(sample(1));
    first.resize(10);
    RecordScanner scanner;
    scanRaw(scanner, first);
    scanner.reset();
    const auto got = scanRaw(scanner, raw(sample(2)));
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].seq, 2);
}

TEST(Line, EncodesMarkerHexNewline) {
    Record r;
    const std::string line = text(r);
    ASSERT_EQ(line.size(), kLineSize);
    EXPECT_EQ(line.substr(0, 5), "ENC1:");
    EXPECT_EQ(line.substr(5, 4), "E75A");
    EXPECT_EQ(line.back(), '\n');
    // Never a 'p': the PROS serial daemon command prefix.
    EXPECT_EQ(line.find('p'), std::string::npos);
}

TEST(LineScanner, DecodesLinesAmidConsoleText) {
    const std::string stream = "hello\n" + text(sample(1)) + "junk ENC1:" +
                               text(sample(2)).substr(5, 32) + "\r\n" + text(sample(3));
    LineScanner scanner;
    const auto got = scanText(scanner, stream);
    ASSERT_EQ(got.size(), 3u);
    EXPECT_EQ(got[0].seq, 1);
    EXPECT_EQ(got[1].seq, 2);
    EXPECT_EQ(got[2].seq, 3);
    EXPECT_EQ(scanner.stats().skipped, 1u); // "hello"
}

TEST(LineScanner, RejectsBadLines) {
    std::string bad_crc = text(sample(1));
    bad_crc[10]         = bad_crc[10] == '0' ? '1' : '0';
    std::string lower   = text(sample(2));
    for (char& c : lower) {
        if (c >= 'A' && c <= 'F') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    const std::string shorter = text(sample(3)).substr(0, kLineSize - 3) + "\n";
    const std::string longer(200, 'A');
    const std::string stream = bad_crc + lower + shorter + longer + "\n" + text(sample(4));
    LineScanner scanner;
    const auto got = scanText(scanner, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].seq, 4);
    EXPECT_EQ(scanner.stats().skipped, 4u);
}

TEST(LineScanner, ResetDropsPartialLine) {
    LineScanner scanner;
    scanText(scanner, text(sample(1)).substr(0, 20));
    scanner.reset();
    const auto got = scanText(scanner, text(sample(2)));
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].seq, 2);
}

} // namespace
} // namespace encoder_record
