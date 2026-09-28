// usb_line_gtest.cpp
// NG1 line codec: known lines, the Pi pros_usb_link parsing rules, and
// round trips of real brain link frames.

#include "communigatr/usb_line.h"

#include <gtest/gtest.h>
#include <string>
#include <vector>

#include "common/frame_codec.h"

using namespace communigatr;

namespace
{

// Every frame carried by the lines in text.
std::vector<std::vector<uint8_t>> decodeAll(UsbLineDecoder& decoder, const std::string& text) {
    std::vector<std::vector<uint8_t>> frames;
    for (char c : text) {
        if (decoder.push(c)) {
            frames.emplace_back(decoder.bytes(), decoder.bytes() + decoder.length());
        }
    }
    return frames;
}

std::vector<std::vector<uint8_t>> decodeAll(const std::string& text) {
    UsbLineDecoder decoder;
    return decodeAll(decoder, text);
}

std::string encode(const std::vector<uint8_t>& frame) {
    char              line[kUsbLineMax];
    const std::size_t n = encodeUsbLine(frame.data(), frame.size(), line, sizeof(line));
    return std::string(line, n);
}

} // namespace

TEST(UsbLine, EncodesUppercaseHexWithMarkerAndNewline) {
    const std::vector<uint8_t> frame = {0xAA, 0x55, 0x10, 0x0A, 0x00, 0xFF, 0x3C};
    EXPECT_EQ(encode(frame), "NG1:AA55100A00FF3C\n");
}

TEST(UsbLine, EncoderRefusesEmptyOversizedAndShortBuffer) {
    uint8_t frame[gatr2::kMaxFrameLen + 1] = {};
    char    line[kUsbLineMax + 2];
    EXPECT_EQ(encodeUsbLine(frame, 0, line, sizeof(line)), 0u);
    EXPECT_EQ(encodeUsbLine(frame, gatr2::kMaxFrameLen + 1, line, sizeof(line)), 0u);
    EXPECT_EQ(encodeUsbLine(frame, gatr2::kMaxFrameLen, line, kUsbLineMax - 1), 0u);
    EXPECT_EQ(encodeUsbLine(frame, gatr2::kMaxFrameLen, line, kUsbLineMax), kUsbLineMax);
    EXPECT_EQ(kUsbLineMax, 4u + 256u + 1u);
}

TEST(UsbLine, DecodesPlainLineAndStripsCarriageReturn) {
    auto frames = decodeAll("NG1:0102FE\nNG1:A0B1\r\n");
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], (std::vector<uint8_t>{0x01, 0x02, 0xFE}));
    EXPECT_EQ(frames[1], (std::vector<uint8_t>{0xA0, 0xB1}));
}

TEST(UsbLine, TextBeforeMarkerIsIgnoredAndLastMarkerCounts) {
    UsbLineDecoder decoder;
    auto           frames = decodeAll(decoder, "kernel: hello NG1:0102\n"
                                                         "NG1:FFNG1:0A0B\n"
                                                         "no marker here\n");
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], (std::vector<uint8_t>{0x01, 0x02}));
    EXPECT_EQ(frames[1], (std::vector<uint8_t>{0x0A, 0x0B})); // as the Pi's rfind
    EXPECT_EQ(decoder.stats().frames, 2u);
    EXPECT_EQ(decoder.stats().ignored, 1u);
    EXPECT_EQ(decoder.stats().dropped, 0u);
}

TEST(UsbLine, BadDigitsDropTheWholeLine) {
    UsbLineDecoder decoder;
    const auto     frames = decodeAll(decoder, "NG1:0a0B\n"    // lowercase
                                               "NG1:012\n"     // odd
                                               "NG1:\n"        // empty
                                               "NG1:01 02\n"   // space
                                               "NG1:0G\n"      // not hex
                                               "NG1:0102 \n"); // trailing space
    EXPECT_TRUE(frames.empty());
    EXPECT_EQ(decoder.stats().dropped, 6u);
    EXPECT_EQ(decoder.stats().frames, 0u);
}

TEST(UsbLine, DigitLimitIs256) {
    const std::string full(256, 'A');
    const std::string over(258, 'A');
    UsbLineDecoder    decoder;
    auto              frames = decodeAll(decoder, "NG1:" + full + "\nNG1:" + over + "\n");
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].size(), 128u);
    EXPECT_EQ(decoder.stats().dropped, 1u);
}

TEST(UsbLine, OverlongLineIsDroppedWholeAndNextLineDecodes) {
    // A diagnostic prefix is allowed up to the line limit.
    const std::string prefix(kUsbLineLimit - 8, '.');
    UsbLineDecoder    decoder;
    auto              frames = decodeAll(decoder, prefix + "NG1:0102\n");
    ASSERT_EQ(frames.size(), 1u);

    const std::string too_long(kUsbLineLimit - 7, '.');
    frames = decodeAll(decoder, too_long + "NG1:0102\nNG1:0304\n");
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], (std::vector<uint8_t>{0x03, 0x04}));
    EXPECT_EQ(decoder.stats().dropped, 1u);
    EXPECT_EQ(kUsbLineLimit, 4u + 256u + 256u); // pros_usb_link kLineLimit
}

TEST(UsbLine, ResetForgetsPartialLine) {
    UsbLineDecoder decoder;
    EXPECT_TRUE(decodeAll(decoder, "NG1:01").empty());
    decoder.reset();
    const auto frames = decodeAll(decoder, "02\nNG1:05\n");
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], (std::vector<uint8_t>{0x05}));
    EXPECT_EQ(decoder.stats().ignored, 1u);
}

TEST(UsbLine, RoundTripsLinkFramesOfEverySize) {
    UsbLineDecoder decoder;
    std::string    stream;
    std::vector<std::vector<uint8_t>> sent;
    // Largest reply: READ_DOC with 96 data bytes, a 128 byte frame.
    for (uint8_t n = 1; n <= gatr2::kDocChunkMax; n = static_cast<uint8_t>(n + 19)) {
        gatr2::BrainReply reply;
        reply.op            = gatr2::kOpReadDoc;
        reply.session       = 0x01020304;
        reply.request_id    = n;
        reply.pi_instance   = 0xA0B0C0D0;
        reply.doc_kind      = gatr2::kDocFieldMap;
        reply.doc_id        = 0x0A55AA55;
        reply.doc_total_len = 500;
        reply.data_len      = n;
        for (uint8_t i = 0; i < n; ++i) {
            reply.data[i] = static_cast<uint8_t>(i * 37 + 0xAA); // sync bytes inside data
        }
        std::vector<uint8_t> frame(gatr2::kMaxFrameLen);
        frame.resize(gatr2::encodeBrainReply(reply, frame.data(), gatr2::kMaxFrameLen));
        ASSERT_GT(frame.size(), 0u);
        sent.push_back(frame);
        stream += "PROS diag\n" + encode(frame);
    }
    gatr2::BrainReply full;
    full.op       = gatr2::kOpReadDoc;
    full.data_len = gatr2::kDocChunkMax;
    full.doc_kind = gatr2::kDocFieldEstimate;
    std::vector<uint8_t> frame(gatr2::kMaxFrameLen);
    frame.resize(gatr2::encodeBrainReply(full, frame.data(), gatr2::kMaxFrameLen));
    ASSERT_EQ(frame.size(), gatr2::kMaxFrameLen);
    sent.push_back(frame);
    stream += encode(frame);

    const auto frames = decodeAll(decoder, stream);
    ASSERT_EQ(frames.size(), sent.size());
    for (std::size_t i = 0; i < sent.size(); ++i) {
        EXPECT_EQ(frames[i], sent[i]);
        gatr2::BrainReply decoded;
        EXPECT_TRUE(gatr2::decodeBrainReply(frames[i].data(),
                                            static_cast<uint16_t>(frames[i].size()), decoded));
    }
}
