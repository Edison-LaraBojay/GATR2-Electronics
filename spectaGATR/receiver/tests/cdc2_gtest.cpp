// cdc2_gtest.cpp
// Request bytes as pros-cli computes them, reply scanning and parsing.

#include "receiver/sources/cdc2.h"

#include <gtest/gtest.h>

#include <string>

#include "receiver/sources/fakes/fake_controller.h"

namespace spectagatr::receiver::sources
{
namespace
{

using fakes::extendedReply;
using fakes::systemVersionReply;

Bytes bytesOf(const std::string& text) { return Bytes(text.begin(), text.end()); }

Bytes joined(Bytes first, const Bytes& second) {
    first.insert(first.end(), second.begin(), second.end());
    return first;
}

CdcScan scanAll(CdcReplyScanner& scanner, const Bytes& bytes) {
    return scanner.push(bytes.data(), bytes.size());
}

Bytes userReadReply(const Bytes& data) {
    return extendedReply(kExtendedUserRead, kAck, joined({kUserChannel}, data));
}

TEST(Cdc2, Crc16XmodemCheckValue) {
    const Bytes check = bytesOf("123456789");
    EXPECT_EQ(crc16Xmodem(check.data(), check.size()), 0x31C3);
}

TEST(Cdc2, RequestsMatchProsCliBytes) {
    EXPECT_EQ(systemVersionRequest(), (Bytes{0xC9, 0x36, 0xB8, 0x47, 0xA4}));
    EXPECT_EQ(userReadRequest(kUserChannel, kProsCliReadLength),
              (Bytes{0xC9, 0x36, 0xB8, 0x47, 0x56, 0x27, 0x02, 0x01, 0x40, 0x8C, 0x69}));
    EXPECT_EQ(userReadRequest(kUserChannel, 0x00),
              (Bytes{0xC9, 0x36, 0xB8, 0x47, 0x56, 0x27, 0x02, 0x01, 0x00, 0xC4, 0xAD}));
    EXPECT_EQ(radioStatusRequest(), (Bytes{0xC9, 0x36, 0xB8, 0x47, 0x56, 0x26, 0x00, 0xAC, 0x38}));
}

// Bytes from pros-cli ft_transfer_channel, CRC computed with pros-cli's own CRC class.
TEST(Cdc2, RadioChannelRequestsMatchProsCliBytes) {
    EXPECT_EQ(radioChannelRequest(true),
              (Bytes{0xC9, 0x36, 0xB8, 0x47, 0x56, 0x10, 0x02, 0x01, 0x01, 0xA9, 0x48}));
    EXPECT_EQ(radioChannelRequest(false),
              (Bytes{0xC9, 0x36, 0xB8, 0x47, 0x56, 0x10, 0x02, 0x01, 0x00, 0xB9, 0x69}));
}

TEST(Cdc2, ExtendedReplyHasZeroCrcResidue) {
    const Bytes reply = userReadReply(bytesOf("SGP1 00\n"));
    EXPECT_EQ(crc16Xmodem(reply.data(), reply.size()), 0);
}

TEST(Cdc2, ScannerJoinsAReplySplitAcrossReads) {
    CdcReplyScanner scanner;
    const Bytes     reply = userReadReply(bytesOf("hello"));
    CdcReplies      replies;
    for (const Byte byte : reply) {
        CdcScan scan = scanner.push(&byte, 1);
        replies.insert(replies.end(), scan.replies.begin(), scan.replies.end());
        EXPECT_EQ(scan.bad_replies, 0u);
        EXPECT_EQ(scan.stray_bytes, 0u);
    }
    ASSERT_EQ(replies.size(), 1u);
    const UserRead read = parseUserRead(replies.front());
    EXPECT_EQ(read.error, UserReadError::kNone);
    EXPECT_EQ(read.data, bytesOf("hello"));
}

TEST(Cdc2, ScannerResyncsAfterGarbageAndABadCrc) {
    CdcReplyScanner scanner;
    Bytes           corrupt = userReadReply(bytesOf("lost"));
    corrupt.back() ^= 0x5A;
    const Bytes   stream = joined(joined(joined(Bytes{0x01, 0x02, 0x03}, corrupt), Bytes{0x00}),
                                  userReadReply(bytesOf("kept")));
    const CdcScan scan   = scanAll(scanner, stream);
    ASSERT_EQ(scan.replies.size(), 1u);
    EXPECT_EQ(parseUserRead(scan.replies.front()).data, bytesOf("kept"));
    EXPECT_EQ(scan.bad_replies, 1u);
    EXPECT_GE(scan.stray_bytes, 3u);
}

TEST(Cdc2, ScannerSkipsRepliesNobodyAskedFor) {
    CdcReplyScanner scanner;
    const Bytes   stream = joined(Bytes{0xAA, 0x55, 0x99, 0xFF, 0x00}, systemVersionReply(0x11, 2));
    const CdcScan scan   = scanAll(scanner, stream);
    ASSERT_EQ(scan.replies.size(), 1u);
    EXPECT_EQ(scan.replies.front().command, kCommandSystemVersion);
    EXPECT_GT(scan.stray_bytes, 0u);
}

TEST(Cdc2, ScannerRefusesOversizeReplies) {
    CdcReplyScanner scanner;
    const Bytes stream = joined(Bytes{0xAA, 0x55, 0x56, 0xFF, 0xFF}, userReadReply(bytesOf("ok")));
    const CdcScan scan = scanAll(scanner, stream);
    ASSERT_EQ(scan.replies.size(), 1u);
    EXPECT_EQ(parseUserRead(scan.replies.front()).data, bytesOf("ok"));
    EXPECT_EQ(scan.bad_replies, 1u);
}

TEST(Cdc2, LongSizeRepliesUseTwoSizeBytes) {
    CdcReplyScanner scanner;
    const Bytes     data(200, 'x');
    const Bytes     reply = userReadReply(data);
    EXPECT_EQ(reply[3] & 0x80, 0x80);
    const CdcScan scan = scanAll(scanner, reply);
    ASSERT_EQ(scan.replies.size(), 1u);
    EXPECT_EQ(parseUserRead(scan.replies.front()).data, data);
}

TEST(Cdc2, SystemVersionGivesTheLink) {
    CdcReplyScanner scanner;
    const auto      linkOf = [&scanner](Byte product, Byte flags) {
        const CdcScan scan = scanAll(scanner, systemVersionReply(product, flags));
        EXPECT_EQ(scan.replies.size(), 1u);
        const std::optional<SystemVersion> version = parseSystemVersion(scan.replies.front());
        EXPECT_TRUE(version);
        return version ? controllerLink(*version) : LinkKind::kNone;
    };
    EXPECT_EQ(linkOf(kProductController, kLinkedByRadio), LinkKind::kRadio);
    EXPECT_EQ(linkOf(kProductController, kLinkedByCable), LinkKind::kCable);
    EXPECT_EQ(linkOf(kProductController, 0x00), LinkKind::kNone);
    EXPECT_EQ(linkOf(kProductBrain, kLinkedByRadio), LinkKind::kNone);
}

TEST(Cdc2, UserReadErrors) {
    CdcReply nack;
    nack.command = kCommandExtended;
    nack.payload = {kExtendedUserRead, 0xFF};
    EXPECT_EQ(parseUserRead(nack).error, UserReadError::kNack);

    CdcReply other = nack;
    other.payload  = {kExtendedRadioStatus, kAck, 0x01};
    EXPECT_EQ(parseUserRead(other).error, UserReadError::kNotUserRead);

    CdcReply cut = nack;
    cut.payload  = {kExtendedUserRead};
    EXPECT_EQ(parseUserRead(cut).error, UserReadError::kShort);

    CdcReply empty      = nack;
    empty.payload       = {kExtendedUserRead, kAck, kUserChannel};
    const UserRead read = parseUserRead(empty);
    EXPECT_EQ(read.error, UserReadError::kNone);
    EXPECT_TRUE(read.data.empty());
}

TEST(Cdc2, RadioStatusFields) {
    CdcReplyScanner scanner;
    const CdcScan   scan = scanAll(
        scanner, extendedReply(kExtendedRadioStatus, kAck, Bytes{4, 90, 0, 0xD0, 0xFF, 55, 1}));
    ASSERT_EQ(scan.replies.size(), 1u);
    const std::optional<RadioStatus> radio = parseRadioStatus(scan.replies.front());
    ASSERT_TRUE(radio);
    EXPECT_EQ(radio->device, 4);
    EXPECT_EQ(radio->quality, 90);
    EXPECT_EQ(radio->strength, -48);
    EXPECT_EQ(radio->channel, 55);
    EXPECT_EQ(radio->timeslot, 1);

    const CdcScan nacked = scanAll(scanner, extendedReply(kExtendedRadioStatus, 0xFF, Bytes{}));
    ASSERT_EQ(nacked.replies.size(), 1u);
    EXPECT_FALSE(parseRadioStatus(nacked.replies.front()));
}

TEST(Cdc2, EnumNamesAreComplete) {
    for (const LinkKind kind : {LinkKind::kNone, LinkKind::kCable, LinkKind::kRadio}) {
        EXPECT_STRNE(linkKindName(kind), "unknown");
    }
    for (const UserReadError error : {UserReadError::kNone, UserReadError::kNotUserRead,
                                      UserReadError::kNack, UserReadError::kShort}) {
        EXPECT_STRNE(userReadErrorName(error), "unknown");
    }
    for (const PortState state :
         {PortState::kSearching, PortState::kOpen, PortState::kNoBrainLinked, PortState::kClosed,
          PortState::kStopped}) {
        EXPECT_STRNE(portStateName(state), "unknown");
    }
    EXPECT_STREQ(sourceKindName(SourceKind::kNone), "none");
    EXPECT_STREQ(sourceKindName(SourceKind::kBrainUsb), "brain");
    EXPECT_STREQ(sourceKindName(SourceKind::kController), "controller");
    EXPECT_STREQ(sourceKindName(SourceKind::kSim), "sim");
    EXPECT_STREQ(sourceKindName(SourceKind::kFile), "file");
}

} // namespace
} // namespace spectagatr::receiver::sources
