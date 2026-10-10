// fakes_gtest.cpp
// The test fakes against independent references: a golden PROS COBS packet (independent Python
// port of PROS cobs_encode), the protocol decoder and the CDC2 scanner.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "receiver/sources/cdc2.h"
#include "receiver/sources/fakes/brain_stdio.h"
#include "receiver/sources/fakes/fake_controller.h"
#include "receiver/sources/fakes/fake_port.h"
#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::receiver::sources::fakes
{
namespace
{

using Lines = std::vector<std::string>;

Bytes bytesOfHex(const std::string& hex) {
    Bytes bytes;
    for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
        bytes.push_back(static_cast<Byte>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
}

Lines scanAll(const Bytes& stream) {
    protocol::LineScanner scanner;
    Lines                 lines;
    scanner.feed(stream.data(), stream.size(),
                 [&lines](std::string_view line) { lines.emplace_back(line); });
    return lines;
}

TEST(Fakes, CobsPacketMatchesTheGoldenProsEncoding) {
    EXPECT_EQ(cobsPacket("hello from printf\n", "sout"),
              bytesOfHex("17736F757468656C6C6F2066726F6D207072696E74660A00"));
}

TEST(Fakes, BrainStdioLinesDecode) {
    FakeBrainStdio brain(0xA1B2C3D4);
    const Bytes    line = brain.pose(1200, -40, 9000, 1000);
    EXPECT_EQ(line.back(), '\n');
    const Lines lines = scanAll(line);
    ASSERT_EQ(lines.size(), 1u);
    const protocol::DecodeResult result = protocol::decodeLine(lines[0]);
    ASSERT_EQ(result.error, protocol::DecodeError::kNone);
    EXPECT_EQ(result.record.session, 0xA1B2C3D4u);
    EXPECT_EQ(result.record.seq, 0u);
    EXPECT_EQ(result.record.t_ms, 1000u);
    EXPECT_EQ(result.record.x_mm, 1200);
    EXPECT_EQ(result.record.y_mm, -40);
    EXPECT_EQ(result.record.yaw_cdeg, 9000);
    EXPECT_EQ(brain.nextSeq(), 1u);
}

TEST(Fakes, BrainStdioCobsLinesSurviveConsoleText) {
    BrainStdioOptions options;
    options.cobs = true;
    FakeBrainStdio brain(7, options);
    Bytes          stream = brain.pose(1, 2, 3, 4);
    EXPECT_EQ(stream.back(), 0x00);
    const Bytes text = brain.consoleText("printf output\n");
    stream.insert(stream.end(), text.begin(), text.end());
    const Bytes second = brain.pose(5, 6, 7, 8);
    stream.insert(stream.end(), second.begin(), second.end());
    EXPECT_EQ(scanAll(stream).size(), 2u);
}

TEST(Fakes, RestartsAndResetsMoveSessionEpochAndSequence) {
    FakeBrainStdio brain(1);
    (void)brain.pose(0, 0, 0, 0);
    brain.skip(4);
    EXPECT_EQ(brain.nextSeq(), 5u);
    brain.resetPose();
    EXPECT_EQ(brain.epoch(), 1);
    brain.restart(2);
    EXPECT_EQ(brain.session(), 2u);
    EXPECT_EQ(brain.epoch(), 0);
    EXPECT_EQ(brain.nextSeq(), 0u);
}

TEST(Fakes, InvalidRecordsWriteNothing) {
    FakeBrainStdio       brain(1);
    protocol::PoseRecord record;
    record.session = 0; // never valid
    EXPECT_TRUE(brain.write(record).empty());
}

TEST(Fakes, ControllerAnswersUserReadWithQueuedBrainBytes) {
    FakeClock      clock;
    FakeController controller(clock);
    controller.brainWrites(Bytes(100, 'q'));
    Port        port    = controller.port();
    const Bytes request = userReadRequest(kUserChannel, kUserReadLength);
    ASSERT_EQ(port.write(request.data(), request.size(), Milliseconds(10)).status, IoStatus::kOk);
    Bytes          buffer(256);
    const IoResult got = port.read(buffer.data(), buffer.size(), Milliseconds(100));
    ASSERT_EQ(got.status, IoStatus::kOk);
    EXPECT_EQ(clock.now(), 20000);
    CdcReplyScanner scanner;
    const CdcScan   scan = scanner.push(buffer.data(), got.bytes);
    ASSERT_EQ(scan.replies.size(), 1u);
    const UserRead read = parseUserRead(scan.replies.front());
    EXPECT_EQ(read.error, UserReadError::kNone);
    EXPECT_EQ(read.data, Bytes(64, 'q'));
    EXPECT_EQ(controller.pendingBrainBytes(), 36u);
}

TEST(Fakes, ControllerPadsRepliesToAWord) {
    FakeClock             clock;
    FakeControllerOptions options;
    options.pad_to_word = true;
    FakeController controller(clock, options);
    controller.brainWrites(Bytes(5, 'p'));
    Port        port    = controller.port();
    const Bytes request = userReadRequest(kUserChannel, kUserReadLength);
    (void)port.write(request.data(), request.size(), Milliseconds(10));
    Bytes           buffer(256);
    const IoResult  got = port.read(buffer.data(), buffer.size(), Milliseconds(100));
    CdcReplyScanner scanner;
    const CdcScan   scan = scanner.push(buffer.data(), got.bytes);
    ASSERT_EQ(scan.replies.size(), 1u);
    EXPECT_EQ(parseUserRead(scan.replies.front()).data, (Bytes{'p', 'p', 'p', 'p', 'p', 0, 0, 0}));
}

TEST(Fakes, ScriptedPortTimesOutInFakeTime) {
    FakeClock    clock;
    ScriptedPort port(clock, {Arrive{250000, Bytes{1, 2}}});
    Port         opened = port.port();
    Bytes        buffer(8);
    EXPECT_EQ(opened.read(buffer.data(), buffer.size(), Milliseconds(100)).status,
              IoStatus::kTimedOut);
    EXPECT_EQ(clock.now(), 100000);
    EXPECT_EQ(opened.read(buffer.data(), buffer.size(), Milliseconds(100)).status,
              IoStatus::kTimedOut);
    const IoResult got = opened.read(buffer.data(), buffer.size(), Milliseconds(100));
    EXPECT_EQ(got.status, IoStatus::kOk);
    EXPECT_EQ(got.bytes, 2u);
    EXPECT_EQ(clock.now(), 250000);
}

TEST(Fakes, WithNulsPutsANulAfterEachGroup) {
    EXPECT_EQ(withNuls(Bytes{1, 2, 3, 4, 5}, 2), (Bytes{1, 2, 0, 3, 4, 0, 5}));
}

} // namespace
} // namespace spectagatr::receiver::sources::fakes
