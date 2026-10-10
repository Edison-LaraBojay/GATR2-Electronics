// pose_feed_gtest.cpp
// End to end on the host: Brain stdout bytes (raw lines, PROS COBS packets, controller USER_READ
// replies through the fake controller) -> LineScanner -> tracker -> browser JSON. Fragmentation,
// garbage, truncation at a disconnect and reconnect.

#include "receiver/feed/pose_feed.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "receiver/app/live_feed.h"
#include "receiver/feed/feed_json.h"
#include "receiver/sources/fakes/brain_stdio.h"
#include "receiver/sources/fakes/fake_controller.h"
#include "receiver/sources/fakes/fake_port.h"
#include "receiver/sources/serial_source.h"

namespace spectagatr::receiver::feed
{
namespace
{

using sources::Bytes;
using sources::fakes::BrainStdioOptions;
using sources::fakes::bytesOf;
using sources::fakes::FakeBrainStdio;
using Poses = std::vector<TrackedPose>;

Bytes joined(std::initializer_list<Bytes> parts) {
    Bytes all;
    for (const Bytes& part : parts) {
        all.insert(all.end(), part.begin(), part.end());
    }
    return all;
}

Poses pushAll(PoseFeed& feed, const Bytes& bytes, RxMillis rx_ms = 0) {
    Poses poses;
    feed.push(bytes.data(), bytes.size(), rx_ms,
              [&poses](const TrackedPose& pose) { poses.push_back(pose); });
    return poses;
}

protocol::PoseRecord sampleRecord() {
    protocol::PoseRecord r;
    r.session    = 0x5A3C19E2;
    r.epoch      = 3;
    r.seq        = 42;
    r.t_ms       = 123456;
    r.x_mm       = 1500;
    r.y_mm       = -250;
    r.yaw_cdeg   = 9000;
    r.roll_cdeg  = -150;
    r.pitch_cdeg = 75;
    r.flags      = protocol::kFullAttitude;
    return r;
}

TEST(PoseFeed, RawLineToBrowserJson) {
    FakeBrainStdio brain(1);
    PoseFeed       feed;
    const Poses    poses = pushAll(feed, brain.write(sampleRecord()), 98765.4);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poseJson(poses[0], "controller"),
              "{\"type\":\"pose\",\"session\":\"5A3C19E2\",\"epoch\":3,\"seq\":42,"
              "\"t_ms\":123456,\"x\":1.5,\"y\":-0.25,\"yaw\":1.570796,\"roll\":-0.02618,"
              "\"pitch\":0.01309,\"attitude\":true,\"restart\":true,\"rx_ms\":98765.4,"
              "\"source\":\"controller\"}");
}

TEST(PoseFeed, ConsoleTextBetweenAndBeforeLinesIsSkipped) {
    FakeBrainStdio brain(7);
    PoseFeed       feed;
    const Bytes    stream =
        joined({bytesOf("hello from printf\nmotor 3 hot: "), brain.pose(1, 2, 3, 4),
                bytesOf("odom: 1.0 2.0\n"), brain.pose(5, 6, 7, 8)});
    const Poses poses = pushAll(feed, stream);
    ASSERT_EQ(poses.size(), 2u);
    EXPECT_EQ(poses[1].record.x_mm, 5);
    EXPECT_EQ(feed.counts().poses, 2u);
    EXPECT_EQ(feed.counts().malformed + feed.counts().bad_crc + feed.counts().garbage, 0u);
    EXPECT_EQ(feed.counts().bytes, stream.size());
}

TEST(PoseFeed, ProsCobsPacketsDecodeWithoutACobsDecoder) {
    BrainStdioOptions options;
    options.cobs = true;
    FakeBrainStdio brain(0xCAFE, options);
    PoseFeed       feed;
    const Bytes    stream = joined({brain.pose(100, 200, -9000, 10), brain.consoleText("text\n"),
                                    brain.pose(110, 210, -8900, 20)});
    const Poses    poses  = pushAll(feed, stream);
    ASSERT_EQ(poses.size(), 2u);
    EXPECT_EQ(poses[0].record.yaw_cdeg, -9000);
    EXPECT_EQ(poses[1].record.seq, 1u);
    EXPECT_EQ(feed.counts().truncated, 0u);
}

TEST(PoseFeed, ByteByByteFragmentsGiveTheSamePoses) {
    BrainStdioOptions options;
    options.cobs = true;
    FakeBrainStdio brain(3, options);
    Bytes          stream;
    for (int i = 0; i < 20; ++i) {
        stream = joined({stream, brain.pose(i, -i, i * 100, static_cast<uint32_t>(i))});
    }
    PoseFeed whole;
    PoseFeed pieces;
    Poses    from_pieces;
    for (const sources::Byte byte : stream) {
        const Poses got = pushAll(pieces, Bytes{byte});
        from_pieces.insert(from_pieces.end(), got.begin(), got.end());
    }
    const Poses from_whole = pushAll(whole, stream);
    ASSERT_EQ(from_whole.size(), 20u);
    ASSERT_EQ(from_pieces.size(), 20u);
    for (std::size_t i = 0; i < 20; ++i) {
        EXPECT_EQ(from_pieces[i].record, from_whole[i].record);
    }
}

TEST(PoseFeed, GarbageAndCorruptLinesAreCountedAndLaterLinesSurvive) {
    FakeBrainStdio brain(9);
    PoseFeed       feed;
    Bytes          corrupt = brain.pose(1, 1, 1, 1);
    corrupt[10] ^= 0x01; // a digit of the session changes: crc mismatch
    Bytes control = brain.pose(2, 2, 2, 2);
    control[8]    = 0x07; // a control byte inside the line
    Bytes noise;
    for (int i = 0; i < 256; ++i) {
        noise.push_back(static_cast<sources::Byte>((i * 37) & 0xFF));
    }
    const Bytes stream = joined(
        {noise, corrupt, control, bytesOf("SGP1 not a pose*0000\n"), brain.pose(3, 3, 3, 3)});
    const Poses poses = pushAll(feed, stream);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].record.x_mm, 3);
    EXPECT_EQ(feed.counts().bad_crc, 2u); // the flipped digit, and the fake line's crc
    EXPECT_GE(feed.counts().garbage, 1u);
}

TEST(PoseFeed, OversizedLineIsDroppedAsTooLong) {
    FakeBrainStdio brain(9);
    PoseFeed       feed;
    const Bytes    stream =
        joined({bytesOf("SGP1 " + std::string(200, '1') + "\n"), brain.pose(4, 4, 4, 4)});
    const Poses poses = pushAll(feed, stream);
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(feed.counts().too_long, 1u);
}

TEST(PoseFeed, ALineCutByANulOrANewMarkerIsTruncated) {
    BrainStdioOptions options;
    options.cobs = true;
    FakeBrainStdio brain(4, options);
    Bytes          cut = brain.pose(1, 1, 1, 1);
    cut.resize(cut.size() / 2);
    cut.push_back(0x00); // the COBS packet ended inside the line
    Bytes raw = FakeBrainStdio(4).pose(9, 9, 9, 9);
    raw.resize(raw.size() / 2); // and a raw line cut by the next marker
    PoseFeed    feed;
    const Poses poses = pushAll(feed, joined({cut, raw, brain.pose(2, 2, 2, 2)}));
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0].record.x_mm, 2);
    EXPECT_EQ(feed.counts().truncated, 2u);
}

TEST(PoseFeed, ControllerUserReadRepliesCarryTheLines) {
    sources::fakes::FakeClock      clock;
    sources::fakes::FakeController controller(clock);
    BrainStdioOptions              options;
    options.cobs = true;
    FakeBrainStdio brain(0x77, options);
    for (int i = 0; i < 10; ++i) {
        controller.brainWrites(brain.consoleText("tick\n"));
        controller.brainWrites(brain.pose(i * 10, 0, 0, static_cast<uint32_t>(i)));
    }
    platform::StopSignal stop;
    controller.onIdle([&stop] { stop.request(); });
    PoseFeed             feed;
    Poses                poses;
    sources::SourceHooks hooks;
    hooks.open_port = [&controller](std::string&) -> std::optional<sources::Port> {
        return controller.port();
    };
    hooks.clock    = clock.clock();
    hooks.on_chunk = [&](const sources::SourceChunk& chunk) {
        EXPECT_LE(chunk.bytes.size(), 64u);
        const Poses got = pushAll(feed, chunk.bytes, static_cast<double>(chunk.rx_us) / 1000.0);
        poses.insert(poses.end(), got.begin(), got.end());
    };
    hooks.on_status = [](const sources::SourceStatus&) {};
    sources::SerialSourceConfig config;
    config.reconnect_wait = sources::Milliseconds(0);
    sources::runSerialSource(config, hooks, stop);
    ASSERT_EQ(poses.size(), 10u);
    EXPECT_EQ(poses[9].record.x_mm, 90);
    EXPECT_EQ(feed.counts().truncated + feed.counts().bad_crc, 0u);
    EXPECT_GT(poses[9].rx_ms, poses[0].rx_ms); // stamped on the receiver clock as they came
}

// Through the live hub: a line cut by an unplug is not continued by the next port's bytes, and
// the same publisher run carries on without a restart.
TEST(PoseFeed, TruncationAtDisconnectThenReconnect) {
    FakeBrainStdio brain(0x55);
    Bytes          first = joined({brain.pose(1, 0, 0, 1), brain.pose(2, 0, 0, 2)});
    Bytes          half  = brain.pose(3, 0, 0, 3);
    half.resize(half.size() - 9);
    first            = joined({first, half});
    const Bytes rest = joined(
        {bytesOf("console after reconnect\n"), brain.pose(4, 0, 0, 4), brain.pose(5, 0, 0, 5)});

    sources::fakes::FakeClock    clock;
    sources::fakes::ScriptedPort before(
        clock, {sources::fakes::Arrive{1000, first}, sources::fakes::Unplug{1000, "unplugged"}});
    sources::fakes::ScriptedPort after(clock, {sources::fakes::Arrive{1000, rest}});
    platform::StopSignal         stop;
    after.onEnd([&stop] { stop.request(); });

    server::HttpServer     server(server::HttpServerConfig{}, nullptr); // never started: no clients
    app::LiveFeed          live(server, app::FeedSettings{});
    std::vector<LinkState> links;
    int                    opens = 0;
    sources::SourceHooks   hooks;
    hooks.open_port = [&](std::string&) -> std::optional<sources::Port> {
        return ++opens == 1 ? before.port() : after.port();
    };
    hooks.clock     = clock.clock();
    hooks.on_chunk  = [&live](const sources::SourceChunk& chunk) { live.onChunk(chunk); };
    hooks.on_status = [&](const sources::SourceStatus& status) {
        live.onStatus(status);
        links.push_back(live.status(0).link);
    };
    sources::SerialSourceConfig config;
    config.reconnect_wait = sources::Milliseconds(0);
    sources::runSerialSource(config, hooks, stop);

    const FeedStatus status = live.status(static_cast<double>(clock.now()) / 1000.0);
    EXPECT_EQ(status.counts.poses, 4u);
    EXPECT_EQ(status.counts.sessions, 1u); // same run before and after: no restart
    EXPECT_EQ(status.counts.bad_crc + status.counts.malformed, 0u);
    ASSERT_GE(links.size(), 3u);
    EXPECT_EQ(links[0], LinkState::kOpen);
    EXPECT_EQ(links[1], LinkState::kError);
    EXPECT_EQ(links[2], LinkState::kOpen);
}

TEST(PoseFeed, WithoutTheResetACutLineSwallowsTheNextText) {
    FakeBrainStdio brain(0x55);
    Bytes          half = brain.pose(3, 0, 0, 3);
    half.resize(half.size() - 9);
    PoseFeed feed;
    (void)pushAll(feed, half);
    (void)pushAll(feed, bytesOf("console after reconnect\n"));
    EXPECT_EQ(feed.counts().bad_crc + feed.counts().malformed, 1u); // what resetTransport avoids

    PoseFeed reset;
    (void)pushAll(reset, half);
    reset.resetTransport();
    (void)pushAll(reset, bytesOf("console after reconnect\n"));
    EXPECT_EQ(reset.counts().bad_crc + reset.counts().malformed, 0u);
}

} // namespace
} // namespace spectagatr::receiver::feed
