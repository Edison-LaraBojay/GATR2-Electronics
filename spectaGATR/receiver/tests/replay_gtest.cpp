// replay_gtest.cpp
// The file source: a file of raw lines and COBS packets replayed in chunks reaches the feed
// unchanged; open while bytes flow, searching at the end; --loop rewinds and restarts ordering.

#include "receiver/sources/replay.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <vector>

#include "receiver/feed/pose_feed.h"
#include "receiver/sources/fakes/brain_stdio.h"

namespace spectagatr::receiver::sources
{
namespace
{

namespace fs = std::filesystem;

class ReplayFile : public ::testing::Test {
  protected:
    fs::path path;
    Bytes    bytes;

    void SetUp() override {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("spectagatr-replay-" + std::to_string(stamp) + ".bin");
        fakes::BrainStdioOptions cobs;
        cobs.cobs = true;
        const fakes::FakeBrainStdio raw(0x1111);
        const fakes::FakeBrainStdio packets(0x1111, cobs);
        for (int i = 0; i < 6; ++i) {
            protocol::PoseRecord record;
            record.session   = 0x1111;
            record.seq       = static_cast<protocol::Sequence>(i);
            record.x_mm      = i;
            const Bytes line = i % 2 == 0 ? raw.write(record) : packets.write(record);
            const Bytes text = raw.consoleText("console line\n");
            bytes.insert(bytes.end(), line.begin(), line.end());
            bytes.insert(bytes.end(), text.begin(), text.end());
        }
        std::ofstream(path, std::ios::binary)
            .write(reinterpret_cast<const char*>(bytes.data()),
                   static_cast<std::streamsize>(bytes.size()));
    }

    void TearDown() override {
        std::error_code error;
        fs::remove(path, error);
    }
};

struct Recorder {
    std::mutex                mutex;
    Bytes                     received;
    std::vector<SourceStatus> statuses;
};

SourceHooks hooksFor(Recorder& record, platform::StopSignal& stop, int stop_after_opens) {
    SourceHooks hooks;
    hooks.clock = [] {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };
    hooks.on_chunk = [&record](const SourceChunk& chunk) {
        EXPECT_EQ(chunk.src, SourceKind::kFile);
        record.received.insert(record.received.end(), chunk.bytes.begin(), chunk.bytes.end());
    };
    hooks.on_status = [&record, &stop, stop_after_opens](const SourceStatus& status) {
        record.statuses.push_back(status);
        int opens = 0;
        for (const SourceStatus& seen : record.statuses) {
            opens += seen.state == PortState::kOpen ? 1 : 0;
        }
        if (status.state == PortState::kSearching || opens > stop_after_opens) {
            stop.request();
        }
    };
    return hooks;
}

TEST_F(ReplayFile, BytesArriveUnchangedAndTheEndIsSearching) {
    ReplayConfig config;
    config.path        = path.string();
    config.chunk_bytes = 64;
    config.interval    = Milliseconds(1);
    platform::StopSignal stop;
    Recorder             record;
    runReplaySource(config, hooksFor(record, stop, 1), stop);

    EXPECT_EQ(record.received, bytes);
    ASSERT_GE(record.statuses.size(), 3u);
    EXPECT_EQ(record.statuses[0].state, PortState::kOpen);
    EXPECT_EQ(record.statuses[0].src, SourceKind::kFile);
    EXPECT_EQ(record.statuses[0].port, path.string());
    EXPECT_EQ(record.statuses[1].state, PortState::kSearching);
    EXPECT_NE(record.statuses[1].detail.find("end of"), std::string::npos);
    EXPECT_EQ(record.statuses.back().state, PortState::kStopped);

    feed::PoseFeed feed;
    std::size_t    poses = 0;
    feed.push(record.received.data(), record.received.size(), 0.0,
              [&poses](const feed::TrackedPose&) { ++poses; });
    EXPECT_EQ(poses, 6u);
    EXPECT_EQ(feed.counts().bad_crc + feed.counts().truncated + feed.counts().malformed, 0u);
}

TEST_F(ReplayFile, LoopRewindsAndMarksTheStreamRestarted) {
    ReplayConfig config;
    config.path        = path.string();
    config.chunk_bytes = replayChunkBytes(1e9, Milliseconds(1)); // as fast as it goes
    config.interval    = Milliseconds(1);
    config.loop        = true;
    platform::StopSignal stop;
    Recorder             record;
    runReplaySource(config, hooksFor(record, stop, 1), stop);
    ASSERT_GE(record.statuses.size(), 2u);
    EXPECT_FALSE(record.statuses[0].stream_restarted);
    EXPECT_EQ(record.statuses[1].state, PortState::kOpen);
    EXPECT_TRUE(record.statuses[1].stream_restarted);
}

TEST(Replay, MissingFileIsSearchingWithTheReason) {
    ReplayConfig config;
    config.path = (fs::temp_directory_path() / "spectagatr-no-such-replay.bin").string();
    platform::StopSignal stop;
    Recorder             record;
    runReplaySource(config, hooksFor(record, stop, 0), stop);
    ASSERT_FALSE(record.statuses.empty());
    EXPECT_EQ(record.statuses[0].state, PortState::kSearching);
    EXPECT_NE(record.statuses[0].detail.find("cannot open"), std::string::npos);
}

// The pace follows elapsed time, not the number of waits: on Windows a 5 ms
// wait lasts about 15.6 ms, which once made replays about 3x too slow.
TEST(Replay, AverageRateFollowsTheRequestedRate) {
    const fs::path path = fs::temp_directory_path() / "spectagatr-replay-pace.bin";
    std::ofstream(path, std::ios::binary) << std::string(8000, 'x');
    ReplayConfig config;
    config.path        = path.string();
    config.chunk_bytes = replayChunkBytes(8000, config.interval); // 1 s for the file
    platform::StopSignal stop;
    Recorder             record;
    const auto           start = std::chrono::steady_clock::now();
    runReplaySource(config, hooksFor(record, stop, 1), stop);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::error_code error;
    fs::remove(path, error);
    EXPECT_EQ(record.received.size(), 8000u);
    EXPECT_GT(seconds, 0.6);
    EXPECT_LT(seconds, 1.8);
}

TEST(Replay, ChunkSizeFollowsTheRate) {
    EXPECT_EQ(replayChunkBytes(12800, Milliseconds(5)), 64u);
    EXPECT_EQ(replayChunkBytes(1000, Milliseconds(5)), 5u);
    EXPECT_EQ(replayChunkBytes(10, Milliseconds(5)), 1u); // never 0
}

} // namespace
} // namespace spectagatr::receiver::sources
