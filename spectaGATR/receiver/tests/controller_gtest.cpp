// controller_gtest.cpp
// The controller poll loop against a fake controller, in fake time.

#include "receiver/sources/controller.h"

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

#include "receiver/feed/pose_feed.h"
#include "receiver/sources/fakes/brain_stdio.h"
#include "receiver/sources/fakes/fake_controller.h"
#include "receiver/sources/serial_source.h"

namespace spectagatr::receiver::sources
{
namespace
{

using fakes::FakeClock;
using fakes::FakeController;
using fakes::FakeControllerOptions;
using Statuses      = std::vector<SourceStatus>;
using StatusWatcher = std::function<void(const SourceStatus&)>;

Bytes patternBytes(std::size_t count) {
    Bytes bytes(count);
    for (std::size_t i = 0; i < count; ++i) {
        bytes[i] = static_cast<Byte>('A' + i % 26);
    }
    return bytes;
}

ControllerConfig quickConfig() { return ControllerConfig{}; }

void runController(const ControllerConfig& controller, const SourceHooks& hooks,
                   platform::StopSignal& stop) {
    SerialSourceConfig config;
    config.controller     = controller;
    config.reconnect_wait = Milliseconds(0);
    runSerialSource(config, hooks, stop);
}

// One fake controller behind the source; the loop runs on the test thread in fake time.
struct Rig {
    explicit Rig(FakeControllerOptions options = {}) : controller(clock, options) {}

    SourceHooks hooks() {
        SourceHooks hooks;
        hooks.open_port = [this](std::string&) -> std::optional<Port> {
            ++opens;
            return controller.port();
        };
        hooks.clock    = clock.clock();
        hooks.on_chunk = [this](const SourceChunk& chunk) {
            chunks.push_back(chunk);
            received.insert(received.end(), chunk.bytes.begin(), chunk.bytes.end());
        };
        hooks.on_status = [this](const SourceStatus& status) {
            statuses.push_back(status);
            if (watch) {
                watch(status);
            }
        };
        return hooks;
    }

    // Stops once the Brain has been idle for this many polls.
    void stopWhenIdle(int polls) {
        controller.onIdle([this, polls] {
            if (++idle >= polls) {
                stop.request();
            }
        });
    }

    void run(const ControllerConfig& config = quickConfig()) {
        runController(config, hooks(), stop);
    }

    std::size_t requestsOf(const Bytes& request) const {
        std::size_t count = 0;
        for (const Bytes& sent : controller.requests()) {
            count += sent == request ? 1 : 0;
        }
        return count;
    }

    FakeClock            clock;
    FakeController       controller;
    platform::StopSignal stop;
    SourceChunks         chunks;
    Bytes                received;
    Statuses             statuses;
    StatusWatcher        watch;
    int                  opens = 0;
    int                  idle  = 0;
};

TEST(ControllerSource, DeliversBrainBytesInOrderAcrossPolls) {
    Rig         rig;
    const Bytes written = patternBytes(300);
    rig.controller.brainWrites(written);
    rig.stopWhenIdle(1);
    rig.run();
    EXPECT_EQ(rig.received, written);
    for (const SourceChunk& chunk : rig.chunks) {
        EXPECT_EQ(chunk.src, SourceKind::kController);
        EXPECT_LE(chunk.bytes.size(), 64u);
    }
    ASSERT_GE(rig.statuses.size(), 2u);
    EXPECT_EQ(rig.statuses.front().state, PortState::kOpen);
    EXPECT_NE(rig.statuses.front().detail.find("linked by radio"), std::string::npos);
    EXPECT_EQ(rig.statuses.back().state, PortState::kStopped);
    ASSERT_TRUE(rig.statuses.back().stats.rtt_us);
    EXPECT_EQ(*rig.statuses.back().stats.rtt_us, 20000);
}

TEST(ControllerSource, WritesOnlyQueriesNeverToTheBrain) {
    Rig rig;
    rig.controller.brainWrites(patternBytes(100));
    rig.stopWhenIdle(2);
    rig.run();
    const Bytes version = systemVersionRequest();
    const Bytes read    = userReadRequest(kUserChannel, kUserReadLength);
    const Bytes radio   = radioStatusRequest();
    ASSERT_FALSE(rig.controller.requests().empty());
    EXPECT_EQ(rig.controller.requests().front(), version);
    for (const Bytes& request : rig.controller.requests()) {
        EXPECT_TRUE(request == version || request == read || request == radio);
    }
}

TEST(ControllerSource, NoBrainLinkedIsReportedThenPollingStartsOnceLinked) {
    FakeControllerOptions options;
    options.flags = 0;
    Rig rig(options);
    rig.controller.brainWrites(patternBytes(10));
    int unlinked = 0;
    rig.watch    = [&rig, &unlinked](const SourceStatus& status) {
        if (status.state == PortState::kNoBrainLinked && ++unlinked == 1) {
            rig.controller.setFlags(kLinkedByCable);
        }
    };
    rig.stopWhenIdle(1);
    rig.run();
    EXPECT_EQ(unlinked, 1);
    EXPECT_NE(rig.statuses.front().detail.find("no Brain linked"), std::string::npos);
    EXPECT_EQ(rig.received, patternBytes(10));
    EXPECT_EQ(rig.requestsOf(systemVersionRequest()), 2u);
}

TEST(ControllerSource, TimeoutsInARowGoBackToSystemVersion) {
    Rig              rig;
    ControllerConfig config = quickConfig();
    config.radio_period_us  = 0;
    rig.controller.brainWrites(patternBytes(40));
    bool dropped = false;
    rig.watch    = [&rig, &dropped](const SourceStatus& status) {
        if (status.state == PortState::kOpen && !dropped) {
            dropped = true;
            rig.controller.dropReplies(3);
        }
    };
    rig.stopWhenIdle(1);
    rig.run(config);
    EXPECT_EQ(rig.statuses.back().stats.timeouts, 3u);
    EXPECT_EQ(rig.requestsOf(systemVersionRequest()), 2u);
    EXPECT_EQ(rig.received, patternBytes(40));
}

TEST(ControllerSource, CorruptRepliesAreCountedAndPollingGoesOn) {
    Rig              rig;
    ControllerConfig config = quickConfig();
    config.radio_period_us  = 0;
    rig.controller.brainWrites(patternBytes(20));
    int idle = 0;
    rig.controller.onIdle([&rig, &idle] {
        ++idle;
        if (idle == 1) {
            rig.controller.corruptNextReply();
        } else if (idle == 2) {
            rig.controller.brainWrites(patternBytes(5));
        } else {
            rig.stop.request();
        }
    });
    rig.run(config);
    Bytes       expected = patternBytes(20);
    const Bytes more     = patternBytes(5);
    expected.insert(expected.end(), more.begin(), more.end());
    EXPECT_EQ(rig.received, expected);
    EXPECT_EQ(rig.statuses.back().stats.bad_replies, 1u);
    EXPECT_EQ(rig.statuses.back().stats.timeouts, 1u);
}

TEST(ControllerSource, NackedPollsAreBadRepliesNotEmptyPolls) {
    Rig              rig;
    ControllerConfig config = quickConfig();
    config.radio_period_us  = 0;
    rig.controller.brainWrites(patternBytes(10));
    rig.controller.nackNextUserRead(0xFF);
    rig.stopWhenIdle(2);
    rig.run(config);
    EXPECT_EQ(rig.statuses.back().stats.bad_replies, 1u);
    EXPECT_EQ(rig.statuses.back().stats.empty_polls, 1u);
    EXPECT_EQ(rig.received, patternBytes(10));
}

TEST(ControllerSource, GarbageBeforeAReplyIsStray) {
    Rig rig;
    rig.controller.brainWrites(patternBytes(30));
    rig.controller.garbageBeforeNextReply({0x13, 0x37, 0x00});
    rig.stopWhenIdle(1);
    rig.run();
    EXPECT_GE(rig.statuses.back().stats.stray_bytes, 3u);
    EXPECT_EQ(rig.received, patternBytes(30));
}

TEST(ControllerSource, LateReplyKeepsTheFifoOrder) {
    Rig              rig;
    ControllerConfig config = quickConfig();
    config.radio_period_us  = 0;
    rig.controller.brainWrites(patternBytes(200));
    bool delayed = false;
    rig.watch    = [&rig, &delayed](const SourceStatus& status) {
        if (status.state == PortState::kOpen && !delayed) {
            delayed = true;
            rig.controller.delayNextReply(400000);
        }
    };
    rig.stopWhenIdle(2);
    rig.run(config);
    EXPECT_EQ(rig.statuses.back().stats.timeouts, 1u);
    EXPECT_EQ(rig.statuses.back().stats.late_replies, 1u);
    EXPECT_EQ(rig.received, patternBytes(200));
}

TEST(ControllerSource, ALateUserReadDuringARadioQueryStillDelivers) {
    Rig              rig;
    ControllerConfig config = quickConfig();
    config.radio_period_us  = 250000;
    rig.controller.brainWrites(patternBytes(100));
    rig.controller.delayNextUserRead(300000);
    rig.stopWhenIdle(2);
    rig.run(config);
    EXPECT_EQ(rig.received, patternBytes(100));
    EXPECT_GE(rig.statuses.back().stats.late_replies, 1u);
    EXPECT_TRUE(rig.statuses.back().stats.radio);
}

TEST(ControllerSource, NulPaddedRepliesStillDecode) {
    FakeControllerOptions options;
    options.pad_to_word = true;
    Rig                   rig(options);
    fakes::FakeBrainStdio brain(0x1234);
    for (int i = 0; i < 5; ++i) {
        rig.controller.brainWrites(brain.pose(10 * i, 20, 0, 100u * static_cast<unsigned>(i)));
    }
    rig.stopWhenIdle(1);
    rig.run();
    feed::PoseFeed feed;
    std::size_t    poses = 0;
    for (const SourceChunk& chunk : rig.chunks) {
        feed.push(chunk.bytes.data(), chunk.bytes.size(), 0.0,
                  [&poses](const feed::TrackedPose&) { ++poses; });
    }
    EXPECT_EQ(poses, 5u);
    EXPECT_GT(rig.statuses.back().stats.padding_bytes, 0u);
    EXPECT_EQ(feed.counts().truncated, 0u);
    EXPECT_EQ(feed.counts().bad_crc, 0u);
}

TEST(ControllerSource, StatusCarriesThePortAndTheLink) {
    Rig rig;
    rig.controller.brainWrites(patternBytes(10));
    rig.stopWhenIdle(1);
    rig.run();
    EXPECT_EQ(rig.statuses.front().src, SourceKind::kController);
    EXPECT_EQ(rig.statuses.front().port, "FAKE1");
    EXPECT_EQ(rig.statuses.front().detail, "controller: Brain linked by radio");
}

TEST(ControllerSource, UnplugReportsClosedAndOpensAgain) {
    FakeClock      clock;
    FakeController first(clock);
    FakeController second(clock);
    first.brainWrites(patternBytes(50));
    first.onIdle([&first] { first.unplug(); });
    second.brainWrites(patternBytes(7));
    platform::StopSignal stop;
    second.onIdle([&stop] { stop.request(); });
    int         opens = 0;
    Statuses    statuses;
    Bytes       received;
    SourceHooks hooks;
    hooks.open_port = [&](std::string&) -> std::optional<Port> {
        return ++opens == 1 ? first.port() : second.port();
    };
    hooks.clock    = clock.clock();
    hooks.on_chunk = [&received](const SourceChunk& chunk) {
        received.insert(received.end(), chunk.bytes.begin(), chunk.bytes.end());
    };
    hooks.on_status = [&statuses](const SourceStatus& status) { statuses.push_back(status); };
    runController(quickConfig(), hooks, stop);
    Bytes       expected = patternBytes(50);
    const Bytes more     = patternBytes(7);
    expected.insert(expected.end(), more.begin(), more.end());
    EXPECT_EQ(received, expected);
    EXPECT_EQ(opens, 2);
    bool closed = false;
    for (const SourceStatus& status : statuses) {
        closed = closed ||
                 (status.state == PortState::kClosed && status.detail == "controller unplugged");
    }
    EXPECT_TRUE(closed);
}

TEST(ControllerSource, RadioStatusIsReported) {
    Rig rig;
    rig.controller.brainWrites(patternBytes(10));
    rig.stopWhenIdle(1);
    rig.run();
    const std::optional<RadioStatus>& radio = rig.statuses.back().stats.radio;
    ASSERT_TRUE(radio);
    EXPECT_EQ(radio->device, 4);
    EXPECT_EQ(radio->quality, 90);
    EXPECT_EQ(radio->strength, -48);
    EXPECT_EQ(radio->channel, 55);
}

TEST(ControllerSource, UnansweredRadioStatusDoesNotStopPolling) {
    FakeControllerOptions options;
    options.answer_radio = false;
    Rig rig(options);
    rig.controller.brainWrites(patternBytes(10));
    rig.stopWhenIdle(1);
    rig.run();
    EXPECT_EQ(rig.received, patternBytes(10));
    EXPECT_FALSE(rig.statuses.back().stats.radio);
    EXPECT_EQ(rig.requestsOf(systemVersionRequest()), 1u);
}

TEST(ControllerSource, AnIdleLinkIsCheckedAgain) {
    Rig rig;
    rig.stopWhenIdle(200); // over 5 s of empty polls
    rig.run();
    EXPECT_GE(rig.requestsOf(systemVersionRequest()), 3u);
    EXPECT_GT(rig.statuses.back().stats.empty_polls, 100u);
}

TEST(ControllerSource, AStopBeforeTheFirstPollEndsTheLoop) {
    Rig rig;
    rig.watch = [&rig](const SourceStatus&) { rig.stop.request(); };
    rig.run();
    EXPECT_EQ(rig.statuses.back().state, PortState::kStopped);
    EXPECT_TRUE(rig.received.empty());
}

} // namespace
} // namespace spectagatr::receiver::sources
