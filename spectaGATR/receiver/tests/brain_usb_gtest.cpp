// brain_usb_gtest.cpp
// The serial source loop with a Brain user port: scripted ports, fake time.

#include "receiver/sources/brain_usb.h"
#include "receiver/sources/serial_source.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "receiver/sources/fakes/fake_port.h"

namespace spectagatr::receiver::sources
{
namespace
{

using fakes::Arrive;
using fakes::FakeClock;
using fakes::ScriptedPort;
using fakes::Unplug;
using Statuses = std::vector<SourceStatus>;
using States   = std::vector<PortState>;

SerialSourceConfig quickConfig() {
    SerialSourceConfig config;
    config.reconnect_wait = Milliseconds(0);
    return config;
}

Bytes bytesOf(const std::string& text) { return Bytes(text.begin(), text.end()); }

struct Record {
    SourceChunks chunks;
    Statuses     statuses;

    SourceHooks hooks(FakeClock& clock, PortOpener opener) {
        SourceHooks hooks;
        hooks.open_port = std::move(opener);
        hooks.clock     = clock.clock();
        hooks.on_chunk  = [this](const SourceChunk& chunk) { chunks.push_back(chunk); };
        hooks.on_status = [this](const SourceStatus& status) { statuses.push_back(status); };
        return hooks;
    }

    States states() const {
        States states;
        for (const SourceStatus& status : statuses) {
            if (states.empty() || states.back() != status.state) {
                states.push_back(status.state);
            }
        }
        return states;
    }
};

TEST(BrainUsbSource, ChunksAreStampedWhenTheyArrive) {
    FakeClock            clock;
    platform::StopSignal stop;
    ScriptedPort port(clock, {Arrive{1000, bytesOf("SGP1 01")}, Arrive{5000, bytesOf("\n")}},
                      "brain user port");
    port.onEnd([&stop] { stop.request(); });
    Record record;
    runSerialSource(quickConfig(),
                    record.hooks(clock, [&port](std::string&) { return port.port(); }), stop);
    ASSERT_EQ(record.chunks.size(), 2u);
    EXPECT_EQ(record.chunks[0].rx_us, 1000);
    EXPECT_EQ(record.chunks[0].bytes, bytesOf("SGP1 01"));
    EXPECT_EQ(record.chunks[1].rx_us, 6000);
    EXPECT_EQ(record.chunks[1].src, SourceKind::kBrainUsb);
    EXPECT_EQ(record.states(), (States{PortState::kOpen, PortState::kStopped}));
    EXPECT_EQ(record.statuses.front().port, "brain user port");
    EXPECT_EQ(record.statuses.front().src, SourceKind::kBrainUsb);
}

TEST(BrainUsbSource, UnplugReportsClosedAndOpensTheDeviceAgain) {
    FakeClock            clock;
    platform::StopSignal stop;
    ScriptedPort         first(clock, {Arrive{100, bytesOf("a")}, Unplug{200, "gone"}});
    ScriptedPort         second(clock, {Arrive{100, bytesOf("b")}});
    second.onEnd([&stop] { stop.request(); });
    int    opens = 0;
    Record record;
    runSerialSource(quickConfig(),
                    record.hooks(clock,
                                 [&](std::string&) -> std::optional<Port> {
                                     return ++opens == 1 ? first.port() : second.port();
                                 }),
                    stop);
    EXPECT_EQ(opens, 2);
    EXPECT_EQ(record.states(), (States{PortState::kOpen, PortState::kClosed, PortState::kOpen,
                                       PortState::kStopped}));
    EXPECT_EQ(record.statuses[1].detail, "gone");
    ASSERT_EQ(record.chunks.size(), 2u);
    EXPECT_EQ(record.chunks[1].bytes, bytesOf("b"));
}

TEST(BrainUsbSource, AMissingDeviceIsSearchedForAgain) {
    FakeClock            clock;
    platform::StopSignal stop;
    ScriptedPort         port(clock, {Arrive{0, bytesOf("x")}});
    port.onEnd([&stop] { stop.request(); });
    int    tries = 0;
    Record record;
    runSerialSource(quickConfig(),
                    record.hooks(clock,
                                 [&](std::string& err) -> std::optional<Port> {
                                     if (++tries < 3) {
                                         err = "no V5 Brain user port";
                                         return std::nullopt;
                                     }
                                     return port.port();
                                 }),
                    stop);
    EXPECT_EQ(tries, 3);
    EXPECT_EQ(record.statuses[0].state, PortState::kSearching);
    EXPECT_EQ(record.statuses[0].detail, "no V5 Brain user port");
    EXPECT_EQ(record.states(),
              (States{PortState::kSearching, PortState::kOpen, PortState::kStopped}));
}

TEST(BrainUsbSource, NeverWritesToTheBrain) {
    FakeClock            clock;
    platform::StopSignal stop;
    ScriptedPort         port(clock, {Arrive{10, bytesOf("SGP1 00\nSGP1 01\n")}});
    port.onEnd([&stop] { stop.request(); });
    Record record;
    runSerialSource(quickConfig(),
                    record.hooks(clock, [&port](std::string&) { return port.port(); }), stop);
    EXPECT_TRUE(port.written().empty());
}

TEST(BrainUsbSource, ReadsOfAMultipleOf512ThenSilenceAreCounted) {
    FakeClock            clock;
    platform::StopSignal stop;
    ScriptedPort         port(clock, {Arrive{0, Bytes(512, 'x')}, Arrive{300000, bytesOf("y")}});
    port.onEnd([&stop] { stop.request(); });
    Record record;
    runSerialSource(quickConfig(),
                    record.hooks(clock, [&port](std::string&) { return port.port(); }), stop);
    const TransportStats& stats = record.statuses.back().stats;
    EXPECT_EQ(stats.reads, 2u);
    EXPECT_EQ(stats.reads_of_512, 1u);
    EXPECT_EQ(stats.holdback_suspect, 1u);
}

TEST(BrainUsbSource, StatsAreReportedWhileOpen) {
    FakeClock            clock;
    platform::StopSignal stop;
    ScriptedPort         port(clock, {Arrive{2500000, bytesOf("z")}});
    port.onEnd([&stop] { stop.request(); });
    Record record;
    runSerialSource(quickConfig(),
                    record.hooks(clock, [&port](std::string&) { return port.port(); }), stop);
    std::size_t open_reports = 0;
    for (const SourceStatus& status : record.statuses) {
        open_reports += status.state == PortState::kOpen ? 1 : 0;
    }
    EXPECT_GE(open_reports, 3u);
}

} // namespace
} // namespace spectagatr::receiver::sources
