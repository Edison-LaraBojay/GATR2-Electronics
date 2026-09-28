// inspection_feed_gtest.cpp
// The inspect/2 live feed on loopback. At the server: latest-wins slots
// deliver the newest state after a backlog, frames stay intact and
// reliable messages keep their order under a slow reader, the send order
// after the in-flight frame follows the channel priorities, a reliable
// overflow closes the client while one large message still goes, and
// ping/pong flow control keeps old states out of the OS buffers. With a
// System behind the service: late join and reconnect (hello, history,
// capture, then state and diag), a session reset sending hello and history
// before any new state, subscriptions gating diag, instrumentation and the
// link monitors, ping/pong, event push, the unchanged-diag skip (also with
// the workers running), a paused and a rate-limited client receiving fresh
// states at the shipped defaults, reset labels under load, telemetry, and
// the capture routes.
// Host tests only; timing bounds are loose because the suite shares the
// machine and Windows timers are coarse.

#include "inspection_test_client.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "core/host_clock.h"
#include "diagnostics/hub.h"
#include "inspection/http_server.h"
#include "inspection/inspection_document.h"
#include "inspection/inspection_service.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "translaGATR/frame_codec.h"

using namespace navigatr;
using namespace inspection_client;

namespace
{

// ---- server-level helpers ------------------------------------------------------

HttpResponse notFoundHandler(const HttpRequest&) {
    HttpResponse r;
    r.status = 404;
    return r;
}

HttpServer::ClientId waitForClient(HttpServer& server) {
    const auto deadline = Clock::now() + Ms(3000);
    while (server.clients().empty() && Clock::now() < deadline) {
        std::this_thread::sleep_for(Ms(5));
    }
    return server.clients().empty() ? 0 : server.clients()[0];
}

// A payload whose body can be checked byte for byte: {"k":"<kind>","n":N,"pad":"..."}
std::string patterned(const std::string& kind, int n, std::size_t pad) {
    std::string s = "{\"k\":\"" + kind + "\",\"n\":" + std::to_string(n) + ",\"pad\":\"";
    for (std::size_t j = 0; j < pad; ++j) {
        s.push_back(static_cast<char>('a' + (static_cast<std::size_t>(n) + j) % 26));
    }
    s += "\"}";
    return s;
}

bool checkPatterned(const std::string& payload, std::string& kind, int& n) {
    kind = jsonString(payload, "k");
    long long v = 0;
    if (kind.empty() || !jsonNumber(payload, "n", v)) {
        return false;
    }
    n                     = static_cast<int>(v);
    const std::size_t pad = payload.find("\"pad\":\"");
    if (pad == std::string::npos) {
        return false;
    }
    const std::size_t start = pad + 7;
    const std::size_t end   = payload.find('"', start);
    if (end == std::string::npos || payload.substr(end) != "\"}") {
        return false;
    }
    for (std::size_t j = 0; start + j < end; ++j) {
        if (payload[start + j] != static_cast<char>('a' + (static_cast<std::size_t>(n) + j) % 26)) {
            return false;
        }
    }
    return true;
}

// ---- service-level fixture ------------------------------------------------------

// The synthetic rig without a camera: wheels, a gyro with bias to calibrate,
// measured attitude, noop commands and world estimation.
std::string rigXml(const std::string& inspection) {
    return R"(
<System>
    <Loop rate_hz="100"/>
    )" + inspection +
           R"(
    <Resources>
        <Resource id="field" type="field_map">
            <Landmark id="goal">
                <NominalPose calibration_status="verified" x_m="1.7832" y_m="1.7832" heading_deg="0"/>
            </Landmark>
        </Resource>
        <Resource id="wheel_geometry" type="wheel_geometry">
            <Wheel id="left_wheel" sensor_id="enc_a" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="0.13"
                   measurement_angle_deg="0" direction="positive"/>
            <Wheel id="right_wheel" sensor_id="enc_b" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="-0.13"
                   measurement_angle_deg="0" direction="positive"/>
            <Wheel id="rear_wheel" sensor_id="enc_c" calibration_status="verified"
                   radius_m="0.0254" position_x_m="-0.12" position_y_m="0"
                   measurement_angle_deg="90" direction="positive"/>
        </Resource>
        <Resource id="rig" type="synthetic_rig">
            <Field resource_id="field"/>
            <Wheels resource_id="wheel_geometry" counts_per_revolution="4000"/>
            <Trajectory center_x_m="1.7832" center_y_m="1.7832" radius_m="0.5"
                        period_s="30" facing="center" start_deg="180" hold_s="0.5"/>
            <Telemetry tick_hz="50" device_offset_ms="5000" gyro_bias_mdps="800"/>
            <Attitude mode="measured" rock_deg="3" period_s="2.5"/>
            <Output id="encoder_a" wheel_id="left_wheel"/>
            <Output id="encoder_b" wheel_id="right_wheel"/>
            <Output id="encoder_c" wheel_id="rear_wheel"/>
            <Output id="imu" channel="imu"/>
            <Output id="attitude" channel="attitude"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="rig" output_id="imu"/>
        </Sensor>
        <Sensor id="robot_attitude" type="attitude_channel">
            <Source resource_id="rig" output_id="attitude"/>
            <Mounting calibration_status="verified" roll_deg="0" pitch_deg="0" yaw_deg="0"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <Wheels resource_id="wheel_geometry">
                    <Use wheel_id="left_wheel"/>
                    <Use wheel_id="right_wheel"/>
                    <Use wheel_id="rear_wheel"/>
                </Wheels>
                <HeadingConstraint sensor_id="robot_imu" bias_samples="10" window_ms="150"
                                   max_calibration_travel_m="0.005"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Observation id="attitude" type="attitude_reference">
                <Input sensor_id="robot_attitude"/>
                <Freshness max_age_ms="100"/>
                <Output observation_id="attitude"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
                <Attitude observation_id="attitude" max_age_ms="200"/>
            </Estimator>
            <History retention_s="5" capacity="1024" max_interpolation_gap_ms="100"/>
            <InitialPlacement x_m="1.2832" y_m="1.7832" heading_deg="0"/>
        </Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
}

const char* kFeedInspection =
    "<Inspection enabled=\"true\" port=\"0\" state_hz=\"30\" diag_hz=\"4\" preview_hz=\"0\"/>";

struct Feed {
    FunctionRegistry                   functions;
    std::unique_ptr<System>            system;   // outlives the service
    std::unique_ptr<InspectionService> service;
    int64_t                            now_ms = 0;

    explicit Feed(const char* inspection = kFeedInspection) {
        registerAll(functions);
        std::string err;
        system = System::buildFromString(rigXml(inspection).c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system != nullptr) {
            service = InspectionService::create(*system, system->inspection(), err);
            EXPECT_NE(service, nullptr) << err;
            std::string start_err;
            EXPECT_TRUE(service != nullptr && service->start(start_err)) << start_err;
        }
    }

    // Inline cycles on a synthetic clock from 0, as the rig's trajectory
    // expects; document ages are then not meaningful and are not checked.
    void step(int cycles) {
        for (int i = 0; i < cycles; ++i) {
            now_ms += 10;
            system->step(hostTime(now_ms));
        }
    }
};

struct Received {
    std::string type;
    std::string payload;
};

// Reads text messages for duration, stepping the runtime between reads when
// cycles > 0.
std::vector<Received> collect(WsClient& ws, Ms duration, Feed* feed = nullptr, int cycles = 0) {
    std::vector<Received> out;
    WsMessage             m;
    const auto            until = Clock::now() + duration;
    while (Clock::now() < until) {
        if (feed != nullptr && cycles > 0) {
            feed->step(cycles);
        }
        while (ws.next(m, Ms(5))) {
            if (m.opcode == 1) {
                out.push_back(Received{messageType(m.payload), m.payload});
            }
        }
    }
    return out;
}

std::size_t countType(const std::vector<Received>& msgs, const std::string& type) {
    return static_cast<std::size_t>(std::count_if(
        msgs.begin(), msgs.end(), [&](const Received& r) { return r.type == type; }));
}

// The first messages a feed client gets, in order.
void expectOpening(WsClient& ws, const std::string& session_id, std::string* history = nullptr) {
    WsMessage m;
    ASSERT_TRUE(ws.next(m, Ms(5000)));
    ASSERT_EQ(messageType(m.payload), "hello");
    EXPECT_NE(m.payload.find("\"contract\":\"navigatr.inspect/2\""), std::string::npos);
    EXPECT_NE(m.payload.find("\"id\":\"" + session_id + "\""), std::string::npos);
    ASSERT_TRUE(ws.next(m, Ms(5000)));
    ASSERT_EQ(messageType(m.payload), "history");
    if (history != nullptr) {
        *history = m.payload;
    }
    ASSERT_TRUE(ws.next(m, Ms(5000)));
    ASSERT_EQ(messageType(m.payload), "capture");
}

} // namespace

// ---- 1. server queues ---------------------------------------------------------------

TEST(InspectionFeed, LatestWinsDeliversTheNewestStateAfterABacklog) {
    HttpServerConfig hc;
    hc.port                = 0;
    hc.client_buffer_bytes = 1024 * 1024;
    hc.send_buffer_bytes   = 4096;
    HttpServer  server(hc, notFoundHandler);
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    WsClient slow;
    ASSERT_TRUE(slow.connect(server.port(), 4096));
    const HttpServer::ClientId id = waitForClient(server);
    ASSERT_NE(id, 0u);

    // the client reads nothing while 3000 states are produced
    const int   kStates  = 3000;
    std::size_t max_seen = 0;
    for (int i = 1; i <= kStates; ++i) {
        const auto r = server.sendLatest(id, WsChannel{"state", 0},
                                         HttpServer::textFrame(patterned("state", i, 1000)));
        ASSERT_TRUE(r == HttpServer::Enqueue::kQueued || r == HttpServer::Enqueue::kReplaced);
        max_seen = std::max(max_seen, server.queued(id));
    }
    // one frame in flight plus one waiting in the slot, never a backlog
    EXPECT_LE(max_seen, 2u * 1100u);
    EXPECT_GT(server.stats().messages_replaced, static_cast<uint64_t>(kStates / 2));

    std::vector<int> seen;
    WsMessage        m;
    while (slow.next(m, Ms(1000))) {
        std::string kind;
        int         n = 0;
        ASSERT_TRUE(checkPatterned(m.payload, kind, n)) << m.payload.substr(0, 80);
        seen.push_back(n);
        if (n == kStates) {
            break;
        }
    }
    ASSERT_FALSE(seen.empty());
    EXPECT_EQ(seen.back(), kStates);   // the newest always arrives
    EXPECT_TRUE(std::is_sorted(seen.begin(), seen.end()));
    EXPECT_EQ(std::adjacent_find(seen.begin(), seen.end()), seen.end());
    EXPECT_LT(seen.size(), static_cast<std::size_t>(kStates / 2));   // the backlog was coalesced
    EXPECT_FALSE(slow.bad_frame);

    const auto stats = server.clientStats();
    ASSERT_EQ(stats.size(), 1u);
    const auto ch = std::find_if(stats[0].channels.begin(), stats[0].channels.end(),
                                 [](const WsChannelStats& c) { return c.name == "state"; });
    ASSERT_NE(ch, stats[0].channels.end());
    EXPECT_TRUE(ch->replaceable);
    EXPECT_EQ(ch->queued, static_cast<uint64_t>(kStates));
    EXPECT_EQ(ch->sent + ch->replaced, static_cast<uint64_t>(kStates));
    EXPECT_GE(ch->last_latency_ms, 0.0);
}

TEST(InspectionFeed, PartialFramesStayIntactAndReliableOrderHoldsUnderASlowReader) {
    HttpServerConfig hc;
    hc.port                = 0;
    hc.client_buffer_bytes = 1024 * 1024;
    hc.reliable_bytes      = 512 * 1024;
    hc.send_buffer_bytes   = 4096;
    HttpServer  server(hc, notFoundHandler);
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    WsClient ws;
    ASSERT_TRUE(ws.connect(server.port(), 4096));
    const HttpServer::ClientId id = waitForClient(server);
    ASSERT_NE(id, 0u);

    const int         kRounds = 300;
    std::atomic<bool> done{false};
    // a producer interleaving reliable, state and diag messages while the
    // reader takes 700 bytes at a time
    std::thread producer([&] {
        for (int i = 1; i <= kRounds; ++i) {
            server.sendReliable(id, HttpServer::textFrame(patterned("event", i, 200)), "event");
            server.sendLatest(id, WsChannel{"state", 0},
                              HttpServer::textFrame(patterned("state", i, 3000)));
            server.sendLatest(id, WsChannel{"diag", 2},
                              HttpServer::textFrame(patterned("diag", i, 7000)));
            if (i % 10 == 0) {
                std::this_thread::sleep_for(Ms(2));
            }
        }
        done = true;
    });

    std::vector<int> events, states, diags;
    WsMessage        m;
    auto             idle_since = Clock::now();
    while (Clock::now() - idle_since < Ms(1500)) {
        if (!ws.next(m, Ms(50), 700)) {
            continue;
        }
        idle_since = Clock::now();
        std::string kind;
        int         n = 0;
        ASSERT_TRUE(checkPatterned(m.payload, kind, n)) << m.payload.substr(0, 80);
        if (kind == "event") {
            events.push_back(n);
        } else if (kind == "state") {
            states.push_back(n);
        } else if (kind == "diag") {
            diags.push_back(n);
        }
    }
    producer.join();
    EXPECT_TRUE(done.load());
    EXPECT_FALSE(ws.bad_frame);

    // every reliable message, once, in order
    ASSERT_EQ(events.size(), static_cast<std::size_t>(kRounds));
    for (int i = 0; i < kRounds; ++i) {
        EXPECT_EQ(events[static_cast<std::size_t>(i)], i + 1);
    }
    // replaceable channels only move forward and end at the newest
    ASSERT_FALSE(states.empty());
    ASSERT_FALSE(diags.empty());
    EXPECT_TRUE(std::is_sorted(states.begin(), states.end()));
    EXPECT_TRUE(std::is_sorted(diags.begin(), diags.end()));
    EXPECT_EQ(states.back(), kRounds);
    EXPECT_EQ(diags.back(), kRounds);
    EXPECT_EQ(server.queued(id), 0u);
}

TEST(InspectionFeed, SendOrderAfterTheInFlightFrameFollowsPriorities) {
    HttpServerConfig hc;
    hc.port                = 0;
    hc.client_buffer_bytes = 8 * 1024 * 1024;
    hc.reliable_bytes      = 4 * 1024 * 1024;
    hc.send_buffer_bytes   = 4096;
    HttpServer  server(hc, notFoundHandler);
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    WsClient ws;
    ASSERT_TRUE(ws.connect(server.port(), 4096));
    const HttpServer::ClientId id = waitForClient(server);
    ASSERT_NE(id, 0u);

    // a large reliable frame goes in flight and stalls against the reader
    ASSERT_EQ(server.sendReliable(id, HttpServer::textFrame(patterned("big", 1, 2000000)), "big"),
              HttpServer::Enqueue::kQueued);
    ASSERT_GT(server.queued(id), 100000u);
    const auto send = [&](const char* name, int priority, int n) {
        return server.sendLatest(id, WsChannel{name, priority},
                                 HttpServer::textFrame(patterned(name, n, 100)));
    };
    send("preview:b", 4, 1);
    send("preview:a", 4, 1);
    send("instrumentation", 3, 1);
    send("diag", 2, 1);
    send("telemetry", 1, 1);
    send("state", 0, 1);
    server.sendReliable(id, HttpServer::textFrame(patterned("event", 1, 100)), "event");
    server.sendReliable(id, HttpServer::textFrame(patterned("event", 2, 100)), "event");
    EXPECT_EQ(send("state", 0, 2), HttpServer::Enqueue::kReplaced);
    server.clearLatest(id, "instrumentation");

    std::vector<std::string> order;
    WsMessage                m;
    while (order.size() < 8 && ws.next(m, Ms(5000))) {
        std::string kind;
        int         n = 0;
        ASSERT_TRUE(checkPatterned(m.payload, kind, n));
        order.push_back(kind + ":" + std::to_string(n));
    }
    const std::vector<std::string> expected = {"big:1",       "event:1",     "event:2",
                                               "state:2",     "telemetry:1", "diag:1",
                                               "preview:a:1", "preview:b:1"};
    EXPECT_EQ(order, expected);
    EXPECT_FALSE(ws.next(m, Ms(300)));   // the cleared instrumentation never went
    EXPECT_FALSE(ws.bad_frame);

    const auto stats = server.clientStats();
    ASSERT_EQ(stats.size(), 1u);
    for (const WsChannelStats& c : stats[0].channels) {
        if (c.name == "instrumentation") {
            EXPECT_EQ(c.dropped, 1u);
            EXPECT_EQ(c.sent, 0u);
        }
        if (c.name == "state") {
            EXPECT_EQ(c.replaced, 1u);
            EXPECT_EQ(c.sent, 1u);
        }
    }
}

TEST(InspectionFeed, ReliableOverflowClosesTheClientAndCountsIt) {
    HttpServerConfig hc;
    hc.port              = 0;
    hc.reliable_bytes    = 16 * 1024;
    hc.send_buffer_bytes = 4096;
    HttpServer  server(hc, notFoundHandler);
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    WsClient stuck;
    ASSERT_TRUE(stuck.connect(server.port(), 4096));
    const HttpServer::ClientId id = waitForClient(server);
    ASSERT_NE(id, 0u);

    HttpServer::Enqueue last = HttpServer::Enqueue::kQueued;
    int                 sent = 0;
    while (last == HttpServer::Enqueue::kQueued && sent < 5000) {
        last = server.sendReliable(id, HttpServer::textFrame(patterned("event", sent, 1000)),
                                   "event");
        ++sent;
        EXPECT_LE(server.clientStats().empty() ? 0u : server.clientStats()[0].reliable_bytes,
                  hc.reliable_bytes);
    }
    EXPECT_EQ(last, HttpServer::Enqueue::kClosed);
    EXPECT_EQ(server.stats().clients_closed_reliable_overflow, 1u);
    // nothing more is accepted, and the socket goes within the close grace
    EXPECT_EQ(server.sendReliable(id, HttpServer::textFrame("x"), "event"),
              HttpServer::Enqueue::kUnknownClient);
    const auto deadline = Clock::now() + Ms(5000);
    while (!server.clients().empty() && Clock::now() < deadline) {
        std::this_thread::sleep_for(Ms(10));
    }
    const auto wait_reap = Clock::now() + Ms(3000);
    while (server.stats().recent_closes.empty() && Clock::now() < wait_reap) {
        std::this_thread::sleep_for(Ms(10));
    }
    ASSERT_FALSE(server.stats().recent_closes.empty());
    EXPECT_EQ(server.stats().recent_closes.back().reason, "reliable_overflow");
    EXPECT_EQ(server.stats().websocket_clients, 0u);
    // the client sees the connection end (a close frame when it got through)
    WsMessage m;
    while (stuck.next(m, Ms(2000))) {
    }
    EXPECT_TRUE(stuck.closed());
}

// Flow control: once the client answers pings, at most two states are ever
// unconfirmed, so the OS buffers (default sizes here, on both ends) never
// fill with old states. A client that does not answer pings keeps the old
// kernel-bounded behavior and still gets the newest state.
TEST(InspectionFeed, FlowControlKeepsOldStatesOutOfTheNetwork) {
    HttpServerConfig hc;
    hc.port = 0;   // default buffers and window
    HttpServer  server(hc, notFoundHandler);
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    WsClient ws;
    ASSERT_TRUE(ws.connect(server.port()));
    const HttpServer::ClientId id = waitForClient(server);
    ASSERT_NE(id, 0u);
    WsMessage m;
    EXPECT_FALSE(ws.next(m, Ms(300)));   // the probe ping is answered, nothing else came
    ASSERT_EQ(ws.pings_answered, 1);
    const auto acking = Clock::now() + Ms(2000);
    while ((server.clientStats().empty() || !server.clientStats()[0].flow_control) &&
           Clock::now() < acking) {
        std::this_thread::sleep_for(Ms(5));
    }
    ASSERT_TRUE(server.clientStats()[0].flow_control);

    // the client reads nothing while 2000 states go by
    const int kStates      = 2000;
    uint64_t  max_unacked  = 0;
    for (int i = 1; i <= kStates; ++i) {
        server.sendLatest(id, WsChannel{"state", 0},
                          HttpServer::textFrame(patterned("state", i, 1000)));
        max_unacked = std::max(max_unacked, server.clientStats()[0].unacked_bytes);
        if (i % 100 == 0) {
            std::this_thread::sleep_for(Ms(2));
        }
    }
    EXPECT_LE(max_unacked, 2u * (1100u + 16u));   // two states and their pings

    std::vector<int> seen;
    while (ws.next(m, Ms(1000))) {
        std::string kind;
        int         n = 0;
        ASSERT_TRUE(checkPatterned(m.payload, kind, n));
        seen.push_back(n);
        if (n == kStates) {
            break;
        }
    }
    ASSERT_FALSE(seen.empty());
    EXPECT_EQ(seen.back(), kStates);
    EXPECT_LE(seen.size(), 3u) << "old states waited in the network";
    EXPECT_TRUE(std::is_sorted(seen.begin(), seen.end()));
    EXPECT_FALSE(ws.bad_frame);
    const auto stats = server.clientStats();
    ASSERT_EQ(stats.size(), 1u);
    EXPECT_GE(stats[0].ack_rtt_ms, 0.0);
    for (const WsChannelStats& c : stats[0].channels) {
        if (c.name == "state") {
            EXPECT_GE(c.last_acked_ms, 0.0);
        }
    }

    // a client that never answers: no flow control, the newest still arrives
    WsClient legacy;
    legacy.answer_pings = false;
    ASSERT_TRUE(legacy.connect(server.port()));
    HttpServer::ClientId lid = 0;
    const auto           wait_legacy = Clock::now() + Ms(3000);
    while (lid == 0 && Clock::now() < wait_legacy) {
        for (HttpServer::ClientId c : server.clients()) {
            if (c != id) {
                lid = c;
            }
        }
        std::this_thread::sleep_for(Ms(5));
    }
    ASSERT_NE(lid, 0u);
    for (int i = 1; i <= 50; ++i) {
        server.sendLatest(lid, WsChannel{"state", 0},
                          HttpServer::textFrame(patterned("state", i, 1000)));
    }
    int last = 0, pings = 0;
    while (legacy.next(m, Ms(1000))) {
        if (m.opcode == 0x9) {
            ++pings;
            continue;
        }
        std::string kind;
        ASSERT_TRUE(checkPatterned(m.payload, kind, last));
        if (last == 50) {
            break;
        }
    }
    EXPECT_EQ(last, 50);
    EXPECT_EQ(pings, 1);   // the probe only
    EXPECT_EQ(server.stats().flow_control_clients, 1u);
}

// A reliable message larger than the whole budget (a big hello or history)
// still goes when nothing waits; the budget bounds the backlog.
TEST(InspectionFeed, ReliableBudgetBoundsTheBacklogNotOneMessage) {
    HttpServerConfig hc;
    hc.port              = 0;
    hc.reliable_bytes    = 16 * 1024;
    hc.send_buffer_bytes = 4096;
    HttpServer  server(hc, notFoundHandler);
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    WsClient stuck;
    ASSERT_TRUE(stuck.connect(server.port(), 4096));
    const HttpServer::ClientId id = waitForClient(server);
    ASSERT_NE(id, 0u);

    // goes in flight against the stalled reader
    EXPECT_EQ(server.sendReliable(id, HttpServer::textFrame(patterned("hello", 1, 2000000)), "hello"),
              HttpServer::Enqueue::kQueued);
    ASSERT_GT(server.queued(id), 100000u);
    // nothing waits in the FIFO: admitted though larger than the budget
    EXPECT_EQ(server.sendReliable(id, HttpServer::textFrame(patterned("history", 1, 40000)),
                                  "history"),
              HttpServer::Enqueue::kQueued);
    // now something waits: the budget applies
    EXPECT_EQ(server.sendReliable(id, HttpServer::textFrame(patterned("event", 1, 100)), "event"),
              HttpServer::Enqueue::kClosed);
    EXPECT_EQ(server.stats().clients_closed_reliable_overflow, 1u);
}

// ---- 2. the feed with a System ---------------------------------------------------------

TEST(InspectionFeed, LateJoinAndReconnectGetHelloHistoryCaptureThenStateAndDiag) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    f.step(80);

    WsClient    a;
    std::string history;
    ASSERT_TRUE(a.connect(f.service->port()));
    expectOpening(a, f.system->sessionId(), &history);
    EXPECT_NE(history.find("\"trail\":[{\"host_ms\":"), std::string::npos) << history;
    auto msgs = collect(a, Ms(800), &f, 1);
    EXPECT_GE(countType(msgs, "state"), 3u);
    EXPECT_GE(countType(msgs, "diag"), 1u);
    for (const Received& r : msgs) {
        EXPECT_NE(r.type, "hello");   // one hello per session
        EXPECT_NE(r.type, "snapshot");
    }

    // a second viewer joins late, the first reconnects
    WsClient b;
    ASSERT_TRUE(b.connect(f.service->port()));
    expectOpening(b, f.system->sessionId());
    a.sendClose();
    a.close();
    WsClient a2;
    ASSERT_TRUE(a2.connect(f.service->port()));
    expectOpening(a2, f.system->sessionId());
    msgs = collect(a2, Ms(800), &f, 1);
    EXPECT_GE(countType(msgs, "state"), 3u);
    EXPECT_GE(countType(msgs, "diag"), 1u);
    const auto b_msgs = collect(b, Ms(300));
    EXPECT_GE(countType(b_msgs, "state") + countType(msgs, "state"), 3u);

    const InspectionServiceStats s = f.service->stats();
    EXPECT_EQ(s.hellos_queued, 3u);
    EXPECT_EQ(s.clients_total, 3u);
}

// The review case: reliable_kb passed validation at 16 while hello alone
// can exceed it, so every connection was closed at open. The floor is now
// 128 KiB and a single large message goes whenever nothing waits.
TEST(InspectionFeed, MinimumReliableBudgetOpensEveryClientWithAFullTrail) {
    Feed f("<Inspection enabled=\"true\" port=\"0\" preview_hz=\"0\" reliable_kb=\"128\"/>");
    ASSERT_NE(f.service, nullptr);
    f.step(400);   // a full trail, as after 4 s of running
    for (int attempt = 0; attempt < 3; ++attempt) {
        WsClient    ws;
        std::string history;
        ASSERT_TRUE(ws.connect(f.service->port()));
        expectOpening(ws, f.system->sessionId(), &history);
        EXPECT_GT(history.size(), 10000u);
        const auto msgs = collect(ws, Ms(400), &f, 1);
        EXPECT_GE(countType(msgs, "state"), 1u);
        EXPECT_FALSE(ws.closed());
    }
    EXPECT_EQ(f.service->stats().closed_reliable_overflow, 0u);
}

TEST(InspectionFeed, SessionResetSendsHelloAndHistoryBeforeAnyNewState) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());
    auto before = collect(ws, Ms(500), &f, 1);
    ASSERT_GE(countType(before, "state"), 1u);
    for (const Received& r : before) {
        if (r.type == "state") {
            EXPECT_NE(r.payload.find("\"reset_count\":0"), std::string::npos);
        }
    }

    f.system->reset();
    const auto after = collect(ws, Ms(1500), &f, 1);
    std::size_t hello_at = after.size();
    for (std::size_t i = 0; i < after.size(); ++i) {
        if (after[i].type == "hello") {
            hello_at = i;
            break;
        }
    }
    ASSERT_LT(hello_at, after.size()) << "no hello after the reset";
    EXPECT_NE(after[hello_at].payload.find("\"reset_count\":1"), std::string::npos);
    ASSERT_LT(hello_at + 1, after.size());
    EXPECT_EQ(after[hello_at + 1].type, "history");
    EXPECT_NE(after[hello_at + 1].payload.find("\"reset_count\":1"), std::string::npos);
    std::size_t new_states = 0;
    for (std::size_t i = hello_at + 1; i < after.size(); ++i) {
        if (after[i].type == "state" || after[i].type == "diag") {
            // nothing of the old session follows the new hello
            EXPECT_EQ(after[i].payload.find("\"reset_count\":0"), std::string::npos) << i;
            new_states += after[i].type == "state";
        }
    }
    EXPECT_GE(new_states, 1u);
}

TEST(InspectionFeed, SubscribeGatesDiagInstrumentationStateAndTheLinkMonitors) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());

    // off by default
    auto msgs = collect(ws, Ms(700), &f, 1);
    EXPECT_EQ(countType(msgs, "instrumentation"), 0u);
    EXPECT_FALSE(f.system->diagHub().links().raw());

    ws.sendText("{\"type\":\"subscribe\",\"instrumentation\":true,\"raw\":true,\"decoded\":true}");
    msgs = collect(ws, Ms(2000), &f, 1);
    const std::size_t instrumentation = countType(msgs, "instrumentation");
    EXPECT_GE(instrumentation, 3u);
    EXPECT_LE(instrumentation, 11u);   // 4 Hz at most, with slack
    EXPECT_TRUE(f.system->diagHub().links().raw());
    EXPECT_TRUE(f.system->diagHub().links().decoded());
    for (const Received& r : msgs) {
        if (r.type == "instrumentation") {
            EXPECT_EQ(r.payload.rfind("{\"type\":\"instrumentation\",\"seq\":", 0), 0u);
            EXPECT_NE(r.payload.find("\"links\":["), std::string::npos);
        }
    }

    // hidden panel: off again, and the rings with it
    ws.sendText("{\"type\":\"subscribe\",\"instrumentation\":false,\"diag\":false}");
    collect(ws, Ms(400), &f, 1);   // what was in flight
    msgs = collect(ws, Ms(1500), &f, 1);
    EXPECT_EQ(countType(msgs, "instrumentation"), 0u);
    EXPECT_EQ(countType(msgs, "diag"), 0u);
    EXPECT_GE(countType(msgs, "state"), 3u);   // state is untouched
    EXPECT_FALSE(f.system->diagHub().links().raw());
    EXPECT_FALSE(f.system->diagHub().links().decoded());

    // back on: a diag right away; state off stops states
    ws.sendText("{\"type\":\"subscribe\",\"diag\":true,\"state_hz\":0}");
    collect(ws, Ms(300), &f, 1);
    msgs = collect(ws, Ms(1200), &f, 1);
    EXPECT_GE(countType(msgs, "diag"), 1u);
    EXPECT_EQ(countType(msgs, "state"), 0u);
}

TEST(InspectionFeed, PingIsAnsweredWithPongEchoingTheClientTime) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());

    const int64_t before_ms = HostClock::now().ms;
    ws.sendText("{\"type\":\"ping\",\"id\":7,\"client_ms\":1234.5678}");
    ws.sendText("{\"type\":\"ping\",\"id\":\"abc\",\"client_ms\":99}");
    std::vector<std::string> pongs;
    WsMessage                m;
    const auto               deadline = Clock::now() + Ms(3000);
    while (pongs.size() < 2 && Clock::now() < deadline) {
        if (ws.next(m, Ms(100)) && messageType(m.payload) == "pong") {
            pongs.push_back(m.payload);
        }
    }
    ASSERT_EQ(pongs.size(), 2u);
    EXPECT_NE(pongs[0].find("\"id\":7,\"client_ms\":1234.5678,"), std::string::npos) << pongs[0];
    EXPECT_NE(pongs[1].find("\"id\":\"abc\",\"client_ms\":99,"), std::string::npos) << pongs[1];
    long long host_ms = 0, host_us = 0;
    ASSERT_TRUE(jsonNumber(pongs[0], "host_ms", host_ms));
    ASSERT_TRUE(jsonNumber(pongs[0], "host_us", host_us));
    EXPECT_GE(host_ms, before_ms);
    EXPECT_LE(host_ms, HostClock::now().ms);
    EXPECT_NEAR(static_cast<double>(host_us) / 1000.0, static_cast<double>(host_ms), 2.0);
    // the counter moves right after the send, so the pong can arrive first
    const auto counted = Clock::now() + Ms(2000);
    while (f.service->stats().pongs_queued < 2 && Clock::now() < counted) {
        std::this_thread::sleep_for(Ms(5));
    }
    EXPECT_EQ(f.service->stats().pongs_queued, 2u);
}

TEST(InspectionFeed, EventsArePushedOnceInOrder) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());

    // the gyro bias calibration completes and logs an event
    std::vector<Received> msgs;
    const auto            deadline = Clock::now() + Ms(15000);
    while (countType(msgs, "event") == 0 && Clock::now() < deadline) {
        const auto more = collect(ws, Ms(200), &f, 2);
        msgs.insert(msgs.end(), more.begin(), more.end());
    }
    const auto more = collect(ws, Ms(300));
    msgs.insert(msgs.end(), more.begin(), more.end());
    ASSERT_GE(countType(msgs, "event"), 1u);

    const std::vector<RuntimeEvent> events = f.system->events();
    long long                       last   = 0;
    for (const Received& r : msgs) {
        if (r.type != "event") {
            continue;
        }
        long long seq = 0;
        ASSERT_TRUE(jsonNumber(r.payload, "seq", seq));
        EXPECT_GT(seq, last);   // in order, never twice
        last = seq;
        const auto it = std::find_if(events.begin(), events.end(), [&](const RuntimeEvent& e) {
            return static_cast<long long>(e.sequence) == seq;
        });
        ASSERT_NE(it, events.end());
        EXPECT_EQ(jsonString(r.payload, "text"), it->text);
    }
}

TEST(InspectionFeed, IdenticalDiagIsSkippedUntilTheRefresh) {
    Feed f;   // never stepped: nothing in the diag content changes
    ASSERT_NE(f.service, nullptr);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());
    auto msgs = collect(ws, Ms(300));
    ASSERT_GE(countType(msgs, "diag"), 1u);
    msgs = collect(ws, Ms(500));
    EXPECT_EQ(countType(msgs, "diag"), 0u);
    EXPECT_GT(f.service->stats().diags_skipped, 0u);
    // states keep coming at the keepalive even with no new publication
    msgs = collect(ws, Ms(1500));
    EXPECT_GE(countType(msgs, "diag"), 1u);   // the 1 s diag refresh
    EXPECT_GE(countType(msgs, "state"), 1u);  // the 1 s state keepalive
}

// What the unchanged-diag skip hashes: the calibration and a reset are
// content; the moving robot, clocks, ages, counters and sequences are not.
TEST(InspectionFeed, DiagHashIgnoresTheMovingRobotButNotContent) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    const InspectionServiceStats service;
    const InspectionFeedStats    feed;
    uint64_t                     power_on = 0, calibrated = 0;
    diagDocument(*f.system, service, feed, 1, hostTime(f.now_ms), &power_on);
    f.step(40);   // the rig holds still for 0.5 s: the gyro bias calibrates
    const std::string done =
        diagDocument(*f.system, service, feed, 2, hostTime(f.now_ms), &calibrated);
    ASSERT_NE(done.find("\"calibration\":\"done\""), std::string::npos);
    EXPECT_NE(calibrated, power_on);
    f.step(300);

    uint64_t          moving = 0, later = 0;
    const std::string first =
        diagDocument(*f.system, service, feed, 3, hostTime(f.now_ms), &moving);
    f.step(3);
    const std::string second =
        diagDocument(*f.system, service, feed, 4, hostTime(f.now_ms + 7), &later);
    EXPECT_NE(first, second);   // pose, cycle, ages, counters moved
    EXPECT_EQ(moving, later);

    f.system->reset();
    uint64_t after_reset = 0;
    diagDocument(*f.system, service, feed, 5, hostTime(f.now_ms + 8), &after_reset);
    EXPECT_NE(after_reset, later);
}

// The review case: with the workers running every diag differed (cycle,
// ages, counters, the moving robot), so none was ever skipped. Only clocks,
// ages, counters and the robot change here once the rig moves steadily.
TEST(InspectionFeed, UnchangedDiagIsSkippedWhileTheWorkersRun) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    std::string err;
    ASSERT_TRUE(f.system->start(err)) << err;
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());
    // settle: calibration (or, when the rig already moves because the
    // process clock passed its hold, discarded motion) and the first move
    collect(ws, Ms(2500));
    const uint64_t skipped_before = f.service->stats().diags_skipped;
    const auto     msgs           = collect(ws, Ms(3000));
    const uint64_t skipped        = f.service->stats().diags_skipped - skipped_before;
    const std::size_t diags       = countType(msgs, "diag");
    f.service->stop();
    f.system->stop();
    EXPECT_GT(skipped, 0u);
    // 4 Hz would be 12; the 1 s refresh alone about 3
    EXPECT_GE(diags, 2u);
    EXPECT_LE(diags, 8u);
    EXPECT_GE(countType(msgs, "state"), 30u);   // the robot itself goes by state
}

TEST(InspectionFeed, HistoryIsSentAgainOnRequest) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    f.step(80);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());
    ws.sendText("{\"type\":\"history\"}");
    const auto msgs = collect(ws, Ms(800));
    ASSERT_EQ(countType(msgs, "history"), 1u);
    for (const Received& r : msgs) {
        if (r.type == "history") {
            EXPECT_NE(r.payload.find("\"trail\":[{"), std::string::npos);
        }
    }
}

namespace
{

// Age of a state at receipt: the service and this test share the process
// and HostClock, so the difference is exact.
int64_t stateAgeMs(const std::string& payload) {
    long long host_ms = 0;
    EXPECT_TRUE(jsonNumber(payload, "host_ms", host_ms));
    return HostClock::now().ms - host_ms;
}

int64_t percentile(std::vector<int64_t> v, double p) {
    if (v.empty()) {
        return -1;
    }
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
}

} // namespace

// The review case at the shipped defaults (OS receive buffer, no grace
// period): a client that stops reading for 3 s and then reads everything
// gets at most the few frames flow control let into the network, then
// fresh states. Before flow control, 50 to 74 stale states arrived first.
TEST(InspectionFeed, PausedClientResumesWithFreshStatesNotABacklog) {
    Feed f("<Inspection enabled=\"true\" port=\"0\" preview_hz=\"0\"/>");
    ASSERT_NE(f.service, nullptr);
    std::string err;
    ASSERT_TRUE(f.system->start(err)) << err;
    WsClient slow;
    ASSERT_TRUE(slow.connect(f.service->port()));
    collect(slow, Ms(500));
    ASSERT_GE(slow.pings_answered, 1);

    std::this_thread::sleep_for(Ms(3000));
    const InspectionServiceStats paused = f.service->stats();
    EXPECT_GT(paused.states_replaced, 10u);
    EXPECT_EQ(paused.flow_control_clients, 1u);
    EXPECT_EQ(paused.closed_reliable_overflow, 0u);

    WsMessage             m;
    int                   stale_first = 0;   // older than 250 ms, before the first fresh one
    std::size_t           stale_bytes = 0;
    bool                  fresh_seen  = false;
    std::vector<int64_t>  after_fresh;
    const auto            start = Clock::now();
    while (Clock::now() - start < Ms(1500)) {
        if (!slow.next(m, Ms(50)) || m.opcode != 1) {
            continue;
        }
        const bool state = messageType(m.payload) == "state";
        const int64_t age = state ? stateAgeMs(m.payload) : 0;
        if (!fresh_seen) {
            stale_bytes += m.payload.size();
            if (state && age > 250) {
                ++stale_first;
            } else if (state) {
                fresh_seen = true;
            }
        } else if (state) {
            after_fresh.push_back(age);
        }
    }
    f.service->stop();
    f.system->stop();
    EXPECT_TRUE(fresh_seen);
    EXPECT_LE(stale_first, 2);                  // two unconfirmed states at most
    EXPECT_LE(stale_bytes, 64u * 1024u + 24u * 1024u);   // the window and one frame
    ASSERT_GE(after_fresh.size(), 10u);
    EXPECT_LT(percentile(after_fresh, 1.0), 250);
}

// The review case: a reader taking about 60 KB/s, slower than state plus
// diag at 4 Hz (about 110 KB/s), measured from the first message. Before
// the fix the median state was about 2 s old at receipt; with flow control
// off it is still 0.3 to 0.4 s. Host numbers (Windows loopback, this
// build): p50 1 to 16 ms, p95 about 185 ms, max about 270 ms.
TEST(InspectionFeed, RateLimitedReaderGetsFreshStatesAtTheDefaults) {
    Feed f("<Inspection enabled=\"true\" port=\"0\" preview_hz=\"0\"/>");
    ASSERT_NE(f.service, nullptr);
    std::string err;
    ASSERT_TRUE(f.system->start(err)) << err;
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));

    std::vector<int64_t> ages;
    int                  diags = 0;
    WsMessage            m;
    const auto           start = Clock::now();
    while (Clock::now() - start < Ms(6000)) {
        if (!ws.next(m, Ms(50), 600)) {
            continue;
        }
        if (m.opcode == 1 && messageType(m.payload) == "state") {
            ages.push_back(stateAgeMs(m.payload));
        }
        diags += m.opcode == 1 && messageType(m.payload) == "diag";
        // about 60 KB/s: time in proportion to each message taken
        std::this_thread::sleep_for(std::chrono::microseconds(
            static_cast<int64_t>(m.payload.size()) * 1000000 / 60000));
    }
    const InspectionServiceStats s = f.service->stats();
    f.service->stop();
    f.system->stop();
    ASSERT_GE(ages.size(), 60u);
    EXPECT_EQ(s.flow_control_clients, 1u);
    EXPECT_GE(diags, 2);   // diagnostics still arrive, paced
    // a state can wait behind one diag in flight (about 20 KB, 330 ms here);
    // the bounds leave room for a loaded machine
    EXPECT_LT(percentile(ages, 0.5), 100) << "p95 " << percentile(ages, 0.95);
    EXPECT_LT(percentile(ages, 0.95), 400) << "max " << percentile(ages, 1.0);
    EXPECT_LT(percentile(ages, 1.0), 1000);
}

// System::reset publishes the power-on state before it counts the reset.
// A state or diag must never label a pose of the old session with the new
// count: for counts r < r2, every state labeled r2 carries a newer odometry
// epoch than any valid state labeled r (the power-on state is invalid). The
// interleaving is a few instructions wide, so this is a guard under load,
// not a reproduction.
TEST(InspectionFeed, ResetsUnderLoadNeverLabelAnOldPoseWithTheNewSession) {
    Feed f("<Inspection enabled=\"true\" port=\"0\" state_hz=\"60\" diag_hz=\"20\" "
           "preview_hz=\"0\"/>");
    ASSERT_NE(f.service, nullptr);
    std::string err;
    ASSERT_TRUE(f.system->start(err)) << err;
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    ws.sendText("{\"type\":\"subscribe\",\"state_hz\":60}");

    std::atomic<bool> stop{false};
    std::thread       resetter([&] {
        while (!stop.load()) {
            std::this_thread::sleep_for(Ms(25));
            f.system->reset();
        }
    });
    struct Seen {
        long long max_valid_epoch = -1;
        long long min_epoch       = -1;
    };
    std::map<long long, Seen> by_reset;
    long long                 last_reset = -1;
    int                       labeled    = 0;
    WsMessage                 m;
    const auto                start = Clock::now();
    while (Clock::now() - start < Ms(3000)) {
        if (!ws.next(m, Ms(50)) || m.opcode != 1) {
            continue;
        }
        const std::string type = messageType(m.payload);
        if (type != "state" && type != "diag") {
            continue;
        }
        long long reset = 0, epoch = 0;
        const std::size_t robot = m.payload.find("\"robot\":{");
        ASSERT_NE(robot, std::string::npos);
        ASSERT_TRUE(jsonNumber(m.payload, "reset_count", reset));
        ASSERT_TRUE(jsonNumber(m.payload, "odometry_epoch", epoch, robot));
        const bool valid = m.payload.compare(robot + 9, 13, "\"valid\":true,") == 0;
        if (type == "state") {
            EXPECT_GE(reset, last_reset) << "state labels go back";
            last_reset = reset;
        }
        Seen& s = by_reset[reset];
        if (valid) {
            s.max_valid_epoch = std::max(s.max_valid_epoch, epoch);
        }
        s.min_epoch = s.min_epoch < 0 ? epoch : std::min(s.min_epoch, epoch);
        ++labeled;
    }
    stop = true;
    resetter.join();
    f.service->stop();
    f.system->stop();
    EXPECT_GE(labeled, 50);
    EXPECT_GE(by_reset.size(), 10u);
    for (auto a = by_reset.begin(); a != by_reset.end(); ++a) {
        for (auto b = std::next(a); b != by_reset.end(); ++b) {
            if (a->second.max_valid_epoch >= 0) {
                EXPECT_LT(a->second.max_valid_epoch, b->second.min_epoch)
                    << "reset " << b->first << " labels a pose of reset " << a->first;
            }
        }
    }
}

TEST(InspectionFeed, TelemetryGoesToSubscribersDecoded) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    expectOpening(ws, f.system->sessionId());

    const auto post = [&](int16_t roll_cdeg) {
        translagatr::BrainRequest req;
        req.op                   = translagatr::kOpTelemetry;
        req.session              = 5;
        req.request_id           = 1;
        req.telemetry.flags      = translagatr::kTelemetryAttitude;
        req.telemetry.roll_cdeg  = roll_cdeg;
        req.telemetry.pitch_cdeg = 50;
        uint8_t frame[translagatr::kMaxFrameLen];
        ASSERT_GT(translagatr::encodeBrainRequest(req, frame, sizeof(frame)), 0u);
        DiagBrainTelemetry body;
        body.session = 5;
        body.len     = translagatr::kTelemetryBodyLen;
        std::memcpy(body.body, frame + 4 + translagatr::kBrainRequestHeaderLen, body.len);
        DiagRecord r;
        r.kind    = DiagKind::kBrainTelemetry;
        r.payload = body;
        f.system->diagHub().post(r);
    };
    post(300);
    auto msgs = collect(ws, Ms(800));
    ASSERT_EQ(countType(msgs, "telemetry"), 1u);
    for (const Received& r : msgs) {
        if (r.type == "telemetry") {
            EXPECT_NE(r.payload.find("\"attitude\":{\"roll_deg\":3,\"pitch_deg\":0.5}"),
                      std::string::npos)
                << r.payload;
        }
    }
    // no new record: nothing more
    msgs = collect(ws, Ms(400));
    EXPECT_EQ(countType(msgs, "telemetry"), 0u);

    ws.sendText("{\"type\":\"subscribe\",\"telemetry\":false}");
    collect(ws, Ms(200));
    post(400);
    msgs = collect(ws, Ms(600));
    EXPECT_EQ(countType(msgs, "telemetry"), 0u);
}

TEST(InspectionFeed, CaptureRoutesAndMethodChecks) {
    Feed f;
    ASSERT_NE(f.service, nullptr);
    const int port = f.service->port();

    HttpReply status = httpRequest(port, "GET", "/api/capture/status");
    ASSERT_TRUE(status.ok);
    EXPECT_EQ(status.status, 200);
    EXPECT_NE(status.headers["content-type"].find("application/json"), std::string::npos);
    EXPECT_NE(status.body.find("\"available\":"), std::string::npos) << status.body;

    HttpReply start = httpRequest(port, "POST", "/api/capture/start?pre_s=1&post_s=1");
    ASSERT_TRUE(start.ok);
    EXPECT_NE(start.body.find("\"ok\":"), std::string::npos) << start.body;

    // POST reaches only the capture routes; other methods never reach a handler
    HttpReply post = httpRequest(port, "POST", "/api/snapshot");
    EXPECT_EQ(post.status, 405);
    EXPECT_EQ(post.headers["allow"], "GET");
    HttpReply put = httpRequest(port, "PUT", "/api/hello");
    EXPECT_EQ(put.status, 405);
    EXPECT_EQ(put.headers["allow"], "GET, POST");
    HttpReply big = httpRequest(port, "POST", "/api/capture/start", std::string(70000, 'x'));
    EXPECT_EQ(big.status, 413);
}
