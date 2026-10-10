// http_server_gtest.cpp
// The server over loopback: static files and /api/fields over HTTP, the WebSocket handshake,
// hello first then pose and status through the live hub, and latest-wins with bounded memory for
// a client that does not read.

#include "test_client.h" // winsock2 before anything that pulls in windows.h

#include "receiver/server/http_server.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "receiver/app/live_feed.h"
#include "receiver/server/static_site.h"
#include "receiver/sources/fakes/brain_stdio.h"

namespace spectagatr::receiver::server
{
namespace
{

namespace fs = std::filesystem;
using test_client::Ms;
using test_client::WsClient;

TEST(WebSocket, AcceptKeyMatchesRfc6455) {
    EXPECT_EQ(websocketAccept("dGhlIHNhbXBsZSBub25jZQ=="), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
}

TEST(WebSocket, UrlDecodeKeepsPlusAndBadEscapes) {
    EXPECT_EQ(urlDecode("/a%20b+c"), "/a b+c");
    EXPECT_EQ(urlDecode("/%2e%2E/x"), "/../x");
    EXPECT_EQ(urlDecode("/bad%zz%4"), "/bad%zz%4");
}

class Loopback : public ::testing::Test {
  protected:
    fs::path                       root;
    std::unique_ptr<HttpServer>    server;
    std::unique_ptr<app::LiveFeed> live;

    void SetUp() override {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        root             = fs::temp_directory_path() / ("spectagatr-loop-" + std::to_string(stamp));
        fs::create_directories(root / "web");
        fs::create_directories(root / "fields");
        std::ofstream(root / "web" / "index.html") << "<html>viewer</html>";
        std::ofstream(root / "fields" / "override.json") << "{}";
        std::ofstream(root / "secret.txt") << "outside";
    }

    void TearDown() override {
        if (server) {
            server->stop(); // its close handlers still reach the live feed
        }
        live.reset();
        std::error_code error;
        fs::remove_all(root, error);
    }

    void start(HttpServerConfig config = {}) {
        config.port = 0;
        SiteConfig site;
        site.web_dir    = root / "web";
        site.fields_dir = root / "fields";
        server          = std::make_unique<HttpServer>(
            config, [site](const HttpRequest& request) { return serveSite(site, request); });
        app::FeedSettings settings;
        settings.hello.fields = {"override"};
        live                  = std::make_unique<app::LiveFeed>(*server, settings);
        server->setWebSocketHandlers({[this](app::ClientId client) { live->onOpen(client); },
                                      [this](app::ClientId client) { live->onClose(client); }});
        std::string err;
        ASSERT_TRUE(server->start(err)) << err;
    }

    void waitForClients(std::size_t count) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (server->clients().size() != count && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void sendPose(sources::fakes::FakeBrainStdio& brain, int x_mm, double rx_ms) {
        sources::SourceChunk chunk;
        chunk.src   = sources::SourceKind::kController;
        chunk.rx_us = static_cast<sources::ReceiverUs>(rx_ms * 1000.0);
        chunk.bytes = brain.pose(x_mm, 0, 0, 0);
        live->onChunk(chunk);
    }
};

TEST_F(Loopback, StaticFilesFieldsAndErrorsOverHttp) {
    start();
    const test_client::HttpReply index = test_client::httpGet(server->port(), "/");
    EXPECT_EQ(index.status, 200);
    EXPECT_EQ(index.body, "<html>viewer</html>");
    EXPECT_NE(index.head.find("Content-Type: text/html"), std::string::npos);
    const test_client::HttpReply fields = test_client::httpGet(server->port(), "/api/fields");
    EXPECT_EQ(fields.status, 200);
    EXPECT_EQ(fields.body, "{\"fields\":[\"override\"],\"default\":\"override\"}");
    EXPECT_EQ(test_client::httpGet(server->port(), "/fields/override.json").status, 200);
    EXPECT_EQ(test_client::httpGet(server->port(), "/missing.js").status, 404);
    const test_client::HttpReply traversal =
        test_client::httpGet(server->port(), "/%2e%2e/secret.txt");
    EXPECT_EQ(traversal.status, 403);
    EXPECT_EQ(traversal.body.find("outside"), std::string::npos);
    EXPECT_EQ(test_client::httpGet(server->port(), "/", "POST").status, 405);
}

TEST_F(Loopback, HandshakeHelloThenPoseThenStatus) {
    start();
    WsClient client;
    ASSERT_TRUE(client.connect(server->port()));
    EXPECT_NE(client.head.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo="),
              std::string::npos);
    const std::optional<test_client::WsMessage> hello = client.next(Ms(3000));
    ASSERT_TRUE(hello);
    EXPECT_EQ(hello->opcode, 1);
    EXPECT_EQ(hello->payload.rfind("{\"type\":\"hello\",\"protocol\":1,", 0), 0u);
    EXPECT_NE(hello->payload.find("\"fields\":[\"override\"]"), std::string::npos);
    waitForClients(1);

    sources::SourceStatus open;
    open.src    = sources::SourceKind::kController;
    open.state  = sources::PortState::kOpen;
    open.port   = "COM7";
    open.detail = "controller: Brain linked by radio";
    live->onStatus(open);

    sources::fakes::FakeBrainStdio brain(0x5A3C19E2);
    sendPose(brain, 1500, 1000.0);
    const std::optional<std::string> pose = client.nextContaining("\"type\":\"pose\"", Ms(3000));
    ASSERT_TRUE(pose);
    EXPECT_NE(pose->find("\"session\":\"5A3C19E2\""), std::string::npos);
    EXPECT_NE(pose->find("\"x\":1.5,"), std::string::npos);
    EXPECT_NE(pose->find("\"restart\":true"), std::string::npos);
    EXPECT_NE(pose->find("\"source\":\"controller\""), std::string::npos);

    live->tick(1010.0);
    const std::optional<std::string> status =
        client.nextContaining("\"type\":\"status\"", Ms(3000));
    ASSERT_TRUE(status);
    EXPECT_NE(status->find("\"link\":\"open\""), std::string::npos);
    EXPECT_NE(status->find("\"port\":\"COM7\""), std::string::npos);
    EXPECT_NE(status->find("\"stale\":false"), std::string::npos);
    EXPECT_NE(status->find("\"pose_age_ms\":10,"), std::string::npos);
    EXPECT_NE(status->find("\"poses\":1,"), std::string::npos);

    // Nothing new: the next status waits for the 250 ms period.
    live->tick(1100.0);
    EXPECT_FALSE(client.nextContaining("\"type\":\"status\"", Ms(150)));
    live->tick(1300.0);
    EXPECT_TRUE(client.nextContaining("\"type\":\"status\"", Ms(3000)));
    // Stale is a change: sent at once.
    live->tick(1800.0);
    const std::optional<std::string> stale = client.nextContaining("\"stale\":true", Ms(3000));
    ASSERT_TRUE(stale);
}

TEST_F(Loopback, ALateClientGetsHelloThenTheNewestState) {
    start();
    sources::fakes::FakeBrainStdio brain(0x99);
    sendPose(brain, 100, 10.0);
    sendPose(brain, 200, 20.0);
    live->tick(25.0);
    WsClient client;
    ASSERT_TRUE(client.connect(server->port()));
    const std::optional<test_client::WsMessage> first = client.next(Ms(3000));
    ASSERT_TRUE(first);
    EXPECT_NE(first->payload.find("\"type\":\"hello\""), std::string::npos);
    const std::optional<std::string> pose = client.nextContaining("\"type\":\"pose\"", Ms(3000));
    ASSERT_TRUE(pose);
    EXPECT_NE(pose->find("\"x\":0.2,"), std::string::npos); // the newest, not a backlog
}

TEST_F(Loopback, ARestartPoseIsNeverReplacedAway) {
    start();
    WsClient client;
    ASSERT_TRUE(client.connect(server->port()));
    ASSERT_TRUE(client.next(Ms(3000))); // hello
    waitForClients(1);
    sources::fakes::FakeBrainStdio brain(0x1);
    for (int i = 0; i < 50; ++i) {
        sendPose(brain, i, i);
    }
    brain.restart(0x2);
    sendPose(brain, 1000, 60.0);
    for (int i = 0; i < 50; ++i) {
        sendPose(brain, 2000 + i, 70.0 + i);
    }
    const std::optional<std::string> restart =
        client.nextContaining("\"session\":\"00000002\",\"epoch\":0,\"seq\":0,", Ms(3000));
    ASSERT_TRUE(restart);
    EXPECT_NE(restart->find("\"restart\":true"), std::string::npos);
    EXPECT_TRUE(client.nextContaining("\"x\":2.049,", Ms(3000))); // and the newest after it
}

TEST_F(Loopback, AClientThatDoesNotReadGetsLatestOnlyThenIsClosed) {
    HttpServerConfig config;
    config.send_buffer_bytes      = 4096;
    config.close_after_stalled_ms = 400;
    start(config);
    WsClient client;
    ASSERT_TRUE(client.connect(server->port(), 4096));
    waitForClients(1);
    const HttpServer::ClientId id = server->clients().front();

    const std::string   filler(8000, 'x');
    std::size_t         most_queued = 0;
    HttpServer::Enqueue last        = HttpServer::Enqueue::kQueued;
    const auto          deadline    = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    // Never read: the kernel buffers fill, then each new frame replaces the unsent one.
    while (server->stats().messages_replaced < 20 && std::chrono::steady_clock::now() < deadline) {
        last = server->sendLatest(
            id, "pose", 0, HttpServer::textFrame("{\"type\":\"pose\",\"pad\":\"" + filler + "\"}"));
        most_queued = std::max(most_queued, server->queued(id));
        if (last == HttpServer::Enqueue::kUnknownClient) {
            break;
        }
    }
    EXPECT_GE(server->stats().messages_replaced, 20u);
    // At most the frame in flight and one waiting, plus small frames such as an
    // unflushed hello under load: never a backlog.
    EXPECT_LE(most_queued, 2u * (filler.size() + 32u) + 1024u);

    // Still blocked: closed after close_after_stalled_ms.
    const auto close_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!server->clients().empty() && std::chrono::steady_clock::now() < close_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_TRUE(server->clients().empty());
    EXPECT_EQ(server->stats().clients_closed_stalled, 1u);
}

} // namespace
} // namespace spectagatr::receiver::server
