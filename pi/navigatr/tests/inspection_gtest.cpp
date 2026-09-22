// inspection_gtest.cpp
// The inspection service end to end on loopback: a real System built from
// the synthetic rig, the real HTTP and WebSocket server on an ephemeral
// port, and a minimal blocking client on the test side. Routes, the feed
// (hello, snapshots, bound JPEG previews), per-client preview budgets, the
// slow-client bound, reconnection, shutdown order and the config checks.
// Timing assertions are deliberately loose: Windows timers are coarse and
// the suite shares the machine.

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// the test target has no stb include path; the core library keeps it private
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO
#define STBI_ONLY_JPEG
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#include "../third_party/stb/stb_image.h"
#pragma GCC diagnostic pop

#include "core/host_clock.h"
#include "inspection/http_server.h"
#include "inspection/inspection_service.h"
#include "inspection/viewer_assets.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

// ---- configuration ----------------------------------------------------------

// The synthetic rig of synthetic_rig_gtest.cpp with an Inspection element.
std::string rigXml(const std::string& inspection) {
    return R"(
<System>
    <Loop rate_hz="100"/>
    )" + inspection +
           R"(
    <Resources>
        <Resource id="robot_geometry" type="robot_frame_map">
            <Frame id="front_camera_engineering" parent_frame_id="robot_body"
                   calibration_status="verified">
                <PoseOfChildInParent x_m="0.15" y_m="0" z_m="0.20"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </Frame>
        </Resource>
        <Resource id="field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified" x_m="1.7832" y_m="1.7832" heading_deg="0"/>
                <TagMount instance_id="center_east" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="0"/>
                </TagMount>
                <TagMount instance_id="center_north" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="0" y_m="0.05" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="90"/>
                </TagMount>
                <TagMount instance_id="center_west" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="-0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="180"/>
                </TagMount>
                <TagMount instance_id="center_south" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="0" y_m="-0.05" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="-90"/>
                </TagMount>
            </Landmark>
            <Landmark id="far_goal">
                <NominalPose calibration_status="verified" x_m="3.0" y_m="1.7832" heading_deg="0"/>
                <TagMount instance_id="far_west" calibration_status="verified"
                          family="tagCircle21h7" observed_id="1" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="-0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="180"/>
                </TagMount>
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
                        period_s="30" facing="center" start_deg="180" hold_s="1.5"/>
            <Displace landmark_id="center_goal" dx_m="0.06" dy_m="-0.04" dyaw_deg="5"/>
            <Telemetry tick_hz="50" device_offset_ms="5000" gyro_bias_mdps="800"/>
            <Attitude mode="measured" rock_deg="3" period_s="2.5"/>
            <Camera frame_id="front_camera_engineering" robot_frames_resource_id="robot_geometry"
                    width_px="640" height_px="480" fx_px="600" fy_px="600" cx_px="320" cy_px="240"
                    k1="-0.1" k2="0.02" frame_rate_hz="10" latency_ms="30"/>
            <Output id="encoder_a" wheel_id="left_wheel"/>
            <Output id="encoder_b" wheel_id="right_wheel"/>
            <Output id="encoder_c" wheel_id="rear_wheel"/>
            <Output id="imu" channel="imu"/>
            <Output id="attitude" channel="attitude"/>
            <Output id="frame" channel="camera"/>
        </Resource>
        <Resource id="tag_detector" type="apriltag_detector">
            <Family name="tagCircle21h7" detection_size_m="0.03"/>
            <Detector quad_decimate="1.0" nthreads="2"/>
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
        <Sensor id="front_camera" type="camera_frame">
            <Source resource_id="rig" output_id="frame"/>
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
                <HeadingConstraint sensor_id="robot_imu" bias_samples="40"
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
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="tag_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association>
                    <Observations observation_id="tag_observations"/>
                    <FieldMap resource_id="field"/>
                    <RobotFrames resource_id="robot_geometry"/>
                    <Attitude policy="assume_level"/>
                    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
                           ambiguity_margin_m="0.15" max_range_m="3.0"
                           min_decision_margin="10" max_hamming="0"
                           min_facing_cos="0.1" min_projected_size_px="8"/>
                    <Output association_id="landmark_pose_observations"/>
                    <Trace association_id="tag_association_trace"/>
                </Association>
                <LandmarkEstimation commit="always" blend="0.5"/>
            </Estimator>
        </WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
}

const char* kDefaultInspection =
    "<Inspection enabled=\"true\" port=\"0\" snapshot_hz=\"20\" preview_hz=\"10\" "
    "preview_quality=\"60\" preview_max_width=\"320\" max_clients=\"4\" "
    "client_buffer_kb=\"1024\" stall_close_ms=\"10000\"/>";

struct Fixture {
    FunctionRegistry                   functions;
    std::unique_ptr<System>            system;   // outlives the service
    std::unique_ptr<InspectionService> service;
    int64_t                            now_ms = 0;

    explicit Fixture(const char* inspection = kDefaultInspection) {
        registerAll(functions);
        std::string err;
        system = System::buildFromString(rigXml(inspection).c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system != nullptr) {
            service = InspectionService::create(*system, system->inspection(), err);
            EXPECT_NE(service, nullptr) << err;
        }
    }

    bool start() {
        std::string err;
        const bool  ok = service != nullptr && service->start(err);
        EXPECT_TRUE(ok) << err;
        return ok;
    }

    void step(int cycles) {
        for (int i = 0; i < cycles; ++i) {
            now_ms += 10;
            system->step(hostTime(now_ms));
        }
    }
};

// ---- sockets ----------------------------------------------------------------------

#ifdef _WIN32
using Sock = SOCKET;
const Sock kBadSock = INVALID_SOCKET;
void       closeSock(Sock s) { closesocket(s); }
struct SocketsInit {
    SocketsInit() {
        WSADATA d;
        WSAStartup(MAKEWORD(2, 2), &d);
    }
    ~SocketsInit() { WSACleanup(); }
};
#else
using Sock = int;
const Sock kBadSock = -1;
void       closeSock(Sock s) { ::close(s); }
struct SocketsInit {};
#endif

[[maybe_unused]] SocketsInit g_sockets;

class TestConnection
{
public:
    ~TestConnection() { close(); }

    // A small receive buffer makes the kernel stop absorbing quickly, so a
    // client that never reads stalls the server within the test budget.
    bool connect(int port, int recv_buffer_bytes = 0) {
        sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == kBadSock) {
            return false;
        }
        if (recv_buffer_bytes > 0) {
            setsockopt(sock_, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&recv_buffer_bytes),
                       sizeof(recv_buffer_bytes));
        }
        sockaddr_in a;
        std::memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET;
        a.sin_port   = htons(static_cast<uint16_t>(port));
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        if (::connect(sock_, reinterpret_cast<const sockaddr*>(&a), sizeof(a)) != 0) {
            close();
            return false;
        }
        const int one = 1;
        setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one),
                   sizeof(one));
        return true;
    }

    bool sendAll(const std::string& bytes) {
        std::size_t off = 0;
        while (off < bytes.size()) {
            const auto n = ::send(sock_, bytes.data() + off,
                                  static_cast<int>(bytes.size() - off), 0);
            if (n <= 0) {
                return false;
            }
            off += static_cast<std::size_t>(n);
        }
        return true;
    }

    // Appends what arrives within timeout. False on timeout or close.
    bool readSome(Ms timeout) {
        if (sock_ == kBadSock || closed) {
            return false;
        }
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(sock_, &rd);
        timeval tv;
        tv.tv_sec  = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
#ifdef _WIN32
        const int r = select(0, &rd, nullptr, nullptr, &tv);
#else
        const int r = select(sock_ + 1, &rd, nullptr, nullptr, &tv);
#endif
        if (r <= 0) {
            return false;
        }
        char       buf[64 * 1024];
        const auto n = ::recv(sock_, buf, sizeof(buf), 0);
        if (n <= 0) {
            closed = true;
            return false;
        }
        buffer.append(buf, static_cast<std::size_t>(n));
        return true;
    }

    void close() {
        if (sock_ != kBadSock) {
            closeSock(sock_);
            sock_ = kBadSock;
        }
    }

    std::string buffer;
    bool        closed = false;

private:
    Sock sock_ = kBadSock;
};

struct HttpReply {
    bool                               ok     = false;
    int                                status = 0;
    std::map<std::string, std::string> headers;   // lower-case names
    std::string                        body;
};

HttpReply httpRequest(int port, const std::string& method, const std::string& path) {
    HttpReply      reply;
    TestConnection c;
    if (!c.connect(port)) {
        return reply;
    }
    c.sendAll(method + " " + path + " HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n");
    const auto deadline = Clock::now() + Ms(10000);
    while (!c.closed && Clock::now() < deadline) {
        c.readSome(Ms(200));
    }
    const std::size_t head_end = c.buffer.find("\r\n\r\n");
    if (head_end == std::string::npos) {
        return reply;
    }
    const std::string head = c.buffer.substr(0, head_end);
    reply.body             = c.buffer.substr(head_end + 4);
    std::size_t pos        = head.find("\r\n");
    std::string line       = head.substr(0, pos);
    if (line.size() > 12) {
        reply.status = std::atoi(line.substr(9, 3).c_str());
    }
    while (pos != std::string::npos) {
        const std::size_t next  = head.find("\r\n", pos + 2);
        const std::string hline = head.substr(pos + 2, next == std::string::npos
                                                          ? std::string::npos
                                                          : next - pos - 2);
        const std::size_t colon = hline.find(':');
        if (colon != std::string::npos) {
            std::string name = hline.substr(0, colon);
            for (char& ch : name) {
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            }
            std::size_t v = colon + 1;
            while (v < hline.size() && hline[v] == ' ') {
                ++v;
            }
            reply.headers[name] = hline.substr(v);
        }
        pos = next;
    }
    reply.ok = reply.status != 0;
    return reply;
}

struct WsMessage {
    int         opcode = -1;
    std::string payload;
};

class WsClient
{
public:
    bool connect(int port, int recv_buffer_bytes = 0) {
        if (!conn_.connect(port, recv_buffer_bytes)) {
            return false;
        }
        // the RFC 6455 sample key: the accept value is known
        conn_.sendAll("GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
                      "Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                      "Sec-WebSocket-Version: 13\r\n\r\n");
        const auto deadline = Clock::now() + Ms(5000);
        while (conn_.buffer.find("\r\n\r\n") == std::string::npos && !conn_.closed &&
               Clock::now() < deadline) {
            conn_.readSome(Ms(200));
        }
        const std::size_t end = conn_.buffer.find("\r\n\r\n");
        if (end == std::string::npos) {
            return false;
        }
        const std::string head = conn_.buffer.substr(0, end);
        conn_.buffer.erase(0, end + 4);
        if (head.find("HTTP/1.1 101") != 0) {
            return false;
        }
        std::string lower = head;
        for (char& ch : lower) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        const std::size_t accept = lower.find("sec-websocket-accept:");
        if (accept == std::string::npos) {
            return false;
        }
        return head.find("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", accept) != std::string::npos;
    }

    // The next complete message within timeout.
    bool next(WsMessage& out, Ms timeout) {
        const auto deadline = Clock::now() + timeout;
        for (;;) {
            if (parseFrame(out)) {
                return true;
            }
            if (conn_.closed) {
                return false;
            }
            const auto now = Clock::now();
            if (now >= deadline) {
                return false;
            }
            const auto remaining = std::chrono::duration_cast<Ms>(deadline - now);
            conn_.readSome(std::min(remaining, Ms(100)));
        }
    }

    void sendText(const std::string& text) { sendFrame(0x1, text, true); }
    void sendClose() { sendFrame(0x8, std::string("\x03\xe8", 2), true); }
    // A protocol violation: client frames must be masked.
    void sendUnmaskedText(const std::string& text) { sendFrame(0x1, text, false); }
    void close() { conn_.close(); }
    bool closed() const { return conn_.closed; }

private:
    void sendFrame(uint8_t opcode, const std::string& payload, bool masked) {
        std::string f;
        f.push_back(static_cast<char>(0x80 | opcode));
        const std::size_t len      = payload.size();
        const uint8_t     mask_bit = masked ? 0x80 : 0x00;
        if (len < 126) {
            f.push_back(static_cast<char>(mask_bit | len));
        } else {
            f.push_back(static_cast<char>(mask_bit | 126));
            f.push_back(static_cast<char>(len >> 8));
            f.push_back(static_cast<char>(len & 0xff));
        }
        if (!masked) {
            f += payload;
            conn_.sendAll(f);
            return;
        }
        const uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
        f.append(reinterpret_cast<const char*>(mask), 4);
        for (std::size_t i = 0; i < len; ++i) {
            f.push_back(static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]));
        }
        conn_.sendAll(f);
    }

    bool parseFrame(WsMessage& out) {
        std::string& b = conn_.buffer;
        if (b.size() < 2) {
            return false;
        }
        const uint8_t b0  = static_cast<uint8_t>(b[0]);
        const uint8_t b1  = static_cast<uint8_t>(b[1]);
        uint64_t      len = b1 & 0x7f;
        std::size_t   pos = 2;
        if (len == 126) {
            if (b.size() < 4) {
                return false;
            }
            len = (static_cast<uint64_t>(static_cast<uint8_t>(b[2])) << 8) |
                  static_cast<uint8_t>(b[3]);
            pos = 4;
        } else if (len == 127) {
            if (b.size() < 10) {
                return false;
            }
            len = 0;
            for (int i = 0; i < 8; ++i) {
                len = (len << 8) | static_cast<uint8_t>(b[2 + i]);
            }
            pos = 10;
        }
        EXPECT_EQ(b1 & 0x80, 0) << "server frames must not be masked";
        EXPECT_NE(b0 & 0x80, 0) << "server frames must not be fragmented";
        const std::size_t total = pos + static_cast<std::size_t>(len);
        if (b.size() < total) {
            return false;
        }
        out.opcode  = b0 & 0x0f;
        out.payload = b.substr(pos, static_cast<std::size_t>(len));
        b.erase(0, total);
        return true;
    }

    TestConnection conn_;
};

// ---- json picking -------------------------------------------------------------

// Value of the first "key":"..." at or after from; empty when absent.
std::string jsonString(const std::string& json, const std::string& key, std::size_t from = 0) {
    const std::string needle = "\"" + key + "\":\"";
    const std::size_t pos    = json.find(needle, from);
    if (pos == std::string::npos) {
        return {};
    }
    const std::size_t start = pos + needle.size();
    const std::size_t end   = json.find('"', start);
    return end == std::string::npos ? std::string() : json.substr(start, end - start);
}

bool jsonNumber(const std::string& json, const std::string& key, long long& out,
                std::size_t from = 0) {
    const std::string needle = "\"" + key + "\":";
    const std::size_t pos    = json.find(needle, from);
    if (pos == std::string::npos) {
        return false;
    }
    out = std::atoll(json.c_str() + pos + needle.size());
    return true;
}

bool jsonBool(const std::string& json, const std::string& key, std::size_t from = 0) {
    return json.find("\"" + key + "\":true", from) != std::string::npos;
}

std::string messageType(const std::string& json) { return jsonString(json, "type"); }

struct FrameIdentity {
    std::string camera;
    long long   epoch    = -1;
    long long   sequence = -1;
    bool        set      = false;

    bool operator==(const FrameIdentity& o) const {
        return set && o.set && camera == o.camera && epoch == o.epoch && sequence == o.sequence;
    }
};

// (camera, epoch, sequence) of detection_frames[0] in a snapshot.
FrameIdentity snapshotFrameIdentity(const std::string& snapshot) {
    FrameIdentity     id;
    const std::size_t pos = snapshot.find("\"detection_frames\":[{");
    if (pos == std::string::npos) {
        return id;
    }
    id.camera = jsonString(snapshot, "camera", pos);
    id.set    = jsonNumber(snapshot, "epoch", id.epoch, pos) &&
             jsonNumber(snapshot, "sequence", id.sequence, pos) && !id.camera.empty();
    return id;
}

struct BinaryFrame {
    std::string   header;
    std::string   jpeg;
    FrameIdentity identity;
};

bool parseBinaryFrame(const std::string& payload, BinaryFrame& out) {
    if (payload.size() < 4) {
        return false;
    }
    const uint32_t len = static_cast<uint8_t>(payload[0]) |
                         (static_cast<uint32_t>(static_cast<uint8_t>(payload[1])) << 8) |
                         (static_cast<uint32_t>(static_cast<uint8_t>(payload[2])) << 16) |
                         (static_cast<uint32_t>(static_cast<uint8_t>(payload[3])) << 24);
    if (payload.size() < 4 + len) {
        return false;
    }
    out.header          = payload.substr(4, len);
    out.jpeg            = payload.substr(4 + len);
    out.identity.camera = jsonString(out.header, "camera");
    out.identity.set    = jsonNumber(out.header, "epoch", out.identity.epoch) &&
                       jsonNumber(out.header, "sequence", out.identity.sequence);
    return messageType(out.header) == "frame";
}

const EmbeddedAsset* embeddedAsset(const std::string& path) {
    std::size_t          count  = 0;
    const EmbeddedAsset* assets = embeddedViewerAssets(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (path == assets[i].path) {
            return &assets[i];
        }
    }
    return nullptr;
}

} // namespace

// ---- 1. HTTP routes -----------------------------------------------------------------

TEST(Inspection, HttpRoutesServeDocumentsAndTheEmbeddedViewer) {
    Fixture f;
    ASSERT_TRUE(f.start());
    const int port = f.service->port();
    ASSERT_GT(port, 0);
    EXPECT_EQ(f.service->url(), "http://127.0.0.1:" + std::to_string(port) + "/");

    HttpReply hello = httpRequest(port, "GET", "/api/hello");
    ASSERT_TRUE(hello.ok);
    EXPECT_EQ(hello.status, 200);
    EXPECT_NE(hello.headers["content-type"].find("application/json"), std::string::npos);
    EXPECT_EQ(hello.headers["cache-control"], "no-store");
    EXPECT_EQ(hello.headers["connection"], "close");
    EXPECT_EQ(std::atoi(hello.headers["content-length"].c_str()),
              static_cast<int>(hello.body.size()));
    EXPECT_NE(hello.body.find("\"contract\":\"navigatr.inspect/1\""), std::string::npos);
    EXPECT_EQ(messageType(hello.body), "hello");
    EXPECT_NE(hello.body.find("\"id\":\"" + f.system->sessionId() + "\""), std::string::npos);
    EXPECT_NE(hello.body.find("\"fields\":["), std::string::npos);
    EXPECT_NE(hello.body.find("\"resource_id\":\"field\""), std::string::npos);

    const EmbeddedAsset* index = embeddedAsset("index.html");
    ASSERT_NE(index, nullptr);
    HttpReply root = httpRequest(port, "GET", "/");
    ASSERT_TRUE(root.ok);
    EXPECT_EQ(root.status, 200);
    EXPECT_NE(root.headers["content-type"].find("text/html"), std::string::npos);
    EXPECT_EQ(root.body, std::string(reinterpret_cast<const char*>(index->data), index->size));

    const EmbeddedAsset* three = embeddedAsset("vendor/three.module.min.js");
    ASSERT_NE(three, nullptr);
    HttpReply vendor = httpRequest(port, "GET", "/vendor/three.module.min.js");
    ASSERT_TRUE(vendor.ok);
    EXPECT_EQ(vendor.status, 200);
    EXPECT_NE(vendor.headers["content-type"].find("text/javascript"), std::string::npos);
    EXPECT_EQ(vendor.body.size(), three->size);
    EXPECT_EQ(std::memcmp(vendor.body.data(), three->data, three->size), 0);

    HttpReply nope = httpRequest(port, "GET", "/nope");
    ASSERT_TRUE(nope.ok);
    EXPECT_EQ(nope.status, 404);
    HttpReply traversal = httpRequest(port, "GET", "/../CMakeLists.txt");
    ASSERT_TRUE(traversal.ok);
    EXPECT_EQ(traversal.status, 404);

    HttpReply health = httpRequest(port, "GET", "/api/health");
    ASSERT_TRUE(health.ok);
    EXPECT_EQ(health.status, 200);
    EXPECT_TRUE(jsonBool(health.body, "ok"));
    EXPECT_TRUE(jsonBool(health.body, "running"));
    long long health_port = 0;
    EXPECT_TRUE(jsonNumber(health.body, "port", health_port));
    EXPECT_EQ(health_port, port);

    HttpReply snapshot = httpRequest(port, "GET", "/api/snapshot");
    ASSERT_TRUE(snapshot.ok);
    EXPECT_EQ(snapshot.status, 200);
    EXPECT_EQ(messageType(snapshot.body), "snapshot");
    EXPECT_NE(snapshot.body.find("\"workers\":{"), std::string::npos);

    // no detection frame yet: nothing to encode
    HttpReply frame = httpRequest(port, "GET", "/api/frame.jpg");
    ASSERT_TRUE(frame.ok);
    EXPECT_EQ(frame.status, 404);

    HttpReply post = httpRequest(port, "POST", "/api/hello");
    ASSERT_TRUE(post.ok);
    EXPECT_EQ(post.status, 405);
    EXPECT_EQ(post.headers["allow"], "GET");

    EXPECT_EQ(f.service->stats().clients_total, 0u);   // plain HTTP is not a feed client
}

// ---- 2. the feed ------------------------------------------------------------------------

TEST(Inspection, WebSocketFeedSendsHelloSnapshotsAndFramesBoundToIdentity) {
    Fixture f;
    ASSERT_TRUE(f.start());
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));

    WsMessage m;
    ASSERT_TRUE(ws.next(m, Ms(5000)));
    EXPECT_EQ(m.opcode, 1);
    EXPECT_EQ(messageType(m.payload), "hello");
    EXPECT_NE(m.payload.find("\"session\":{\"id\":\"" + f.system->sessionId()), std::string::npos);

    // snapshots at roughly snapshot_hz (20): count over one second
    int        snapshots = 0;
    const auto until     = Clock::now() + Ms(1000);
    while (Clock::now() < until) {
        if (ws.next(m, Ms(100)) && m.opcode == 1 && messageType(m.payload) == "snapshot") {
            ++snapshots;
            // inline mode: the top-level running flag (before robot) is false
            EXPECT_NE(m.payload.find("\"running\":false,\"robot\":{"), std::string::npos);
        }
    }
    EXPECT_GE(snapshots, 8);
    EXPECT_LE(snapshots, 60);

    // drive the runtime inline until a detection frame exists and a
    // preview for it arrives
    BinaryFrame frame;
    bool        got_frame = false;
    const auto  deadline  = Clock::now() + Ms(30000);
    while (!got_frame && Clock::now() < deadline) {
        f.step(10);
        while (ws.next(m, Ms(10))) {
            if (m.opcode == 2) {
                ASSERT_TRUE(parseBinaryFrame(m.payload, frame));
                got_frame = true;
                break;
            }
        }
    }
    ASSERT_TRUE(got_frame);
    EXPECT_FALSE(f.system->detectionFrames().empty());

    // no more stepping: the newest identity is fixed, so the last preview
    // and a later snapshot must name the same frame
    FrameIdentity snapshot_identity;
    const auto    settle = Clock::now() + Ms(5000);
    while (Clock::now() < settle) {
        if (!ws.next(m, Ms(200))) {
            continue;
        }
        if (m.opcode == 2) {
            ASSERT_TRUE(parseBinaryFrame(m.payload, frame));
        } else if (messageType(m.payload) == "snapshot") {
            snapshot_identity = snapshotFrameIdentity(m.payload);
        }
        if (snapshot_identity == frame.identity) {
            break;
        }
    }
    ASSERT_TRUE(frame.identity.set);
    ASSERT_TRUE(snapshot_identity.set);
    EXPECT_EQ(frame.identity.camera, "front_camera");
    EXPECT_EQ(frame.identity.camera, snapshot_identity.camera);
    EXPECT_EQ(frame.identity.epoch, snapshot_identity.epoch);
    EXPECT_EQ(frame.identity.sequence, snapshot_identity.sequence);

    // the JPEG decodes to the dimensions the header promises, within the
    // configured preview width
    long long width = 0, height = 0, preview_w = 0, preview_h = 0, quality = 0;
    EXPECT_TRUE(jsonNumber(frame.header, "width_px", width));
    EXPECT_TRUE(jsonNumber(frame.header, "height_px", height));
    EXPECT_TRUE(jsonNumber(frame.header, "preview_width_px", preview_w));
    EXPECT_TRUE(jsonNumber(frame.header, "preview_height_px", preview_h));
    EXPECT_TRUE(jsonNumber(frame.header, "quality", quality));
    EXPECT_EQ(width, 640);
    EXPECT_EQ(height, 480);
    EXPECT_EQ(quality, 60);
    EXPECT_LE(preview_w, 320);
    EXPECT_EQ(jsonString(frame.header, "format"), "image/jpeg");
    int w = 0, h = 0, comp = 0;
    stbi_uc* pixels = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(frame.jpeg.data()),
                                            static_cast<int>(frame.jpeg.size()), &w, &h, &comp, 1);
    ASSERT_NE(pixels, nullptr) << stbi_failure_reason();
    EXPECT_EQ(w, preview_w);
    EXPECT_EQ(h, preview_h);
    EXPECT_EQ(w * 2, width);   // 640 wide into 320: an exact factor of two
    stbi_image_free(pixels);

    const InspectionServiceStats s = f.service->stats();
    EXPECT_TRUE(s.running);
    EXPECT_EQ(s.port, f.service->port());
    EXPECT_EQ(s.clients, 1u);
    EXPECT_GE(s.frames_sent, 1u);
    EXPECT_GE(s.encodes, 1u);
    EXPECT_GT(s.last_encode_ms, 0.0);
    EXPECT_GT(s.snapshots_sent, 10u);
    EXPECT_GT(s.bytes_sent, 0u);

    // the HTTP preview of the same frame decodes too
    HttpReply jpg = httpRequest(f.service->port(), "GET", "/api/frame.jpg?camera=front_camera");
    ASSERT_TRUE(jpg.ok);
    EXPECT_EQ(jpg.status, 200);
    EXPECT_EQ(jpg.headers["content-type"], "image/jpeg");
    pixels = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(jpg.body.data()),
                                   static_cast<int>(jpg.body.size()), &w, &h, &comp, 1);
    ASSERT_NE(pixels, nullptr);
    EXPECT_EQ(w, 320);
    stbi_image_free(pixels);
    HttpReply unknown = httpRequest(f.service->port(), "GET", "/api/frame.jpg?camera=nope");
    EXPECT_EQ(unknown.status, 404);
}

// ---- 3. preview budget ----------------------------------------------------------------

TEST(Inspection, PreviewBudgetZeroStopsFramesAndANewBudgetResumesThem) {
    Fixture f;
    ASSERT_TRUE(f.start());
    WsClient ws;
    ASSERT_TRUE(ws.connect(f.service->port()));
    WsMessage m;
    ASSERT_TRUE(ws.next(m, Ms(5000)));
    ASSERT_EQ(messageType(m.payload), "hello");

    // no frame exists yet, so the budget is in place before any could go
    ws.sendText("{\"type\":\"preview\",\"hz\":0}");
    std::this_thread::sleep_for(Ms(300));
    while (ws.next(m, Ms(10))) {
        EXPECT_NE(m.opcode, 2);
    }

    int        binaries = 0;
    const auto until    = Clock::now() + Ms(1500);
    while (Clock::now() < until) {
        f.step(10);
        while (ws.next(m, Ms(10))) {
            if (m.opcode == 2) {
                ++binaries;
            }
        }
    }
    EXPECT_FALSE(f.system->detectionFrames().empty());   // frames existed to send
    EXPECT_EQ(binaries, 0);
    EXPECT_EQ(f.service->stats().frames_sent, 0u);

    // a budget with an over-range width is clamped, not rejected
    ws.sendText("{ \"type\" : \"preview\", \"hz\" : 20, \"quality\" : 50, \"max_width\" : 10 }");
    BinaryFrame frame;
    bool        got_frame = false;
    const auto  deadline  = Clock::now() + Ms(20000);
    while (!got_frame && Clock::now() < deadline) {
        f.step(10);
        while (ws.next(m, Ms(10))) {
            if (m.opcode == 2) {
                ASSERT_TRUE(parseBinaryFrame(m.payload, frame));
                got_frame = true;
                break;
            }
        }
    }
    ASSERT_TRUE(got_frame);
    long long preview_w = 0, quality = 0;
    EXPECT_TRUE(jsonNumber(frame.header, "preview_width_px", preview_w));
    EXPECT_TRUE(jsonNumber(frame.header, "quality", quality));
    EXPECT_EQ(quality, 50);
    EXPECT_LE(preview_w, 64);   // clamped to the 64 px floor: 640 / 10
    EXPECT_GE(preview_w, 32);
    EXPECT_GE(f.service->stats().frames_sent, 1u);
}

// ---- 4. slow client -----------------------------------------------------------------------

TEST(Inspection, SlowClientIsBoundedAndClosedWhileEstimationKeepsRunning) {
    Fixture f("<Inspection enabled=\"true\" port=\"0\" snapshot_hz=\"20\" preview_hz=\"10\" "
              "preview_quality=\"60\" preview_max_width=\"640\" client_buffer_kb=\"64\" "
              "stall_close_ms=\"500\"/>");
    ASSERT_NE(f.system, nullptr);
    std::string err;
    ASSERT_TRUE(f.system->start(err)) << err;
    ASSERT_TRUE(f.start());

    WsClient slow;
    ASSERT_TRUE(slow.connect(f.service->port(), 4096));   // never reads again

    uint64_t   last_cycles = f.system->cycle();
    bool       closed      = false;
    const auto deadline    = Clock::now() + Ms(15000);
    while (Clock::now() < deadline) {
        std::this_thread::sleep_for(Ms(250));
        const uint64_t cycles = f.system->cycle();
        EXPECT_GT(cycles, last_cycles);   // estimation never waits for the browser
        last_cycles = cycles;
        const InspectionServiceStats s = f.service->stats();
        if (s.client_disconnects > 0) {
            closed = true;
            break;
        }
    }
    EXPECT_TRUE(closed);
    const InspectionServiceStats s = f.service->stats();
    EXPECT_GT(s.frames_skipped + s.snapshots_skipped, 0u);
    EXPECT_EQ(s.clients_total, 1u);
    EXPECT_EQ(s.client_disconnects, 1u);
    EXPECT_GT(s.snapshots_sent, 0u);
    // the estimation worker ran the whole time (the rate itself is not
    // asserted: the detector and the encoder share a loaded test machine)
    EXPECT_TRUE(f.system->estimationStats().running);
    EXPECT_GE(f.system->estimationStats().cycles, 10u);

    f.service->stop();
    f.system->stop();
    EXPECT_FALSE(f.service->stats().running);
}

// The bound itself, at the server: queued bytes never pass the limit,
// refused messages are counted, the stalled client is closed, and a
// connection over max_clients gets 503.
TEST(Inspection, HttpServerBoundsQueuedBytesRefusesAndClosesStalledClients) {
    HttpServerConfig hc;
    hc.port                   = 0;
    hc.max_clients            = 1;
    hc.client_buffer_bytes    = 16 * 1024;
    hc.close_after_stalled_ms = 300;
    HttpServer  server(hc, [](const HttpRequest&) {
        HttpResponse r;
        r.status = 404;
        return r;
    });
    std::string err;
    ASSERT_TRUE(server.start(err)) << err;
    ASSERT_GT(server.port(), 0);

    WsClient slow;
    ASSERT_TRUE(slow.connect(server.port(), 4096));
    const auto wait_client = Clock::now() + Ms(3000);
    while (server.clients().empty() && Clock::now() < wait_client) {
        std::this_thread::sleep_for(Ms(10));
    }
    ASSERT_EQ(server.clients().size(), 1u);
    const HttpServer::ClientId id = server.clients()[0];

    HttpReply over = httpRequest(server.port(), "GET", "/");
    ASSERT_TRUE(over.ok);
    EXPECT_EQ(over.status, 503);
    EXPECT_EQ(server.stats().refused_connections, 1u);

    const std::string message(4000, 'x');
    std::size_t       refused  = 0;
    std::size_t       max_seen = 0;
    const auto        fill     = Clock::now() + Ms(5000);
    while (Clock::now() < fill) {
        const bool ok = server.sendText(id, message);
        max_seen      = std::max(max_seen, server.queued(id));
        if (!ok && ++refused >= 3) {
            break;
        }
    }
    EXPECT_GE(refused, 3u);
    EXPECT_LE(max_seen, hc.client_buffer_bytes);
    EXPECT_GT(max_seen, 0u);
    EXPECT_GE(server.stats().messages_refused, refused);

    const auto stall = Clock::now() + Ms(5000);
    while (server.stats().clients_closed_stalled == 0 && Clock::now() < stall) {
        std::this_thread::sleep_for(Ms(20));
    }
    EXPECT_EQ(server.stats().clients_closed_stalled, 1u);
    EXPECT_TRUE(server.clients().empty());
    EXPECT_EQ(server.queued(id), 0u);
    EXPECT_FALSE(server.sendText(id, "gone"));

    // an unmasked client frame ends the connection
    WsClient bad;
    ASSERT_TRUE(bad.connect(server.port()));
    const auto wait_bad = Clock::now() + Ms(3000);
    while (server.clients().empty() && Clock::now() < wait_bad) {
        std::this_thread::sleep_for(Ms(10));
    }
    ASSERT_EQ(server.clients().size(), 1u);
    bad.sendUnmaskedText("{\"type\":\"preview\"}");
    const auto wait_gone = Clock::now() + Ms(3000);
    while (!server.clients().empty() && Clock::now() < wait_gone) {
        std::this_thread::sleep_for(Ms(10));
    }
    EXPECT_TRUE(server.clients().empty());
    WsMessage m;
    while (bad.next(m, Ms(2000))) {
    }
    EXPECT_TRUE(bad.closed());

    // a well-formed client stays; ping is answered with pong; stop drops it
    WsClient good;
    ASSERT_TRUE(good.connect(server.port()));
    const auto wait_good = Clock::now() + Ms(3000);
    while (server.clients().empty() && Clock::now() < wait_good) {
        std::this_thread::sleep_for(Ms(10));
    }
    ASSERT_EQ(server.clients().size(), 1u);
    EXPECT_EQ(server.stats().websocket_total, 3u);
    server.stop();
    EXPECT_FALSE(server.running());
    EXPECT_TRUE(server.clients().empty());
    server.stop();   // idempotent
}

// ---- 5. reconnect ------------------------------------------------------------------------

TEST(Inspection, ReconnectingClientGetsHelloWithTheSameSession) {
    Fixture f;
    ASSERT_TRUE(f.start());
    WsMessage m;

    WsClient a;
    ASSERT_TRUE(a.connect(f.service->port()));
    ASSERT_TRUE(a.next(m, Ms(5000)));
    ASSERT_EQ(messageType(m.payload), "hello");
    const std::string session = jsonString(m.payload, "id");
    EXPECT_EQ(session, f.system->sessionId());
    a.sendClose();
    // the server echoes the close and drops the connection
    bool echoed = false;
    while (a.next(m, Ms(2000))) {
        if (m.opcode == 8) {
            echoed = true;
            break;
        }
    }
    EXPECT_TRUE(echoed);
    a.close();

    WsClient b;
    ASSERT_TRUE(b.connect(f.service->port()));
    ASSERT_TRUE(b.next(m, Ms(5000)));
    ASSERT_EQ(messageType(m.payload), "hello");
    EXPECT_EQ(jsonString(m.payload, "id"), session);

    const auto deadline = Clock::now() + Ms(5000);
    while (Clock::now() < deadline) {
        const InspectionServiceStats s = f.service->stats();
        if (s.clients_total == 2 && s.client_disconnects == 1 && s.clients == 1) {
            break;
        }
        std::this_thread::sleep_for(Ms(20));
    }
    const InspectionServiceStats s = f.service->stats();
    EXPECT_EQ(s.clients_total, 2u);
    EXPECT_EQ(s.client_disconnects, 1u);
    EXPECT_EQ(s.clients, 1u);
}

// ---- 6. shutdown ---------------------------------------------------------------------------

TEST(Inspection, StopIsPromptWithAConnectedClientAndDestructorsStop) {
    {
        Fixture f;
        ASSERT_TRUE(f.start());
        WsClient  ws;
        WsMessage m;
        ASSERT_TRUE(ws.connect(f.service->port()));
        ASSERT_TRUE(ws.next(m, Ms(5000)));
        EXPECT_TRUE(f.service->running());

        const auto t0 = Clock::now();
        f.service->stop();
        const auto elapsed = std::chrono::duration_cast<Ms>(Clock::now() - t0);
        EXPECT_LT(elapsed.count(), 1000);
        EXPECT_FALSE(f.service->running());
        f.service->stop();   // idempotent
        // the client sees the connection go
        while (ws.next(m, Ms(2000))) {
        }
        EXPECT_TRUE(ws.closed());

        f.system->stop();   // never started: harmless
        f.service.reset();
        f.system.reset();
    }
    {
        // no stop() call at all: the destructor does it
        Fixture f;
        ASSERT_TRUE(f.start());
        WsClient  ws;
        WsMessage m;
        ASSERT_TRUE(ws.connect(f.service->port()));
        ASSERT_TRUE(ws.next(m, Ms(5000)));
        f.service.reset();
        f.system.reset();
        SUCCEED();
    }
}

// ---- 7. helpers ------------------------------------------------------------------------------

TEST(Inspection, UrlDecodeAndParseQuery) {
    EXPECT_EQ(urlDecode("a%20b%2Fc+d"), "a b/c d");
    EXPECT_EQ(urlDecode("plain"), "plain");
    EXPECT_EQ(urlDecode("%zz%4"), "%zz%4");   // malformed escapes stay literal
    EXPECT_EQ(urlDecode(""), "");

    const std::map<std::string, std::string> q = parseQuery("camera=front%20cam&x=1&flag&=v&y=a=b");
    EXPECT_EQ(q.size(), 4u);
    EXPECT_EQ(q.at("camera"), "front cam");
    EXPECT_EQ(q.at("x"), "1");
    EXPECT_EQ(q.at("flag"), "");
    EXPECT_EQ(q.at("y"), "a=b");
    EXPECT_TRUE(parseQuery("").empty());

    HttpRequest req;
    req.query = "camera=cam_a&n=3";
    EXPECT_EQ(req.param("camera"), "cam_a");
    EXPECT_EQ(req.param("missing", "dflt"), "dflt");
    EXPECT_EQ(req.param("missing"), "");
}

// ---- 8. configuration ----------------------------------------------------------------------

TEST(Inspection, ConfigRejectsOutOfRangePortAndStallClose) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    EXPECT_EQ(System::buildFromString(rigXml("<Inspection enabled=\"true\" port=\"70000\"/>").c_str(),
                                      functions, err),
              nullptr);
    EXPECT_NE(err.find("port"), std::string::npos) << err;
    err.clear();
    EXPECT_EQ(System::buildFromString(
                  rigXml("<Inspection enabled=\"true\" port=\"0\" stall_close_ms=\"10\"/>").c_str(),
                  functions, err),
              nullptr);
    EXPECT_NE(err.find("stall_close_ms"), std::string::npos) << err;

    // the disabled default builds and reports defaults
    err.clear();
    std::unique_ptr<System> plain = System::buildFromString(rigXml("").c_str(), functions, err);
    ASSERT_NE(plain, nullptr) << err;
    EXPECT_FALSE(plain->inspection().enabled);
    EXPECT_EQ(plain->inspection().port, 8765);

    // a static_root that is not a directory is a create() error
    Fixture f;
    ASSERT_NE(f.system, nullptr);
    InspectionConfig bad = f.system->inspection();
    bad.static_root      = "definitely/not/a/directory";
    err.clear();
    EXPECT_EQ(InspectionService::create(*f.system, bad, err), nullptr);
    EXPECT_NE(err.find("static_root"), std::string::npos) << err;
}

// ---- 9. static_root -------------------------------------------------------------------

TEST(Inspection, StaticRootServesFilesFromADirectoryAndRejectsTraversal) {
    namespace fs = std::filesystem;
    const fs::path root =
        fs::temp_directory_path() /
        ("navigatr_inspection_static_" + std::to_string(std::chrono::steady_clock::now()
                                                            .time_since_epoch()
                                                            .count()));
    ASSERT_TRUE(fs::create_directories(root));
    const std::string page = "<html>development copy</html>\n";
    {
        std::ofstream out(root / "index.html", std::ios::binary);
        out << page;
    }
    {
        std::ofstream out(root / "extra.json", std::ios::binary);
        out << "{\"extra\":true}";
    }

    Fixture f;
    ASSERT_NE(f.system, nullptr);
    InspectionConfig cfg = f.system->inspection();
    cfg.static_root      = root.string();
    std::string err;
    f.service = InspectionService::create(*f.system, cfg, err);
    ASSERT_NE(f.service, nullptr) << err;
    ASSERT_TRUE(f.start());
    const int port = f.service->port();

    HttpReply index = httpRequest(port, "GET", "/");
    ASSERT_TRUE(index.ok);
    EXPECT_EQ(index.status, 200);
    EXPECT_EQ(index.body, page);   // the directory copy, not the embedded one
    EXPECT_NE(index.headers["content-type"].find("text/html"), std::string::npos);

    HttpReply extra = httpRequest(port, "GET", "/extra.json");
    ASSERT_TRUE(extra.ok);
    EXPECT_EQ(extra.status, 200);
    EXPECT_EQ(extra.body, "{\"extra\":true}");
    EXPECT_EQ(extra.headers["content-type"], "application/json");

    // files the directory lacks still come from the binary
    const EmbeddedAsset* three = embeddedAsset("vendor/three.module.min.js");
    ASSERT_NE(three, nullptr);
    HttpReply vendor = httpRequest(port, "GET", "/vendor/three.module.min.js");
    ASSERT_TRUE(vendor.ok);
    EXPECT_EQ(vendor.status, 200);
    EXPECT_EQ(vendor.body.size(), three->size);

    // traversal and backslashes never reach the filesystem
    {
        std::ofstream out(root.parent_path() / "navigatr_inspection_outside.txt", std::ios::binary);
        out << "outside";
    }
    EXPECT_EQ(httpRequest(port, "GET", "/../navigatr_inspection_outside.txt").status, 404);
    EXPECT_EQ(httpRequest(port, "GET", "/%2e%2e/navigatr_inspection_outside.txt").status, 404);
    EXPECT_EQ(httpRequest(port, "GET", "/..%5Cnavigatr_inspection_outside.txt").status, 404);
    EXPECT_EQ(httpRequest(port, "GET", "/missing.css").status, 404);

    f.service->stop();
    std::error_code ec;
    fs::remove_all(root, ec);
    fs::remove(root.parent_path() / "navigatr_inspection_outside.txt", ec);
}
