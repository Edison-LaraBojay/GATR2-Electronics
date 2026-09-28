// capture_gtest.cpp
// Host tests of the capture recorder, bundle, ZIP and HTTP handlers
// (docs/capture.md). Records are posted with explicit Pi host stamps and the
// recorder runs on a fake clock, so windows and limits are exact. A test-side
// ZIP reader with its own CRC table checks the archive independently.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "capture/capture_bundle.h"
#include "capture/capture_config.h"
#include "capture/capture_http.h"
#include "capture/capture_recorder.h"
#include "capture/zip_writer.h"
#include "config/composition.h"
#include "config/config_node.h"
#include "core/host_clock.h"
#include "diagnostics/hub.h"
#include "impl/resources/serial_links.h"
#include "inspection/json_writer.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "state/robot_state_feed.h"
#include "tinyxml2/tinyxml2.h"
#include "translaGATR/frame_codec.h"

using namespace navigatr;

namespace
{

constexpr double kTestPi = 3.14159265358979323846;

// ---- test-side ZIP reader ---------------------------------------------------------

uint32_t refCrc32(const std::string& data) {
    static std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            }
            t[i] = c;
        }
        return t;
    }();
    uint32_t c = 0xFFFFFFFFu;
    for (unsigned char b : data) {
        c = table[(c ^ b) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

uint32_t rd16(const std::string& s, std::size_t at) {
    return static_cast<uint32_t>(static_cast<unsigned char>(s[at])) |
           static_cast<uint32_t>(static_cast<unsigned char>(s[at + 1])) << 8;
}

uint32_t rd32(const std::string& s, std::size_t at) { return rd16(s, at) | rd16(s, at + 2) << 16; }

struct ZipRead {
    std::map<std::string, std::string> files;
    std::vector<std::string>           order;
};

// Store-only archives: end record, central directory, local headers, CRCs.
bool readZip(const std::string& z, ZipRead& out, std::string& err) {
    if (z.size() < 22) {
        err = "too short";
        return false;
    }
    std::size_t eocd = std::string::npos;
    for (std::size_t i = z.size() - 22 + 1; i-- > 0;) {
        if (rd32(z, i) == 0x06054b50u) {
            eocd = i;
            break;
        }
    }
    if (eocd == std::string::npos) {
        err = "no end record";
        return false;
    }
    const uint32_t count = rd16(z, eocd + 10);
    const uint32_t cd_size = rd32(z, eocd + 12);
    std::size_t    at    = rd32(z, eocd + 16);
    if (at + cd_size != eocd) {
        err = "central directory size mismatch";
        return false;
    }
    for (uint32_t n = 0; n < count; ++n) {
        if (rd32(z, at) != 0x02014b50u) {
            err = "bad central header";
            return false;
        }
        const uint32_t method = rd16(z, at + 10);
        const uint32_t crc    = rd32(z, at + 16);
        const uint32_t csize  = rd32(z, at + 20);
        const uint32_t usize  = rd32(z, at + 24);
        const uint32_t nlen   = rd16(z, at + 28);
        const uint32_t elen   = rd16(z, at + 30);
        const uint32_t clen   = rd16(z, at + 32);
        const uint32_t local  = rd32(z, at + 42);
        const std::string name = z.substr(at + 46, nlen);
        at += 46 + nlen + elen + clen;
        if (method != 0 || csize != usize) {
            err = name + ": not stored";
            return false;
        }
        if (rd32(z, local) != 0x04034b50u || rd16(z, local + 26) != nlen ||
            z.compare(local + 30, nlen, name) != 0 || rd32(z, local + 14) != crc ||
            rd32(z, local + 18) != csize) {
            err = name + ": local header disagrees";
            return false;
        }
        const std::size_t data = local + 30 + nlen + rd16(z, local + 28);
        if (data + usize > z.size()) {
            err = name + ": truncated";
            return false;
        }
        const std::string body = z.substr(data, usize);
        if (refCrc32(body) != crc) {
            err = name + ": crc mismatch";
            return false;
        }
        out.files[name] = body;
        out.order.push_back(name);
    }
    return true;
}

// ---- CSV ------------------------------------------------------------------------

struct Table {
    std::vector<std::string>              header;
    std::vector<std::vector<std::string>> rows;

    int col(const std::string& name) const {
        for (std::size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) {
                return static_cast<int>(i);
            }
        }
        ADD_FAILURE() << "no column " << name;
        return -1;
    }
    const std::string& at(std::size_t row, const std::string& name) const {
        static const std::string kNone = "<missing column>";
        const int c = col(name);
        // a short row (a writer bug) fails its own check, never aborts the run
        return c < 0 || row >= rows.size() || static_cast<std::size_t>(c) >= rows[row].size()
                   ? kNone
                   : rows[row][static_cast<std::size_t>(c)];
    }
    double num(std::size_t row, const std::string& name) const {
        const std::string& s = at(row, name);
        EXPECT_FALSE(s.empty()) << name << " row " << row;
        return std::strtod(s.c_str(), nullptr);
    }
};

// RFC 4180 with \n line ends; false on a stray \r or an unterminated quote.
bool parseCsv(const std::string& text, Table& out) {
    std::vector<std::vector<std::string>> lines;
    std::vector<std::string>              fields;
    std::string                           field;
    bool                                  quoted = false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (quoted) {
            if (c == '"') {
                if (i + 1 < text.size() && text[i + 1] == '"') {
                    field += '"';
                    ++i;
                } else {
                    quoted = false;
                }
            } else {
                field += c;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == ',') {
            fields.push_back(field);
            field.clear();
        } else if (c == '\n') {
            fields.push_back(field);
            field.clear();
            lines.push_back(fields);
            fields.clear();
        } else if (c == '\r') {
            return false;
        } else {
            field += c;
        }
    }
    if (quoted || !field.empty() || !fields.empty() || lines.empty()) {
        return false;
    }
    out.header = lines.front();
    out.rows.assign(lines.begin() + 1, lines.end());
    return true;
}

// ---- recorder on a fake clock --------------------------------------------------------

class FakeHost final : public CaptureHost
{
public:
    std::string sessionId() const override { return "0123456789abcdef"; }
    uint64_t    resetCount() const override { return 0; }
    void        writeMetadata(JsonWriter& w) const override { w.field("host", "fake"); }
};

// A host whose profile changes while the recorder runs.
class MutableHost final : public CaptureHost
{
public:
    std::string sessionId() const override { return "0123456789abcdef"; }
    uint64_t    resetCount() const override { return 0; }
    void        writeMetadata(JsonWriter& w) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        w.field("profile_probe", profile_);
    }
    void set(const std::string& profile) {
        std::lock_guard<std::mutex> lock(mutex_);
        profile_ = profile;
    }

private:
    mutable std::mutex mutex_;
    std::string        profile_ = "A";
};

// The integer after "key": in a JSON text, from `from` on; -1 when absent.
int64_t jsonInt(const std::string& json, const std::string& key, std::size_t from = 0) {
    const std::size_t at = json.find("\"" + key + "\":", from);
    if (at == std::string::npos) {
        ADD_FAILURE() << "no " << key;
        return -1;
    }
    return std::strtoll(json.c_str() + at + key.size() + 3, nullptr, 10);
}

CaptureConfig testConfig() {
    CaptureConfig c;
    c.rolling_s      = 10;
    c.max_pre_s      = 30;
    c.max_post_s     = 60;
    c.default_pre_s  = 2;
    c.default_post_s = 1;
    return c;
}

struct Rig {
    explicit Rig(CaptureConfig config = testConfig(), std::size_t hub_capacity = 16384,
                 std::shared_ptr<const CaptureHost> host = std::make_shared<FakeHost>())
        : hub(hub_capacity) {
        CaptureRecorder::Options o;
        o.thread  = false;
        o.clock   = [this] { return now; };
        o.wall_ms = [] { return int64_t{1790000000000}; };
        robot_src = hub.sourceId("localization");
        pico_src  = hub.sourceId("pico");
        brain_src = hub.sourceId("brain_link");
        rec = std::make_unique<CaptureRecorder>(config, hub, std::move(host), o);
    }

    DiagnosticsHub                   hub;
    int64_t                          now = 1000000;
    uint16_t                         robot_src = 0, pico_src = 0, brain_src = 0;
    std::unique_ptr<CaptureRecorder> rec;

    bool post(DiagKind kind, DiagPayload payload, int64_t at, uint16_t source) {
        DiagRecord r;
        r.kind    = kind;
        r.source  = source;
        r.host_us = at;
        r.payload = std::move(payload);
        return hub.post(std::move(r));
    }

    DiagRobotState robot(uint64_t pub, double x_m, double heading_deg, uint64_t epoch = 1,
                         uint64_t anchor = 1) const {
        DiagRobotState s;
        s.publication       = pub;
        s.measured_ms       = 5000 + static_cast<int64_t>(pub);
        s.measured_clock    = DiagClock::kPico;
        s.measured_host_ms  = now / 1000 - 4;
        s.odom_x_m          = x_m;
        s.odom_y_m          = -x_m / 2;
        s.odom_heading_rad  = heading_deg * kTestPi / 180.0;
        s.field_x_m         = x_m + 1.0;
        s.field_y_m         = 0.25;
        s.field_heading_rad = s.odom_heading_rad;
        s.vx_m_s            = 0.5f;
        s.vy_m_s            = -0.125f;
        s.yaw_rate_rad_s    = 0.1f;
        s.confidence        = 0.75f;
        s.odometry_epoch    = epoch;
        s.anchor_revision   = anchor;
        s.placement_session = 7;
        s.placement_sequence = 3;
        s.valid             = true;
        s.initialized       = true;
        s.attitude_assumed_level = true;
        s.advanced          = true;
        return s;
    }

    // one robot state every step_us for span_us, pumping every 20 ms
    void run(int64_t span_us, int64_t step_us = 10000) {
        const int64_t end = now + span_us;
        int64_t       next_pump = now + 20000;
        while (now < end) {
            ++pub;
            post(DiagKind::kRobotState, robot(pub, static_cast<double>(pub) * 0.001, 0.0), now,
                 robot_src);
            stamps.push_back(now);
            now += step_us;
            if (now >= next_pump) {
                rec->pump();
                next_pump += 20000;
            }
        }
        rec->pump();
    }

    // time passes with nothing posted
    void wait(int64_t span_us) {
        const int64_t end = now + span_us;
        while (now < end) {
            now += 20000;
            rec->pump();
        }
    }

    std::string status() const {
        JsonWriter w;
        rec->writeStatus(w);
        return w.take();
    }

    std::string start(double pre_s, double post_s, uint32_t kinds = kCaptureDefaultKinds) {
        CaptureRequest q;
        q.pre_s     = pre_s;
        q.post_s    = post_s;
        q.kinds     = kinds;
        q.requester = "test";
        std::string id, err;
        EXPECT_TRUE(rec->start(q, id, err)) << err;
        return id;
    }

    ZipRead bundle(const std::string& id) const {
        ZipRead                                  z;
        const std::shared_ptr<const std::string> zip = rec->bundle(id);
        EXPECT_NE(zip, nullptr) << id << " " << status();
        if (zip != nullptr) {
            std::string err;
            EXPECT_TRUE(readZip(*zip, z, err)) << err;
        }
        return z;
    }

    uint64_t             pub = 0;
    std::vector<int64_t> stamps; // of every state run() posted
};

Table table(const ZipRead& z, const std::string& file) {
    Table      t;
    const auto it = z.files.find(file);
    EXPECT_NE(it, z.files.end()) << file;
    if (it != z.files.end()) {
        EXPECT_TRUE(parseCsv(it->second, t)) << file;
        for (std::size_t r = 0; r < t.rows.size(); ++r) {
            EXPECT_EQ(t.rows[r].size(), t.header.size()) << file << " row " << r;
        }
    }
    return t;
}

bool contains(const std::string& s, const std::string& part) {
    return s.find(part) != std::string::npos;
}

std::size_t count(const std::string& s, const std::string& part) {
    std::size_t n = 0;
    for (std::size_t at = s.find(part); at != std::string::npos; at = s.find(part, at + 1)) {
        ++n;
    }
    return n;
}

bool parseConfig(const char* xml, CaptureConfig& out, std::string& err) {
    tinyxml2::XMLDocument doc;
    EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
    return parseCaptureConfig(ConfigNode{doc.RootElement()}, out, err);
}

const char* kAllNoop = R"(
<System>
    CAPTURE
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";

std::unique_ptr<System> buildNoop(const std::string& capture, std::string& err) {
    std::string xml(kAllNoop);
    xml.replace(xml.find("CAPTURE"), 7, capture);
    FunctionRegistry functions;
    registerAll(functions);
    return System::buildFromString(xml.c_str(), functions, err);
}

HttpRequest request(const std::string& method, const std::string& path,
                    const std::string& query = "") {
    HttpRequest r;
    r.method = method;
    r.path   = path;
    r.query  = query;
    return r;
}

} // namespace

// ---- configuration --------------------------------------------------------------

TEST(CaptureConfig, DefaultsWhenAbsentAndStrictAttributes) {
    CaptureConfig c;
    std::string   err;
    ASSERT_TRUE(parseCaptureConfig(ConfigNode{}, c, err)) << err;
    EXPECT_TRUE(c.enabled);
    EXPECT_EQ(c.rolling_s, 10.0);
    EXPECT_EQ(c.default_post_s, 15.0);
    EXPECT_EQ(c.max_records, 400000);
    EXPECT_EQ(c.max_mb, 64);
    EXPECT_EQ(c.keep, 3);
    EXPECT_EQ(c.auto_faults, 0u);

    ASSERT_TRUE(parseConfig(R"(<Capture enabled="false" rolling_s="5" max_pre_s="5"
        max_post_s="30" default_pre_s="3" default_post_s="20" max_records="5000" max_mb="8"
        keep="2" directory="caps" auto="link_lost, pico_reboot" auto_cooldown_s="60"
        auto_max_per_hour="4"/>)",
                            c, err))
        << err;
    EXPECT_FALSE(c.enabled);
    EXPECT_EQ(c.directory, "caps");
    EXPECT_EQ(c.auto_faults, captureFaultBit(CaptureFault::kLinkLost) |
                                 captureFaultBit(CaptureFault::kPicoReboot));
    EXPECT_EQ(c.auto_max_per_hour, 4);

    EXPECT_FALSE(parseConfig(R"(<Capture auto="link_lost,brownout"/>)", c, err));
    EXPECT_TRUE(contains(err, "brownout")) << err;
    EXPECT_FALSE(parseConfig(R"(<Capture colour="red"/>)", c, err));
    EXPECT_FALSE(parseConfig(R"(<Capture max_post_s="0"/>)", c, err));
    EXPECT_FALSE(parseConfig(R"(<Capture default_pre_s="40"/>)", c, err));
    EXPECT_FALSE(parseConfig(R"(<Capture keep="0"/>)", c, err));
    EXPECT_FALSE(parseConfig(R"(<Capture rolling_s="nan"/>)", c, err));
}

TEST(CaptureConfig, SystemTakesTheOptionalCaptureElement) {
    std::string err;
    auto        system = buildNoop("", err);
    ASSERT_NE(system, nullptr) << err;
    ASSERT_NE(system->capture(), nullptr);
    EXPECT_TRUE(system->captureConfig().enabled);

    system = buildNoop(R"(<Capture enabled="false"/>)", err);
    ASSERT_NE(system, nullptr) << err;
    std::string id;
    EXPECT_FALSE(system->capture()->start(CaptureRequest{}, id, err));
    EXPECT_TRUE(contains(err, "disabled")) << err;
    // a disabled capture wants nothing: producers pay one atomic load
    EXPECT_EQ(system->diagHub().wanted(), 0u);

    system = buildNoop(R"(<Capture auto="continuity_lost"/>)", err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->captureConfig().auto_faults,
              captureFaultBit(CaptureFault::kContinuityLost));

    EXPECT_EQ(buildNoop(R"(<Capture enabled="maybe"/>)", err), nullptr);
    EXPECT_EQ(buildNoop(R"(<Capture/><Capture/>)", err), nullptr);
}

// ---- windows and limits --------------------------------------------------------------

TEST(CaptureRecorder, IdleWantsTheDefaultStreamsOnly) {
    Rig rig;
    EXPECT_EQ(rig.hub.wanted(), kCaptureDefaultKinds);
    EXPECT_FALSE(rig.hub.wants(DiagKind::kBytes));
    const std::string id = rig.start(0, 1, kCaptureDefaultKinds | diagBit(DiagKind::kBytes));
    // raw bytes flow only while a capture selected them
    EXPECT_TRUE(rig.hub.wants(DiagKind::kBytes));
    rig.run(1100000);
    EXPECT_TRUE(contains(rig.status(), "\"state\":\"idle\"")) << rig.status();
    EXPECT_FALSE(rig.hub.wants(DiagKind::kBytes));
    EXPECT_NE(rig.rec->bundle(id), nullptr);
}

TEST(CaptureRecorder, RollingWindowIsBoundedByTimeAndRecords) {
    CaptureConfig c = testConfig();
    c.rolling_s     = 1;
    Rig rig(c);
    rig.run(3000000); // 3 s at 100 Hz
    const int64_t     trigger = rig.now;
    const std::string id      = rig.start(5, 0.5); // asks 5 s, the window holds 1 s
    rig.run(600000);
    const ZipRead z = rig.bundle(id);
    const Table   t = table(z, "robot_state.csv");
    std::size_t   pre = 0;
    for (std::size_t r = 0; r < t.rows.size(); ++r) {
        const int64_t at = std::stoll(t.at(r, "pi_host_us"));
        EXPECT_GE(at, trigger - 1000000);
        pre += at < trigger ? 1 : 0;
    }
    EXPECT_GE(pre, 99u);
    EXPECT_LE(pre, 101u);
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"pre_s_requested\":5,")) << meta;
    EXPECT_TRUE(contains(meta, "\"pre_s\":1,")) << meta;

    // the record bound evicts the oldest while idle, and says so
    CaptureConfig small = testConfig();
    small.max_records   = 100;
    Rig tight(small);
    ASSERT_EQ(tight.rec->recordCap(), 100u);
    tight.run(1000000, 2000); // 500 records within the 10 s window
    const std::string id2 = tight.start(10, 0.1);
    tight.wait(200000);
    const ZipRead z2 = tight.bundle(id2);
    EXPECT_EQ(table(z2, "robot_state.csv").rows.size(), 100u);
    EXPECT_TRUE(contains(z2.files.at("metadata.json"), "\"rolling_evicted\":400,"))
        << z2.files.at("metadata.json");
}

// Review fix: rolling_evicted counts what this capture's window lost, not
// the recorder's lifetime total.
TEST(CaptureRecorder, RollingEvictedCountsOnlyThisCapturesWindow) {
    CaptureConfig small = testConfig();
    small.max_records   = 100;
    Rig rig(small);
    rig.run(1000000, 2000);    // 500 in 1 s: the bound evicts 400 while idle
    rig.run(12000000, 200000); // 12 s at 5 Hz: the 10 s window refills with 50
    const std::string a = rig.start(10, 0.1);
    rig.wait(200000);
    const ZipRead za = rig.bundle(a);
    EXPECT_EQ(table(za, "robot_state.csv").rows.size(), 50u);
    EXPECT_TRUE(contains(za.files.at("metadata.json"), "\"rolling_evicted\":0,"))
        << za.files.at("metadata.json"); // its pre-window was complete
    EXPECT_TRUE(contains(za.files.at("metadata.json"), "\"pre_s_actual\":10,"));

    // a burst late in the next window: exactly the window's records that the
    // bound pushed out, not those older than the window
    rig.run(3000000, 200000);
    rig.run(1000000, 2000);
    const int64_t     trigger = rig.now;
    const std::string b       = rig.start(10, 0.1);
    rig.wait(200000);
    const ZipRead zb   = rig.bundle(b);
    const Table   t    = table(zb, "robot_state.csv");
    uint64_t      in_window = 0;
    for (int64_t at : rig.stamps) {
        in_window += at >= trigger - 10000000 && at < trigger ? 1 : 0;
    }
    ASSERT_EQ(t.rows.size(), 100u); // the bound
    const std::string& meta = zb.files.at("metadata.json");
    EXPECT_GT(in_window, 100u);
    // nothing else lost: every window record is a row or an eviction
    EXPECT_TRUE(contains(meta, "\"hub_ring_full\":{\"robot_state\":0,")) << meta;
    EXPECT_EQ(jsonInt(meta, "rolling_evicted"), static_cast<int64_t>(in_window - 100)) << meta;

    // a pre-window shorter than the rolling one counts only its own part:
    // 400 stamped 1.000..1.798 s were evicted, 150 of them from 1.5 s on
    Rig late(small);
    late.run(1000000, 2000);
    late.wait(3000000);
    const std::string c = late.start(3.5, 0.1); // window from 5.0 - 3.5 = 1.5 s
    late.wait(200000);
    const std::string mc = late.bundle(c).files.at("metadata.json");
    EXPECT_EQ(jsonInt(mc, "rolling_evicted"), 150) << mc;
}

TEST(CaptureRecorder, PreWindowTrimmedToTheRequestedInterval) {
    Rig rig;
    rig.run(5000000); // 500 states over 5 s
    const int64_t     trigger = rig.now;
    const std::string id      = rig.start(2, 0.5);
    // records stamped before the trigger but not drained yet join the pre-window
    rig.post(DiagKind::kRobotState, rig.robot(++rig.pub, 9.0, 0.0), trigger - 5000, rig.robot_src);
    rig.run(700000);
    const Table t = table(rig.bundle(id), "robot_state.csv");
    std::size_t pre = 0;
    bool        late = false;
    for (std::size_t r = 0; r < t.rows.size(); ++r) {
        const int64_t at = std::stoll(t.at(r, "pi_host_us"));
        EXPECT_GE(at, trigger - 2000000);
        EXPECT_LE(at, trigger + 500000);
        pre += at < trigger ? 1 : 0;
        late = late || t.at(r, "odom_x_m") == "9.000000";
    }
    EXPECT_GE(pre, 200u);
    EXPECT_LE(pre, 202u);
    EXPECT_TRUE(late);
}

TEST(CaptureRecorder, PostWindowStopsAtTheEnd) {
    Rig rig;
    const int64_t     trigger = rig.now;
    const std::string id      = rig.start(0, 1);
    EXPECT_TRUE(contains(rig.status(), "\"state\":\"recording\""));
    rig.run(990000);
    EXPECT_EQ(rig.rec->bundle(id), nullptr); // not before the window closed
    EXPECT_TRUE(contains(rig.status(), "\"remaining_s\":")) << rig.status();
    rig.run(500000);
    const std::string status = rig.status();
    EXPECT_TRUE(contains(status, "\"state\":\"idle\"")) << status;
    EXPECT_TRUE(contains(status, "\"id\":\"" + id + "\",\"state\":\"ready\"")) << status;
    EXPECT_TRUE(contains(status, "\"url\":\"/api/capture/" + id + ".zip\""));
    const ZipRead z = rig.bundle(id);
    const Table   t = table(z, "robot_state.csv");
    ASSERT_FALSE(t.rows.empty());
    for (std::size_t r = 0; r < t.rows.size(); ++r) {
        EXPECT_LE(std::stoll(t.at(r, "pi_host_us")), trigger + 1000000);
    }
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"stop_reason\":\"post_window\"")) << meta;
    EXPECT_TRUE(contains(meta, "\"truncated\":false,\"truncated_by\":[]"));
    EXPECT_TRUE(contains(status, "\"truncated\":false,\"truncated_by\":[]")) << status;
    EXPECT_TRUE(contains(meta, "\"schema\":\"gatr2.capture/1\""));
    EXPECT_TRUE(contains(meta, "\"host\":\"fake\",\"host_sampled_pi_host_us\":"));
    EXPECT_TRUE(contains(meta, "\"host_at_start\":{\"sampled_pi_host_us\":")) << meta;
    EXPECT_TRUE(contains(meta, "\"requester\":\"test\""));
}

TEST(CaptureRecorder, RecordLimitTruncatesEarly) {
    CaptureConfig c = testConfig();
    c.max_records   = 50;
    Rig rig(c);
    const std::string id = rig.start(0, 10);
    for (int i = 0; i < 80; ++i) {
        rig.post(DiagKind::kRobotState, rig.robot(++rig.pub, 0.0, 0.0), rig.now + i, rig.robot_src);
    }
    rig.now += 1000;
    rig.rec->pump();
    const std::string status = rig.status();
    EXPECT_TRUE(contains(status, "\"truncated\":true")) << status;
    EXPECT_TRUE(contains(status, "\"stop_reason\":\"record_limit\"")) << status;
    const ZipRead z = rig.bundle(id);
    EXPECT_EQ(table(z, "robot_state.csv").rows.size(), 50u);
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"capture_limit\":{\"robot_state\":30,")) << meta;
    EXPECT_TRUE(contains(meta, "\"truncated\":true,\"truncated_by\":[\"record_limit\"]")) << meta;
    EXPECT_TRUE(contains(status, "\"truncated\":true,\"truncated_by\":[\"record_limit\"]"));
}

// Review fix: rows the CSV size bound left out mark metadata.json truncated,
// the same verdict as the status.
TEST(CaptureRecorder, BundleSizeLimitMarksTheBundleTruncated) {
    CaptureConfig c = testConfig();
    c.max_mb        = 1; // 6241 records of 168 B; the CSV text of 6000 is over 1 MiB
    Rig rig(c);
    ASSERT_GT(rig.rec->recordCap(), 6000u);
    const std::string id = rig.start(0, 10);
    for (int i = 0; i < 6000; ++i) {
        rig.post(DiagKind::kRobotState, rig.robot(++rig.pub, 1.0 * i, 0.0), rig.now + i,
                 rig.robot_src);
    }
    rig.now += 20000;
    rig.rec->pump();
    rig.now += 11000000;
    rig.rec->pump();
    const std::string status = rig.status();
    const ZipRead     z      = rig.bundle(id);
    const std::string& meta  = z.files.at("metadata.json");
    const std::string  verdict =
        "\"truncated\":true,\"truncated_by\":[\"bundle_size_limit\"],\"stop_reason\":\"post_window\"";
    EXPECT_TRUE(contains(meta, verdict)) << meta;
    EXPECT_TRUE(contains(status, verdict)) << status;
    const int64_t rows    = static_cast<int64_t>(table(z, "robot_state.csv").rows.size());
    const int64_t omitted = jsonInt(meta, "robot_state", meta.find("\"bundle_size_limit\":{"));
    EXPECT_GT(omitted, 0);
    EXPECT_EQ(rows + omitted, 6000);
    EXPECT_TRUE(contains(meta, "\"capture_limit\":{\"robot_state\":0,")) << meta;
}

TEST(CaptureRecorder, CancelDiscards) {
    Rig rig;
    const std::string id = rig.start(1, 5);
    rig.run(200000);
    std::string err;
    EXPECT_FALSE(rig.rec->cancel("nope", err));
    EXPECT_TRUE(contains(err, "no active capture")) << err;
    EXPECT_TRUE(rig.rec->cancel(id, err)) << err;
    rig.run(100000);
    const std::string status = rig.status();
    EXPECT_TRUE(contains(status, "\"state\":\"idle\"")) << status;
    EXPECT_TRUE(contains(status, "\"outcome\":\"cancelled\"")) << status;
    EXPECT_TRUE(contains(status, "\"captures\":[]")) << status;
    EXPECT_EQ(rig.rec->bundle(id), nullptr);
    EXPECT_FALSE(rig.rec->cancel(id, err)); // nothing left to cancel
    // idle again: a new capture starts
    rig.start(0, 0.1);
}

TEST(CaptureRecorder, OneCaptureAtATime) {
    Rig rig;
    const std::string first = rig.start(0, 0.5);
    CaptureRequest    q;
    std::string       id, err;
    EXPECT_FALSE(rig.rec->start(q, id, err));
    EXPECT_EQ(err.rfind("busy", 0), 0u) << err;
    EXPECT_TRUE(contains(err, first));
    rig.run(600000);
    EXPECT_TRUE(rig.rec->start(q, id, err)) << err;
    EXPECT_NE(id, first);

    // out of range requests
    Rig other;
    q.post_s = 0;
    EXPECT_FALSE(other.rec->start(q, id, err));
    q.post_s = 1;
    q.pre_s  = -1;
    EXPECT_FALSE(other.rec->start(q, id, err));
    q.pre_s = 1;
    q.kinds = 0;
    EXPECT_FALSE(other.rec->start(q, id, err));
    EXPECT_TRUE(contains(err, "no streams")) << err;
    // over the maximum is clamped, not refused
    q.kinds  = kCaptureDefaultKinds;
    q.post_s = 1000;
    EXPECT_TRUE(other.rec->start(q, id, err)) << err;
    EXPECT_TRUE(contains(other.status(), "\"post_s\":60,")) << other.status();
}

TEST(CaptureRecorder, AutoTriggersRespectCooldownAndHourlyCap) {
    CaptureConfig c     = testConfig();
    c.auto_faults       = captureFaultBit(CaptureFault::kLinkLost);
    c.auto_cooldown_s   = 120;
    c.auto_max_per_hour = 2;
    c.default_pre_s     = 0;
    c.default_post_s    = 1;
    Rig rig(c);
    const auto ready = [&] { return count(rig.status(), "\"state\":\"ready\""); };

    rig.rec->fault(CaptureFault::kPicoReboot, "not enabled"); // ignored
    rig.rec->fault(CaptureFault::kLinkLost, "Brain link quiet for 1 s");
    rig.rec->pump();
    std::string status = rig.status();
    EXPECT_TRUE(contains(status, "\"reason\":\"link_lost\",\"requester\":\"auto\"")) << status;
    EXPECT_TRUE(contains(status, "\"detail\":\"Brain link quiet for 1 s\"")) << status;
    rig.rec->fault(CaptureFault::kLinkLost, "again while recording");
    rig.run(1100000);
    EXPECT_EQ(ready(), 1u);

    rig.now += 10000000; // +10 s: inside the cooldown
    rig.rec->fault(CaptureFault::kLinkLost, "flapping");
    rig.run(1100000);
    EXPECT_EQ(ready(), 1u);

    rig.now += 120000000; // past the cooldown: the second of this hour
    rig.rec->fault(CaptureFault::kLinkLost, "lost again");
    rig.run(1100000);
    EXPECT_EQ(ready(), 2u);

    rig.now += 125000000; // past the cooldown, but two already this hour
    rig.rec->fault(CaptureFault::kLinkLost, "persistent");
    rig.run(1100000);
    EXPECT_EQ(ready(), 2u);
    status = rig.status();
    EXPECT_TRUE(contains(status, "\"suppressed\":{\"busy\":1,\"cooldown\":1,\"hourly_cap\":1}"))
        << status;
    EXPECT_TRUE(contains(status, "\"seen\":{\"link_lost\":5,\"continuity_lost\":0,"
                                 "\"pico_reboot\":0,"))
        << status;

    rig.now += 3600000000LL; // an hour later the cap has room again
    rig.rec->fault(CaptureFault::kLinkLost, "next hour");
    rig.run(1100000);
    EXPECT_EQ(ready(), 3u);
    EXPECT_TRUE(contains(rig.status(), "\"fired\":3,")) << rig.status();
}

// Review fix: the host metadata (profile, calibration, Pico identity) is kept
// as it was at the window start, next to the values when the bundle is built.
TEST(CaptureRecorder, HostMetadataAtTheWindowStartAndAtTheEnd) {
    const auto startOf = [](const std::string& meta, int64_t& sampled) {
        const std::size_t at = meta.find("\"host_at_start\":{");
        EXPECT_NE(at, std::string::npos) << meta;
        sampled = at == std::string::npos ? -1 : jsonInt(meta, "sampled_pi_host_us", at);
        return "\"host_at_start\":{\"sampled_pi_host_us\":" + std::to_string(sampled) +
               ",\"profile_probe\":";
    };
    {
        // changed during the post window
        auto host = std::make_shared<MutableHost>();
        Rig  rig(testConfig(), 16384, host);
        rig.run(3000000); // read once a second while idle
        const int64_t     trigger = rig.now;
        const std::string id      = rig.start(2, 1);
        rig.run(300000);
        host->set("B");
        rig.run(900000);
        const std::string meta = rig.bundle(id).files.at("metadata.json");
        int64_t           sampled = -1;
        EXPECT_TRUE(contains(meta, startOf(meta, sampled) + "\"A\"}")) << meta;
        EXPECT_LE(sampled, trigger - 2000000); // at or before the window start...
        EXPECT_GT(sampled, trigger - 2000000 - 1000000 - 20000); // ...by at most a period
        EXPECT_TRUE(contains(meta, "\"profile_probe\":\"B\",\"host_sampled_pi_host_us\":")) << meta;
        EXPECT_GE(jsonInt(meta, "host_sampled_pi_host_us"), trigger + 1000000);
    }
    {
        // changed just before an automatic trigger (a profile applied): the
        // pre-window ran under the old one
        CaptureConfig c  = testConfig();
        c.auto_faults    = captureFaultBit(CaptureFault::kProfileChanged);
        c.default_pre_s  = 2;
        c.default_post_s = 1;
        auto host        = std::make_shared<MutableHost>();
        Rig  rig(c, 16384, host);
        rig.run(3000000);
        host->set("B");
        rig.rec->fault(CaptureFault::kProfileChanged, "profile 0000abcd applied");
        rig.run(1200000);
        const ZipRead z = rig.bundle("01234567-1");
        ASSERT_EQ(z.files.count("metadata.json"), 1u);
        const std::string& meta = z.files.at("metadata.json");
        int64_t            sampled = -1;
        EXPECT_TRUE(contains(meta, startOf(meta, sampled) + "\"A\"}")) << meta;
        EXPECT_TRUE(contains(meta, "\"profile_probe\":\"B\",\"host_sampled_pi_host_us\":")) << meta;
        EXPECT_TRUE(contains(meta, "\"reason\":\"profile_changed\",\"requester\":\"auto\"")) << meta;
    }
    {
        // no rolling window, no pre-window: read when the capture begins
        CaptureConfig off = testConfig();
        off.rolling_s     = 0;
        auto host         = std::make_shared<MutableHost>();
        Rig  rig(off, 16384, host);
        const int64_t     trigger = rig.now;
        const std::string id      = rig.start(0, 1);
        rig.run(200000);
        host->set("B");
        rig.run(900000);
        const std::string meta = rig.bundle(id).files.at("metadata.json");
        int64_t           sampled = -1;
        EXPECT_TRUE(contains(meta, startOf(meta, sampled) + "\"A\"}")) << meta;
        EXPECT_GE(sampled, trigger);
        EXPECT_LE(sampled, trigger + 20000);
    }
}

TEST(CaptureRecorder, SegmentsAtSessionEpochBootAndResetChanges) {
    Rig rig;
    const std::string id = rig.start(0, 2, kCaptureDefaultKinds);
    const auto state = [&](double heading, uint64_t epoch, uint64_t anchor) {
        rig.post(DiagKind::kRobotState, rig.robot(++rig.pub, 0.1, heading, epoch, anchor), rig.now,
                 rig.robot_src);
        rig.now += 10000;
    };
    const auto pico = [&](uint16_t boot, int32_t enc0, uint8_t seq) {
        DiagPicoSensor p;
        p.version  = 2;
        p.boot_id  = boot;
        p.seq      = seq;
        p.mask     = translagatr::kSensorEnc0;
        p.stamp_ms = 100;
        p.enc[0]   = enc0;
        rig.post(DiagKind::kPicoSensor, p, rig.now, rig.pico_src);
        rig.now += 10000;
    };
    const auto brain = [&](uint32_t session) {
        DiagBrainRequest q;
        q.session    = session;
        q.request_id = 1;
        q.op         = translagatr::kOpGetState;
        q.request_len = 23;
        q.reply_len   = 59;
        rig.post(DiagKind::kBrainRequest, q, rig.now, rig.brain_src);
        rig.now += 10000;
    };
    state(170, 1, 1);
    state(179, 1, 1);
    state(-179, 1, 1); // crosses +-180 inside one segment
    pico(100, 10, 7);
    pico(100, 25, 8);
    brain(7);
    rig.rec->pump();
    rig.rec->noteReset(1);
    rig.now += 1000;
    state(-175, 2, 1); // new odometry epoch
    state(-170, 2, 2); // new anchor
    pico(200, 3, 9);   // Pico rebooted: counters rebased (seq happens to follow on)
    brain(8);
    rig.wait(2100000);

    const ZipRead z = rig.bundle(id);
    const Table   t = table(z, "robot_state.csv");
    ASSERT_GE(t.rows.size(), 5u);
    EXPECT_EQ(t.at(0, "segment"), "0");
    EXPECT_EQ(t.at(2, "segment"), "0");
    EXPECT_NEAR(t.num(2, "odom_heading_deg"), -179.0, 1e-4);
    EXPECT_NEAR(t.num(2, "odom_heading_unwrapped_deg"), 181.0, 1e-4);
    EXPECT_EQ(t.at(3, "segment"), "1");
    EXPECT_NEAR(t.num(3, "odom_heading_unwrapped_deg"), -175.0, 1e-4); // restarts, not joined
    EXPECT_EQ(t.at(4, "segment"), "2");
    EXPECT_EQ(t.at(2, "reset_count"), "0");
    EXPECT_EQ(t.at(3, "reset_count"), "1");

    const Table p = table(z, "pico_sensor.csv");
    ASSERT_EQ(p.rows.size(), 3u);
    EXPECT_EQ(p.at(0, "enc0_delta_counts"), "");
    EXPECT_EQ(p.at(1, "enc0_delta_counts"), "15");
    EXPECT_EQ(p.at(2, "enc0_delta_counts"), ""); // never across a reboot
    EXPECT_EQ(p.at(2, "boot_id"), "200");
    EXPECT_EQ(p.at(0, "seq_gap_frames"), "");
    EXPECT_EQ(p.at(1, "seq_gap_frames"), "0");
    EXPECT_EQ(p.at(2, "seq_gap_frames"), ""); // the first frame of that boot

    const std::string& meta = z.files.at("metadata.json");
    EXPECT_EQ(count(meta, "\"domain\":\"robot\""), 3u) << meta;
    EXPECT_EQ(count(meta, "\"domain\":\"pico\""), 2u);
    EXPECT_EQ(count(meta, "\"domain\":\"brain\""), 2u);
    EXPECT_EQ(count(meta, "\"domain\":\"pi\""), 2u);
    EXPECT_TRUE(contains(meta, "\"brain_session\":8")) << meta;
    EXPECT_TRUE(contains(meta, "\"boot_id\":200"));
    const Table e = table(z, "events.csv");
    bool reset_marker = false;
    for (std::size_t r = 0; r < e.rows.size(); ++r) {
        reset_marker = reset_marker || e.at(r, "text") == "Pi reset: reset count 1";
    }
    EXPECT_TRUE(reset_marker);
}

TEST(CaptureRecorder, DropAccounting) {
    Rig rig(testConfig(), 64);
    rig.rec->pump();
    rig.now += 200000;
    const std::string id = rig.start(0, 1, diagBit(DiagKind::kRobotState) | diagBit(DiagKind::kEvent));
    // the recorder falls behind: the hub ring (64) drops the rest
    for (int i = 0; i < 200; ++i) {
        rig.post(DiagKind::kRobotState, rig.robot(++rig.pub, 0, 0), rig.now + i, rig.robot_src);
    }
    rig.now += 1000;
    rig.rec->pump();
    // a producer bug: the payload is not the stream's; skipped and counted
    rig.post(DiagKind::kRobotState, DiagEvent{}, rig.now, rig.robot_src);
    // a stream this capture did not choose
    for (int i = 0; i < 5; ++i) {
        DiagPicoSensor p;
        p.version = 1;
        rig.post(DiagKind::kPicoSensor, p, rig.now, rig.pico_src);
        rig.now += 1000;
    }
    rig.run(1100000, 100000);
    const ZipRead z = rig.bundle(id);
    EXPECT_EQ(z.files.count("pico_sensor.csv"), 0u);
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"hub_ring_full\":{\"robot_state\":136,")) << meta;
    EXPECT_TRUE(contains(meta, "\"malformed\":{\"robot_state\":1,")) << meta;
    EXPECT_TRUE(contains(meta, "\"not_selected\":{\"robot_state\":0,\"pico_sensor\":5,")) << meta;
    EXPECT_TRUE(contains(meta, "\"pico_sensor\":{\"selected\":false,\"file\":null,\"rows\":0"))
        << meta;
    EXPECT_TRUE(contains(rig.status(), "\"dropped\":{\"robot_state\":136}")) << rig.status();
}

// ---- CSV, ZIP and values ------------------------------------------------------------

TEST(CaptureBundle, CsvEscapingAndMissingValues) {
    EXPECT_EQ(csvEscape("plain text"), "plain text");
    EXPECT_EQ(csvEscape("a,b"), "\"a,b\"");
    EXPECT_EQ(csvEscape("say \"hi\""), "\"say \"\"hi\"\"\"");
    EXPECT_EQ(csvEscape("two\nlines"), "\"two\nlines\"");

    Rig rig;
    const std::string id = rig.start(0, 1, kDiagAllKinds);
    DiagEvent         e;
    std::strcpy(e.text, "profile 1a2b applied (two wheels, \"vex\" imu)");
    rig.post(DiagKind::kEvent, e, rig.now, rig.hub.sourceId("system"));
    DiagRobotState invalid = rig.robot(1, 0.5, 10);
    invalid.valid          = false;
    invalid.initialized    = false;
    invalid.measured_clock = DiagClock::kNone;
    invalid.measured_host_ms = -1;
    invalid.advanced         = false;
    rig.post(DiagKind::kRobotState, invalid, rig.now + 1, rig.robot_src);
    DiagRobotState unplaced = rig.robot(2, 0.5, 10);
    unplaced.initialized    = false;
    rig.post(DiagKind::kRobotState, unplaced, rig.now + 2, rig.robot_src);
    DiagRobotState repeated = unplaced; // the held pose published again
    repeated.publication    = 3;
    repeated.advanced       = false;
    rig.post(DiagKind::kRobotState, repeated, rig.now + 3, rig.robot_src);
    DiagPicoSensor partial;
    partial.version = 1;
    partial.mask    = translagatr::kSensorEnc0 | translagatr::kSensorEnc2;
    partial.enc[0]  = 4;
    partial.enc[2]  = -9;
    rig.post(DiagKind::kPicoSensor, partial, rig.now + 3, rig.pico_src);
    DiagVexImu none;
    none.session = 3;
    rig.post(DiagKind::kVexImu, none, rig.now + 4, rig.brain_src);
    DiagPicoDiag garbage;
    garbage.len        = 3;
    garbage.payload[0] = 0xDE;
    rig.post(DiagKind::kPicoDiag, garbage, rig.now + 5, rig.pico_src);
    DiagBrainTelemetry refused;
    refused.session = 77;
    refused.len     = 3;
    refused.body[0] = 0xDE;
    rig.post(DiagKind::kBrainTelemetry, refused, rig.now + 5, rig.brain_src);
    DiagBrainRequest silent;
    silent.op          = translagatr::kOpPathReport;
    silent.request_len = 20;
    rig.post(DiagKind::kBrainRequest, silent, rig.now + 6, rig.brain_src);
    rig.wait(1100000);
    const ZipRead z = rig.bundle(id);

    const Table e_t = table(z, "events.csv");
    bool        found = false;
    for (std::size_t r = 0; r < e_t.rows.size(); ++r) {
        if (e_t.at(r, "kind") == "runtime") {
            EXPECT_EQ(e_t.at(r, "text"), e.text);
            EXPECT_EQ(e_t.at(r, "source"), "system");
            found = true;
        }
    }
    EXPECT_TRUE(found);

    const Table t = table(z, "robot_state.csv");
    ASSERT_EQ(t.rows.size(), 3u);
    for (const char* col : {"source_clock", "source_ms", "measured_pi_host_ms", "odom_x_m",
                            "odom_heading_deg", "field_x_m", "odom_vx_m_s", "confidence",
                            "roll_deg", "placement_sequence"}) {
        EXPECT_EQ(t.at(0, col), "") << col; // missing, never zero
    }
    EXPECT_EQ(t.at(0, "valid"), "0");
    EXPECT_EQ(t.at(0, "new_measurement"), "0"); // not advanced: never empty
    EXPECT_EQ(t.at(0, "attitude_status"), "assumed_level");
    EXPECT_EQ(t.at(1, "odom_x_m"), "0.500000");
    EXPECT_EQ(t.at(1, "field_x_m"), ""); // valid but not placed
    EXPECT_EQ(t.at(1, "placed"), "0");
    EXPECT_EQ(t.at(1, "new_measurement"), "1"); // the first measured row is known too
    EXPECT_EQ(t.at(2, "new_measurement"), "0"); // the same measurement published again
    EXPECT_EQ(t.at(2, "odom_x_m"), "0.500000");  // the held pose is still written

    const Table p = table(z, "pico_sensor.csv");
    ASSERT_EQ(p.rows.size(), 1u);
    EXPECT_EQ(p.at(0, "enc0_counts"), "4");
    EXPECT_EQ(p.at(0, "enc1_counts"), "");
    EXPECT_EQ(p.at(0, "enc2_counts"), "-9");
    EXPECT_EQ(p.at(0, "gyro_z_deg_s"), "");
    EXPECT_EQ(p.at(0, "boot_id"), ""); // v1 frames carry no identity

    const Table v = table(z, "vex_imu.csv");
    ASSERT_EQ(v.rows.size(), 1u);
    EXPECT_EQ(v.at(0, "valid"), "0");
    EXPECT_EQ(v.at(0, "rotation_deg"), "");
    EXPECT_EQ(v.at(0, "source_ms"), "");

    const Table d = table(z, "pico_diag.csv");
    ASSERT_EQ(d.rows.size(), 1u);
    EXPECT_EQ(d.at(0, "decoded"), "0");
    EXPECT_EQ(d.at(0, "pin_pi_rx_level"), "");
    EXPECT_EQ(d.at(0, "payload_hex"), "DE0000");

    const Table bt = table(z, "brain_telemetry.csv");
    ASSERT_EQ(bt.rows.size(), 1u);
    EXPECT_EQ(bt.at(0, "decoded"), "0");
    EXPECT_EQ(bt.at(0, "target_valid"), "");
    EXPECT_EQ(bt.at(0, "wheel5_rpm"), "");
    EXPECT_EQ(bt.at(0, "payload_hex"), "DE0000");
    EXPECT_EQ(bt.rows[0].size(), bt.header.size()); // the filler matches the header

    const Table q = table(z, "brain_requests.csv");
    ASSERT_EQ(q.rows.size(), 1u);
    EXPECT_EQ(q.at(0, "replied"), "0");
    EXPECT_EQ(q.at(0, "result"), "");
    EXPECT_EQ(q.at(0, "op_name"), "PATH_REPORT");

    // every selected stream has a file, even with no rows
    EXPECT_EQ(table(z, "transport_bytes.csv").rows.size(), 0u);
    EXPECT_EQ(table(z, "paths.csv").rows.size(), 0u);
}

// Review fix: a delta is one frame's change; lost frames leave it empty and
// seq_gap_frames says how many are missing.
TEST(CaptureBundle, EncoderDeltasOnlyBetweenConsecutiveFrames) {
    Rig rig;
    const std::string id = rig.start(0, 1, diagBit(DiagKind::kPicoSensor));
    const auto frame = [&](uint8_t seq, int32_t enc0, const int32_t* enc1) {
        DiagPicoSensor p;
        p.version = 2;
        p.boot_id = 77;
        p.seq     = seq;
        p.mask    = translagatr::kSensorEnc0;
        p.enc[0]  = enc0;
        if (enc1 != nullptr) {
            p.mask   = static_cast<uint8_t>(p.mask | translagatr::kSensorEnc1);
            p.enc[1] = *enc1;
        }
        rig.post(DiagKind::kPicoSensor, p, rig.now, rig.pico_src);
        rig.now += 10000;
    };
    const int32_t e1[] = {1000, 1005, 1020, 1030, 1040, 1041};
    frame(1, 100, &e1[0]);
    frame(2, 110, &e1[1]);
    frame(5, 150, &e1[2]); // 3 and 4 lost
    frame(6, 151, nullptr); // port 1 not in this frame
    frame(7, 153, &e1[3]);
    frame(255, 400, &e1[4]);
    frame(0, 405, &e1[5]); // seq wraps: still consecutive
    rig.wait(1100000);
    const Table p = table(rig.bundle(id), "pico_sensor.csv");
    ASSERT_EQ(p.rows.size(), 7u);
    const char* d0[]  = {"", "10", "", "1", "2", "", "5"};
    const char* d1[]  = {"", "5", "", "", "", "", "1"};
    const char* gap[] = {"", "0", "2", "0", "0", "247", "0"};
    for (std::size_t r = 0; r < p.rows.size(); ++r) {
        EXPECT_EQ(p.at(r, "enc0_delta_counts"), d0[r]) << r;
        EXPECT_EQ(p.at(r, "enc1_delta_counts"), d1[r]) << r;
        EXPECT_EQ(p.at(r, "seq_gap_frames"), gap[r]) << r;
    }
    EXPECT_EQ(p.at(2, "enc0_counts"), "150"); // the counts stay across a gap
    EXPECT_EQ(p.at(3, "enc1_counts"), "");
}

TEST(CaptureBundle, ZipRoundTrip) {
    std::vector<ZipEntry> entries = {{"a.txt", "hello\n"},
                                     {"empty.csv", ""},
                                     {"bin.dat", std::string("\0\x01\xff\x7f", 4)}};
    std::string big(100000, 'x');
    for (std::size_t i = 0; i < big.size(); ++i) {
        big[i] = static_cast<char>(i * 31 + 7);
    }
    entries.push_back({"dir/big.bin", big});
    // text built in chunks is stored as its concatenation
    entries.push_back({"chunked.csv", "a,b\n", {"1,2\n", "", big.substr(0, 70000), "3,4\n"}});
    std::string zip, err;
    ASSERT_TRUE(buildStoredZip(entries, zipTimeFromUnixMs(1790000000000), zip, err)) << err;
    ZipRead z;
    ASSERT_TRUE(readZip(zip, z, err)) << err;
    ASSERT_EQ(z.order.size(), entries.size());
    for (const ZipEntry& e : entries) {
        std::string whole = e.data;
        for (const std::string& c : e.chunks) {
            whole += c;
        }
        EXPECT_EQ(z.files.at(e.name), whole) << e.name;
        EXPECT_EQ(translagatr::crc32(reinterpret_cast<const uint8_t*>(whole.data()),
                                     static_cast<uint32_t>(whole.size())),
                  refCrc32(whole));
    }
    const ZipTime t = zipTimeFromUnixMs(1790000000000); // 2026-09-21 14:13:20 UTC
    EXPECT_EQ(t.date, ((2026 - 1980) << 9) | (9 << 5) | 21);
    EXPECT_EQ(t.time, (14 << 11) | (13 << 5) | 10);
    EXPECT_EQ(zipTimeFromUnixMs(-1).date, (1 << 5) | 1); // unknown: 1980-01-01

    // a capture bundle opens the same way and lists what it holds
    Rig rig;
    const std::string id = rig.start(0, 0.2);
    rig.run(300000);
    const ZipRead b = rig.bundle(id);
    EXPECT_EQ(b.order.front(), "metadata.json");
    EXPECT_EQ(b.files.count("README.txt"), 1u);
    for (int k = 0; k < kDiagKindCount; ++k) {
        const auto kind = static_cast<DiagKind>(k);
        EXPECT_EQ(b.files.count(captureStreamFile(kind)),
                  (kCaptureDefaultKinds & diagBit(kind)) != 0 ? 1u : 0u)
            << captureStreamFile(kind);
    }
    EXPECT_TRUE(contains(b.files.at("README.txt"), "An empty field means missing"));
}

TEST(CaptureBundle, ReplayConsistency) {
    Rig rig;
    const std::string id = rig.start(0, 1, kDiagAllKinds);
    const int64_t     t0 = rig.now;

    std::vector<DiagRobotState> states;
    for (int i = 0; i < 20; ++i) {
        DiagRobotState s     = rig.robot(100 + i, 0.123456 * i, -170.0 + 37.5 * i, 4, 2);
        s.attitude_valid     = i % 2 == 0;
        s.attitude_assumed_level = false;
        s.roll_rad           = 0.01f * i;
        s.pitch_rad          = -0.02f * i;
        s.stationary         = i == 5;
        s.advanced           = i % 3 != 1; // source_ms moves every row; the flag decides
        states.push_back(s);
        rig.post(DiagKind::kRobotState, s, t0 + i * 10000, rig.robot_src);
    }
    DiagPicoSensor ps;
    ps.version     = 2;
    ps.boot_id     = 4321;
    ps.acq_epoch   = 2;
    ps.imu_epoch   = 3;
    ps.seq         = 9;
    ps.mask        = 0x0F;
    ps.stamp_ms    = 123456;
    ps.enc[0]      = 1000;
    ps.enc[1]      = -2000;
    ps.enc[2]      = 7;
    ps.gyro_z_mdps = -12345;
    rig.post(DiagKind::kPicoSensor, ps, t0 + 1, rig.pico_src);

    DiagPicoStatus st;
    st.status.boot_id         = 4321;
    st.status.uptime_ms       = 99000;
    st.status.imu_state       = translagatr::kPicoImuReady;
    st.status.imu_attempts    = 2;
    st.status.flags           = translagatr::kPicoImuEnabled;
    st.status.last_request_id = 5;
    st.status.last_op         = translagatr::kPicoOpDiagnostics;
    st.status.last_status     = translagatr::kPicoCommandCompleted;
    st.status.firmware        = translagatr::kPicoFirmwareBno08x;
    rig.post(DiagKind::kPicoStatus, st, t0 + 2, rig.pico_src);

    translagatr::PicoDiag pd;
    pd.boot_id       = 4321;
    pd.seq           = 17;
    pd.firmware      = translagatr::kPicoFirmwareBno08x;
    pd.pins          = translagatr::kPicoPinPiRx | translagatr::kPicoPinImuInt;
    pd.pins_known    = translagatr::kPicoPinPiRx | translagatr::kPicoPinImuInt |
                    translagatr::kPicoPinEnc0A;
    pd.imu_rx        = 1234;
    pd.imu_error     = -3;
    pd.report_age_ms = 0xFFFF;
    pd.ticks_skipped = 2;
    pd.flags         = translagatr::kPicoDiagImuPresent;
    uint8_t frame[64];
    const uint16_t flen = translagatr::encodePicoDiag(pd, frame, sizeof(frame));
    ASSERT_EQ(flen, 32u);
    DiagPicoDiag as_payload; // the 26-byte payload
    as_payload.len = translagatr::kPicoDiagLen;
    std::memcpy(as_payload.payload, frame + 4, as_payload.len);
    rig.post(DiagKind::kPicoDiag, as_payload, t0 + 3, rig.pico_src);
    DiagPicoDiag as_frame; // or the whole frame
    as_frame.len = static_cast<uint8_t>(flen);
    std::memcpy(as_frame.payload, frame, flen);
    rig.post(DiagKind::kPicoDiag, as_frame, t0 + 4, rig.pico_src);

    DiagVexImu vex;
    vex.session       = 77;
    vex.request_id    = 12;
    vex.flags         = translagatr::kBenchImuValid;
    vex.stamp_ms      = 4567;
    vex.rotation_mdeg = -367250;
    vex.accepted      = true;
    rig.post(DiagKind::kVexImu, vex, t0 + 5, rig.brain_src);

    DiagBrainRequest br;
    br.session     = 77;
    br.request_id  = 12;
    br.op          = translagatr::kOpGetState;
    br.result      = translagatr::kResultOk;
    br.request_len = 23;
    br.reply_len   = 59;
    br.duplicate   = true;
    rig.post(DiagKind::kBrainRequest, br, t0 + 6, rig.brain_src);

    translagatr::BrainRequest tr;
    tr.op         = translagatr::kOpTelemetry;
    tr.session    = 77;
    tr.request_id = 13;
    translagatr::BrainTelemetry& tel = tr.telemetry;
    tel.flags              = translagatr::kTelemetryAttitude | translagatr::kTelemetryMotion |
                translagatr::kTelemetryWheels | translagatr::kTelemetryTarget;
    tel.stamp_ms           = 4600;
    tel.roll_cdeg          = 150;
    tel.pitch_cdeg         = -275;
    tel.command_id         = 42;
    tel.motion_state       = 2;
    tel.segment            = 1;
    tel.segment_count      = 3;
    tel.target_x_mm        = 1500;
    tel.target_y_mm        = -250;
    tel.target_heading_cdeg = 9000;
    tel.cmd_vx_mm_s        = 400;
    tel.cmd_omega_cdeg_s   = -1500;
    tel.cross_track_mm     = 12;
    tel.wheel_count        = 2;
    tel.wheel_rpm_x10[0]   = 1234;
    tel.wheel_rpm_x10[1]   = -56;
    uint8_t        tframe[translagatr::kMaxFrameLen];
    const uint16_t tlen = translagatr::encodeBrainRequest(tr, tframe, sizeof(tframe));
    ASSERT_GT(tlen, 0u);
    DiagBrainTelemetry body; // the 54-byte body
    body.session = 77;
    body.len     = translagatr::kTelemetryBodyLen;
    std::memcpy(body.body, tframe + 4 + translagatr::kBrainRequestHeaderLen, body.len);
    rig.post(DiagKind::kBrainTelemetry, body, t0 + 7, rig.brain_src);
    DiagBrainTelemetry whole; // or the whole frame
    whole.session = 77;
    whole.len     = static_cast<uint8_t>(tlen);
    std::memcpy(whole.body, tframe, tlen);
    rig.post(DiagKind::kBrainTelemetry, whole, t0 + 8, rig.brain_src);
    const uint8_t variants[2] = {
        // target bit clear: the wire still carries target_* (the codec keeps it)
        static_cast<uint8_t>(tel.flags & ~translagatr::kTelemetryTarget),
        // motion bit clear: the whole group, target_valid included, is absent
        static_cast<uint8_t>(translagatr::kTelemetryAttitude | translagatr::kTelemetryTarget)};
    for (int k = 0; k < 2; ++k) {
        translagatr::BrainRequest other = tr;
        other.telemetry.flags           = variants[k];
        uint8_t        oframe[translagatr::kMaxFrameLen];
        const uint16_t olen = translagatr::encodeBrainRequest(other, oframe, sizeof(oframe));
        ASSERT_GT(olen, 0u);
        DiagBrainTelemetry ob;
        ob.session = 77;
        ob.len     = static_cast<uint8_t>(olen);
        std::memcpy(ob.body, oframe, olen);
        rig.post(DiagKind::kBrainTelemetry, ob, t0 + 11 + k, rig.brain_src);
    }

    DiagPath path;
    path.session    = 77;
    path.command_id = 42;
    path.mode       = translagatr::kPathAvoiding;
    path.count      = 2;
    path.points[0]  = {100, 200};
    path.points[1]  = {-300, 450};
    rig.post(DiagKind::kPath, path, t0 + 9, rig.brain_src);

    DiagBytes bytes;
    bytes.dir     = DiagDirection::kTxAttempted;
    bytes.len     = 3;
    bytes.data[0] = 0xAA;
    bytes.data[1] = 0x55;
    bytes.data[2] = 0x0F;
    rig.post(DiagKind::kBytes, bytes, t0 + 10, rig.pico_src);

    rig.now = t0 + 300000;
    rig.wait(800000);
    const ZipRead z = rig.bundle(id);

    const Table r = table(z, "robot_state.csv");
    ASSERT_EQ(r.rows.size(), states.size());
    for (std::size_t i = 0; i < states.size(); ++i) {
        const DiagRobotState& s = states[i];
        EXPECT_EQ(r.at(i, "pi_host_us"), std::to_string(t0 + static_cast<int64_t>(i) * 10000));
        EXPECT_EQ(r.at(i, "pi_session"), "0123456789abcdef");
        EXPECT_EQ(r.at(i, "source"), "localization");
        EXPECT_EQ(r.at(i, "publication"), std::to_string(s.publication));
        EXPECT_EQ(r.at(i, "source_clock"), "pico");
        EXPECT_EQ(r.at(i, "source_ms"), std::to_string(s.measured_ms));
        EXPECT_EQ(r.at(i, "odometry_epoch"), "4");
        EXPECT_EQ(r.at(i, "anchor_revision"), "2");
        EXPECT_EQ(r.at(i, "placement_session"), "7");
        EXPECT_NEAR(r.num(i, "odom_x_m"), s.odom_x_m, 1e-6);
        EXPECT_NEAR(r.num(i, "odom_y_m"), s.odom_y_m, 1e-6);
        EXPECT_NEAR(r.num(i, "field_x_m"), s.field_x_m, 1e-6);
        double h = s.odom_heading_rad * 180.0 / kTestPi;
        h        = std::remainder(h, 360.0);
        if (h <= -180.0) {
            h += 360.0;
        }
        EXPECT_NEAR(r.num(i, "odom_heading_deg"), h, 1e-4);
        EXPECT_GT(r.num(i, "odom_heading_deg"), -180.0);
        EXPECT_LE(r.num(i, "odom_heading_deg"), 180.0);
        EXPECT_NEAR(r.num(i, "odom_heading_unwrapped_deg"), -170.0 + 37.5 * i, 1e-3);
        EXPECT_NEAR(r.num(i, "odom_vx_m_s"), s.vx_m_s, 1e-5);
        EXPECT_NEAR(r.num(i, "odom_vy_m_s"), s.vy_m_s, 1e-5);
        EXPECT_NEAR(r.num(i, "yaw_rate_deg_s"), s.yaw_rate_rad_s * 180.0 / kTestPi, 1e-4);
        EXPECT_NEAR(r.num(i, "confidence"), s.confidence, 1e-4);
        EXPECT_EQ(r.at(i, "stationary"), s.stationary ? "1" : "0");
        EXPECT_EQ(r.at(i, "new_measurement"), s.advanced ? "1" : "0") << i;
        if (s.attitude_valid) {
            EXPECT_EQ(r.at(i, "attitude_status"), "measured");
            EXPECT_NEAR(r.num(i, "roll_deg"), s.roll_rad * 180.0 / kTestPi, 1e-4);
            EXPECT_NEAR(r.num(i, "pitch_deg"), s.pitch_rad * 180.0 / kTestPi, 1e-4);
        } else {
            EXPECT_EQ(r.at(i, "attitude_status"), "unavailable");
            EXPECT_EQ(r.at(i, "roll_deg"), "");
        }
    }

    const Table p = table(z, "pico_sensor.csv");
    ASSERT_EQ(p.rows.size(), 1u);
    EXPECT_EQ(p.at(0, "source"), "pico");
    EXPECT_EQ(p.at(0, "source_ms"), "123456");
    EXPECT_EQ(p.at(0, "boot_id"), "4321");
    EXPECT_EQ(p.at(0, "acq_epoch"), "2");
    EXPECT_EQ(p.at(0, "imu_epoch"), "3");
    EXPECT_EQ(p.at(0, "seq"), "9");
    EXPECT_EQ(p.at(0, "enc1_counts"), "-2000");
    EXPECT_EQ(p.at(0, "gyro_z_deg_s"), "-12.345");

    const Table s = table(z, "pico_status.csv");
    ASSERT_EQ(s.rows.size(), 1u);
    EXPECT_EQ(s.at(0, "source_ms"), "99000");
    EXPECT_EQ(s.at(0, "imu_state_name"), "ready");
    EXPECT_EQ(s.at(0, "imu_enabled"), "1");
    EXPECT_EQ(s.at(0, "last_op_name"), "diagnostics");
    EXPECT_EQ(s.at(0, "last_status_name"), "completed");
    EXPECT_EQ(s.at(0, "firmware_name"), "bno08x");

    const Table d = table(z, "pico_diag.csv");
    ASSERT_EQ(d.rows.size(), 2u);
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_EQ(d.at(i, "decoded"), "1") << i;
        EXPECT_EQ(d.at(i, "boot_id"), "4321");
        EXPECT_EQ(d.at(i, "seq"), "17");
        EXPECT_EQ(d.at(i, "pin_pi_rx_level"), "1");
        EXPECT_EQ(d.at(i, "pin_imu_int_level"), "1");
        EXPECT_EQ(d.at(i, "pin_enc0_a_level"), "0");
        EXPECT_EQ(d.at(i, "pin_enc0_b_level"), ""); // not sampled
        EXPECT_EQ(d.at(i, "imu_rx"), "1234");
        EXPECT_EQ(d.at(i, "imu_error"), "-3");
        EXPECT_EQ(d.at(i, "report_age_ms"), ""); // 0xFFFF: no report yet
        EXPECT_EQ(d.at(i, "ticks_skipped"), "2");
        EXPECT_EQ(d.at(i, "imu_present"), "1");
    }

    const Table v = table(z, "vex_imu.csv");
    ASSERT_EQ(v.rows.size(), 1u);
    EXPECT_EQ(v.at(0, "brain_session"), "77");
    EXPECT_EQ(v.at(0, "source_clock"), "brain");
    EXPECT_EQ(v.at(0, "source_ms"), "4567");
    EXPECT_EQ(v.at(0, "rotation_deg"), "-367.250");
    EXPECT_EQ(v.at(0, "accepted"), "1");

    const Table q = table(z, "brain_requests.csv");
    ASSERT_EQ(q.rows.size(), 1u);
    EXPECT_EQ(q.at(0, "op_name"), "GET_STATE");
    EXPECT_EQ(q.at(0, "result_name"), "ok");
    EXPECT_EQ(q.at(0, "reply_len_bytes"), "59");
    EXPECT_EQ(q.at(0, "duplicate"), "1");

    const Table b = table(z, "brain_telemetry.csv");
    ASSERT_EQ(b.rows.size(), 4u);
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_EQ(b.at(i, "decoded"), "1") << i;
        EXPECT_EQ(b.at(i, "source_ms"), "4600");
        EXPECT_EQ(b.at(i, "roll_deg"), "1.50");
        EXPECT_EQ(b.at(i, "pitch_deg"), "-2.75");
        EXPECT_EQ(b.at(i, "command_id"), "42");
        EXPECT_EQ(b.at(i, "path_segment_count"), "3");
        EXPECT_EQ(b.at(i, "target_valid"), "1");
        EXPECT_EQ(b.at(i, "target_field_x_m"), "1.500");
        EXPECT_EQ(b.at(i, "target_field_y_m"), "-0.250");
        EXPECT_EQ(b.at(i, "target_field_heading_deg"), "90.00");
        EXPECT_EQ(b.at(i, "cmd_body_vx_m_s"), "0.400");
        EXPECT_EQ(b.at(i, "cmd_omega_deg_s"), "-15.00");
        EXPECT_EQ(b.at(i, "cross_track_m"), "0.012");
        EXPECT_EQ(b.at(i, "wheel_count"), "2");
        EXPECT_EQ(b.at(i, "wheel0_rpm"), "123.4");
        EXPECT_EQ(b.at(i, "wheel1_rpm"), "-5.6");
        EXPECT_EQ(b.at(i, "wheel2_rpm"), "");
    }
    // target bit clear: no destination shown, the rest of the motion group is
    EXPECT_EQ(b.at(2, "motion_present"), "1");
    EXPECT_EQ(b.at(2, "target_valid"), "0");
    for (const char* col : {"target_field_x_m", "target_field_y_m", "target_field_heading_deg"}) {
        EXPECT_EQ(b.at(2, col), "") << col;
    }
    EXPECT_EQ(b.at(2, "command_id"), "42");
    EXPECT_EQ(b.at(2, "cmd_body_vx_m_s"), "0.400");
    EXPECT_EQ(b.at(2, "wheel0_rpm"), "123.4");
    // no motion group: target_valid is missing like the group, whatever bit 3 says
    EXPECT_EQ(b.at(3, "motion_present"), "0");
    EXPECT_EQ(b.at(3, "target_valid"), "");
    EXPECT_EQ(b.at(3, "target_field_x_m"), "");
    EXPECT_EQ(b.at(3, "cmd_body_vx_m_s"), "");
    EXPECT_EQ(b.at(3, "roll_deg"), "1.50");
    EXPECT_EQ(b.at(3, "wheels_present"), "0");
    EXPECT_EQ(b.at(3, "flags"), std::to_string(translagatr::kTelemetryAttitude |
                                               translagatr::kTelemetryTarget));
    for (std::size_t i = 0; i < b.rows.size(); ++i) { // one field per header column
        EXPECT_EQ(b.rows[i].size(), b.header.size()) << i;
    }

    const Table pa = table(z, "paths.csv");
    ASSERT_EQ(pa.rows.size(), 2u);
    EXPECT_EQ(pa.at(1, "mode_name"), "avoiding");
    EXPECT_EQ(pa.at(1, "point_index"), "1");
    EXPECT_EQ(pa.at(1, "field_x_m"), "-0.300");
    EXPECT_EQ(pa.at(1, "field_y_m"), "0.450");

    const Table tb = table(z, "transport_bytes.csv");
    ASSERT_EQ(tb.rows.size(), 1u);
    EXPECT_EQ(tb.at(0, "direction"), "tx_attempted");
    EXPECT_EQ(tb.at(0, "len_bytes"), "3");
    EXPECT_EQ(tb.at(0, "hex"), "AA550F");

    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"robot_state\":{\"selected\":true,\"file\":\"robot_state.csv\","
                               "\"rows\":20,"))
        << meta;
}

// ---- HTTP -------------------------------------------------------------------------

TEST(CaptureHttp, Routes) {
    Rig          rig;
    HttpResponse r;
    EXPECT_FALSE(captureRoute(rig.rec.get(), request("GET", "/api/snapshot"), r));
    EXPECT_FALSE(isCaptureRoute("/api/captures"));
    EXPECT_TRUE(isCaptureRoute("/api/capture/status"));

    ASSERT_TRUE(captureRoute(rig.rec.get(), request("GET", "/api/capture/status"), r));
    EXPECT_EQ(r.status, 200);
    EXPECT_EQ(r.content_type, "application/json");
    EXPECT_TRUE(contains(r.body, "\"state\":\"idle\""));

    captureRoute(rig.rec.get(), request("GET", "/api/capture/start"), r);
    EXPECT_EQ(r.status, 405);
    captureRoute(rig.rec.get(), request("POST", "/api/capture/start", "streams=robot_state,warp"), r);
    EXPECT_EQ(r.status, 400);
    EXPECT_TRUE(contains(r.body, "warp"));
    captureRoute(rig.rec.get(), request("POST", "/api/capture/start", "pre_s=abc"), r);
    EXPECT_EQ(r.status, 400);

    captureRoute(rig.rec.get(),
                 request("POST", "/api/capture/start",
                         "pre_s=0&post_s=0.5&streams=robot_state,event&requester=viewer%203"),
                 r);
    ASSERT_EQ(r.status, 200) << r.body;
    EXPECT_TRUE(contains(r.body, "\"ok\":true,\"id\":\"01234567-1\"")) << r.body;
    captureRoute(rig.rec.get(), request("POST", "/api/capture/start"), r);
    EXPECT_EQ(r.status, 409);
    EXPECT_TRUE(contains(r.body, "\"ok\":false")) << r.body;

    captureRoute(rig.rec.get(), request("GET", "/api/capture/01234567-1.zip"), r);
    EXPECT_EQ(r.status, 404); // not ready yet
    rig.run(600000);
    captureRoute(rig.rec.get(), request("GET", "/api/capture/01234567-1.zip"), r);
    ASSERT_EQ(r.status, 200) << r.body;
    EXPECT_EQ(r.content_type, "application/zip");
    ASSERT_NE(r.shared_body, nullptr);
    EXPECT_TRUE(r.body.empty());
    EXPECT_EQ(r.shared_body, rig.rec->bundle("01234567-1")); // the kept bundle, not a copy
    ZipRead     z;
    std::string err;
    ASSERT_TRUE(readZip(*r.shared_body, z, err)) << err;
    EXPECT_TRUE(contains(z.files.at("metadata.json"), "\"requester\":\"viewer 3\""));
    EXPECT_EQ(z.files.count("pico_sensor.csv"), 0u);

    captureRoute(rig.rec.get(), request("GET", "/api/capture/..%2F..%2Fetc.zip"), r);
    EXPECT_EQ(r.status, 400);
    captureRoute(rig.rec.get(), request("POST", "/api/capture/cancel", "id=01234567-1"), r);
    EXPECT_EQ(r.status, 404); // finished captures are not cancelled
    captureRoute(rig.rec.get(), request("GET", "/api/capture/other"), r);
    EXPECT_EQ(r.status, 404);

    // defaults come from the configuration when the query leaves them out
    captureRoute(rig.rec.get(), request("POST", "/api/capture/start"), r);
    ASSERT_EQ(r.status, 200) << r.body;
    EXPECT_TRUE(contains(rig.status(), "\"pre_s\":2,\"post_s\":1,")) << rig.status();
    captureRoute(rig.rec.get(), request("POST", "/api/capture/cancel", "id=01234567-2"), r);
    EXPECT_EQ(r.status, 200) << r.body;

    ASSERT_TRUE(captureRoute(static_cast<CaptureService*>(nullptr),
                             request("GET", "/api/capture/status"), r));
    EXPECT_EQ(r.status, 503);
}

// ---- directory, shutdown, threads -------------------------------------------------

TEST(CaptureRecorder, DirectoryKeepsTheNewestAndReportsFailures) {
    namespace fs         = std::filesystem;
    const fs::path dir   = fs::temp_directory_path() /
                         ("navigatr_capture_test_" +
                          std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    CaptureConfig c = testConfig();
    c.directory     = dir.string();
    c.keep          = 1;
    {
        Rig               rig(c);
        const std::string a = rig.start(0, 0.1);
        rig.run(200000);
        ASSERT_TRUE(fs::exists(dir / (a + ".zip"))) << rig.status();
        std::ifstream in(dir / (a + ".zip"), std::ios::binary);
        const std::string on_disk((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        EXPECT_EQ(on_disk, *rig.rec->bundle(a));
        in.close();
        const std::string b = rig.start(0, 0.1);
        rig.run(200000);
        EXPECT_FALSE(fs::exists(dir / (a + ".zip"))); // keep = 1, and it was ours
        EXPECT_TRUE(fs::exists(dir / (b + ".zip")));
        EXPECT_EQ(rig.rec->bundle(a), nullptr);
        EXPECT_TRUE(contains(rig.status(), "\"file_error\":null"));
    }
    // a path that is a file: the write fails, the bundle stays in memory
    std::error_code ec;
    fs::create_directories(dir, ec);
    const fs::path blocker = dir / "not_a_dir";
    std::ofstream(blocker) << "x";
    c.directory = blocker.string();
    {
        Rig               rig(c);
        const std::string a = rig.start(0, 0.1);
        rig.run(200000);
        const std::string status = rig.status();
        EXPECT_TRUE(contains(status, "\"file\":null,\"file_error\":\"")) << status;
        EXPECT_TRUE(contains(status, "\"outcome\":\"ready\"")) << status;
        EXPECT_NE(rig.rec->bundle(a), nullptr);
    }
    fs::remove_all(dir, ec);
}

TEST(CaptureRecorder, StopClosesARecordingCaptureAsTruncated) {
    Rig               rig;
    const std::string id = rig.start(0, 30);
    rig.run(300000);
    rig.rec->stop();
    const std::string status = rig.status();
    EXPECT_TRUE(contains(status, "\"state\":\"stopped\"")) << status;
    EXPECT_TRUE(contains(status, "\"stop_reason\":\"shutdown\"")) << status;
    EXPECT_TRUE(contains(status, "\"truncated\":true"));
    EXPECT_FALSE(table(rig.bundle(id), "robot_state.csv").rows.empty());
    std::string err, other;
    EXPECT_FALSE(rig.rec->start(CaptureRequest{}, other, err));
    EXPECT_EQ(rig.hub.wanted(), 0u);
    rig.rec->stop(); // idempotent
}

TEST(CaptureRecorder, ThreadedRecorderFinishesOnItsOwn) {
    DiagnosticsHub           hub;
    CaptureRecorder::Options o; // real clock, own thread
    CaptureRecorder          rec(testConfig(), hub, std::make_shared<FakeHost>(), o);
    CaptureRequest           q;
    q.pre_s  = 0;
    q.post_s = 0.2;
    std::string id, err;
    ASSERT_TRUE(rec.start(q, id, err)) << err;
    const uint64_t v0       = rec.version();
    const auto     deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    std::thread    producer([&] {
        for (int i = 0; i < 20; ++i) {
            DiagRecord r;
            r.kind = DiagKind::kRobotState;
            DiagRobotState s;
            s.publication = static_cast<uint64_t>(i);
            r.payload     = s;
            hub.post(r);
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    });
    while (rec.bundle(id) == nullptr && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    producer.join();
    ASSERT_NE(rec.bundle(id), nullptr);
    EXPECT_GT(rec.version(), v0);
    ZipRead z;
    ASSERT_TRUE(readZip(*rec.bundle(id), z, err)) << err;
    EXPECT_FALSE(table(z, "robot_state.csv").rows.empty());
}

// ---- the state tap and the System ---------------------------------------------------

TEST(CaptureTap, RobotStateFeedPostsOnlyWhileWanted) {
    auto           hub = std::make_shared<DiagnosticsHub>();
    RobotStateFeed feed;
    feed.setDiagnostics(hub);
    RobotState s;
    s.valid                 = true;
    s.initialized           = true;
    s.odom_pose             = {1.0, 2.0, 0.5};
    s.field_from_odom       = {10.0, 0.0, 0.0};
    s.odometry_epoch        = 3;
    s.anchor_revision       = 4;
    s.measuredAt            = deviceTime(777);
    s.measuredAtHost        = hostTime(555);
    s.vx_m_s                = 0.25;
    s.attitude              = assumedLevelAttitude(0.5);
    feed.publish(s, LocalizationStatus{}, true, 9);
    EXPECT_EQ(hub->stats().posted[static_cast<int>(DiagKind::kRobotState)], 0u);

    hub->setWanted(diagBit(DiagKind::kRobotState));
    feed.publish(s, LocalizationStatus{}, true, 10);
    std::vector<DiagRecord> out;
    ASSERT_EQ(hub->drain(out), 1u);
    EXPECT_EQ(hub->sourceName(out[0].source), "localization");
    EXPECT_GE(out[0].host_us, 0); // stamped by the hub (0 only at the clock origin)
    const DiagRobotState& d = std::get<DiagRobotState>(out[0].payload);
    EXPECT_EQ(d.publication, 10u);
    EXPECT_EQ(d.measured_clock, DiagClock::kPico);
    EXPECT_EQ(d.measured_ms, 777);
    EXPECT_EQ(d.measured_host_ms, 555);
    EXPECT_DOUBLE_EQ(d.odom_x_m, 1.0);
    EXPECT_DOUBLE_EQ(d.field_x_m, 11.0);
    EXPECT_DOUBLE_EQ(d.odom_heading_rad, 0.5);
    EXPECT_EQ(d.odometry_epoch, 3u);
    EXPECT_EQ(d.anchor_revision, 4u);
    EXPECT_FLOAT_EQ(d.vx_m_s, 0.25f);
    EXPECT_TRUE(d.valid);
    EXPECT_TRUE(d.initialized);
    EXPECT_FALSE(d.attitude_valid);
    EXPECT_TRUE(d.attitude_assumed_level);
    EXPECT_TRUE(d.advanced);

    // the held pose published again: no new measurement, no history entry
    const std::size_t history = feed.historySize();
    s.measuredAtHost          = hostTime(565);
    feed.publish(s, LocalizationStatus{}, false, 11);
    out.clear();
    ASSERT_EQ(hub->drain(out), 1u);
    const DiagRobotState& held = std::get<DiagRobotState>(out[0].payload);
    EXPECT_EQ(held.publication, 11u);
    EXPECT_FALSE(held.advanced);
    EXPECT_EQ(feed.historySize(), history);
}

TEST(CaptureSystem, ResetIsASegmentBoundaryAndEventsAreRecorded) {
    std::string err;
    auto system = buildNoop(R"(<Capture rolling_s="1" default_pre_s="0" default_post_s="1"/>)", err);
    ASSERT_NE(system, nullptr) << err;
    HttpResponse r;
    captureRoute(*system, request("POST", "/api/capture/start", "pre_s=0&post_s=0.6&requester=t"),
                 r);
    ASSERT_EQ(r.status, 200) << r.body;
    int64_t ms = 1;
    for (int i = 0; i < 10; ++i) {
        system->step(hostTime(ms++));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    system->reset();
    for (int i = 0; i < 10; ++i) {
        system->step(hostTime(ms++));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    std::shared_ptr<const std::string> zip;
    while ((zip = system->capture()->bundle(system->sessionId().substr(0, 8) + "-1")) == nullptr &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_NE(zip, nullptr);
    ZipRead z;
    ASSERT_TRUE(readZip(*zip, z, err)) << err;
    const Table t = table(z, "robot_state.csv");
    ASSERT_FALSE(t.rows.empty());
    bool before = false, after = false;
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        EXPECT_EQ(t.at(i, "pi_session"), system->sessionId());
        EXPECT_EQ(t.at(i, "valid"), "0"); // the noop estimator never has a pose
        EXPECT_EQ(t.at(i, "odom_x_m"), "");
        before = before || t.at(i, "reset_count") == "0";
        after  = after || t.at(i, "reset_count") == "1";
    }
    EXPECT_TRUE(before);
    EXPECT_TRUE(after);
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"configuration\":{\"id\":\"inline\"")) << meta;
    EXPECT_TRUE(contains(meta, "\"reset_count\":1")) << meta;
    EXPECT_EQ(count(meta, "\"domain\":\"pi\""), 2u) << meta;
    // the capture's own timeline, including the reset, is in events.csv
    const Table e = table(z, "events.csv");
    bool        trigger_marker = false, reset_marker = false;
    for (std::size_t i = 0; i < e.rows.size(); ++i) {
        EXPECT_EQ(e.at(i, "pi_session"), system->sessionId());
        trigger_marker = trigger_marker || (e.at(i, "kind") == "capture" &&
                                            contains(e.at(i, "text"), "trigger: manual by t"));
        reset_marker = reset_marker || (e.at(i, "kind") == "capture" &&
                                        e.at(i, "text") == "Pi reset: reset count 1" &&
                                        e.at(i, "reset_count") == "1");
    }
    EXPECT_TRUE(trigger_marker);
    EXPECT_TRUE(reset_marker);
}

namespace
{

// A plain <System> (composition does not pass <Capture> through a
// <Configuration> yet) with the Brain link over a memory link.
std::string linkSystemXml(const std::string& capture) {
    return R"(
<System>
    <Loop rate_hz="100"/>
    )" + capture + R"(
    <Resources>
        <Resource id="brain_uart" type="memory_link"/>
    </Resources>
    <Pipeline>
        <CommandCollection type="brain_link"><Serial resource_id="brain_uart"/></CommandCollection>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="brain_link"><Serial resource_id="brain_uart"/></Publishing>
    </Pipeline>
</System>
)";
}

// The shipped brain_profile_usb.xml, resolved to its <System> document, with
// memory links for both devices and a <Capture> element added (the System
// takes it; composition does not pass it through a <Configuration> yet).
std::unique_ptr<System> buildUsbProfileWithCapture(FunctionRegistry& functions,
                                                   const std::string& capture, std::string& err) {
    registerAll(functions);
    functions.add<ResourceMakeFunction>(
        FunctionKey{"test_memory_link"},
        [](const ConfigNode&, ResourceInitializationContext&, std::string&) {
            return ResourceInstance::asContract<SerialLink>(std::make_shared<MemoryLink>());
        });
    ResolvedConfiguration config;
    if (!resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) + "/override/brain_profile_usb.xml",
                              config, err)) {
        return nullptr;
    }
    tinyxml2::XMLDocument doc;
    if (doc.Parse(config.xml.c_str()) != tinyxml2::XML_SUCCESS) {
        err = "resolved configuration does not parse";
        return nullptr;
    }
    tinyxml2::XMLElement* resources = doc.RootElement()->FirstChildElement("Resources");
    for (auto* r = resources != nullptr ? resources->FirstChildElement("Resource") : nullptr;
         r != nullptr; r = r->NextSiblingElement("Resource")) {
        const std::string id = ConfigNode{r}.attr("id");
        if (id == "brain_usb" || id == "pico_uart") {
            r->SetAttribute("type", "test_memory_link");
        }
    }
    tinyxml2::XMLDocument cap;
    if (cap.Parse(capture.c_str()) != tinyxml2::XML_SUCCESS) {
        err = "capture element does not parse";
        return nullptr;
    }
    doc.RootElement()->InsertFirstChild(cap.RootElement()->DeepClone(&doc));
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return System::buildFromString(printer.CStr(), functions, err);
}

MemoryLink* memoryLink(System& system, const char* id) {
    std::string err;
    auto        link = system.resources().require<SerialLink>(ResourceId{id}, err);
    EXPECT_NE(link, nullptr) << err;
    return dynamic_cast<MemoryLink*>(link.get());
}

// Every reply frame the Pi wrote since the last call.
std::vector<translagatr::BrainReply> replies(MemoryLink& link) {
    std::vector<translagatr::BrainReply> out;
    translagatr::FrameReader             reader;
    for (uint8_t b : link.output().takeAll()) {
        if (!reader.push(b)) {
            continue;
        }
        do {
            translagatr::BrainReply reply;
            if (translagatr::decodeBrainReply(reader.frame(), reader.frameLen(), reply)) {
                out.push_back(reply);
            }
        } while (reader.next());
    }
    return out;
}

// Polls the status until it holds `part` (the recorder runs on its own
// thread with the real clock).
bool waitStatus(CaptureService& capture, const std::string& part, std::string& status) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    do {
        JsonWriter w;
        capture.writeStatus(w);
        status = w.take();
        if (contains(status, part)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

} // namespace

// Review fix: the System's own fault edge reaches the automatic trigger, and
// System::noteEvent reaches events.csv through the hub.
TEST(CaptureSystem, BrainLinkQuietFiresTheAutomaticCapture) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromString(
        linkSystemXml(R"(<Capture auto="link_lost,profile_changed" auto_cooldown_s="10"
                                  rolling_s="5" default_pre_s="2" default_post_s="1"/>)")
            .c_str(),
        functions, err);
    ASSERT_NE(system, nullptr) << err;
    MemoryLink* brain = memoryLink(*system, "brain_uart");
    ASSERT_NE(brain, nullptr);
    int64_t clock_us = 1000000, ms = 0;
    brain->setClock([&] { return clock_us; });
    const auto step = [&](int64_t step_ms) {
        clock_us += step_ms * 1000;
        ms += step_ms;
        system->step(hostTime(ms));
        return replies(*brain);
    };
    const auto send = [&](const translagatr::BrainRequest& r) {
        std::vector<uint8_t> buf(translagatr::kMaxFrameLen);
        buf.resize(translagatr::encodeBrainRequest(r, buf.data(), translagatr::kMaxFrameLen));
        brain->input().feed(buf);
        const std::vector<translagatr::BrainReply> got = step(10);
        EXPECT_EQ(got.size(), 1u);
        return got.empty() ? translagatr::BrainReply{} : got.front();
    };
    step(10); // the first drain never replies

    translagatr::BrainRequest hello;
    hello.op         = translagatr::kOpHello;
    hello.request_id = 1;
    hello.nonce      = 0x5EED;
    const uint32_t session = send(hello).session;
    ASSERT_NE(session, 0u);
    uint16_t   rid  = 2;
    const auto poll = [&] {
        translagatr::BrainRequest q;
        q.op         = translagatr::kOpGetState;
        q.session    = session;
        q.request_id = rid++;
        send(q);
    };
    for (int i = 0; i < 5; ++i) {
        poll();
    }
    for (int i = 0; i < 25; ++i) {
        step(50); // 1.25 s of host time with no request
    }

    CaptureService& capture = *system->capture();
    std::string     status;
    ASSERT_TRUE(waitStatus(capture, "\"state\":\"ready\"", status)) << status;
    EXPECT_TRUE(contains(status, "\"fired\":1,")) << status;
    EXPECT_TRUE(contains(status, "\"last_reason\":\"link_lost\"")) << status;
    EXPECT_TRUE(contains(status, "\"seen\":{\"link_lost\":1,\"continuity_lost\":0,"
                                 "\"pico_reboot\":0,\"profile_changed\":0}"))
        << status;
    const std::string id  = system->sessionId().substr(0, 8) + "-1";
    const auto        zip = capture.bundle(id);
    ASSERT_NE(zip, nullptr) << status;
    ZipRead z;
    ASSERT_TRUE(readZip(*zip, z, err)) << err;
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"reason\":\"link_lost\",\"requester\":\"auto\","
                               "\"detail\":\"Brain link quiet for 1 s\""))
        << meta;
    const Table e     = table(z, "events.csv");
    bool        quiet = false, opened = false;
    for (std::size_t i = 0; i < e.rows.size(); ++i) {
        if (e.at(i, "kind") == "runtime") {
            EXPECT_EQ(e.at(i, "source"), "system");
            quiet  = quiet || e.at(i, "text") == "Brain link quiet for 1 s";
            opened = opened || e.at(i, "text") == "Brain session opened";
        }
    }
    EXPECT_TRUE(quiet);
    EXPECT_TRUE(opened); // well inside the 2 s pre-window
    EXPECT_FALSE(table(z, "brain_requests.csv").rows.empty());

    // quiet again inside the cooldown: seen and suppressed, no second capture
    poll();
    for (int i = 0; i < 25; ++i) {
        step(50);
    }
    ASSERT_TRUE(waitStatus(capture, "\"seen\":{\"link_lost\":2,", status)) << status;
    EXPECT_TRUE(contains(status, "\"fired\":1,")) << status;
    EXPECT_TRUE(contains(status, "\"suppressed\":{\"busy\":0,\"cooldown\":1,\"hourly_cap\":0}"))
        << status;
    EXPECT_EQ(capture.bundle(system->sessionId().substr(0, 8) + "-2"), nullptr);
}

// Review fix: in the Brain-profile USB configuration, a Pico reboot seen on
// the link fires its capture through the System, the metadata keeps the old
// boot at the window start next to the new one, and the frames-lost edge
// while recording is suppressed as busy.
TEST(CaptureSystem, PicoRebootFiresTheAutomaticCaptureWithBothBoots) {
    FunctionRegistry functions;
    std::string      err;
    auto             system = buildUsbProfileWithCapture(
        functions,
        R"(<Capture auto="pico_reboot,link_lost" auto_cooldown_s="10" rolling_s="5"
                    default_pre_s="0.5" default_post_s="1"/>)",
        err);
    ASSERT_NE(system, nullptr) << err;
    MemoryLink* pico = memoryLink(*system, "pico_uart");
    ASSERT_NE(pico, nullptr);
    int64_t    ms = 0;
    uint8_t    seq = 0;
    uint32_t   stamp_ms = 1000;
    const auto step = [&](uint16_t boot) {
        if (boot != 0) {
            translagatr::SensorSample s{};
            s.seq      = seq++;
            s.stamp_ms = stamp_ms;
            s.mask     = translagatr::kSensorEnc0;
            s.enc[0]   = static_cast<int32_t>(stamp_ms);
            s.identity = true;
            s.boot_id  = boot;
            uint8_t        buf[translagatr::kMaxFrameLen];
            const uint16_t n = translagatr::encodeSensorFrame(s, buf, sizeof(buf));
            ASSERT_GT(n, 0u);
            pico->input().feed(std::vector<uint8_t>(buf, buf + n));
            stamp_ms += 10;
        }
        ms += 10;
        system->step(hostTime(ms));
        pico->output().takeAll();
    };
    for (int i = 0; i < 10; ++i) {
        step(0x1111);
    }
    // the recorder reads the host once a second: let it see boot 0x1111
    // before the window start of the coming trigger
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    seq      = 0;
    stamp_ms = 30;
    for (int i = 0; i < 5; ++i) {
        step(0x2222); // rebooted
    }
    for (int i = 0; i < 30; ++i) {
        step(0); // 300 ms without frames: "Pico frames lost"
    }

    CaptureService& capture = *system->capture();
    std::string     status;
    ASSERT_TRUE(waitStatus(capture, "\"state\":\"ready\"", status)) << status;
    EXPECT_TRUE(contains(status, "\"fired\":1,")) << status;
    EXPECT_TRUE(contains(status, "\"last_reason\":\"pico_reboot\"")) << status;
    EXPECT_TRUE(contains(status, "\"seen\":{\"link_lost\":1,\"continuity_lost\":0,"
                                 "\"pico_reboot\":1,\"profile_changed\":0}"))
        << status;
    EXPECT_TRUE(contains(status, "\"suppressed\":{\"busy\":1,\"cooldown\":0,\"hourly_cap\":0}"))
        << status;
    const auto zip = capture.bundle(system->sessionId().substr(0, 8) + "-1");
    ASSERT_NE(zip, nullptr) << status;
    ZipRead z;
    ASSERT_TRUE(readZip(*zip, z, err)) << err;
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"reason\":\"pico_reboot\",\"requester\":\"auto\","
                               "\"detail\":\"Pico rebooted (boot 8738)\""))
        << meta;
    // 0x1111 = 4369 at the window start, 0x2222 = 8738 when built
    const std::size_t start = meta.find("\"host_at_start\":{");
    ASSERT_NE(start, std::string::npos) << meta;
    EXPECT_EQ(jsonInt(meta, "boot_id", meta.find("\"pico_link\":{")), 8738) << meta;
    EXPECT_EQ(jsonInt(meta, "boot_id", meta.find("\"pico_link\":{", start)), 4369) << meta;
    const Table e      = table(z, "events.csv");
    bool        reboot = false, lost = false;
    for (std::size_t i = 0; i < e.rows.size(); ++i) {
        reboot = reboot || contains(e.at(i, "text"), "Pico rebooted (boot 8738)");
        lost   = lost || e.at(i, "text") == "Pico frames lost";
    }
    EXPECT_TRUE(reboot);
    EXPECT_TRUE(lost);
    const Table p = table(z, "pico_sensor.csv");
    ASSERT_FALSE(p.rows.empty());
    EXPECT_EQ(p.at(p.rows.size() - 1, "boot_id"), "8738");
}

TEST(CaptureSystem, SyntheticRigRunRecordsPlacedStates) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromFile(
        std::string(NAVIGATR_CONFIG_DIR) + "/demo/synthetic_field_demo.xml", functions, err);
    ASSERT_NE(system, nullptr) << err;
    ASSERT_TRUE(system->start(err)) << err;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    HttpResponse r;
    captureRoute(*system,
                 request("POST", "/api/capture/start", "pre_s=0.2&post_s=0.5&requester=rig"), r);
    ASSERT_EQ(r.status, 200) << r.body;
    const std::string id       = system->sessionId().substr(0, 8) + "-1";
    const auto        deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (system->capture()->bundle(id) == nullptr && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    system->capture()->stop();
    system->stop();
    const std::shared_ptr<const std::string> zip = system->capture()->bundle(id);
    ASSERT_NE(zip, nullptr);
    ZipRead z;
    ASSERT_TRUE(readZip(*zip, z, err)) << err;
    const Table t = table(z, "robot_state.csv");
    // about 70 publications at 100 Hz; a loaded host may run fewer cycles
    EXPECT_GT(t.rows.size(), 10u);
    bool    placed = false;
    int64_t last_pub = -1;
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        if (t.at(i, "placed") == "1" && t.at(i, "valid") == "1") {
            placed = true; // the configured initial placement
            EXPECT_FALSE(t.at(i, "field_x_m").empty());
        }
        // a pose held before any measurement has no measurement time: missing, not zero
        EXPECT_EQ(t.at(i, "source_clock").empty(), t.at(i, "source_ms").empty());
        const std::string& fresh = t.at(i, "new_measurement"); // the estimator's flag: never empty
        EXPECT_TRUE(fresh == "0" || fresh == "1") << i << " " << fresh;
        const int64_t pub = std::stoll(t.at(i, "publication"));
        EXPECT_GT(pub, last_pub); // every publication in order, none twice
        last_pub = pub;
    }
    EXPECT_TRUE(placed);
    const std::string& meta = z.files.at("metadata.json");
    EXPECT_TRUE(contains(meta, "\"id\":\"demo_synthetic_field\"") ||
                contains(meta, "\"configuration\":{\"id\":\""))
        << meta;
    EXPECT_TRUE(contains(meta, "\"requester\":\"rig\""));
}
