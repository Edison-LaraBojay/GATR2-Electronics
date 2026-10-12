// brain_profile_gtest.cpp
// Brain robot profiles through the real wire path: the checked-in
// brain_profile_usb.xml (or an edited copy) with the NG1 USB envelope, or
// brain_profile_rs485.xml with raw frames, over memory links, the real
// brain link codec and Pico sensor frames, the System's profile boundary,
// and the inspection documents. Every geometry value here is a test value.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "translaGATR/link_documents.h"
#include "config/composition.h"
#include "core/host_clock.h"
#include "impl/resources/cameras.h"
#include "impl/resources/pros_usb_link.h"
#include "impl/resources/serial_links.h"
#include "inspection/inspection_document.h"
#include "math/angles.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/stationary_precheck.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

constexpr double  kRadius = 0.024;
constexpr int32_t kCpr    = 4000;
constexpr double  kMetersPerCount = 2.0 * kPi * kRadius / kCpr;
constexpr double  kRevolution     = 2.0 * kPi * kRadius;
constexpr int64_t kCycleMs        = 20;

// ---- profiles --------------------------------------------------------------

translagatr::ProfileWheel wheel(uint8_t port, int32_t x_um, int32_t y_um, int32_t angle_mdeg) {
    translagatr::ProfileWheel w;
    w.encoder_port   = port;
    w.counts_per_rev = kCpr;
    w.radius_um      = 24000;
    w.x_um           = x_um;
    w.y_um           = y_um;
    w.angle_mdeg     = angle_mdeg;
    return w;
}

translagatr::RobotProfileDoc baseProfile(uint8_t topology, uint8_t imu_source) {
    translagatr::RobotProfileDoc p;
    p.topology           = topology;
    p.imu_source         = imu_source;
    p.vex_smart_port     = imu_source == translagatr::kImuSourceBrainVex ? 1 : 0;
    p.footprint_front_um = 200000;
    p.footprint_back_um  = 200000;
    p.footprint_left_um  = 200000;
    p.footprint_right_um = 200000;
    p.calibration_window_ms = 500;   // the shortest window, for test time
    return p;
}

// Forward wheel on port 0 at (0, 0.15), sideways wheel on port 1 at (0.10, 0).
translagatr::RobotProfileDoc perpendicular(uint8_t imu_source) {
    translagatr::RobotProfileDoc p = baseProfile(translagatr::kTopologyTwoWheelImu, imu_source);
    p.wheel_count            = 2;
    p.wheels[0]              = wheel(0, 0, 150000, 0);
    p.wheels[1]              = wheel(1, 100000, 0, 90000);
    return p;
}

// Two forward-measuring wheels; the right one is mounted to measure backward.
translagatr::RobotProfileDoc twoForward(uint8_t imu_source) {
    translagatr::RobotProfileDoc p = baseProfile(translagatr::kTopologyTwoForwardWheelImu, imu_source);
    p.wheel_count            = 2;
    p.wheels[0]              = wheel(0, 0, 150000, 0);
    p.wheels[1]              = wheel(1, 0, -150000, 180000);
    return p;
}

translagatr::RobotProfileDoc threeWheel(uint8_t imu_source) {
    translagatr::RobotProfileDoc p = baseProfile(translagatr::kTopologyThreeWheel, imu_source);
    p.wheel_count            = 3;
    p.wheels[0]              = wheel(0, 0, 130000, 0);
    p.wheels[1]              = wheel(1, 0, -130000, 0);
    p.wheels[2]              = wheel(2, -120000, 0, 90000);
    return p;
}

std::vector<uint8_t> bytesOf(const translagatr::RobotProfileDoc& p) {
    std::vector<uint8_t> bytes(translagatr::kProfileMaxLen);
    bytes.resize(translagatr::encodeRobotProfile(p, bytes.data(), translagatr::kProfileMaxLen));
    EXPECT_FALSE(bytes.empty());
    return bytes;
}

uint32_t idOf(const std::vector<uint8_t>& doc) {
    return translagatr::crc32(doc.data(), static_cast<uint32_t>(doc.size()));
}

// Lever arm k = x uy - y ux of a profile wheel, meters.
double leverArm(const translagatr::ProfileWheel& w) {
    const double a = w.angle_mdeg * kPi / 180000.0;
    return w.x_um * 1e-6 * std::sin(a) - w.y_um * 1e-6 * std::cos(a);
}

using Edit = std::function<void(tinyxml2::XMLElement* root)>;

tinyxml2::XMLElement* findChild(tinyxml2::XMLElement* parent, const char* path) {
    tinyxml2::XMLElement* e = parent;
    std::string           rest(path);
    while (e != nullptr && !rest.empty()) {
        const std::size_t slash = rest.find('/');
        const std::string name  = rest.substr(0, slash);
        e                       = e->FirstChildElement(name.c_str());
        rest = slash == std::string::npos ? std::string() : rest.substr(slash + 1);
    }
    return e;
}

void removeChild(tinyxml2::XMLElement* root, const char* parent_path, const char* name) {
    tinyxml2::XMLElement* parent = findChild(root, parent_path);
    ASSERT_NE(parent, nullptr) << parent_path;
    tinyxml2::XMLElement* child = parent->FirstChildElement(name);
    ASSERT_NE(child, nullptr) << name;
    parent->DeleteChild(child);
}

const char* kProfilePath = "Pipeline/Localization/BrainProfile";

// A scripted Pico link: link state set by the test, every submitted command
// recorded, request states settled by the test.
struct FakePico : PicoControl {
    PicoLinkState state;
    struct Command {
        uint8_t  op     = 0;
        uint8_t  arg    = 0;
        uint32_t handle = 0;
    };
    std::vector<Command>                  commands;
    std::map<uint32_t, PicoRequestStatus> requests;
    uint32_t                              next   = 1;
    bool                                  refuse = false;

    FakePico() {
        state.frames_fresh = true;
        state.identity     = true;
        state.boot_id      = 0x1234;
    }

    PicoLinkState link() const override { return state; }

    uint32_t submit(uint8_t op, uint8_t arg, MonotonicTime, double) override {
        if (refuse) {
            return 0;
        }
        commands.push_back({op, arg, next});
        requests[next] = PicoRequestStatus{PicoRequestState::kSending, 0};
        return next++;
    }

    PicoRequestStatus request(uint32_t handle) const override {
        const auto it = requests.find(handle);
        return it == requests.end() ? PicoRequestStatus{} : it->second;
    }

    int count(uint8_t op) const {
        int n = 0;
        for (const Command& c : commands) {
            n += c.op == op ? 1 : 0;
        }
        return n;
    }

    // The newest command with op; the test fails without one.
    const Command& last(uint8_t op) const {
        for (auto it = commands.rbegin(); it != commands.rend(); ++it) {
            if (it->op == op) {
                return *it;
            }
        }
        ADD_FAILURE() << "no command with op " << int(op);
        static const Command none;
        return none;
    }

    void settle(uint8_t op, PicoRequestState s, uint8_t detail = translagatr::kControlDetailNone) {
        requests[last(op).handle] = PicoRequestStatus{s, detail};
    }
};

// Every Pico link reference goes to the rig's FakePico.
void useFakePico(tinyxml2::XMLElement* root) {
    tinyxml2::XMLElement* resources = root->FirstChildElement("Resources");
    ASSERT_NE(resources, nullptr);
    tinyxml2::XMLElement* fake = root->GetDocument()->NewElement("Resource");
    fake->SetAttribute("id", "fake_pico");
    fake->SetAttribute("type", "fake_pico");
    resources->InsertEndChild(fake);
    for (const char* path : {"Pipeline/CommandCollection/Pico", "Pipeline/Publishing/Pico"}) {
        tinyxml2::XMLElement* pico = findChild(root, path);
        ASSERT_NE(pico, nullptr) << path;
        pico->SetAttribute("resource_id", "fake_pico");
    }
}

// The Pi end of the Brain link, which a test can pull like a USB cable: the
// device then reads as closed.
struct PullableLink : SerialLink {
    explicit PullableLink(std::shared_ptr<MemoryLink> link) : inner(std::move(link)) {}

    std::shared_ptr<MemoryLink> inner;
    bool                        pulled = false;

    SerialReadResult readAvailable(MutableByteSpan destination) override {
        return pulled ? SerialReadResult{0, true} : inner->readAvailable(destination);
    }
    SerialWriteResult write(ByteSpan source) override { return inner->write(source); }
    SerialWriteResult write(ByteSpan source, const TransmitWindow& window) override {
        return inner->write(source, window);
    }
    bool    inputPending() override { return inner->inputPending(); }
    int64_t nowUs() override { return inner->nowUs(); }
};

// ---- rig -------------------------------------------------------------------

// The Pi built from a checked-in Brain-profile config with the Pico UART and
// A camera for the preview configuration. Dead: the device never opened,
// as libcamera_camera reports a missing or failed camera. Live: a 16x12 Y8
// frame at every poll, stamped with the host clock.
class TestCamera : public CameraDevice
{
public:
    explicit TestCamera(bool live) : live_(live) {}
    bool        alive() const override { return live_; }
    std::string diagnostic() const override { return live_ ? "" : "camera not connected"; }
    const CameraIntrinsics* intrinsics() const override { return nullptr; }
    FrameId engineeringFrame() const override { return FrameId{}; }
    std::optional<CameraFrameData> latestFrame(uint64_t, uint32_t) override {
        if (!live_) {
            return std::nullopt;
        }
        CameraFrameData f;
        f.sequence   = ++sequence_;
        f.epoch      = 1;
        f.exposureAt = HostClock::now();
        f.receivedAt = f.exposureAt;
        f.width_px   = 16;
        f.height_px  = 12;
        f.y8         = std::make_shared<const std::vector<uint8_t>>(16 * 12, uint8_t{128});
        return f;
    }

private:
    bool     live_;
    uint32_t sequence_ = 0;
};

// the Brain link device replaced by memory links. brain_profile_usb.xml: the
// Brain side speaks NG1 through its own ProsUsbLink, exactly as the Brain app
// does. brain_profile_rs485.xml: raw frames, as on the Smart Port.
struct Rig {
    FunctionRegistry              functions;
    std::shared_ptr<FakePico>     fake      = std::make_shared<FakePico>();
    std::shared_ptr<MemoryLink>   pi_raw    = std::make_shared<MemoryLink>();
    std::shared_ptr<MemoryLink>   brain_raw = std::make_shared<MemoryLink>();
    std::shared_ptr<PullableLink> pi_link   = std::make_shared<PullableLink>(pi_raw);
    std::shared_ptr<SerialLink>   brain;
    std::unique_ptr<System>       system;
    std::shared_ptr<MemoryLink>   pico;
    std::string                   build_error;

    int64_t  now       = 1000;   // host clock, ms
    int64_t  link_ms   = 1000;   // the Brain link clock; only steps advance it
    int64_t  pico_zero = 0;      // host ms of the Pico's clock zero, a reboot moves it
    uint32_t session = 0;
    uint16_t rid     = 1;
    uint8_t  seq     = 0;

    // simulated body: raw counts per port, gyro rate over its bias, rotation
    std::array<double, 3> counts{};   // fractional, rounded on the wire
    double                bias_mdps  = 40.0;
    double                rate_mdps  = 0.0;   // above the bias
    double                theta_mdeg = 0.0;   // trapezoid of the rate, as the Pi integrates it
    bool                  send_gyro  = true;
    bool                  vex_valid  = true;
    uint8_t               mask_drop  = 0;       // encoder bits left out of Pico frames
    double                gyro_sign  = 1.0;   // -1: an IMU mounted upside down

    explicit Rig(const Edit& edit = nullptr) : Rig("brain_profile_usb.xml", edit) {}

    Rig(const std::string& config_name, const Edit& edit) {
        registerAll(functions);
        auto raw = pi_link;
        EXPECT_TRUE(functions.add<ResourceMakeFunction>(
            FunctionKey{"test_usb"},
            [raw](const ConfigNode&, ResourceInitializationContext&, std::string&) {
                return ResourceInstance::asContract<SerialLink>(std::make_shared<ProsUsbLink>(raw));
            }));
        EXPECT_TRUE(functions.add<ResourceMakeFunction>(
            FunctionKey{"test_rs485"},
            [raw](const ConfigNode&, ResourceInitializationContext&, std::string&) {
                return ResourceInstance::asContract<SerialLink>(raw);
            }));
        auto pico_control = fake;
        EXPECT_TRUE(functions.add<ResourceMakeFunction>(
            FunctionKey{"fake_pico"},
            [pico_control](const ConfigNode&, ResourceInitializationContext&, std::string&) {
                return ResourceInstance::asContract<PicoControl>(pico_control);
            }));
        // cameras for the preview configuration: one that never opened, one
        // that streams small Y8 frames
        EXPECT_TRUE(functions.add<ResourceMakeFunction>(
            FunctionKey{"test_dead_camera"},
            [](const ConfigNode&, ResourceInitializationContext&, std::string&) {
                return cameraResource(std::make_shared<TestCamera>(false), OutputId{"frame"});
            }));
        EXPECT_TRUE(functions.add<ResourceMakeFunction>(
            FunctionKey{"test_live_camera"},
            [](const ConfigNode&, ResourceInitializationContext&, std::string&) {
                return cameraResource(std::make_shared<TestCamera>(true), OutputId{"frame"});
            }));
        const bool usb = config_name.find("_usb") != std::string::npos;
        if (usb) {
            brain = std::make_shared<ProsUsbLink>(brain_raw);
        } else {
            brain = brain_raw;
        }
        pi_raw->setClock([this] { return link_ms * 1000; });

        ResolvedConfiguration config;
        if (!resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) + "/override/" + config_name,
                                  config, build_error)) {
            ADD_FAILURE() << build_error;
            return;
        }
        tinyxml2::XMLDocument doc;
        EXPECT_EQ(doc.Parse(config.xml.c_str()), tinyxml2::XML_SUCCESS);
        tinyxml2::XMLElement* resources = doc.RootElement()->FirstChildElement("Resources");
        for (auto* r = resources->FirstChildElement("Resource"); r != nullptr;
             r = r->NextSiblingElement("Resource")) {
            const std::string id = ConfigNode{r}.attr("id");
            if (id == "brain_usb") {
                r->SetAttribute("type", "test_usb");
            }
            if (id == "brain_uart") {
                r->SetAttribute("type", "test_rs485");
            }
            if (id == "pico_uart") {
                r->SetAttribute("type", "memory_link");
            }
        }
        if (edit) {
            edit(doc.RootElement());
        }
        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        system = System::buildFromString(printer.CStr(), functions, build_error);
        if (system == nullptr) {
            return;
        }
        std::string err;
        pico = std::dynamic_pointer_cast<MemoryLink>(
            system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err));
        EXPECT_NE(pico, nullptr) << err;
        system->step(hostTime(now));   // the first drain never answers
    }

    bool ok() const { return system != nullptr && pico != nullptr; }

    void step() {
        now += kCycleMs;
        link_ms += kCycleMs;
        system->step(hostTime(now));
    }

    void picoFrame() {
        translagatr::SensorSample f{};
        f.seq      = seq++;
        f.stamp_ms = static_cast<uint32_t>(now - pico_zero);
        f.mask     = (translagatr::kSensorEnc0 | translagatr::kSensorEnc1 | translagatr::kSensorEnc2) & ~mask_drop;
        for (int i = 0; i < 3; ++i) {
            f.enc[i] = static_cast<int32_t>(std::llround(counts[i]));
        }
        if (send_gyro) {
            f.mask |= translagatr::kSensorGyroZ;
            f.gyro_z = static_cast<int32_t>(std::llround(gyro_sign * (bias_mdps + rate_mdps)));
        }
        std::vector<uint8_t> bytes(translagatr::kMaxFrameLen);
        bytes.resize(translagatr::encodeSensorFrame(f, bytes.data(), translagatr::kMaxFrameLen));
        EXPECT_FALSE(bytes.empty());
        pico->input().feed(bytes);
    }

    std::vector<translagatr::BrainReply> exchange(const translagatr::BrainRequest& r) {
        send(r);
        step();
        return replies();
    }

    void send(const translagatr::BrainRequest& r) {
        std::array<uint8_t, translagatr::kMaxFrameLen> bytes{};
        const uint16_t len = translagatr::encodeBrainRequest(r, bytes.data(), bytes.size());
        EXPECT_GT(len, 0);
        EXPECT_TRUE(brain->write({bytes.data(), len}).ok);
        pi_raw->input().feed(brain_raw->output().takeAll());
    }

    // Worker mode for a moment: the workers answer what waits in the link,
    // whose clock stands still meanwhile so the reply window holds. They run
    // on the real host clock, and nothing but commands flows meanwhile. No
    // profile boundary runs unless the test asks for one.
    std::vector<translagatr::BrainReply> exchangeWithWorkers(const translagatr::BrainRequest& r) {
        send(r);
        std::string err;
        EXPECT_TRUE(system->start(err)) << err;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        system->stop();
        now = std::max(now, HostClock::now().ms) + kCycleMs;   // inline time never regresses
        return replies();
    }

    std::vector<translagatr::BrainReply> replies() {
        brain_raw->input().feed(pi_raw->output().takeAll());
        std::vector<translagatr::BrainReply> replies;
        translagatr::FrameReader             reader;
        for (;;) {
            std::array<uint8_t, 256> in{};
            const SerialReadResult   read = brain->readAvailable({in.data(), in.size()});
            if (read.bytes == 0) {
                break;
            }
            for (std::size_t i = 0; i < read.bytes; ++i) {
                if (!reader.push(in[i])) {
                    continue;
                }
                do {
                    translagatr::BrainReply reply;
                    EXPECT_TRUE(translagatr::decodeBrainReply(reader.frame(), reader.frameLen(), reply));
                    replies.push_back(reply);
                } while (reader.next());
            }
        }
        return replies;
    }

    translagatr::BrainReply one(const translagatr::BrainRequest& r) {
        const std::vector<translagatr::BrainReply> replies = exchange(r);
        EXPECT_EQ(replies.size(), 1u) << "op " << int(r.op);
        return replies.empty() ? translagatr::BrainReply{} : replies.front();
    }

    translagatr::BrainRequest request(uint8_t op) {
        translagatr::BrainRequest r;
        r.op         = op;
        r.session    = session;
        r.request_id = rid++;
        return r;
    }

    uint32_t hello(uint32_t nonce = 0x5EED0001) {
        translagatr::BrainRequest r = request(translagatr::kOpHello);
        r.session             = 0;
        r.nonce               = nonce;
        const translagatr::BrainReply reply = one(r);
        EXPECT_EQ(reply.result, translagatr::kResultOk);
        session = reply.session;
        return session;
    }

    // One pipeline cycle: a Pico frame and a state poll carrying the Brain
    // VEX IMU sample, as the Brain app sends it.
    translagatr::BrainReply cycle() {
        picoFrame();
        translagatr::BrainRequest r = request(translagatr::kOpGetState);
        r.imu_flags           = vex_valid ? translagatr::kBenchImuValid : 0;
        r.imu_stamp_ms        = static_cast<uint32_t>(now);
        r.imu_rotation_mdeg   = static_cast<int32_t>(std::llround(theta_mdeg));
        return one(r);
    }

    translagatr::BrainState still(int cycles) {
        translagatr::BrainReply last;
        for (int i = 0; i < cycles; ++i) {
            last = cycle();
        }
        return last.state;
    }

    // Raw count deltas per port spread evenly over the cycles, no rotation.
    translagatr::BrainState translate(std::array<double, 3> delta, int cycles) {
        translagatr::BrainReply last;
        for (int c = 0; c < cycles; ++c) {
            for (int i = 0; i < 3; ++i) {
                counts[i] += delta[i] / cycles;
            }
            last = cycle();
        }
        return last.state;
    }

    // A turn at rate_mdps for the cycles, then one cycle back at the bias.
    // counts_per_rad per port make each wheel follow the rotation the Pi
    // integrates from the gyro (trapezoid between frames), so the lever arms
    // see exactly that rotation.
    translagatr::BrainState turn(double turn_mdps, int cycles, std::array<double, 3> counts_per_rad,
                           double wheel_gain = 1.0) {
        translagatr::BrainReply last;
        for (int c = 0; c <= cycles; ++c) {
            const double prev = rate_mdps;
            rate_mdps         = c < cycles ? turn_mdps : 0.0;
            const double d_mdeg = 0.5 * (prev + rate_mdps) * kCycleMs / 1000.0;
            theta_mdeg += d_mdeg;
            const double d_rad = d_mdeg / 1000.0 * kPi / 180.0;
            for (int i = 0; i < 3; ++i) {
                counts[i] += counts_per_rad[i] * d_rad * wheel_gain;
            }
            last = cycle();
        }
        return last.state;
    }

    // Stage in chunks of at most chunk bytes, then APPLY; the APPLY reply.
    translagatr::BrainReply stageAndApply(const std::vector<uint8_t>& doc,
                                    uint16_t chunk = translagatr::kProfileChunkMax) {
        stage(doc, 0, static_cast<uint16_t>(doc.size()), chunk);
        return apply(doc);
    }

    void stage(const std::vector<uint8_t>& doc, uint16_t from, uint16_t to, uint16_t chunk) {
        for (uint16_t offset = from; offset < to; offset = static_cast<uint16_t>(offset + chunk)) {
            const uint16_t      n = static_cast<uint16_t>(std::min<int>(chunk, to - offset));
            translagatr::BrainRequest r = request(translagatr::kOpProfileWrite);
            r.profile_id          = idOf(doc);
            r.total_len           = static_cast<uint16_t>(doc.size());
            r.offset              = offset;
            r.data_len            = static_cast<uint8_t>(n);
            std::memcpy(r.data, doc.data() + offset, n);
            const translagatr::BrainReply reply = one(r);
            EXPECT_EQ(reply.result, translagatr::kResultOk);
        }
    }

    translagatr::BrainReply apply(const std::vector<uint8_t>& doc) { return one(applyRequest(doc)); }

    translagatr::BrainRequest applyRequest(const std::vector<uint8_t>& doc) {
        translagatr::BrainRequest r = request(translagatr::kOpProfileApply);
        r.profile_id          = idOf(doc);
        r.total_len           = static_cast<uint16_t>(doc.size());
        return r;
    }

    // One chunk holding the whole document.
    translagatr::BrainRequest writeRequest(const std::vector<uint8_t>& doc) {
        EXPECT_LE(doc.size(), static_cast<std::size_t>(translagatr::kProfileChunkMax));
        translagatr::BrainRequest r = request(translagatr::kOpProfileWrite);
        r.profile_id          = idOf(doc);
        r.total_len           = static_cast<uint16_t>(doc.size());
        r.data_len            = static_cast<uint8_t>(doc.size());
        std::memcpy(r.data, doc.data(), doc.size());
        return r;
    }

    // The whole exchange a Brain runs: stage, APPLY (Pending), one cycle for
    // the boundary, APPLY again (Ok).
    bool applyProfile(const translagatr::RobotProfileDoc& p) {
        const std::vector<uint8_t> doc     = bytesOf(p);
        const translagatr::BrainReply    pending = stageAndApply(doc);
        EXPECT_EQ(pending.result, translagatr::kResultPending);
        if (pending.result != translagatr::kResultPending) {
            return false;
        }
        cycle();
        const translagatr::BrainReply done = apply(doc);
        EXPECT_EQ(done.result, translagatr::kResultOk);
        EXPECT_EQ(done.profile_state, translagatr::kProfileApplied);
        return done.result == translagatr::kResultOk;
    }

    translagatr::BrainReply place(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg) {
        picoFrame();
        translagatr::BrainRequest r = request(translagatr::kOpSetPose);
        r.x_mm                = x_mm;
        r.y_mm                = y_mm;
        r.heading_cdeg        = heading_cdeg;
        return one(r);
    }

    translagatr::BrainReply control(uint8_t action) {
        picoFrame();
        translagatr::BrainRequest r = request(translagatr::kOpControl);
        r.action              = action;
        return one(r);
    }

    Pose2D pose() const { return system->robot().fieldPose(); }
};

// Applies a profile, places the robot at (1, 0.5, 0) and lets it settle.
void ready(Rig& r, const translagatr::RobotProfileDoc& p, int settle = 40) {
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    r.still(3);
    ASSERT_TRUE(r.applyProfile(p));
    r.still(settle);   // baselines, and a Pico gyro calibrates here
    const translagatr::BrainReply placed = r.place(1000, 500, 0);
    ASSERT_EQ(placed.result, translagatr::kResultOk);
    r.still(3);
    ASSERT_NEAR(r.pose().x_m, 1.0, 1e-9);
    ASSERT_NEAR(r.pose().y_m, 0.5, 1e-9);
}

std::array<double, 3> countsPerRad(const translagatr::RobotProfileDoc& p) {
    std::array<double, 3> out{};
    for (uint8_t i = 0; i < p.wheel_count; ++i) {
        out[p.wheels[i].encoder_port] = leverArm(p.wheels[i]) / kMetersPerCount;
    }
    return out;
}

// Each axis on its own, both signs, and a turn whose wheel travel is all
// lever arm: forward travel moves only x, sideways only y, the turn neither.
void perpendicularWheelsMeasureEachAxisOnce(uint8_t imu_source) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = perpendicular(imu_source);
    ready(r, p);

    r.translate({kCpr, 0, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 0.5, 1e-6);

    r.translate({0, kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 0.5 + kRevolution, 1e-6);

    r.translate({-kCpr / 2.0, -kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution / 2.0, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 0.5, 1e-6);

    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));   // 10 degrees CCW
    r.still(3);
    const Pose2D after = r.pose();
    EXPECT_NEAR(wrapAngle(after.heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
    // one count of rounding per wheel at most
    EXPECT_NEAR(after.x_m, before.x_m, 1e-4);
    EXPECT_NEAR(after.y_m, before.y_m, 1e-4);
}

double headingDeg(const Rig& r) { return r.pose().heading_rad * 180.0 / kPi; }

} // namespace

// ---- waiting ---------------------------------------------------------------

TEST(BrainProfile, WaitsWithAcquisitionCommandsAndInspectionAlive) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    const translagatr::BrainState s = r.still(5);
    EXPECT_EQ(s.profile_state, translagatr::kProfileNone);
    EXPECT_EQ(s.robot_flags & translagatr::kRobotPoseValid, 0);
    EXPECT_EQ(s.health & translagatr::kHealthEncodersFresh, 0);   // no profile encoders yet
    EXPECT_EQ(s.calibration, translagatr::kCalibrationNone);
    EXPECT_NE(s.map_id, 0u);   // the field is served while waiting
    EXPECT_EQ(r.system->localization().estimatorType(), "noop");

    EXPECT_EQ(r.place(1000, 500, 0).result, translagatr::kResultNotReady);
    EXPECT_EQ(r.control(translagatr::kControlRecalibrate).result, translagatr::kResultNotReady);

    // raw acquisition stays visible
    const auto health = r.system->sourceHealth();
    ASSERT_NE(health, nullptr);
    bool encoder_seen = false;
    for (const SourceHealthEntry& e : health->entries) {
        if (e.id == "pico_telemetry.encoder_0") {
            encoder_seen = e.has_sample && e.state == SourceState::kValid;
        }
    }
    EXPECT_TRUE(encoder_seen);

    const std::string hello = helloDocument(*r.system, hostTime(r.now));
    EXPECT_NE(hello.find("\"estimator_type\":\"noop\""), std::string::npos) << hello;
    const std::string snapshot =
        snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_NE(snapshot.find("pico_telemetry.encoder_0"), std::string::npos);
}

// ---- topologies ------------------------------------------------------------

TEST(BrainProfile, PerpendicularWheelsWithTheBrainVexImu) {
    perpendicularWheelsMeasureEachAxisOnce(translagatr::kImuSourceBrainVex);
}

TEST(BrainProfile, PerpendicularWheelsWithThePicoGyro) {
    perpendicularWheelsMeasureEachAxisOnce(translagatr::kImuSourcePico);
}

TEST(BrainProfile, PicoImuInversionAppliesOnce) {
    Rig r;
    r.gyro_sign              = -1.0;   // the chip reports CW positive
    translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourcePico);
    p.imu_flags              = translagatr::kImuInvert;
    ready(r, p);
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));   // 10 degrees CCW
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
    EXPECT_NEAR(r.pose().x_m, before.x_m, 1e-4);
    EXPECT_NEAR(r.pose().y_m, before.y_m, 1e-4);
}

TEST(BrainProfile, VexProfileIsReadyWithNoPicoImuFramesAtAll) {
    Rig r;
    r.send_gyro = false;   // an absent or broken BNO08X: encoder frames only
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState s = r.translate({kCpr, 0, 0}, 10);
    EXPECT_EQ(s.profile_state, translagatr::kProfileApplied);
    EXPECT_NE(s.robot_flags & translagatr::kRobotPoseValid, 0);
    EXPECT_NE(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NE(s.health & translagatr::kHealthEncodersFresh, 0);
    EXPECT_NE(s.health & translagatr::kHealthGyroFresh, 0);   // the Brain bench sample
    EXPECT_EQ(s.calibration, translagatr::kCalibrationNone);  // VEX firmware owns its calibration
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);

    // an invalid VEX sample drops the gyro bit and holds the pose
    r.vex_valid = false;
    const translagatr::BrainState stale = r.translate({kCpr, 0, 0}, 5);
    EXPECT_EQ(stale.health & translagatr::kHealthGyroFresh, 0);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
}

TEST(BrainProfile, TwoForwardWheelsWithThePicoGyro) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = twoForward(translagatr::kImuSourcePico);
    ready(r, p);
    // the right wheel measures backward: forward travel counts down on it
    r.translate({kCpr, -kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 0.5, 1e-9);   // sideways is assumed zero
    const Pose2D before = r.pose();
    r.turn(-25000.0, 20, countsPerRad(p));
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(-10.0), 2e-4);
    EXPECT_NEAR(r.pose().x_m, before.x_m, 1e-4);
    EXPECT_NEAR(r.pose().y_m, before.y_m, 1e-4);
}

TEST(BrainProfile, TwoForwardWheelsWithTheBrainVexImu) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = twoForward(translagatr::kImuSourceBrainVex);
    ready(r, p);
    EXPECT_EQ(r.system->localization().functionStatus().front().type,
              "brain_imu_parallel_bench");
    r.translate({kCpr, -kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 0.5, 1e-9);
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
    EXPECT_NEAR(r.pose().x_m, before.x_m, 1e-4);
}

TEST(BrainProfile, ThreeWheelsFuseAnIndependentPicoGyroOnce) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = threeWheel(translagatr::kImuSourcePico);
    ready(r, p);
    EXPECT_EQ(r.system->localization().estimatorType(), "weighted_planar_fusion");
    const std::vector<ObservationFunctionStatus> functions =
        r.system->localization().functionStatus();
    ASSERT_EQ(functions.size(), 2u);
    EXPECT_EQ(functions[0].type, "tracking_wheel_motion");   // no heading constraint
    EXPECT_EQ(functions[1].type, "imu_heading_increment");   // the independent gyro
    EXPECT_TRUE(functions[1].ready);

    // agreeing wheels and gyro: the rotation, no translation
    Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
    EXPECT_NEAR(r.pose().x_m, before.x_m, 1e-4);
    EXPECT_NEAR(r.pose().y_m, before.y_m, 1e-4);
    EXPECT_TRUE(r.system->robot().has_covariance);

    // wheels that see 20 percent more rotation than the gyro: the fused turn
    // lies strictly between, so both counted, and each once (a gyro counted
    // twice, or folded into the wheels, would give the gyro's value)
    before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p), 1.2);
    r.still(3);
    const double turned = wrapAngle(r.pose().heading_rad - before.heading_rad);
    EXPECT_GT(turned, degToRad(10.0) + 1e-3);
    EXPECT_LT(turned, degToRad(12.0) - 1e-3);

    // straight travel still resolves
    before = r.pose();
    const double c = std::cos(r.pose().heading_rad);
    const double s = std::sin(r.pose().heading_rad);
    r.translate({kCpr, kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m - before.x_m, kRevolution * c, 1e-5);
    EXPECT_NEAR(r.pose().y_m - before.y_m, kRevolution * s, 1e-5);
}

TEST(BrainProfile, ThreeWheelRecalibrationRestartsTheFusedGyroBias) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = threeWheel(translagatr::kImuSourcePico);
    ready(r, p);
    EXPECT_EQ(r.still(20).calibration, translagatr::kCalibrationDone);
    const Pose2D held = r.pose();
    // the bias drifted; its first sample arrives with the request, which the
    // new window measures from
    r.bias_mdps                = 90.0;
    const translagatr::BrainReply ok = r.control(translagatr::kControlRecalibrate);
    EXPECT_EQ(ok.result, translagatr::kResultOk);
    EXPECT_EQ(ok.calibration, translagatr::kCalibrationRunning);
    // 20 samples arrive in 400 ms; the profile's 500 ms window still runs
    EXPECT_EQ(r.still(22).calibration, translagatr::kCalibrationRunning);
    EXPECT_EQ(r.still(10).calibration, translagatr::kCalibrationDone);
    r.still(50);   // the new bias holds the heading still
    EXPECT_NEAR(r.pose().x_m, held.x_m, 1e-9);
    EXPECT_NEAR(r.pose().y_m, held.y_m, 1e-9);
    EXPECT_NEAR(r.pose().heading_rad, held.heading_rad, 1e-9);
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
}

TEST(BrainProfile, ThreeWheelsWithoutAnImu) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = threeWheel(translagatr::kImuSourceNone);
    ready(r, p);
    EXPECT_EQ(r.system->localization().estimatorType(), "planar_motion_integrator");
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));   // the gyro is ignored; the wheels turn
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
    const translagatr::BrainState s = r.still(1);
    EXPECT_EQ(s.calibration, translagatr::kCalibrationNone);
    EXPECT_EQ(s.health & translagatr::kHealthGyroFresh, 0);
}

// ---- one application per correction --------------------------------------

namespace
{

// Port 0 forward and port 1 sideways, one revolution of raw counts each, with
// only wheel 0 changed by the knob: wheel 1 stays the reference.
void knobAppliesOnce(const std::function<void(translagatr::ProfileWheel&)>& knob, double forward,
                     uint8_t imu_source = translagatr::kImuSourceBrainVex) {
    Rig                    r;
    translagatr::RobotProfileDoc p = perpendicular(imu_source);
    knob(p.wheels[0]);
    ready(r, p);
    r.translate({kCpr, kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m - 1.0, forward, 1e-7);
    EXPECT_NEAR(r.pose().y_m - 0.5, kRevolution, 1e-7);
}

// Forward and sideways wheels each calibrated on their own.
void scalesArePerWheel(uint8_t imu_source) {
    Rig                    r;
    translagatr::RobotProfileDoc p     = perpendicular(imu_source);
    p.wheels[0].travel_scale_ppm = 1040000;
    p.wheels[1].travel_scale_ppm = 970000;
    ready(r, p);
    r.translate({kCpr, 0, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m - 1.0, 1.04 * kRevolution, 1e-7);
    EXPECT_NEAR(r.pose().y_m - 0.5, 0.0, 1e-9);
    r.translate({0, kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m - 1.0, 1.04 * kRevolution, 1e-7);
    EXPECT_NEAR(r.pose().y_m - 0.5, 0.97 * kRevolution, 1e-7);
}

const auto kAllCorrections = [](translagatr::ProfileWheel& w) {
    w.counts_per_rev   = 8192;
    w.gear_micro       = 1500000;
    w.flags            = translagatr::kWheelReversed;
    w.travel_scale_ppm = 950000;
    w.radius_um        = 30000;
};
const double kAllCorrectionsForward = -(kCpr * 2.0 * kPi / (8192 * 1.5)) * 0.030 * 0.95;

} // namespace

TEST(BrainProfile, CountsPerRevolutionApplyOnce) {
    knobAppliesOnce([](translagatr::ProfileWheel& w) { w.counts_per_rev = 2 * kCpr; },
                    kRevolution / 2.0);
}

TEST(BrainProfile, GearingAppliesOnce) {
    knobAppliesOnce([](translagatr::ProfileWheel& w) { w.gear_micro = 2500000; },   // 2.5:1
                    kRevolution / 2.5);
}

TEST(BrainProfile, PolarityAppliesOnce) {
    knobAppliesOnce([](translagatr::ProfileWheel& w) { w.flags = translagatr::kWheelReversed; },
                    -kRevolution);
}

TEST(BrainProfile, TravelScaleAppliesOnce) {
    knobAppliesOnce([](translagatr::ProfileWheel& w) { w.travel_scale_ppm = 1050000; },
                    1.05 * kRevolution);
}

TEST(BrainProfile, AllCorrectionsTogetherApplyOnceEach) {
    knobAppliesOnce(kAllCorrections, kAllCorrectionsForward);
}

TEST(BrainProfile, AllCorrectionsApplyOnceInTheTrackingWheelModel) {
    knobAppliesOnce(kAllCorrections, kAllCorrectionsForward, translagatr::kImuSourcePico);
}

TEST(BrainProfile, ForwardAndSidewaysScalesArePerWheelWithTheVexImu) {
    scalesArePerWheel(translagatr::kImuSourceBrainVex);
}

TEST(BrainProfile, ForwardAndSidewaysScalesArePerWheelWithThePicoGyro) {
    scalesArePerWheel(translagatr::kImuSourcePico);
}

// ---- idempotence, continuity -----------------------------------------------

// With the Pico IMU a Brain restart touches no source the profile uses; with
// the Brain VEX IMU it does (SensorLoss.ABrainRestartRestartsTheVexImu).
TEST(BrainProfile, SameProfileReappliesWithoutResetting) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourcePico);
    ready(r, p);
    r.translate({kCpr, 0, 0}, 10);
    const translagatr::BrainState before = r.still(2);
    const Pose2D            pose   = r.pose();

    const std::vector<uint8_t> doc = bytesOf(p);
    const translagatr::BrainReply    again = r.apply(doc);
    EXPECT_EQ(again.result, translagatr::kResultOk);   // at once, no boundary
    EXPECT_EQ(again.profile_state, translagatr::kProfileApplied);

    // a restarted Brain: new session, stages and applies the same profile
    const uint32_t old_session = r.session;
    r.rid                      = 1;
    EXPECT_NE(r.hello(0x5EED0002), old_session);
    EXPECT_EQ(r.stageAndApply(doc).result, translagatr::kResultOk);
    const translagatr::BrainState after = r.still(3);
    EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    EXPECT_EQ(after.anchor_revision, before.anchor_revision);
    EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);   // the placement holds
    EXPECT_NEAR(r.pose().x_m, pose.x_m, 1e-9);
    EXPECT_NEAR(r.pose().y_m, pose.y_m, 1e-9);
    EXPECT_EQ(r.system->profileBinding()->generation, 1u);   // applied once
}

TEST(BrainProfile, NewProfileResetsAndWithdrawsPlacement) {
    Rig                          r;
    const translagatr::RobotProfileDoc a = perpendicular(translagatr::kImuSourceBrainVex);
    ready(r, a);
    r.translate({kCpr, 0, 0}, 10);
    const translagatr::BrainState placed = r.still(2);
    EXPECT_NE(placed.robot_flags & translagatr::kRobotLocalized, 0);
    const SensorId encoder{"profile_encoder_0"};
    const uint64_t encoder_epoch = r.system->sensorMap().at(encoder).latest->epoch;
    translagatr::RobotProfileDoc b = a;
    b.wheels[0].radius_um    = 25000;   // a different robot description
    const std::vector<uint8_t> doc = bytesOf(b);
    EXPECT_EQ(r.stageAndApply(doc).result, translagatr::kResultPending);
    const translagatr::BrainState swapped = r.still(1);
    EXPECT_EQ(swapped.profile_state, translagatr::kProfileApplied);
    EXPECT_EQ(swapped.profile_id, idOf(doc));
    EXPECT_GT(swapped.odometry_epoch, placed.odometry_epoch);
    EXPECT_EQ(swapped.anchor_revision, placed.anchor_revision);   // continued, never reused
    EXPECT_EQ(swapped.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_EQ(r.system->robotFeed()->historySize(), 0u);
    // the rebuilt encoder sensor restarts under a new record epoch
    EXPECT_GT(r.system->sensorMap().at(encoder).latest->epoch, encoder_epoch);

    // the old SET_POSE never re-applies to the new odometry origin
    const translagatr::BrainState moving = r.translate({kCpr, 0, 0}, 15);
    EXPECT_EQ(moving.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NE(moving.robot_flags & translagatr::kRobotPoseValid, 0);

    // a new placement is required and works
    EXPECT_EQ(r.place(2000, 1000, 9000).result, translagatr::kResultOk);
    const translagatr::BrainState replaced = r.still(2);
    EXPECT_NE(replaced.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(replaced.anchor_revision, placed.anchor_revision);
    EXPECT_NEAR(r.pose().x_m, 2.0, 1e-9);
    EXPECT_NEAR(headingDeg(r), 90.0, 1e-9);
}

TEST(BrainProfile, StagingInterruptedByANewSessionResumes) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    const std::vector<uint8_t> doc = bytesOf(perpendicular(translagatr::kImuSourceBrainVex));
    r.stage(doc, 0, 40, 40);   // interrupted after one chunk
    r.rid = 1;
    r.hello(0x5EED0003);        // the Brain reconnects under a new session
    r.stage(doc, 40, static_cast<uint16_t>(doc.size()), 40);   // resumes at received
    EXPECT_EQ(r.apply(doc).result, translagatr::kResultPending);
    r.cycle();
    EXPECT_EQ(r.apply(doc).result, translagatr::kResultOk);
}

TEST(BrainProfile, TheLatestApplyWinsOverACandidateWaitingForTheBoundary) {
    // the worker phases run on the real host clock with no sensor traffic;
    // sensor loss would end the placement this test watches, so it only warns
    Rig r([](tinyxml2::XMLElement* root) {
        findChild(root, (std::string(kProfilePath) + "/Timing").c_str())
            ->SetAttribute("on_sensor_loss", "warn");
    });
    const translagatr::RobotProfileDoc a = perpendicular(translagatr::kImuSourceBrainVex);
    ready(r, a);
    const translagatr::BrainState    before = r.still(2);
    const Pose2D               pose   = r.pose();
    translagatr::RobotProfileDoc     b      = a;
    b.wheels[0].radius_um             = 25000;
    const std::vector<uint8_t> doc_a  = bytesOf(a);
    const std::vector<uint8_t> doc_b  = bytesOf(b);
    const auto only = [](const std::vector<translagatr::BrainReply>& replies) {
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? translagatr::BrainReply{} : replies.front();
    };
    const auto dropped = [&r] {
        int n = 0;
        for (const RuntimeEvent& e : r.system->events()) {
            n += e.text.find("dropped") != std::string::npos ? 1 : 0;
        }
        return n;
    };

    // worker mode: B is built, then the Brain stages and applies A again
    // before the main thread reaches the boundary
    r.stage(doc_b, 0, static_cast<uint16_t>(doc_b.size()), translagatr::kProfileChunkMax);
    EXPECT_EQ(only(r.exchangeWithWorkers(r.applyRequest(doc_b))).result,
              translagatr::kResultPending);
    EXPECT_EQ(only(r.exchangeWithWorkers(r.writeRequest(doc_a))).result, translagatr::kResultOk);
    const translagatr::BrainReply again = only(r.exchangeWithWorkers(r.applyRequest(doc_a)));
    EXPECT_EQ(again.result, translagatr::kResultOk);
    EXPECT_EQ(again.profile_state, translagatr::kProfileApplied);
    EXPECT_TRUE(r.system->applyPendingProfile());
    EXPECT_EQ(dropped(), 1);
    ASSERT_NE(r.system->profileBinding(), nullptr);
    EXPECT_EQ(r.system->profileBinding()->id, idOf(doc_a));
    EXPECT_EQ(r.system->profileBinding()->generation, 1u);
    const translagatr::BrainState kept = r.still(2);
    EXPECT_EQ(kept.profile_state, translagatr::kProfileApplied);
    EXPECT_EQ(kept.profile_id, idOf(doc_a));
    EXPECT_EQ(kept.odometry_epoch, before.odometry_epoch);
    EXPECT_NE(kept.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NEAR(r.pose().x_m, pose.x_m, 1e-9);

    // B applied later is built anew
    ASSERT_TRUE(r.applyProfile(b));
    EXPECT_EQ(r.system->profileBinding()->id, idOf(doc_b));
    const translagatr::BrainState on_b = r.still(1);
    EXPECT_GT(on_b.odometry_epoch, before.odometry_epoch);

    // a refused APPLY supersedes a waiting candidate too; B keeps running
    translagatr::RobotProfileDoc bad = perpendicular(translagatr::kImuSourcePico);
    bad.imu_port               = 1;   // no such Pico IMU port on this Pi
    const std::vector<uint8_t> doc_bad = bytesOf(bad);
    r.stage(doc_a, 0, static_cast<uint16_t>(doc_a.size()), translagatr::kProfileChunkMax);
    EXPECT_EQ(only(r.exchangeWithWorkers(r.applyRequest(doc_a))).result,
              translagatr::kResultPending);
    EXPECT_EQ(only(r.exchangeWithWorkers(r.writeRequest(doc_bad))).result, translagatr::kResultOk);
    const translagatr::BrainReply refused = only(r.exchangeWithWorkers(r.applyRequest(doc_bad)));
    EXPECT_EQ(refused.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(refused.profile_reason, translagatr::kProfileReasonImuPort);
    EXPECT_TRUE(r.system->applyPendingProfile());
    EXPECT_EQ(dropped(), 2);
    EXPECT_EQ(r.system->profileBinding()->id, idOf(doc_b));
    const translagatr::BrainState after = r.still(1);
    EXPECT_EQ(after.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(after.profile_id, idOf(doc_bad));
    EXPECT_EQ(after.odometry_epoch, on_b.odometry_epoch);
}

// ---- refusals --------------------------------------------------------------

namespace
{

struct Refusal {
    const char*            what;
    translagatr::RobotProfileDoc profile;
    uint8_t                reason;
    uint8_t                detail;
};

void expectRefused(Rig& r, const Refusal& refusal) {
    SCOPED_TRACE(refusal.what);
    const std::vector<uint8_t> doc   = bytesOf(refusal.profile);
    const translagatr::BrainReply    reply = r.stageAndApply(doc);
    EXPECT_EQ(reply.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(reply.profile_reason, refusal.reason);
    EXPECT_EQ(reply.profile_detail, refusal.detail);
    const translagatr::BrainState s = r.still(1);
    EXPECT_EQ(s.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(s.profile_id, idOf(doc));
    EXPECT_EQ(r.system->profileBinding(), nullptr);   // still waiting
    EXPECT_EQ(r.system->localization().estimatorType(), "noop");
    // remembered: the same id answers the same without another build
    EXPECT_EQ(r.apply(doc).profile_reason, refusal.reason);
}

} // namespace

TEST(BrainProfile, UnsupportedCombinationsAreRefusedExplicitly) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();

    translagatr::RobotProfileDoc two_none = perpendicular(translagatr::kImuSourceNone);
    two_none.vex_smart_port         = 0;
    translagatr::RobotProfileDoc three_vex = threeWheel(translagatr::kImuSourceBrainVex);
    translagatr::RobotProfileDoc collinear = perpendicular(translagatr::kImuSourcePico);
    collinear.wheels[1].angle_mdeg   = 180000;
    translagatr::RobotProfileDoc sideways_forward = twoForward(translagatr::kImuSourcePico);
    sideways_forward.wheels[1].angle_mdeg   = 90000;
    translagatr::RobotProfileDoc same_port        = perpendicular(translagatr::kImuSourcePico);
    same_port.wheels[1].encoder_port        = 0;
    translagatr::RobotProfileDoc unwired          = perpendicular(translagatr::kImuSourcePico);
    unwired.wheels[1].encoder_port          = 3;
    translagatr::RobotProfileDoc imu_port         = perpendicular(translagatr::kImuSourcePico);
    imu_port.imu_port                       = 1;
    translagatr::RobotProfileDoc camera           = perpendicular(translagatr::kImuSourcePico);
    camera.camera_count                     = 1;

    for (const Refusal& refusal : std::vector<Refusal>{
             {"two wheels without an IMU", two_none, translagatr::kProfileReasonImuCombination, 0},
             {"three wheels with the Brain clock IMU", three_vex,
              translagatr::kProfileReasonImuCombination, 0},
             {"collinear wheels", collinear, translagatr::kProfileReasonObservability, 0},
             {"a sideways wheel in the forward topology", sideways_forward,
              translagatr::kProfileReasonObservability, 0},
             {"one port twice", same_port, translagatr::kProfileReasonEncoderPort, 1},
             {"a port this Pi does not have", unwired, translagatr::kProfileReasonEncoderPort, 1},
             {"a Pico IMU port that does not exist", imu_port, translagatr::kProfileReasonImuPort, 0},
             {"a camera slot this Pi does not have", camera, translagatr::kProfileReasonCamera, 0},
         }) {
        expectRefused(r, refusal);
    }
}

TEST(BrainProfile, CapabilitiesFollowThePiConfiguration) {
    {
        // port 2 not wired: a three-wheel profile names it
        Rig r([](tinyxml2::XMLElement* root) {
            tinyxml2::XMLElement* encoders =
                findChild(root, "Pipeline/Localization/BrainProfile/Encoders");
            ASSERT_NE(encoders, nullptr);
            tinyxml2::XMLElement* port = encoders->FirstChildElement("Port");
            port = port->NextSiblingElement("Port")->NextSiblingElement("Port");
            encoders->DeleteChild(port);
        });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"port 2 unwired", threeWheel(translagatr::kImuSourceNone),
                          translagatr::kProfileReasonEncoderPort, 2});
    }
    {
        Rig r([](tinyxml2::XMLElement* root) { removeChild(root, kProfilePath, "Imu"); });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"no Pico IMU", perpendicular(translagatr::kImuSourcePico),
                          translagatr::kProfileReasonImuSource, 0});
    }
    {
        Rig r([](tinyxml2::XMLElement* root) {
            removeChild(root, kProfilePath, "BrainImu");
            removeChild(root, "Pipeline/CommandCollection", "BenchImu");
        });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"no Brain IMU mailbox", perpendicular(translagatr::kImuSourceBrainVex),
                          translagatr::kProfileReasonImuSource, 0});
    }
    {
        Rig r([](tinyxml2::XMLElement* root) { removeChild(root, kProfilePath, "Fusion"); });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"no fusion tuning", threeWheel(translagatr::kImuSourcePico),
                          translagatr::kProfileReasonImuCombination, 0});
    }
}

TEST(BrainProfile, ARefusedProfileLeavesTheRunningOneRunning) {
    Rig                          r;
    const translagatr::RobotProfileDoc a = perpendicular(translagatr::kImuSourceBrainVex);
    ready(r, a);
    const translagatr::BrainState before = r.still(1);
    translagatr::RobotProfileDoc  bad    = a;
    bad.camera_count               = 1;
    const translagatr::BrainReply reply  = r.stageAndApply(bytesOf(bad));
    EXPECT_EQ(reply.profile_reason, translagatr::kProfileReasonCamera);
    const translagatr::BrainState after = r.translate({kCpr, 0, 0}, 10);
    EXPECT_EQ(after.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_EQ(r.system->profileBinding()->id, idOf(bytesOf(a)));
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
}

// ---- CONTROL ---------------------------------------------------------------

TEST(BrainProfile, RecalibrateNeedsStillnessAndHoldsThePose) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourcePico);
    ready(r, p);
    EXPECT_EQ(r.still(1).calibration, translagatr::kCalibrationDone);

    // moving now: nothing starts
    for (int i = 0; i < 4; ++i) {
        r.counts[0] += 200;
        r.cycle();
    }
    r.counts[0] += 200;
    const translagatr::BrainReply moving = r.control(translagatr::kControlRecalibrate);
    EXPECT_EQ(moving.result, translagatr::kResultNotStationary);
    EXPECT_EQ(r.still(1).calibration, translagatr::kCalibrationDone);

    // turning in place counts as moving too
    r.still(20);
    r.rate_mdps               = 5000.0;   // 5 deg/s
    r.still(20);
    EXPECT_EQ(r.control(translagatr::kControlRecalibrate).result, translagatr::kResultNotStationary);
    r.rate_mdps = 0.0;

    // still for longer than the window: it restarts
    r.still(20);
    const Pose2D            held = r.pose();
    const translagatr::BrainReply ok   = r.control(translagatr::kControlRecalibrate);
    EXPECT_EQ(ok.result, translagatr::kResultOk);
    EXPECT_EQ(ok.action, translagatr::kControlRecalibrate);
    EXPECT_EQ(ok.calibration, translagatr::kCalibrationRunning);
    const uint16_t control_rid = static_cast<uint16_t>(r.rid - 1);

    const translagatr::BrainState done = r.still(30);
    EXPECT_EQ(done.calibration, translagatr::kCalibrationDone);
    EXPECT_NE(done.health & translagatr::kHealthBiasCalibrated, 0);
    EXPECT_NE(done.robot_flags & translagatr::kRobotLocalized, 0);   // still throughout
    EXPECT_NEAR(r.pose().x_m, held.x_m, 1e-9);
    EXPECT_NEAR(r.pose().y_m, held.y_m, 1e-9);

    // a lost acknowledgement: the same request id reports, never reruns
    translagatr::BrainRequest dup = r.request(translagatr::kOpControl);
    r.rid--;
    dup.request_id = control_rid;
    dup.action     = translagatr::kControlRecalibrate;
    r.picoFrame();
    const translagatr::BrainReply again = r.one(dup);
    EXPECT_EQ(again.result, translagatr::kResultOk);
    EXPECT_EQ(again.calibration, translagatr::kCalibrationDone);   // not restarted

    // the calibrated bias is the one it measured
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
}

TEST(BrainProfile, ReinitializeStartsANewUnplacedOdometry) {
    Rig                          r;
    const translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourcePico);
    ready(r, p);
    r.still(20);
    const translagatr::BrainState before = r.still(1);
    const translagatr::BrainReply reply  = r.control(translagatr::kControlReinitialize);
    EXPECT_EQ(reply.result, translagatr::kResultOk);
    EXPECT_EQ(reply.calibration, translagatr::kCalibrationRunning);
    const translagatr::BrainState after = r.still(30);
    EXPECT_GT(after.odometry_epoch, before.odometry_epoch);
    EXPECT_EQ(after.robot_flags & translagatr::kRobotLocalized, 0);   // the old SET_POSE is withdrawn
    EXPECT_EQ(after.calibration, translagatr::kCalibrationDone);
    EXPECT_EQ(r.place(500, 500, 0).result, translagatr::kResultOk);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);
}

TEST(BrainProfile, CalibrationSettingsFromTheProfileReachTheModel) {
    Rig                    r;
    translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourcePico);
    p.calibration_window_ms  = 1000;   // longer than the samples the Pi needs
    p.still_travel_um        = 20;     // under one encoder count of travel
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    r.still(3);
    ASSERT_TRUE(r.applyProfile(p));
    EXPECT_EQ(r.still(30).calibration, translagatr::kCalibrationRunning);   // 600 ms of samples
    EXPECT_EQ(r.still(25).calibration, translagatr::kCalibrationDone);

    // one count of travel restarts the window under this profile's limit
    ASSERT_EQ(r.control(translagatr::kControlRecalibrate).result, translagatr::kResultOk);
    r.still(30);
    r.counts[0] += 1;
    EXPECT_EQ(r.still(30).calibration, translagatr::kCalibrationRunning);
    EXPECT_EQ(r.still(30).calibration, translagatr::kCalibrationDone);
}

TEST(BrainProfile, VexRecalibrateHasNothingToCalibrateOnThePi) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    r.still(20);
    const translagatr::BrainReply reply = r.control(translagatr::kControlRecalibrate);
    EXPECT_EQ(reply.result, translagatr::kResultOk);
    EXPECT_EQ(reply.calibration, translagatr::kCalibrationNone);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);   // nothing reset
    // the Pico IMU is no part of this profile
    const translagatr::BrainReply reinit = r.control(translagatr::kControlReinitImu);
    EXPECT_EQ(reinit.result, translagatr::kResultFailed);
    EXPECT_EQ(reinit.control_detail, translagatr::kControlDetailImuUnused);
}

// ---- configuration ---------------------------------------------------------

TEST(BrainProfile, ConfigurationSchemaIsStrict) {
    const auto fails = [](const char* what, const Edit& edit, const char* expect) {
        SCOPED_TRACE(what);
        Rig r(edit);
        EXPECT_EQ(r.system, nullptr);
        EXPECT_NE(r.build_error.find(expect), std::string::npos) << r.build_error;
    };
    fails("models beside the profile",
          [](tinyxml2::XMLElement* root) {
              tinyxml2::XMLElement* localization = findChild(root, "Pipeline/Localization");
              tinyxml2::XMLElement* estimator =
                  localization->GetDocument()->NewElement("Estimator");
              estimator->SetAttribute("type", "noop");
              localization->InsertEndChild(estimator);
          },
          "only BrainProfile");
    fails("no encoders",
          [](tinyxml2::XMLElement* root) { removeChild(root, kProfilePath, "Encoders"); },
          "Encoders");
    fails("port out of range",
          [](tinyxml2::XMLElement* root) {
              findChild(root, "Pipeline/Localization/BrainProfile/Encoders/Port")
                  ->SetAttribute("index", 3);
          },
          "index must be 0..2");
    fails("an output that is not an encoder",
          [](tinyxml2::XMLElement* root) {
              findChild(root, "Pipeline/Localization/BrainProfile/Encoders/Port")
                  ->SetAttribute("output_id", "imu_0");
          },
          "different payload");
    fails("a Brain IMU the commands never feed",
          [](tinyxml2::XMLElement* root) {
              removeChild(root, "Pipeline/CommandCollection", "BenchImu");
          },
          "BenchImu");
    fails("health references beside a profile",
          [](tinyxml2::XMLElement* root) {
              tinyxml2::XMLElement* health = findChild(root, "Pipeline/Publishing/Health");
              tinyxml2::XMLElement* encoder = health->GetDocument()->NewElement("Encoder");
              encoder->SetAttribute("sensor_id", "profile_encoder_0");
              health->InsertEndChild(encoder);
          },
          "follow the profile");
    fails("an unknown element",
          [](tinyxml2::XMLElement* root) {
              tinyxml2::XMLElement* profile = findChild(root, kProfilePath);
              profile->InsertEndChild(profile->GetDocument()->NewElement("CameraSlot"));
          },
          "CameraSlot");
    fails("noise missing",
          [](tinyxml2::XMLElement* root) {
              removeChild(root, "Pipeline/Localization/BrainProfile/Fusion", "HeadingNoise");
          },
          "HeadingNoise");
    fails("a calibration window of no time",
          [](tinyxml2::XMLElement* root) {
              findChild(root, "Pipeline/Localization/BrainProfile/Calibration")
                  ->SetAttribute("window_ms", 0);
          },
          "window_ms");
}

TEST(BrainProfile, Rs485ConfigDiffersFromUsbOnlyInTheBrainLink) {
    const auto load = [](const char* name, tinyxml2::XMLDocument& doc) {
        ResolvedConfiguration config;
        std::string           err;
        ASSERT_TRUE(resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) + "/override/" + name,
                                         config, err))
            << err;
        ASSERT_EQ(doc.Parse(config.xml.c_str()), tinyxml2::XML_SUCCESS);
    };
    const auto print = [](const tinyxml2::XMLNode* e) {
        tinyxml2::XMLPrinter printer;
        e->Accept(&printer);
        return std::string(printer.CStr());
    };
    // the transport resource goes, and the slots name one link id
    const auto normalize = [](tinyxml2::XMLDocument& doc, const char* link) {
        tinyxml2::XMLElement* root      = doc.RootElement();
        tinyxml2::XMLElement* resources = root->FirstChildElement("Resources");
        for (auto* r = resources->FirstChildElement("Resource"); r != nullptr;) {
            auto* next = r->NextSiblingElement("Resource");
            if (ConfigNode{r}.attr("id") == link) {
                resources->DeleteChild(r);
            }
            r = next;
        }
        for (const char* slot :
             {"Pipeline/CommandCollection/Serial", "Pipeline/Publishing/Serial"}) {
            tinyxml2::XMLElement* serial = findChild(root, slot);
            ASSERT_NE(serial, nullptr) << slot;
            EXPECT_EQ(ConfigNode{serial}.attr("resource_id"), link);
            serial->SetAttribute("resource_id", "brain_link");
        }
    };
    tinyxml2::XMLDocument usb, rs485;
    load("brain_profile_usb.xml", usb);
    load("brain_profile_rs485.xml", rs485);
    normalize(usb, "brain_usb");
    normalize(rs485, "brain_uart");
    EXPECT_EQ(print(findChild(usb.RootElement(), "Pipeline")),
              print(findChild(rs485.RootElement(), "Pipeline")));
    EXPECT_EQ(print(findChild(usb.RootElement(), "Resources")),
              print(findChild(rs485.RootElement(), "Resources")));

    // it builds with memory links, and waits for a profile like the USB one
    FunctionRegistry functions;
    registerAll(functions);
    tinyxml2::XMLDocument doc;
    load("brain_profile_rs485.xml", doc);
    tinyxml2::XMLElement* resources = doc.RootElement()->FirstChildElement("Resources");
    for (auto* e = resources->FirstChildElement("Resource"); e != nullptr;
         e = e->NextSiblingElement("Resource")) {
        const std::string id = ConfigNode{e}.attr("id");
        if (id == "brain_uart" || id == "pico_uart") {
            e->DeleteChildren();
            e->SetAttribute("type", "memory_link");
        }
    }
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    std::string                   err;
    const std::unique_ptr<System> system =
        System::buildFromString(printer.CStr(), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_TRUE(system->bindingView()->brain_profile);
    EXPECT_EQ(system->bindingView()->estimator_type, "noop");
    EXPECT_EQ(system->profileBinding(), nullptr);
}

TEST(BrainProfile, InspectionFollowsTheBoundary) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const std::shared_ptr<const BindingView> view = r.system->bindingView();
    ASSERT_NE(view, nullptr);
    EXPECT_TRUE(view->brain_profile);
    ASSERT_NE(view->profile, nullptr);
    EXPECT_EQ(view->estimator_type, "planar_motion_integrator");
    EXPECT_NE(view->sensors.payloadOf(SensorId{"profile_encoder_0"}), nullptr);
    EXPECT_NE(view->profile->summary.find("Brain VEX smart port 1"), std::string::npos)
        << view->profile->summary;
    bool bench_warning = false;
    for (const std::string& w : view->warnings) {
        bench_warning = bench_warning || w.find("BENCH ONLY") != std::string::npos;
    }
    EXPECT_TRUE(bench_warning);
    const std::string hello = helloDocument(*r.system, hostTime(r.now));
    EXPECT_NE(hello.find("\"estimator_type\":\"planar_motion_integrator\""), std::string::npos);
    bool applied_event = false;
    for (const RuntimeEvent& e : r.system->events()) {
        applied_event = applied_event || e.text.find("applied") != std::string::npos;
    }
    EXPECT_TRUE(applied_event);
}

// ---- checked-in configs and inspection ---------------------------------------

namespace inspection_test
{
bool validJson(const std::string& s);   // inspection_document_gtest.cpp
} // namespace inspection_test

namespace
{

std::string hex8(uint32_t v) {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%08x", static_cast<unsigned>(v));
    return buf;
}

bool has(const std::string& doc, const std::string& text) {
    return doc.find(text) != std::string::npos;
}

// The robot profile the Brain programs send today (brain/robot/gatr2_robot.h,
// kTwoWheelVexImu): forward wheel on port 0 at (0, 0.15), sideways wheel on
// port 1 at (0.15, 0), 0.024 m, 4000 counts, VEX IMU on Smart Port 1, 0.23 m
// to each side, Pi calibration defaults. Placeholders there, test values here.
translagatr::RobotProfileDoc currentBrainProfile() {
    translagatr::RobotProfileDoc p =
        baseProfile(translagatr::kTopologyTwoWheelImu, translagatr::kImuSourceBrainVex);
    p.wheel_count           = 2;
    p.wheels[0]             = wheel(0, 0, 150000, 0);
    p.wheels[1]             = wheel(1, 150000, 0, 90000);
    p.footprint_front_um    = 230000;
    p.footprint_back_um     = 230000;
    p.footprint_left_um     = 230000;
    p.footprint_right_um    = 230000;
    p.calibration_window_ms = 0;
    return p;
}

// A checked-in config waits, takes the current profile over its own
// transport, reports every readiness condition, and localizes.
void checkedInConfigRunsTheCurrentProfile(const char* config) {
    Rig r(config, nullptr);
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();

    // waiting: acquisition, the link and the field run; no pose, no placement
    translagatr::BrainState s = r.still(5);
    EXPECT_EQ(s.profile_state, translagatr::kProfileNone);
    EXPECT_EQ(s.robot_flags & translagatr::kRobotPoseValid, 0);
    EXPECT_NE(s.map_id, 0u);
    EXPECT_EQ(r.place(1200, 1800, 0).result, translagatr::kResultNotReady);
    std::string snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_TRUE(inspection_test::validJson(snap));
    EXPECT_TRUE(has(snap, "\"profile\":{\"state\":\"none\",\"id\":null")) << snap;

    const translagatr::RobotProfileDoc p = currentBrainProfile();
    ASSERT_TRUE(r.applyProfile(p));
    s = r.still(3);
    EXPECT_EQ(s.profile_state, translagatr::kProfileApplied);
    EXPECT_EQ(s.profile_id, idOf(bytesOf(p)));
    EXPECT_EQ(s.robot_flags & translagatr::kRobotLocalized, 0);   // needs placement

    // ready: fresh encoders and VEX IMU, nothing to calibrate on the Pi, placed
    ASSERT_EQ(r.place(1200, 1800, 0).result, translagatr::kResultOk);
    s = r.still(3);
    EXPECT_EQ(s.health & (translagatr::kHealthEncodersFresh | translagatr::kHealthGyroFresh),
              translagatr::kHealthEncodersFresh | translagatr::kHealthGyroFresh);
    EXPECT_EQ(s.calibration, translagatr::kCalibrationNone);
    EXPECT_EQ(s.robot_flags & (translagatr::kRobotPoseValid | translagatr::kRobotLocalized),
              translagatr::kRobotPoseValid | translagatr::kRobotLocalized);

    // each wheel measures its own axis
    r.translate({kCpr, 0, 0}, 10);
    r.translate({0, -kCpr / 2.0, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.2 + kRevolution, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 1.8 - kRevolution / 2.0, 1e-6);

    snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_TRUE(inspection_test::validJson(snap));
    EXPECT_TRUE(has(snap, "\"state\":\"applied\",\"id\":\"" + hex8(idOf(bytesOf(p))) + "\""))
        << snap;
    EXPECT_TRUE(has(snap, "\"pose_valid\":true,\"localized\":true"));
    EXPECT_TRUE(has(snap, "\"encoders_fresh\":true,\"gyro_fresh\":true"));
    EXPECT_TRUE(has(snap, "\"source\":\"brain_vex\",\"port\":0,\"vex_smart_port\":1"));
    EXPECT_TRUE(has(snap, "\"front_m\":0.23,\"back_m\":0.23"));
}

} // namespace

TEST(BrainProfileConfigs, UsbRunsTheCurrentBrainProfile) {
    checkedInConfigRunsTheCurrentProfile("brain_profile_usb.xml");
}

TEST(BrainProfileConfigs, Rs485RunsTheCurrentBrainProfile) {
    checkedInConfigRunsTheCurrentProfile("brain_profile_rs485.xml");
}

TEST(BrainProfileInspection, HelloCarriesThePlanningFieldTheBrainReads) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    const translagatr::BrainState s     = r.still(2);
    const std::string       hello = helloDocument(*r.system, hostTime(r.now));
    ASSERT_TRUE(inspection_test::validJson(hello)) << hello;
    // the map the Brain is served, by the same id
    EXPECT_TRUE(has(hello, "\"map_id\":\"" + hex8(s.map_id) + "\",\"map_error\":null")) << hello;
    EXPECT_TRUE(has(hello, "\"commands_type\":\"brain_link\",\"brain_profile\":true"));
    EXPECT_TRUE(has(hello, "\"revision\":1"));
    EXPECT_TRUE(has(hello, "\"boundary\":{\"min_x_m\":0,\"min_y_m\":0,\"max_x_m\":3.5664"));
    // a goal: wire id and a box in its own frame; a fixed obstacle with its box
    EXPECT_TRUE(has(hello, "\"wire_id\":5,\"nominal\":{\"x_m\":1.7832,\"y_m\":1.7832"));
    EXPECT_TRUE(has(hello, "\"collision_box\":{\"x_m\":0,\"y_m\":0,\"yaw_deg\":0,"
                           "\"size_x_m\":0.1543,\"size_y_m\":0.1543"))
        << hello;
    EXPECT_TRUE(has(hello, "\"obstacles\":[{\"id\":\"loader_red_south\",\"wire_id\":101"))
        << hello;
    const auto snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_TRUE(has(snap, "\"map_id\":\"" + hex8(s.map_id) + "\""));
}

TEST(BrainProfileInspection, SnapshotShowsProfileCalibrationWheelsPicoEventsAndPath) {
    Rig r(useFakePico);
    r.fake->state.status_known           = true;
    r.fake->state.status.imu_state       = translagatr::kPicoImuReady;
    r.fake->state.status.flags           = translagatr::kPicoImuEnabled;
    r.fake->state.status.firmware        = translagatr::kPicoFirmwareBno08x;
    r.fake->state.status.last_request_id = 7;
    r.fake->state.status.last_op         = translagatr::kPicoOpReinitImu;
    r.fake->state.status.last_status     = translagatr::kPicoCommandCompleted;
    translagatr::RobotProfileDoc p             = perpendicular(translagatr::kImuSourcePico);
    p.wheels[1].gear_micro               = 2 * translagatr::kUnitMicro;
    p.wheels[1].flags                    = translagatr::kWheelReversed;
    p.wheels[1].travel_scale_ppm         = 1010000;
    ready(r, p);
    r.counts[0] += kCpr;
    r.cycle();

    // the Brain reports its planned path
    translagatr::BrainRequest path = r.request(translagatr::kOpPathReport);
    path.command_id          = 42;
    path.path_mode           = translagatr::kPathAvoiding;
    path.point_count         = 3;
    path.points[0]           = {1000, 500};
    path.points[1]           = {1500, 900};
    path.points[2]           = {2000, 900};
    const translagatr::BrainReply path_reply = r.one(path);
    EXPECT_EQ(path_reply.result, translagatr::kResultOk);
    // an IMU reinitialization running on the Pico
    ASSERT_EQ(r.control(translagatr::kControlReinitImu).result, translagatr::kResultPending);

    const std::string snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    ASSERT_TRUE(inspection_test::validJson(snap)) << snap;
    const uint32_t id = idOf(bytesOf(p));
    EXPECT_TRUE(has(snap, "\"brain_link\":{\"takes_profile\":true,\"link_open\":true,"
                          "\"session\":" + std::to_string(r.session) + ",\"pi_instance\":" +
                              std::to_string(path_reply.pi_instance) + ","))
        << snap;
    EXPECT_TRUE(has(snap, "\"last_request_age_ms\":0"));
    EXPECT_TRUE(has(snap, "\"state\":\"applied\",\"id\":\"" + hex8(id) + "\",\"applied_id\":\"" +
                              hex8(id) + "\""));
    EXPECT_TRUE(has(snap, "\"topology\":\"two wheels + IMU\""));
    // the active corrections, each once, on the wheel that has them
    EXPECT_TRUE(has(snap, "\"port\":1,\"sensor_id\":\"profile_encoder_1\",\"counts_per_rev\":4000,"
                          "\"gear\":2,\"reversed\":true,\"radius_m\":0.024"))
        << snap;
    EXPECT_TRUE(has(snap, "\"travel_scale\":1.01"));
    EXPECT_TRUE(has(snap, "\"source\":\"pico\",\"port\":0"));
    // calibration of the model that owns the bias
    EXPECT_TRUE(has(snap, "\"calibration\":{\"function\":\"profile_motion\",\"ready\":true,"
                          "\"stillness\":{\"monitored\":true"))
        << snap;
    EXPECT_TRUE(has(snap, "\"calibration\":\"done\""));
    // raw readings as READ_WHEELS gives them: travel without the scale
    EXPECT_TRUE(has(snap, "\"port\":0,\"valid\":true,\"fresh\":true,\"counts\":4000,\"travel_m\":" +
                              std::to_string(kRevolution).substr(0, 7)))
        << snap;
    // each reading carries its own wheel's corrections
    EXPECT_TRUE(has(snap, "\"counts_per_rev\":4000,\"gear\":2,\"reversed\":true,"
                          "\"radius_m\":0.024,\"travel_scale\":1.01}"));
    EXPECT_TRUE(has(snap, "\"counts_per_rev\":4000,\"gear\":1,\"reversed\":false,"
                          "\"radius_m\":0.024,\"travel_scale\":1}"));
    EXPECT_TRUE(has(snap, "\"operation\":{\"action\":\"reinit_imu\",\"result\":\"pending\""));
    EXPECT_TRUE(has(snap, "\"path\":{\"session\":" + std::to_string(r.session) +
                              ",\"command_id\":42,\"mode\":\"avoiding\""));
    EXPECT_TRUE(has(snap, "\"points\":[{\"x_m\":1,\"y_m\":0.5},{\"x_m\":1.5,\"y_m\":0.9},"
                          "{\"x_m\":2,\"y_m\":0.9}]"));
    EXPECT_TRUE(has(snap, "\"pico\":{\"resource_id\":\"fake_pico\",\"frames_fresh\":true,"
                          "\"identity\":true,\"boot_id\":4660"))
        << snap;
    EXPECT_TRUE(has(snap, "\"firmware\":\"bno08x\""));
    EXPECT_TRUE(has(snap, "\"imu\":{\"enabled\":true,\"state\":\"ready\",\"reason\":\"none\""));
    EXPECT_TRUE(has(snap, "\"last_command\":{\"request_id\":7,\"op\":\"reinit_imu\","
                          "\"status\":\"completed\""));
    EXPECT_TRUE(has(snap, "\"text\":\"profile " + hex8(id) + " applied"));
    EXPECT_TRUE(has(snap, "\"text\":\"Pico IMU reinitialization requested\""));

    // a mode none report clears the path
    translagatr::BrainRequest clear = r.request(translagatr::kOpPathReport);
    clear.command_id          = 42;
    EXPECT_EQ(r.one(clear).result, translagatr::kResultOk);
    EXPECT_TRUE(has(snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now)),
                    "\"path\":null"));
}

TEST(BrainProfile, ResetWaitsForAProfileAgain) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    r.system->reset();
    EXPECT_EQ(r.system->profileBinding(), nullptr);
    EXPECT_EQ(r.system->localization().estimatorType(), "noop");
    EXPECT_GT(r.system->robot().odometry_epoch, before.odometry_epoch);
    EXPECT_EQ(r.system->sensorCatalog().payloadOf(SensorId{"profile_encoder_0"}), nullptr);
}

// ---- worker mode -----------------------------------------------------------

TEST(BrainProfile, WorkerModeBoundaryKeepsTheSessionAndInspection) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    const uint32_t session = r.hello();
    r.still(2);
    const uint32_t             pi_instance = r.one(r.request(translagatr::kOpGetState)).pi_instance;
    const std::vector<uint8_t> doc         = bytesOf(perpendicular(translagatr::kImuSourceBrainVex));
    r.stage(doc, 0, static_cast<uint16_t>(doc.size()), translagatr::kProfileChunkMax);

    // the APPLY waits in the link; the estimation worker prepares it
    translagatr::BrainRequest apply = r.request(translagatr::kOpProfileApply);
    apply.profile_id          = idOf(doc);
    apply.total_len           = static_cast<uint16_t>(doc.size());
    std::array<uint8_t, translagatr::kMaxFrameLen> bytes{};
    const uint16_t len = translagatr::encodeBrainRequest(apply, bytes.data(), bytes.size());
    ASSERT_TRUE(r.brain->write({bytes.data(), len}).ok);
    r.pi_raw->input().feed(r.brain_raw->output().takeAll());
    r.pi_raw->setClock({});   // the workers run on real time

    std::atomic<bool> done{false};
    std::atomic<int>  documents{0};
    std::thread       inspector([&] {
        while (!done.load()) {
            const std::string hello = helloDocument(*r.system, HostClock::now());
            const std::string snap =
                snapshotDocument(*r.system, InspectionServiceStats{}, HostClock::now());
            if (!hello.empty() && !snap.empty()) {
                documents.fetch_add(1);
            }
        }
    });
    std::string err;
    ASSERT_TRUE(r.system->start(err)) << err;
    bool       applied  = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!applied && std::chrono::steady_clock::now() < deadline) {
        applied = r.system->applyPendingProfile();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));   // workers run again
    r.system->stop();
    done.store(true);
    inspector.join();

    EXPECT_TRUE(applied);
    EXPECT_GT(documents.load(), 0);
    ASSERT_NE(r.system->profileBinding(), nullptr);
    EXPECT_EQ(r.system->profileBinding()->id, idOf(doc));
    EXPECT_EQ(r.system->command().profile.state, translagatr::kProfileApplied);
    EXPECT_EQ(r.system->command().session, session);

    // back inline: the same session and pi_instance answer
    r.pi_raw->setClock([&r] { return r.link_ms * 1000; });
    r.pi_raw->output().takeAll();
    r.now = std::max(r.now, HostClock::now().ms) + 1000;
    r.step();
    const translagatr::BrainReply state = r.one(r.request(translagatr::kOpGetState));
    EXPECT_EQ(state.result, translagatr::kResultOk);
    EXPECT_EQ(state.pi_instance, pi_instance);
    EXPECT_EQ(state.state.profile_state, translagatr::kProfileApplied);
}

// ---- stationary precheck ---------------------------------------------------

namespace
{

struct PrecheckFeed {
    SensorMap          map;
    StationaryPrecheck check;
    uint64_t           sequence = 0;

    PrecheckFeed() {
        check.configure({{SensorId{"enc"}, 0.05}}, SensorId{"imu"});
    }

    void sample(int64_t at_ms, double angle_rad, double rate_rad_s, uint64_t discontinuity = 0,
                bool imu = true) {
        ++sequence;
        EncoderSample e;
        e.angle_rad           = angle_rad;
        e.discontinuity_epoch = discontinuity;
        MeasurementRecord& enc = map[SensorId{"enc"}];
        enc.state              = SourceState::kValid;
        StoredSample s;
        s.receivedAt = hostTime(at_ms);
        s.sequence   = sequence;
        s.payload    = TypedPayload::store(e, payload_names::kEncoderSample);
        enc.latest   = s;
        if (imu) {
            ImuSample g;
            g.yaw_rate_rad_s        = rate_rad_s;
            MeasurementRecord& rec  = map[SensorId{"imu"}];
            rec.state               = SourceState::kValid;
            StoredSample gs;
            gs.receivedAt = hostTime(at_ms);
            gs.sequence   = sequence;
            gs.payload    = TypedPayload::store(g, payload_names::kImuSample);
            rec.latest    = gs;
        }
        check.update(map, hostTime(at_ms));
    }
};

} // namespace

TEST(StationaryPrecheck, NeedsFreshContinuousCoverageOfTheWindow) {
    PrecheckFeed f;
    std::string  why;
    EXPECT_FALSE(f.check.still(hostTime(0), &why));   // nothing yet
    for (int64_t t = 0; t <= 200; t += 20) {
        f.sample(t, 0.0, 0.0);
    }
    EXPECT_FALSE(f.check.still(hostTime(200), &why));   // the window is not covered
    EXPECT_NE(why.find("cover"), std::string::npos) << why;
    for (int64_t t = 220; t <= 400; t += 20) {
        f.sample(t, 0.0, 0.0);
    }
    EXPECT_TRUE(f.check.still(hostTime(400), &why)) << why;
    EXPECT_FALSE(f.check.still(hostTime(600), &why));   // stale now
    EXPECT_NE(why.find("fresh"), std::string::npos) << why;

    // a repeated record is not new evidence: time passes, nothing arrives
    const uint64_t seq = f.sequence;
    for (int64_t t = 420; t <= 700; t += 20) {
        f.check.update(f.map, hostTime(t));
    }
    EXPECT_EQ(f.sequence, seq);
    EXPECT_FALSE(f.check.still(hostTime(700), &why));
}

TEST(StationaryPrecheck, MovementTurningGapsAndDiscontinuitiesAreNotStill) {
    {
        PrecheckFeed f;
        for (int64_t t = 0; t <= 400; t += 20) {
            f.sample(t, t < 300 ? 0.0 : 0.03, 0.0);   // 1.5 mm at 0.05 m radius
        }
        std::string why;
        EXPECT_FALSE(f.check.still(hostTime(400), &why));
        EXPECT_NE(why.find("moving"), std::string::npos) << why;
        // stops, and once the window holds only still samples it is still
        for (int64_t t = 420; t <= 800; t += 20) {
            f.sample(t, 0.03, 0.0);
        }
        EXPECT_TRUE(f.check.still(hostTime(800), &why)) << why;
    }
    {
        PrecheckFeed f;
        for (int64_t t = 0; t <= 400; t += 20) {
            f.sample(t, 0.0, degToRad(t == 300 ? 3.0 : 0.5));
        }
        std::string why;
        EXPECT_FALSE(f.check.still(hostTime(400), &why));
        EXPECT_NE(why.find("turning"), std::string::npos) << why;
    }
    {
        PrecheckFeed f;
        for (int64_t t = 0; t <= 400; t += 20) {
            if (t < 200 || t > 340) {
                f.sample(t, 0.0, 0.0);
            }
        }
        std::string why;
        EXPECT_FALSE(f.check.still(hostTime(400), &why));
        EXPECT_NE(why.find("gap"), std::string::npos) << why;
    }
    {
        PrecheckFeed f;
        for (int64_t t = 0; t <= 400; t += 20) {
            f.sample(t, 0.0, 0.0, t < 300 ? 0 : 1);   // an encoder restart
        }
        std::string why;
        EXPECT_FALSE(f.check.still(hostTime(400), &why));
        EXPECT_NE(why.find("discontinuity"), std::string::npos) << why;
    }
    {
        PrecheckFeed f;
        for (int64_t t = 0; t <= 400; t += 20) {
            f.sample(t, 0.0, 0.0, 0, false);   // the gyro never arrives
        }
        std::string why;
        EXPECT_FALSE(f.check.still(hostTime(400), &why));
        EXPECT_NE(why.find("imu"), std::string::npos) << why;
    }
}

TEST(StationaryPrecheck, TheVexRotationMustHoldToo) {
    StationaryPrecheck                             check;
    auto                                           vex = std::make_shared<BrainImuBench>();
    SensorMap                                      map;
    const std::vector<StationaryPrecheck::Wheel> none;
    check.configure(none, SensorId{}, vex);
    std::string why;
    int32_t     mdeg = 0;
    const auto  feed = [&](int64_t t, int32_t step_mdeg) {
        mdeg += step_mdeg;
        vex->accept(1, translagatr::kBenchImuValid, static_cast<uint32_t>(t), mdeg, hostTime(t));
        check.update(map, hostTime(t));
    };
    for (int64_t t = 0; t <= 400; t += 20) {
        feed(t, 10);   // 0.5 deg/s of drift
    }
    EXPECT_TRUE(check.still(hostTime(400), &why)) << why;
    for (int64_t t = 420; t <= 800; t += 20) {
        feed(t, t == 700 ? 1000 : 10);   // a one degree step
    }
    EXPECT_FALSE(check.still(hostTime(800), &why));
    EXPECT_NE(why.find("turning"), std::string::npos) << why;
    vex->reset(1);   // the Brain recalibrates its IMU: no samples, never still
    check.update(map, hostTime(1000));
    EXPECT_FALSE(check.still(hostTime(1000), &why));
}

// ---- calibration and recovery ----------------------------------------------

namespace
{

// The Brain resends a CONTROL under its request id after a lost reply.
translagatr::BrainReply repeatControl(Rig& r, uint16_t rid, uint8_t action) {
    translagatr::BrainRequest dup = r.request(translagatr::kOpControl);
    r.rid--;
    dup.request_id = rid;
    dup.action     = action;
    r.picoFrame();
    return r.one(dup);
}

translagatr::BrainReply readWheels(Rig& r) {
    r.picoFrame();
    return r.one(r.request(translagatr::kOpReadWheels));
}

// Cycles with Pico frames and no Brain request, as when the cable is out.
void quietLink(Rig& r, int cycles, std::array<double, 3> delta = {}) {
    for (int c = 0; c < cycles; ++c) {
        for (int i = 0; i < 3; ++i) {
            r.counts[i] += delta[i] / cycles;
        }
        r.picoFrame();
        r.step();
    }
}

// Cycles with Brain requests and no Pico frames, as when the Pico link is out.
void quietPico(Rig& r, int cycles, std::array<double, 3> delta = {}) {
    for (int c = 0; c < cycles; ++c) {
        for (int i = 0; i < 3; ++i) {
            r.counts[i] += delta[i] / cycles;
        }
        translagatr::BrainRequest q = r.request(translagatr::kOpGetState);
        q.imu_flags           = translagatr::kBenchImuValid;
        q.imu_stamp_ms        = static_cast<uint32_t>(r.now);
        r.one(q);
    }
}

} // namespace

TEST(BrainRecovery, WheelReadingsReportRawTravelInProfileOrder) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    r.still(3);
    EXPECT_EQ(readWheels(r).result, translagatr::kResultNotReady);   // no profile yet

    // profile order port 1 then port 0; port 1 geared 2:1 and reversed,
    // port 0 with a travel scale the readings never include
    translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourceBrainVex);
    std::swap(p.wheels[0], p.wheels[1]);
    p.wheels[0].gear_micro       = 2 * translagatr::kUnitMicro;
    p.wheels[0].flags            = translagatr::kWheelReversed;
    p.wheels[1].travel_scale_ppm = 1020000;
    ASSERT_TRUE(r.applyProfile(p));
    r.still(3);
    r.counts[0] += kCpr;   // one wheel revolution on port 0
    r.counts[1] += 1000;   // an eighth of one on port 1
    r.cycle();

    const translagatr::BrainReply reply = readWheels(r);
    ASSERT_EQ(reply.result, translagatr::kResultOk);
    ASSERT_EQ(reply.wheel_count, 2);
    const translagatr::WheelReading& a = reply.wheels[0];
    const translagatr::WheelReading& b = reply.wheels[1];
    EXPECT_EQ(a.port, 1);
    EXPECT_EQ(b.port, 0);
    EXPECT_EQ(a.counts, 1000);
    EXPECT_EQ(b.counts, kCpr);
    EXPECT_EQ(a.flags, translagatr::kWheelFresh | translagatr::kWheelValid);
    EXPECT_EQ(b.flags, translagatr::kWheelFresh | translagatr::kWheelValid);
    EXPECT_EQ(a.discontinuity, 0);
    EXPECT_LE(a.age_ms, 2 * kCycleMs);
    EXPECT_NEAR(a.travel_um, -1000.0 / (kCpr * 2.0) * kRevolution * 1e6, 1.0);
    EXPECT_NEAR(b.travel_um, kRevolution * 1e6, 1.0);   // no 1.02

    // the same request again reads again
    translagatr::BrainRequest again = r.request(translagatr::kOpReadWheels);
    r.rid--;
    again.request_id = static_cast<uint16_t>(r.rid - 1);
    r.counts[0] += kCpr;
    r.picoFrame();
    const translagatr::BrainReply repeated = r.one(again);
    ASSERT_EQ(repeated.result, translagatr::kResultOk);
    EXPECT_EQ(repeated.wheels[1].counts, 2 * kCpr);

    // a Pico restart shows as a new discontinuity; the travel stays continuous
    r.pico_zero = r.now - 20;
    r.counts    = {};
    r.still(3);
    const translagatr::BrainReply restarted = readWheels(r);
    ASSERT_EQ(restarted.result, translagatr::kResultOk);
    EXPECT_NE(restarted.wheels[1].discontinuity, repeated.wheels[1].discontinuity);
    EXPECT_NEAR(restarted.wheels[1].travel_um, 2.0 * kRevolution * 1e6, 1.0);
}


TEST(BrainRecovery, ReinitImuRunsOnThePicoOnceThenNeedsAPlacement) {
    Rig r(useFakePico);
    ready(r, perpendicular(translagatr::kImuSourcePico));
    r.still(20);
    const Pose2D held = r.pose();

    const translagatr::BrainReply first = r.control(translagatr::kControlReinitImu);
    EXPECT_EQ(first.result, translagatr::kResultPending);
    EXPECT_EQ(first.action, translagatr::kControlReinitImu);
    const uint16_t rid = static_cast<uint16_t>(r.rid - 1);
    ASSERT_EQ(r.fake->count(translagatr::kPicoOpReinitImu), 1);
    EXPECT_EQ(r.fake->last(translagatr::kPicoOpReinitImu).arg, 0);   // IMU port 0

    // lost replies: the Brain resends the same id; the Pico runs it once
    r.fake->settle(translagatr::kPicoOpReinitImu, PicoRequestState::kRunning);
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(repeatControl(r, rid, translagatr::kControlReinitImu).result,
                  translagatr::kResultPending);
    }
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpReinitImu), 1);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);   // placed while it runs

    r.fake->settle(translagatr::kPicoOpReinitImu, PicoRequestState::kCompleted);
    const translagatr::BrainReply done = repeatControl(r, rid, translagatr::kControlReinitImu);
    EXPECT_EQ(done.result, translagatr::kResultOk);
    EXPECT_NE(done.calibration, translagatr::kCalibrationDone);   // recalibrating
    EXPECT_EQ(repeatControl(r, rid, translagatr::kControlReinitImu).result, translagatr::kResultOk);
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpReinitImu), 1);

    // a restarted IMU the profile uses: the pose needs a placement again
    const translagatr::BrainState s = r.still(40);
    EXPECT_EQ(s.calibration, translagatr::kCalibrationDone);
    EXPECT_EQ(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NEAR(r.pose().x_m, held.x_m, 1e-9);   // shown where it was
    ASSERT_EQ(r.place(1000, 500, 0).result, translagatr::kResultOk);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);
}

TEST(BrainRecovery, PicoCommandsFailClearly) {
    Rig r(useFakePico);
    ready(r, perpendicular(translagatr::kImuSourcePico));

    r.fake->state.identity  = false;   // firmware without commands
    translagatr::BrainReply reply = r.control(translagatr::kControlReinitImu);
    EXPECT_EQ(reply.result, translagatr::kResultFailed);
    EXPECT_EQ(reply.control_detail, translagatr::kControlDetailPicoLink);
    r.fake->state.identity     = true;
    r.fake->state.frames_fresh = false;   // no Pico frames
    reply                      = r.control(translagatr::kControlReinitImu);
    EXPECT_EQ(reply.control_detail, translagatr::kControlDetailPicoLink);
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpReinitImu), 0);
    r.fake->state.frames_fresh = true;

    // the Pico reports its failure, with its reason
    reply = r.control(translagatr::kControlReinitImu);
    ASSERT_EQ(reply.result, translagatr::kResultPending);
    uint16_t rid = static_cast<uint16_t>(r.rid - 1);
    r.fake->settle(translagatr::kPicoOpReinitImu, PicoRequestState::kFailed,
                   translagatr::kControlDetailImuAbsent);
    reply = repeatControl(r, rid, translagatr::kControlReinitImu);
    EXPECT_EQ(reply.result, translagatr::kResultFailed);
    EXPECT_EQ(reply.control_detail, translagatr::kControlDetailImuAbsent);

    // never finishing: the bound ends it
    reply = r.control(translagatr::kControlReinitImu);
    ASSERT_EQ(reply.result, translagatr::kResultPending);
    rid = static_cast<uint16_t>(r.rid - 1);
    r.still(static_cast<int>(System::kPicoOperationMs / kCycleMs) + 2);
    reply = repeatControl(r, rid, translagatr::kControlReinitImu);
    EXPECT_EQ(reply.result, translagatr::kResultFailed);
    EXPECT_EQ(reply.control_detail, translagatr::kControlDetailTimedOut);
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpReinitImu), 2);

    // a VEX profile does not use the Pico IMU
    Rig v(useFakePico);
    ready(v, perpendicular(translagatr::kImuSourceBrainVex));
    reply = v.control(translagatr::kControlReinitImu);
    EXPECT_EQ(reply.result, translagatr::kResultFailed);
    EXPECT_EQ(reply.control_detail, translagatr::kControlDetailImuUnused);
    EXPECT_EQ(v.fake->count(translagatr::kPicoOpReinitImu), 0);
}

TEST(BrainRecovery, AcquisitionRestartNeedsStillnessThenAPlacement) {
    Rig r(useFakePico);
    ready(r, perpendicular(translagatr::kImuSourcePico));
    for (int i = 0; i < 5; ++i) {
        r.counts[0] += 200;
        r.cycle();
    }
    r.counts[0] += 200;
    EXPECT_EQ(r.control(translagatr::kControlRestartAcquisition).result, translagatr::kResultNotStationary);
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpRestartAcquisition), 0);

    r.still(20);
    const translagatr::BrainReply reply = r.control(translagatr::kControlRestartAcquisition);
    ASSERT_EQ(reply.result, translagatr::kResultPending);
    const uint16_t rid = static_cast<uint16_t>(r.rid - 1);
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpRestartAcquisition), 1);

    // completed on the Pico, but no frame of the new epoch yet
    r.fake->settle(translagatr::kPicoOpRestartAcquisition, PicoRequestState::kCompleted);
    EXPECT_EQ(repeatControl(r, rid, translagatr::kControlRestartAcquisition).result,
              translagatr::kResultPending);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);
    r.fake->state.acq_epoch = 1;
    r.fake->state.restarts  = 1;
    EXPECT_EQ(repeatControl(r, rid, translagatr::kControlRestartAcquisition).result,
              translagatr::kResultOk);
    EXPECT_EQ(r.fake->count(translagatr::kPicoOpRestartAcquisition), 1);
    EXPECT_EQ(r.still(3).robot_flags & translagatr::kRobotLocalized, 0);   // counters restarted
}

TEST(BrainRecovery, HealthCarriesThePicoLinkAndItsImuState) {
    // a VEX profile: the Pico IMU state is reported, never used
    Rig r(useFakePico);
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    translagatr::BrainState s = r.still(1);
    EXPECT_NE(s.health & translagatr::kHealthPicoLink, 0);
    EXPECT_EQ(s.health & (translagatr::kHealthImuInitializing | translagatr::kHealthImuFailed), 0);

    r.fake->state.status_known     = true;
    r.fake->state.status.imu_state = translagatr::kPicoImuAligning;
    EXPECT_NE(r.still(1).health & translagatr::kHealthImuInitializing, 0);
    r.fake->state.status.imu_state = translagatr::kPicoImuRetrying;
    EXPECT_NE(r.still(1).health & translagatr::kHealthImuInitializing, 0);
    r.fake->state.status.imu_state = translagatr::kPicoImuFailed;
    s                              = r.still(1);
    EXPECT_NE(s.health & translagatr::kHealthImuFailed, 0);
    EXPECT_EQ(s.health & translagatr::kHealthImuInitializing, 0);
    r.fake->state.status.imu_state = translagatr::kPicoImuReady;
    r.fake->state.frames_fresh     = false;
    s                              = r.still(1);
    EXPECT_EQ(s.health & (translagatr::kHealthPicoLink | translagatr::kHealthImuFailed), 0);
    EXPECT_NE(s.robot_flags & translagatr::kRobotLocalized, 0);   // nothing used was lost
}

TEST(BrainRecovery, StillnessReportsZeroVelocityAndMovementClearsIt) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourcePico));
    translagatr::BrainState s = r.still(1);
    EXPECT_NE(s.health & translagatr::kHealthStationary, 0);   // still since ready
    EXPECT_EQ(r.system->robot().vx_m_s, 0.0);
    EXPECT_EQ(r.system->robot().yaw_rate_rad_s, 0.0);

    // moving: not stationary, and the velocity is what the wheels say
    r.counts[0] += 400;
    s = r.cycle().state;
    EXPECT_EQ(s.health & translagatr::kHealthStationary, 0);
    r.counts[0] += 400;
    r.cycle();
    EXPECT_GT(r.system->robot().vx_m_s, 0.5);
    // still for a window again: stationary, zero velocity, the pose kept
    r.still(30);
    const Pose2D kept = r.pose();
    s                 = r.still(1);
    EXPECT_NE(s.health & translagatr::kHealthStationary, 0);
    EXPECT_EQ(r.system->robot().vx_m_s, 0.0);
    EXPECT_NEAR(r.pose().x_m, kept.x_m, 1e-12);
    EXPECT_NE(s.robot_flags & translagatr::kRobotLocalized, 0);
}

TEST(BrainRecovery, VexStillnessWatchesTheVexRotation) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    EXPECT_NE(r.still(30).health & translagatr::kHealthStationary, 0);
    EXPECT_EQ(r.system->robot().yaw_rate_rad_s, 0.0);
    // the VEX IMU turns while the wheels read nothing: not stationary
    translagatr::BrainState s;
    for (int i = 0; i < 5; ++i) {
        r.theta_mdeg += 200.0;
        s = r.cycle().state;
    }
    EXPECT_EQ(s.health & translagatr::kHealthStationary, 0);
    EXPECT_NE(r.system->robot().yaw_rate_rad_s, 0.0);
}

TEST(BrainRecovery, VexProfileIsReadyWhileThePicoImuHasFailed) {
    Rig r(useFakePico);
    r.send_gyro                    = false;
    r.fake->state.status_known     = true;
    r.fake->state.status.imu_state = translagatr::kPicoImuFailed;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState s = r.translate({kCpr, 0, 0}, 10);
    EXPECT_NE(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NE(s.health & translagatr::kHealthGyroFresh, 0);   // the VEX IMU
    EXPECT_NE(s.health & translagatr::kHealthImuFailed, 0);   // reported, not blocking
    EXPECT_EQ(s.calibration, translagatr::kCalibrationNone);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
}

// ---- sensor loss (spec 8.10) -------------------------------------------------

namespace
{

bool eventNamed(const Rig& r, const std::string& text) {
    for (const RuntimeEvent& e : r.system->events()) {
        if (e.text.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// Expects the robot unplaced in a new epoch, then placed again by a new
// SET_POSE and not before.
void expectLostThenPlacedAgain(Rig& r, const translagatr::BrainState& before) {
    translagatr::BrainState after = r.still(2);
    EXPECT_EQ(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(after.odometry_epoch, before.odometry_epoch);
    after = r.still(10);   // the old SET_POSE never re-applies
    EXPECT_EQ(after.robot_flags & translagatr::kRobotLocalized, 0);
    ASSERT_EQ(r.place(1200, 500, 0).result, translagatr::kResultOk);
    after = r.still(3);
    EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NEAR(r.pose().x_m, 1.2, 1e-9);
}

} // namespace

TEST(SensorLoss, AStaleVexImuEndsContinuityEvenStandingStill) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    const Pose2D            held   = r.pose();
    quietLink(r, 20);   // 400 ms without a Brain request
    EXPECT_NEAR(r.pose().x_m, held.x_m, 1e-9);   // shown where it was
    EXPECT_TRUE(eventNamed(r, "sensor lost: brain_vex_imu stale"));
    EXPECT_TRUE(eventNamed(r, "place again"));
    expectLostThenPlacedAgain(r, before);
}

TEST(SensorLoss, ABlipShorterThanTheLimitKeepsThePlacement) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    quietLink(r, 10);   // 200 ms
    const translagatr::BrainState after = r.still(3);
    EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    EXPECT_FALSE(eventNamed(r, "sensor lost"));
}

TEST(SensorLoss, AnInvalidVexSampleIsARestart) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    r.vex_valid = false;   // the Brain recalibrates its IMU
    r.cycle();
    r.vex_valid = true;
    r.still(1);
    EXPECT_TRUE(eventNamed(r, "sensor lost: brain_vex_imu restarted"));
    expectLostThenPlacedAgain(r, before);
}

// One invalid sample is one restart: the valid samples after it are the
// new epoch, not another restart.
TEST(SensorLoss, AnInvalidVexSampleIsOneRestart) {
    Rig r([](tinyxml2::XMLElement* root) {
        findChild(root, (std::string(kProfilePath) + "/Timing").c_str())
            ->SetAttribute("on_sensor_loss", "warn");
    });
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    r.vex_valid = false;
    r.cycle();
    r.vex_valid = true;
    r.still(5);
    int restarts = 0;
    for (const RuntimeEvent& e : r.system->events()) {
        restarts += e.text.find("brain_vex_imu restarted") != std::string::npos ? 1 : 0;
    }
    EXPECT_EQ(restarts, 1);
    EXPECT_FALSE(eventNamed(r, "motion lost"));   // the sensor loss explains it
}

TEST(SensorLoss, ABrainRestartRestartsTheVexImu) {
    Rig r;
    const translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourceBrainVex);
    ready(r, p);
    const translagatr::BrainState before = r.still(1);
    r.rid = 1;
    r.hello(0x5EED0002);   // a new Brain program: a new session, a new IMU epoch
    EXPECT_EQ(r.stageAndApply(bytesOf(p)).result, translagatr::kResultOk);   // no boundary
    r.still(1);
    EXPECT_TRUE(eventNamed(r, "sensor lost: brain_vex_imu restarted"));
    EXPECT_EQ(r.system->profileBinding()->generation, 1u);
    expectLostThenPlacedAgain(r, before);
}

TEST(SensorLoss, StaleOrMissingEncodersEndContinuity) {
    {
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourceBrainVex));
        const translagatr::BrainState before = r.still(1);
        quietPico(r, 20);   // no Pico frames for 400 ms
        EXPECT_TRUE(eventNamed(r, "sensor lost: encoder port"));
        expectLostThenPlacedAgain(r, before);
    }
    {
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourceBrainVex));
        const translagatr::BrainState before = r.still(1);
        r.mask_drop = translagatr::kSensorEnc1;   // frames keep coming without port 1
        r.still(20);
        r.mask_drop = 0;
        EXPECT_TRUE(eventNamed(r, "sensor lost: encoder port 1 stale"));
        expectLostThenPlacedAgain(r, before);
    }
    {
        // a port the profile does not use never matters
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourceBrainVex));
        const translagatr::BrainState before = r.still(1);
        r.mask_drop = translagatr::kSensorEnc2;
        const translagatr::BrainState after = r.still(20);
        EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
        EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    }
}

TEST(SensorLoss, APicoRebootEndsContinuityAndRestartsTheBias) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourcePico));
    const translagatr::BrainState before = r.still(1);
    ASSERT_EQ(before.calibration, translagatr::kCalibrationDone);
    const Pose2D held = r.pose();
    // the Pico restarts: its clock and its counters start over
    r.pico_zero               = r.now - 20;
    r.counts                  = {};
    const translagatr::BrainState s = r.still(1);
    EXPECT_NE(s.calibration, translagatr::kCalibrationDone);   // the IMU restarted with it
    EXPECT_EQ(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_TRUE(eventNamed(r, "restarted"));
    EXPECT_NEAR(r.pose().x_m, held.x_m, 1e-9);
    EXPECT_NEAR(r.pose().y_m, held.y_m, 1e-9);
    EXPECT_EQ(r.still(40).calibration, translagatr::kCalibrationDone);
    expectLostThenPlacedAgain(r, before);
}

// The encoders' own restart shows the reboot even when the Pico link reports
// no identity (v1 firmware), and a placement sent in that very cycle is
// withdrawn with the old ones.
TEST(SensorLoss, AnEncoderRestartEndsContinuityWithoutPicoIdentity) {
    Rig r(useFakePico);   // the fake link never reports a reboot
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    r.pico_zero = r.now - 20;
    r.counts    = {};
    EXPECT_EQ(r.place(1500, 500, 0).result, translagatr::kResultPending);
    const translagatr::BrainState after = r.still(2);
    EXPECT_TRUE(eventNamed(r, "sensor lost: encoder port 0 restarted"));
    EXPECT_EQ(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(after.odometry_epoch, before.odometry_epoch);
    ASSERT_EQ(r.place(1200, 500, 0).result, translagatr::kResultOk);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);
}

TEST(SensorLoss, APicoImuMattersOnlyWhenTheProfileUsesIt) {
    {
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourcePico));
        const translagatr::BrainState before = r.still(1);
        r.send_gyro = false;   // the IMU drops out, standing still
        r.still(20);
        r.send_gyro = true;
        EXPECT_TRUE(eventNamed(r, "sensor lost: pico_imu port 0 stale"));
        EXPECT_EQ(r.still(1).calibration, translagatr::kCalibrationDone);   // a gap is no restart
        expectLostThenPlacedAgain(r, before);
    }
    {
        Rig r(useFakePico);
        ready(r, perpendicular(translagatr::kImuSourcePico));
        const translagatr::BrainState before = r.still(1);
        r.fake->state.status_known     = true;
        r.fake->state.status.imu_state = translagatr::kPicoImuFailed;
        r.still(1);
        EXPECT_TRUE(eventNamed(r, "sensor lost: pico_imu failed"));
        r.fake->state.status.imu_state = translagatr::kPicoImuReady;
        expectLostThenPlacedAgain(r, before);
    }
    {
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourceBrainVex));
        const translagatr::BrainState before = r.still(1);
        r.send_gyro                    = false;
        const translagatr::BrainState after  = r.still(20);
        EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
        EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    }
}

TEST(SensorLoss, PlacingWhileASourceIsStillLostDoesNotHold) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourcePico));
    r.send_gyro = false;
    r.still(20);
    EXPECT_EQ(r.place(1200, 500, 0).result, translagatr::kResultOk);
    EXPECT_EQ(r.still(2).robot_flags & translagatr::kRobotLocalized, 0);
    r.send_gyro = true;
    r.still(2);
    ASSERT_EQ(r.place(1200, 500, 0).result, translagatr::kResultOk);
    EXPECT_NE(r.still(3).robot_flags & translagatr::kRobotLocalized, 0);
}

TEST(SensorLoss, WarnOnlyKeepsThePoseAndLogsOnce) {
    Rig r([](tinyxml2::XMLElement* root) {
        findChild(root, (std::string(kProfilePath) + "/Timing").c_str())
            ->SetAttribute("on_sensor_loss", "warn");
    });
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    quietLink(r, 40);
    const translagatr::BrainState after = r.still(2);
    EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    int warnings = 0;
    for (const RuntimeEvent& e : r.system->events()) {
        warnings += e.text.find("warn only") != std::string::npos ? 1 : 0;
    }
    EXPECT_EQ(warnings, 1);
}

// Rolling through gaps shorter than sensor_loss_ms (250): nothing is lost,
// the placement holds and the pose is the truth.
TEST(SensorLoss, GapsUnderTheLimitLoseNoTravelWhileRolling) {
    const std::array<double, 3> half = {kCpr / 2.0, 0, 0};
    {
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourceBrainVex));
        const translagatr::BrainState before = r.still(1);
        r.translate(half, 10);
        quietLink(r, 10, half);   // the next VEX sample 220 ms after the last
        r.translate(half, 10);
        quietPico(r, 10, half);   // the next wheel sample 220 ms after the last
        r.translate(half, 10);
        const translagatr::BrainState after = r.still(2);
        EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
        EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
        EXPECT_NEAR(r.pose().x_m, 1.0 + 2.5 * kRevolution, 1e-6);
        EXPECT_FALSE(eventNamed(r, "sensor lost") || eventNamed(r, "motion lost"));
    }
    {
        Rig r;
        ready(r, perpendicular(translagatr::kImuSourcePico));
        const translagatr::BrainState before = r.still(1);
        r.translate(half, 10);
        quietPico(r, 10, half);   // a 220 ms gyro and wheel gap, under max_gap_ms
        r.translate(half, 10);
        const translagatr::BrainState after = r.still(2);
        EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
        EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
        EXPECT_NEAR(r.pose().x_m, 1.0 + 1.5 * kRevolution, 1e-6);
        EXPECT_FALSE(eventNamed(r, "sensor lost") || eventNamed(r, "motion lost"));
    }
}

// Rolling through a 260 ms gap: the source was never quiet longer than
// 250 ms at a cycle start, so only the model that could not measure across
// the gap sees it. It drops the interval, and that ends continuity.
TEST(SensorLoss, AGapPastTheLimitBetweenChecksEndsContinuity) {
    const std::array<double, 3> half = {kCpr / 2.0, 0, 0};
    const auto run = [&](uint8_t imu_source, bool link_gap, const char* why) {
        SCOPED_TRACE(why);
        Rig r;
        ready(r, perpendicular(imu_source));
        const translagatr::BrainState before = r.still(1);
        r.translate(half, 10);
        if (link_gap) {
            quietLink(r, 12, half);
        } else {
            quietPico(r, 12, half);
        }
        r.translate(half, 2);
        EXPECT_TRUE(eventNamed(r, std::string("motion lost: profile_motion: ") + why));
        EXPECT_FALSE(eventNamed(r, "sensor lost"));
        expectLostThenPlacedAgain(r, before);
    };
    run(translagatr::kImuSourceBrainVex, true, "no VEX IMU and wheel pair for 260 ms");
    run(translagatr::kImuSourceBrainVex, false, "no VEX IMU and wheel pair for 260 ms");
    run(translagatr::kImuSourcePico, false, "gyro gap");
}

// A VEX rotation that jumps (an unannounced zeroing) is no measurement: the
// bench model drops that step, and the placement goes with it.
TEST(SensorLoss, AVexRotationJumpEndsContinuity) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    r.theta_mdeg += 90000.0;   // 90 degrees within one 20 ms poll
    r.still(1);
    EXPECT_TRUE(eventNamed(r, "motion lost: profile_motion: VEX IMU rotation jumped"));
    expectLostThenPlacedAgain(r, before);
}

// A placement sent in the cycle whose model drops an interval is withdrawn
// with the earlier ones.
TEST(SensorLoss, APlacementInTheCycleOfADroppedIntervalIsWithdrawn) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourcePico));
    const translagatr::BrainState before = r.still(1);
    quietPico(r, 12, {kCpr / 2.0, 0, 0});
    EXPECT_EQ(r.place(1500, 500, 0).result, translagatr::kResultPending);   // with the 260 ms frame
    EXPECT_FALSE(r.system->robotFeed()->latest().initialized);   // published at once
    const translagatr::BrainState after = r.still(2);
    EXPECT_TRUE(eventNamed(r, "motion lost: profile_motion: gyro gap"));
    EXPECT_EQ(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(after.odometry_epoch, before.odometry_epoch);
    ASSERT_EQ(r.place(1200, 500, 0).result, translagatr::kResultOk);
    EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);
}

// The Brain VEX IMU bench model takes its step limit from sensor_loss_ms,
// never a number of its own.
TEST(SensorLoss, TheVexStepLimitIsTheProfileLossLimit) {
    Rig r([](tinyxml2::XMLElement* root) {
        findChild(root, (std::string(kProfilePath) + "/Timing").c_str())
            ->SetAttribute("sensor_loss_ms", 400);
    });
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    const translagatr::BrainState before = r.still(1);
    const std::array<double, 3> half = {kCpr / 2.0, 0, 0};
    quietLink(r, 16, half);   // 340 ms between VEX samples, rolling
    r.translate(half, 10);
    translagatr::BrainState s = r.still(2);
    EXPECT_NE(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
    quietLink(r, 20, half);   // 420 ms
    s = r.still(2);
    EXPECT_EQ(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(s.odometry_epoch, before.odometry_epoch);

    // the encoder sensors' own freshness (Encoders stale_after_ms, 250) still
    // ends a step: 300 ms without Pico frames loses the placement
    ASSERT_EQ(r.place(1200, 500, 0).result, translagatr::kResultOk);
    const translagatr::BrainState placed = r.still(2);
    ASSERT_NE(placed.robot_flags & translagatr::kRobotLocalized, 0);
    quietPico(r, 15);
    s = r.still(2);
    EXPECT_TRUE(eventNamed(r, "missing or stale: place again"));
    EXPECT_EQ(s.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(s.odometry_epoch, placed.odometry_epoch);
}

// The count of dropped intervals starts over with each profile's models; a
// drop under a new profile still ends continuity.
TEST(SensorLoss, ADroppedIntervalAfterANewProfileStillCounts) {
    Rig                    r;
    translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourceBrainVex);
    ready(r, p);
    quietLink(r, 12, {kCpr / 2.0, 0, 0});
    r.still(2);
    ASSERT_TRUE(eventNamed(r, "motion lost"));
    p.wheels[0].travel_scale_ppm = 1001000;   // a calibrated scale: a new profile
    ASSERT_TRUE(r.applyProfile(p));
    r.still(3);
    ASSERT_EQ(r.place(1000, 500, 0).result, translagatr::kResultOk);
    const translagatr::BrainState before = r.still(2);
    ASSERT_NE(before.robot_flags & translagatr::kRobotLocalized, 0);
    quietLink(r, 12, {kCpr / 2.0, 0, 0});
    const translagatr::BrainState after = r.still(2);
    EXPECT_EQ(after.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_GT(after.odometry_epoch, before.odometry_epoch);
}

// The Pico gyro bias calibrates only while the robot stands still; motion
// meanwhile is never integrated, so a placed robot that moves before the
// window completes is unplaced. Left still, the placement holds.
TEST(BrainRecovery, MovingWhileTheGyroBiasCalibratesEndsContinuity) {
    const translagatr::RobotProfileDoc p = perpendicular(translagatr::kImuSourcePico);
    {
        // CONTROL 1 on a placed robot, then it rolls
        Rig r;
        ready(r, p);
        r.still(20);
        const translagatr::BrainState before = r.still(1);
        ASSERT_EQ(r.control(translagatr::kControlRecalibrate).result, translagatr::kResultOk);
        r.translate({kCpr, 0, 0}, 10);
        EXPECT_TRUE(eventNamed(r, "motion lost: profile_motion:"));
        EXPECT_TRUE(eventNamed(r, "while the gyro bias calibrated"));
        EXPECT_EQ(r.still(60).calibration, translagatr::kCalibrationDone);
        expectLostThenPlacedAgain(r, before);
    }
    {
        // placed during the first calibration after APPLY, then it rolls
        Rig r;
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        r.still(3);
        ASSERT_TRUE(r.applyProfile(p));
        ASSERT_EQ(r.place(1000, 500, 0).result, translagatr::kResultOk);
        const translagatr::BrainState before = r.still(1);
        EXPECT_NE(before.robot_flags & translagatr::kRobotLocalized, 0);
        EXPECT_NE(before.calibration, translagatr::kCalibrationDone);
        r.translate({kCpr, 0, 0}, 10);
        EXPECT_TRUE(eventNamed(r, "moved while the gyro bias calibrated"));
        expectLostThenPlacedAgain(r, before);
    }
    {
        // placed during calibration and left still: the pose holds
        Rig r;
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        r.still(3);
        ASSERT_TRUE(r.applyProfile(p));
        ASSERT_EQ(r.place(1000, 500, 0).result, translagatr::kResultOk);
        const translagatr::BrainState before = r.still(1);
        const translagatr::BrainState after  = r.still(40);
        EXPECT_EQ(after.calibration, translagatr::kCalibrationDone);
        EXPECT_NE(after.robot_flags & translagatr::kRobotLocalized, 0);
        EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
        EXPECT_FALSE(eventNamed(r, "sensor lost") || eventNamed(r, "motion lost"));
        r.translate({kCpr, 0, 0}, 10);   // calibrated: rolling is measured
        r.still(2);
        EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
        EXPECT_NE(r.still(1).robot_flags & translagatr::kRobotLocalized, 0);
    }
}

TEST(BrainRecovery, CalibrationFailsAfterItsBoundAndRecalibrateRetries) {
    Rig r([](tinyxml2::XMLElement* root) {
        findChild(root, (std::string(kProfilePath) + "/Calibration").c_str())
            ->SetAttribute("attempt_s", 1);
    });
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    r.still(3);
    ASSERT_TRUE(r.applyProfile(perpendicular(translagatr::kImuSourcePico)));
    translagatr::BrainState s;
    for (int i = 0; i < 70; ++i) {   // never still for a window
        r.counts[0] += 100;
        s = r.cycle().state;
    }
    EXPECT_EQ(s.calibration, translagatr::kCalibrationFailed);
    EXPECT_EQ(s.robot_flags & translagatr::kRobotPoseValid, 0);            // nothing integrated
    EXPECT_EQ(r.still(40).calibration, translagatr::kCalibrationFailed);   // bounded, stays
    const translagatr::BrainReply retry = r.control(translagatr::kControlRecalibrate);
    EXPECT_EQ(retry.result, translagatr::kResultOk);
    EXPECT_EQ(r.still(40).calibration, translagatr::kCalibrationDone);
}

TEST(BrainProfileInspection, SensorLossAndStillnessShowInTheSnapshot) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    r.still(30);
    std::string snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_TRUE(has(snap, "\"stationary\":true,\"continuity_breaks\":0")) << snap;
    EXPECT_TRUE(has(snap, "\"calibration\":null"));   // the VEX IMU calibrates on the Brain
    EXPECT_TRUE(has(snap, "\"link_open\":true"));
    r.pi_link->pulled = true;   // the USB cable out for 400 ms
    quietLink(r, 20);
    snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    ASSERT_TRUE(inspection_test::validJson(snap));
    EXPECT_TRUE(has(snap, "\"link_open\":false"));
    EXPECT_TRUE(has(snap, "\"continuity_breaks\":1,\"last_break\":\"sensor lost: brain_vex_imu"))
        << snap;
    EXPECT_TRUE(has(snap, "\"localized\":false"));
    EXPECT_TRUE(has(snap, "\"last_request_age_ms\":400"));
    EXPECT_TRUE(has(snap, "place again"));
    // the bench model dropped the stretch it could not measure
    EXPECT_TRUE(has(snap, "\"dropped_intervals\":1,\"dropped_why\":\"VEX IMU stale\"")) << snap;
    EXPECT_TRUE(has(snap, "\"movements\":0,\"attempts\""));
    r.pi_link->pulled = false;
    quietLink(r, 1);   // the first drain after a reopen answers nothing
    r.still(3);
    snap = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_TRUE(has(snap, "\"link_open\":true"));
    EXPECT_TRUE(has(snap, "\"last_request_age_ms\":0"));
}

// ---- camera preview without field correction ---------------------------------

namespace
{

const char* kCameraConfig = "brain_profile_usb_camera.xml";

Edit cameraType(const char* type) {
    return [type](tinyxml2::XMLElement* root) {
        tinyxml2::XMLElement* resources = root->FirstChildElement("Resources");
        for (auto* e = resources->FirstChildElement("Resource"); e != nullptr;
             e = e->NextSiblingElement("Resource")) {
            if (ConfigNode{e}.attr("id") == "front_camera_device") {
                e->DeleteChildren();
                e->SetAttribute("type", type);
            }
        }
    };
}

// The perpendicular VEX profile on the camera configuration: the same
// placement and axis checks as without a camera.
void cameraConfigLocalizes(Rig& r) {
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    r.translate({kCpr, 0, 0}, 10);
    r.translate({0, kCpr, 0}, 10);
    r.still(3);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
    EXPECT_NEAR(r.pose().y_m, 0.5 + kRevolution, 1e-6);
    EXPECT_TRUE(r.system->robot().valid);
    EXPECT_TRUE(r.system->robot().initialized);
    // no field correction exists: the world estimator is noop and the field
    // keeps its nominal definition
    const auto field = r.system->fieldSnapshot();
    ASSERT_NE(field, nullptr);
    EXPECT_TRUE(field->observations.empty());
    EXPECT_TRUE(field->associations.empty());
    for (const auto& kv : field->field.objects) {
        EXPECT_FALSE(kv.second.observed) << kv.first.value;
        EXPECT_NE(kv.second.source, EstimateSource::kObserved) << kv.first.value;
    }
}

} // namespace

TEST(BrainProfileCamera, PreviewConfigIsTheUsbConfigPlusOnlyTheCamera) {
    const auto load = [](const char* name, tinyxml2::XMLDocument& doc) {
        ResolvedConfiguration config;
        std::string           err;
        ASSERT_TRUE(resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) + "/override/" + name,
                                         config, err))
            << err;
        ASSERT_EQ(doc.Parse(config.xml.c_str()), tinyxml2::XML_SUCCESS);
    };
    const auto print = [](const tinyxml2::XMLNode* e) {
        tinyxml2::XMLPrinter printer;
        e->Accept(&printer);
        return std::string(printer.CStr());
    };
    // comments explain each file differently; only elements must match
    std::function<void(tinyxml2::XMLNode*)> uncomment = [&](tinyxml2::XMLNode* n) {
        for (tinyxml2::XMLNode* c = n->FirstChild(); c != nullptr;) {
            tinyxml2::XMLNode* next = c->NextSibling();
            if (c->ToComment() != nullptr) {
                n->DeleteChild(c);
            } else {
                uncomment(c);
            }
            c = next;
        }
    };
    tinyxml2::XMLDocument usb, camera;
    load("brain_profile_usb.xml", usb);
    load(kCameraConfig, camera);
    uncomment(&usb);
    uncomment(&camera);

    // the camera adds one resource and one sensor, and nothing reads the sensor
    tinyxml2::XMLElement* resources = camera.RootElement()->FirstChildElement("Resources");
    tinyxml2::XMLElement* device    = nullptr;
    for (auto* e = resources->FirstChildElement("Resource"); e != nullptr;
         e = e->NextSiblingElement("Resource")) {
        if (ConfigNode{e}.attr("id") == "front_camera_device") {
            device = e;
        }
    }
    ASSERT_NE(device, nullptr);
    EXPECT_EQ(ConfigNode{device}.attr("type"), "libcamera_camera");
    EXPECT_EQ(device->FirstChildElement("Calibration"), nullptr);   // preview needs none
    resources->DeleteChild(device);
    tinyxml2::XMLElement* sensors = camera.RootElement()->FirstChildElement("Sensors");
    ASSERT_NE(sensors, nullptr);
    const std::string pipeline = print(findChild(camera.RootElement(), "Pipeline"));
    EXPECT_EQ(pipeline.find("front_camera"), std::string::npos);
    camera.RootElement()->DeleteChild(sensors);

    EXPECT_EQ(print(findChild(usb.RootElement(), "Resources")),
              print(findChild(camera.RootElement(), "Resources")));
    EXPECT_EQ(print(findChild(usb.RootElement(), "Pipeline")), pipeline);
    // the world estimator stays noop: no detector, no association, no landmark update
    tinyxml2::XMLElement* world = findChild(camera.RootElement(), "Pipeline/WorldEstimation");
    ASSERT_NE(world, nullptr);
    EXPECT_EQ(ConfigNode{world->FirstChildElement("Estimator")}.attr("type"), "noop");

    // preview is bounded by the inspection budget
    const tinyxml2::XMLElement* inspection = camera.RootElement()->FirstChildElement("Inspection");
    ASSERT_NE(inspection, nullptr);
    EXPECT_STREQ(inspection->Attribute("preview_hz"), "5");
    EXPECT_STREQ(inspection->Attribute("preview_max_width"), "640");
}

TEST(BrainProfileCamera, MissingCameraNeverStopsLocalization) {
    Rig r(kCameraConfig, cameraType("test_dead_camera"));
    ASSERT_TRUE(r.ok()) << r.build_error;
    cameraConfigLocalizes(r);
    const SensorMap& sensors = r.system->sensorMap();
    const auto       camera  = sensors.find(SensorId{"front_camera"});
    ASSERT_NE(camera, sensors.end());
    EXPECT_NE(camera->second.state, SourceState::kValid);
    EXPECT_FALSE(camera->second.latest.has_value());
    EXPECT_TRUE(r.system->detectionFrames().empty());

    // the Brain sees a healthy, placed robot throughout
    const translagatr::BrainState s = r.still(2);
    EXPECT_EQ(s.robot_flags & (translagatr::kRobotPoseValid | translagatr::kRobotLocalized),
              translagatr::kRobotPoseValid | translagatr::kRobotLocalized);
    EXPECT_EQ(s.health & translagatr::kHealthVisionAlive, 0);
}

TEST(BrainProfileCamera, LiveCameraPreviewsRawFramesAndCorrectsNothing) {
    Rig r(kCameraConfig, cameraType("test_live_camera"));
    ASSERT_TRUE(r.ok()) << r.build_error;
    cameraConfigLocalizes(r);
    const auto frames = r.system->detectionFrames();
    const auto it     = frames.find(SensorId{"front_camera"});
    ASSERT_NE(it, frames.end());
    EXPECT_FALSE(it->second->has_observations);   // preview only, nothing decoded
    EXPECT_FALSE(it->second->has_trace);
    EXPECT_EQ(it->second->width_px, 16);
    EXPECT_EQ(it->second->intrinsics, nullptr);   // uncalibrated, metric output unavailable
}

TEST(BrainProfileCamera, SameProfileSamePoseWithAndWithoutTheCamera) {
    Rig plain;
    Rig camera(kCameraConfig, cameraType("test_live_camera"));
    ASSERT_TRUE(plain.ok()) << plain.build_error;
    ASSERT_TRUE(camera.ok()) << camera.build_error;
    for (Rig* r : {&plain, &camera}) {
        ready(*r, perpendicular(translagatr::kImuSourceBrainVex));
        r->translate({kCpr / 3.0, -kCpr / 5.0, 0}, 12);
        r->still(2);
    }
    EXPECT_DOUBLE_EQ(plain.pose().x_m, camera.pose().x_m);
    EXPECT_DOUBLE_EQ(plain.pose().y_m, camera.pose().y_m);
    EXPECT_DOUBLE_EQ(plain.pose().heading_rad, camera.pose().heading_rad);
}

// ---- Pico diagnostics and attitude in the Brain profile configs -----------------

TEST(BrainProfileConfigs, EveryBrainProfileConfigAsksForPicoDiagnostics) {
    for (const char* name : {"brain_profile_usb.xml", "brain_profile_rs485.xml", kCameraConfig}) {
        ResolvedConfiguration config;
        std::string           err;
        ASSERT_TRUE(resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) + "/override/" + name,
                                         config, err))
            << err;
        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.Parse(config.xml.c_str()), tinyxml2::XML_SUCCESS);
        const tinyxml2::XMLElement* found = nullptr;
        tinyxml2::XMLElement* resources   = doc.RootElement()->FirstChildElement("Resources");
        for (auto* e = resources->FirstChildElement("Resource"); e != nullptr;
             e = e->NextSiblingElement("Resource")) {
            if (ConfigNode{e}.attr("type") == "pico_telemetry") {
                found = e->FirstChildElement("Diagnostics");
            }
        }
        ASSERT_NE(found, nullptr) << name;
        EXPECT_STREQ(found->Attribute("hz"), "1") << name;
    }
}

TEST(BrainProfile, VexProfileShowsTheTelemetryTiltAsAttitude) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourceBrainVex));
    // configured but nothing measured: unavailable, never an invented tilt
    Attitude a = r.system->robot().attitude;
    EXPECT_FALSE(a.valid);
    EXPECT_EQ(a.source, kProfileAttitudeId);
    EXPECT_FALSE(a.measuredAt.isSet());

    const Pose2D before = r.pose();
    translagatr::BrainRequest t = r.request(translagatr::kOpTelemetry);
    t.telemetry.flags      = translagatr::kTelemetryAttitude | translagatr::kTelemetryMotion;
    t.telemetry.stamp_ms   = static_cast<uint32_t>(r.now);
    t.telemetry.roll_cdeg  = -500;
    t.telemetry.pitch_cdeg = 250;
    t.telemetry.cmd_vx_mm_s = 900;   // a command report changes nothing on the Pi
    r.picoFrame();
    EXPECT_EQ(r.one(t).result, translagatr::kResultOk);
    a = r.system->robot().attitude;
    ASSERT_TRUE(a.valid);
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(a, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), -5.0, 1e-9);
    EXPECT_NEAR(radToDeg(pitch), 2.5, 1e-9);
    EXPECT_DOUBLE_EQ(r.pose().x_m, before.x_m);
    EXPECT_DOUBLE_EQ(r.pose().y_m, before.y_m);

    // no new report: stale after 250 ms, the pose still valid and placed
    r.still(14);
    a = r.system->robot().attitude;
    EXPECT_FALSE(a.valid);
    EXPECT_TRUE(a.assumed_level);
    EXPECT_TRUE(a.measuredAt.isSet());
    EXPECT_TRUE(r.system->robot().valid);
    EXPECT_TRUE(r.system->robot().initialized);
}

TEST(BrainProfile, PicoImuProfilesCarryNoAttitude) {
    Rig r;
    ready(r, perpendicular(translagatr::kImuSourcePico));
    const Attitude a = r.system->robot().attitude;
    EXPECT_FALSE(a.valid);
    EXPECT_TRUE(a.assumed_level);
    EXPECT_TRUE(a.source.empty());   // no source configured: assumed level
}
