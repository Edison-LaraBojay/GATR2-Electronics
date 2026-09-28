// brain_profile_gtest.cpp
// Brain robot profiles through the real wire path: the checked-in
// brain_profile_usb.xml (or an edited copy), the NG1 USB envelope over
// memory links, the real brain link codec and Pico sensor frames, and the
// System's profile boundary. Every geometry value here is a test value.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common/frame_codec.h"
#include "common/link_documents.h"
#include "config/composition.h"
#include "core/host_clock.h"
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

gatr2::ProfileWheel wheel(uint8_t port, int32_t x_um, int32_t y_um, int32_t angle_mdeg) {
    gatr2::ProfileWheel w;
    w.encoder_port   = port;
    w.counts_per_rev = kCpr;
    w.radius_um      = 24000;
    w.x_um           = x_um;
    w.y_um           = y_um;
    w.angle_mdeg     = angle_mdeg;
    return w;
}

gatr2::RobotProfileDoc baseProfile(uint8_t topology, uint8_t imu_source) {
    gatr2::RobotProfileDoc p;
    p.topology           = topology;
    p.imu_source         = imu_source;
    p.vex_smart_port     = imu_source == gatr2::kImuSourceBrainVex ? 1 : 0;
    p.footprint_front_um = 200000;
    p.footprint_back_um  = 200000;
    p.footprint_left_um  = 200000;
    p.footprint_right_um = 200000;
    p.calibration_window_ms = 500;   // the shortest window, for test time
    return p;
}

// Forward wheel on port 0 at (0, 0.15), sideways wheel on port 1 at (0.10, 0).
gatr2::RobotProfileDoc perpendicular(uint8_t imu_source) {
    gatr2::RobotProfileDoc p = baseProfile(gatr2::kTopologyTwoWheelImu, imu_source);
    p.wheel_count            = 2;
    p.wheels[0]              = wheel(0, 0, 150000, 0);
    p.wheels[1]              = wheel(1, 100000, 0, 90000);
    return p;
}

// Two forward-measuring wheels; the right one is mounted to measure backward.
gatr2::RobotProfileDoc twoForward(uint8_t imu_source) {
    gatr2::RobotProfileDoc p = baseProfile(gatr2::kTopologyTwoForwardWheelImu, imu_source);
    p.wheel_count            = 2;
    p.wheels[0]              = wheel(0, 0, 150000, 0);
    p.wheels[1]              = wheel(1, 0, -150000, 180000);
    return p;
}

gatr2::RobotProfileDoc threeWheel(uint8_t imu_source) {
    gatr2::RobotProfileDoc p = baseProfile(gatr2::kTopologyThreeWheel, imu_source);
    p.wheel_count            = 3;
    p.wheels[0]              = wheel(0, 0, 130000, 0);
    p.wheels[1]              = wheel(1, 0, -130000, 0);
    p.wheels[2]              = wheel(2, -120000, 0, 90000);
    return p;
}

std::vector<uint8_t> bytesOf(const gatr2::RobotProfileDoc& p) {
    std::vector<uint8_t> bytes(gatr2::kProfileMaxLen);
    bytes.resize(gatr2::encodeRobotProfile(p, bytes.data(), gatr2::kProfileMaxLen));
    EXPECT_FALSE(bytes.empty());
    return bytes;
}

uint32_t idOf(const std::vector<uint8_t>& doc) {
    return gatr2::crc32(doc.data(), static_cast<uint32_t>(doc.size()));
}

// Lever arm k = x uy - y ux of a profile wheel, meters.
double leverArm(const gatr2::ProfileWheel& w) {
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

// ---- rig -------------------------------------------------------------------

// The Pi built from brain_profile_usb.xml with the Pico UART and the Brain USB
// device replaced by memory links; the Brain side speaks NG1 through its own
// ProsUsbLink, exactly as the Brain app does.
struct Rig {
    FunctionRegistry              functions;
    std::shared_ptr<MemoryLink>   pi_raw    = std::make_shared<MemoryLink>();
    std::shared_ptr<MemoryLink>   brain_raw = std::make_shared<MemoryLink>();
    std::unique_ptr<ProsUsbLink>  brain;
    std::unique_ptr<System>       system;
    std::shared_ptr<MemoryLink>   pico;
    std::string                   build_error;

    int64_t  now     = 1000;
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

    explicit Rig(const Edit& edit = nullptr) {
        registerAll(functions);
        auto raw = pi_raw;
        EXPECT_TRUE(functions.add<ResourceMakeFunction>(
            FunctionKey{"test_usb"},
            [raw](const ConfigNode&, ResourceInitializationContext&, std::string&) {
                return ResourceInstance::asContract<SerialLink>(std::make_shared<ProsUsbLink>(raw));
            }));
        brain = std::make_unique<ProsUsbLink>(brain_raw);
        pi_raw->setClock([this] { return now * 1000; });

        ResolvedConfiguration config;
        if (!resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) +
                                      "/override/brain_profile_usb.xml",
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
        system->step(hostTime(now));
    }

    void picoFrame() {
        gatr2::SensorSample f{};
        f.seq      = seq++;
        f.stamp_ms = static_cast<uint32_t>(now);
        f.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorEnc2;
        for (int i = 0; i < 3; ++i) {
            f.enc[i] = static_cast<int32_t>(std::llround(counts[i]));
        }
        if (send_gyro) {
            f.mask |= gatr2::kSensorGyroZ;
            f.gyro_z = static_cast<int32_t>(std::llround(bias_mdps + rate_mdps));
        }
        std::vector<uint8_t> bytes(gatr2::kMaxFrameLen);
        bytes.resize(gatr2::encodeSensorFrame(f, bytes.data(), gatr2::kMaxFrameLen));
        EXPECT_FALSE(bytes.empty());
        pico->input().feed(bytes);
    }

    std::vector<gatr2::BrainReply> exchange(const gatr2::BrainRequest& r) {
        std::array<uint8_t, gatr2::kMaxFrameLen> bytes{};
        const uint16_t len = gatr2::encodeBrainRequest(r, bytes.data(), bytes.size());
        EXPECT_GT(len, 0);
        EXPECT_TRUE(brain->write({bytes.data(), len}).ok);
        pi_raw->input().feed(brain_raw->output().takeAll());
        step();
        brain_raw->input().feed(pi_raw->output().takeAll());
        std::vector<gatr2::BrainReply> replies;
        gatr2::FrameReader             reader;
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
                    gatr2::BrainReply reply;
                    EXPECT_TRUE(gatr2::decodeBrainReply(reader.frame(), reader.frameLen(), reply));
                    replies.push_back(reply);
                } while (reader.next());
            }
        }
        return replies;
    }

    gatr2::BrainReply one(const gatr2::BrainRequest& r) {
        const std::vector<gatr2::BrainReply> replies = exchange(r);
        EXPECT_EQ(replies.size(), 1u) << "op " << int(r.op);
        return replies.empty() ? gatr2::BrainReply{} : replies.front();
    }

    gatr2::BrainRequest request(uint8_t op) {
        gatr2::BrainRequest r;
        r.op         = op;
        r.session    = session;
        r.request_id = rid++;
        return r;
    }

    uint32_t hello(uint32_t nonce = 0x5EED0001) {
        gatr2::BrainRequest r = request(gatr2::kOpHello);
        r.session             = 0;
        r.nonce               = nonce;
        const gatr2::BrainReply reply = one(r);
        EXPECT_EQ(reply.result, gatr2::kResultOk);
        session = reply.session;
        return session;
    }

    // One pipeline cycle: a Pico frame and a state poll carrying the Brain
    // VEX IMU sample, as the Brain app sends it.
    gatr2::BrainReply cycle() {
        picoFrame();
        gatr2::BrainRequest r = request(gatr2::kOpGetState);
        r.imu_flags           = vex_valid ? gatr2::kBenchImuValid : 0;
        r.imu_stamp_ms        = static_cast<uint32_t>(now);
        r.imu_rotation_mdeg   = static_cast<int32_t>(std::llround(theta_mdeg));
        return one(r);
    }

    gatr2::BrainState still(int cycles) {
        gatr2::BrainReply last;
        for (int i = 0; i < cycles; ++i) {
            last = cycle();
        }
        return last.state;
    }

    // Raw count deltas per port spread evenly over the cycles, no rotation.
    gatr2::BrainState translate(std::array<double, 3> delta, int cycles) {
        gatr2::BrainReply last;
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
    gatr2::BrainState turn(double turn_mdps, int cycles, std::array<double, 3> counts_per_rad,
                           double wheel_gain = 1.0) {
        gatr2::BrainReply last;
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
    gatr2::BrainReply stageAndApply(const std::vector<uint8_t>& doc,
                                    uint16_t chunk = gatr2::kProfileChunkMax) {
        stage(doc, 0, static_cast<uint16_t>(doc.size()), chunk);
        return apply(doc);
    }

    void stage(const std::vector<uint8_t>& doc, uint16_t from, uint16_t to, uint16_t chunk) {
        for (uint16_t offset = from; offset < to; offset = static_cast<uint16_t>(offset + chunk)) {
            const uint16_t      n = static_cast<uint16_t>(std::min<int>(chunk, to - offset));
            gatr2::BrainRequest r = request(gatr2::kOpProfileWrite);
            r.profile_id          = idOf(doc);
            r.total_len           = static_cast<uint16_t>(doc.size());
            r.offset              = offset;
            r.data_len            = static_cast<uint8_t>(n);
            std::memcpy(r.data, doc.data() + offset, n);
            const gatr2::BrainReply reply = one(r);
            EXPECT_EQ(reply.result, gatr2::kResultOk);
        }
    }

    gatr2::BrainReply apply(const std::vector<uint8_t>& doc) {
        gatr2::BrainRequest r = request(gatr2::kOpProfileApply);
        r.profile_id          = idOf(doc);
        r.total_len           = static_cast<uint16_t>(doc.size());
        return one(r);
    }

    // The whole exchange a Brain runs: stage, APPLY (Pending), one cycle for
    // the boundary, APPLY again (Ok).
    bool applyProfile(const gatr2::RobotProfileDoc& p) {
        const std::vector<uint8_t> doc     = bytesOf(p);
        const gatr2::BrainReply    pending = stageAndApply(doc);
        EXPECT_EQ(pending.result, gatr2::kResultPending);
        if (pending.result != gatr2::kResultPending) {
            return false;
        }
        cycle();
        const gatr2::BrainReply done = apply(doc);
        EXPECT_EQ(done.result, gatr2::kResultOk);
        EXPECT_EQ(done.profile_state, gatr2::kProfileApplied);
        return done.result == gatr2::kResultOk;
    }

    gatr2::BrainReply place(int32_t x_mm, int32_t y_mm, int32_t heading_cdeg) {
        picoFrame();
        gatr2::BrainRequest r = request(gatr2::kOpSetPose);
        r.x_mm                = x_mm;
        r.y_mm                = y_mm;
        r.heading_cdeg        = heading_cdeg;
        return one(r);
    }

    gatr2::BrainReply control(uint8_t action) {
        picoFrame();
        gatr2::BrainRequest r = request(gatr2::kOpControl);
        r.action              = action;
        return one(r);
    }

    Pose2D pose() const { return system->robot().fieldPose(); }
};

// Applies a profile, places the robot at (1, 0.5, 0) and lets it settle.
void ready(Rig& r, const gatr2::RobotProfileDoc& p, int settle = 40) {
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    r.still(3);
    ASSERT_TRUE(r.applyProfile(p));
    r.still(settle);   // baselines, and a Pico gyro calibrates here
    const gatr2::BrainReply placed = r.place(1000, 500, 0);
    ASSERT_EQ(placed.result, gatr2::kResultOk);
    r.still(3);
    ASSERT_NEAR(r.pose().x_m, 1.0, 1e-9);
    ASSERT_NEAR(r.pose().y_m, 0.5, 1e-9);
}

std::array<double, 3> countsPerRad(const gatr2::RobotProfileDoc& p) {
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
    const gatr2::RobotProfileDoc p = perpendicular(imu_source);
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
    const gatr2::BrainState s = r.still(5);
    EXPECT_EQ(s.profile_state, gatr2::kProfileNone);
    EXPECT_EQ(s.robot_flags & gatr2::kRobotPoseValid, 0);
    EXPECT_EQ(s.health & gatr2::kHealthEncodersFresh, 0);   // no profile encoders yet
    EXPECT_EQ(s.calibration, gatr2::kCalibrationNone);
    EXPECT_NE(s.map_id, 0u);   // the field is served while waiting
    EXPECT_EQ(r.system->localization().estimatorType(), "noop");

    EXPECT_EQ(r.place(1000, 500, 0).result, gatr2::kResultNotReady);
    EXPECT_EQ(r.control(gatr2::kControlRecalibrate).result, gatr2::kResultNotReady);

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
    const std::string snapshot = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_NE(snapshot.find("pico_telemetry.encoder_0"), std::string::npos);
}

// ---- topologies ------------------------------------------------------------

TEST(BrainProfile, PerpendicularWheelsWithTheBrainVexImu) {
    perpendicularWheelsMeasureEachAxisOnce(gatr2::kImuSourceBrainVex);
}

TEST(BrainProfile, PerpendicularWheelsWithThePicoGyro) {
    perpendicularWheelsMeasureEachAxisOnce(gatr2::kImuSourcePico);
}

TEST(BrainProfile, VexProfileIsReadyWithNoPicoImuFramesAtAll) {
    Rig r;
    r.send_gyro = false;   // an absent or broken BNO08X: encoder frames only
    ready(r, perpendicular(gatr2::kImuSourceBrainVex));
    const gatr2::BrainState s = r.translate({kCpr, 0, 0}, 10);
    EXPECT_EQ(s.profile_state, gatr2::kProfileApplied);
    EXPECT_NE(s.robot_flags & gatr2::kRobotPoseValid, 0);
    EXPECT_NE(s.robot_flags & gatr2::kRobotLocalized, 0);
    EXPECT_NE(s.health & gatr2::kHealthEncodersFresh, 0);
    EXPECT_NE(s.health & gatr2::kHealthGyroFresh, 0);   // the Brain bench sample
    EXPECT_EQ(s.calibration, gatr2::kCalibrationNone);  // VEX firmware owns its calibration
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);

    // an invalid VEX sample drops the gyro bit and holds the pose
    r.vex_valid = false;
    const gatr2::BrainState stale = r.translate({kCpr, 0, 0}, 5);
    EXPECT_EQ(stale.health & gatr2::kHealthGyroFresh, 0);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
}

TEST(BrainProfile, TwoForwardWheelsWithThePicoGyro) {
    Rig                          r;
    const gatr2::RobotProfileDoc p = twoForward(gatr2::kImuSourcePico);
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
    const gatr2::RobotProfileDoc p = twoForward(gatr2::kImuSourceBrainVex);
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
    const gatr2::RobotProfileDoc p = threeWheel(gatr2::kImuSourcePico);
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

TEST(BrainProfile, ThreeWheelsWithoutAnImu) {
    Rig                          r;
    const gatr2::RobotProfileDoc p = threeWheel(gatr2::kImuSourceNone);
    ready(r, p);
    EXPECT_EQ(r.system->localization().estimatorType(), "planar_motion_integrator");
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));   // the gyro is ignored; the wheels turn
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
    const gatr2::BrainState s = r.still(1);
    EXPECT_EQ(s.calibration, gatr2::kCalibrationNone);
    EXPECT_EQ(s.health & gatr2::kHealthGyroFresh, 0);
}

// ---- one application per correction --------------------------------------

namespace
{

// Port 0 forward and port 1 sideways, one revolution of raw counts each, with
// only wheel 0 changed by the knob: wheel 1 stays the reference.
void knobAppliesOnce(const std::function<void(gatr2::ProfileWheel&)>& knob, double forward) {
    Rig                    r;
    gatr2::RobotProfileDoc p = perpendicular(gatr2::kImuSourceBrainVex);
    knob(p.wheels[0]);
    ready(r, p);
    r.translate({kCpr, kCpr, 0}, 10);
    r.still(2);
    EXPECT_NEAR(r.pose().x_m - 1.0, forward, 1e-7);
    EXPECT_NEAR(r.pose().y_m - 0.5, kRevolution, 1e-7);
}

} // namespace

TEST(BrainProfile, CountsPerRevolutionApplyOnce) {
    knobAppliesOnce([](gatr2::ProfileWheel& w) { w.counts_per_rev = 2 * kCpr; },
                    kRevolution / 2.0);
}

TEST(BrainProfile, GearingAppliesOnce) {
    knobAppliesOnce([](gatr2::ProfileWheel& w) { w.gear_micro = 2500000; },   // 2.5:1
                    kRevolution / 2.5);
}

TEST(BrainProfile, PolarityAppliesOnce) {
    knobAppliesOnce([](gatr2::ProfileWheel& w) { w.flags = gatr2::kWheelReversed; },
                    -kRevolution);
}

TEST(BrainProfile, TravelScaleAppliesOnce) {
    knobAppliesOnce([](gatr2::ProfileWheel& w) { w.travel_scale_ppm = 1050000; },
                    1.05 * kRevolution);
}

TEST(BrainProfile, AllCorrectionsTogetherApplyOnceEach) {
    knobAppliesOnce(
        [](gatr2::ProfileWheel& w) {
            w.counts_per_rev   = 8192;
            w.gear_micro       = 1500000;
            w.flags            = gatr2::kWheelReversed;
            w.travel_scale_ppm = 950000;
            w.radius_um        = 30000;
        },
        -(kCpr * 2.0 * kPi / (8192 * 1.5)) * 0.030 * 0.95);
}

// ---- idempotence, continuity -----------------------------------------------

TEST(BrainProfile, SameProfileReappliesWithoutResetting) {
    Rig                          r;
    const gatr2::RobotProfileDoc p = perpendicular(gatr2::kImuSourceBrainVex);
    ready(r, p);
    r.translate({kCpr, 0, 0}, 10);
    const gatr2::BrainState before = r.still(2);
    const Pose2D            pose   = r.pose();

    const std::vector<uint8_t> doc = bytesOf(p);
    const gatr2::BrainReply    again = r.apply(doc);
    EXPECT_EQ(again.result, gatr2::kResultOk);   // at once, no boundary
    EXPECT_EQ(again.profile_state, gatr2::kProfileApplied);

    // a restarted Brain: new session, stages and applies the same profile
    const uint32_t old_session = r.session;
    r.rid                      = 1;
    EXPECT_NE(r.hello(0x5EED0002), old_session);
    EXPECT_EQ(r.stageAndApply(doc).result, gatr2::kResultOk);
    const gatr2::BrainState after = r.still(3);
    EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    EXPECT_EQ(after.anchor_revision, before.anchor_revision);
    EXPECT_NE(after.robot_flags & gatr2::kRobotLocalized, 0);   // the placement holds
    EXPECT_NEAR(r.pose().x_m, pose.x_m, 1e-9);
    EXPECT_NEAR(r.pose().y_m, pose.y_m, 1e-9);
    EXPECT_EQ(r.system->profileBinding()->generation, 1u);   // applied once
}

TEST(BrainProfile, NewProfileResetsAndWithdrawsPlacement) {
    Rig                          r;
    const gatr2::RobotProfileDoc a = perpendicular(gatr2::kImuSourceBrainVex);
    ready(r, a);
    r.translate({kCpr, 0, 0}, 10);
    const gatr2::BrainState placed = r.still(2);
    EXPECT_NE(placed.robot_flags & gatr2::kRobotLocalized, 0);
    gatr2::RobotProfileDoc b = a;
    b.wheels[0].radius_um    = 25000;   // a different robot description
    const std::vector<uint8_t> doc = bytesOf(b);
    EXPECT_EQ(r.stageAndApply(doc).result, gatr2::kResultPending);
    const gatr2::BrainState swapped = r.still(1);
    EXPECT_EQ(swapped.profile_state, gatr2::kProfileApplied);
    EXPECT_EQ(swapped.profile_id, idOf(doc));
    EXPECT_GT(swapped.odometry_epoch, placed.odometry_epoch);
    EXPECT_EQ(swapped.robot_flags & gatr2::kRobotLocalized, 0);
    EXPECT_EQ(r.system->robotFeed()->historySize(), 0u);

    // the old SET_POSE never re-applies to the new odometry origin
    const gatr2::BrainState moving = r.translate({kCpr, 0, 0}, 15);
    EXPECT_EQ(moving.robot_flags & gatr2::kRobotLocalized, 0);
    EXPECT_NE(moving.robot_flags & gatr2::kRobotPoseValid, 0);

    // a new placement is required and works
    EXPECT_EQ(r.place(2000, 1000, 9000).result, gatr2::kResultOk);
    const gatr2::BrainState replaced = r.still(2);
    EXPECT_NE(replaced.robot_flags & gatr2::kRobotLocalized, 0);
    EXPECT_NEAR(r.pose().x_m, 2.0, 1e-9);
    EXPECT_NEAR(headingDeg(r), 90.0, 1e-9);
}

TEST(BrainProfile, StagingInterruptedByANewSessionResumes) {
    Rig r;
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    const std::vector<uint8_t> doc = bytesOf(perpendicular(gatr2::kImuSourceBrainVex));
    r.stage(doc, 0, 40, 40);   // interrupted after one chunk
    r.rid = 1;
    r.hello(0x5EED0003);        // the Brain reconnects under a new session
    r.stage(doc, 40, static_cast<uint16_t>(doc.size()), 40);   // resumes at received
    EXPECT_EQ(r.apply(doc).result, gatr2::kResultPending);
    r.cycle();
    EXPECT_EQ(r.apply(doc).result, gatr2::kResultOk);
}

// ---- refusals --------------------------------------------------------------

namespace
{

struct Refusal {
    const char*            what;
    gatr2::RobotProfileDoc profile;
    uint8_t                reason;
    uint8_t                detail;
};

void expectRefused(Rig& r, const Refusal& refusal) {
    SCOPED_TRACE(refusal.what);
    const std::vector<uint8_t> doc   = bytesOf(refusal.profile);
    const gatr2::BrainReply    reply = r.stageAndApply(doc);
    EXPECT_EQ(reply.result, gatr2::kResultProfileRejected);
    EXPECT_EQ(reply.profile_reason, refusal.reason);
    EXPECT_EQ(reply.profile_detail, refusal.detail);
    const gatr2::BrainState s = r.still(1);
    EXPECT_EQ(s.profile_state, gatr2::kProfileRejected);
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

    gatr2::RobotProfileDoc two_none = perpendicular(gatr2::kImuSourceNone);
    two_none.vex_smart_port         = 0;
    gatr2::RobotProfileDoc three_vex = threeWheel(gatr2::kImuSourceBrainVex);
    gatr2::RobotProfileDoc collinear = perpendicular(gatr2::kImuSourcePico);
    collinear.wheels[1].angle_mdeg   = 180000;
    gatr2::RobotProfileDoc sideways_forward = twoForward(gatr2::kImuSourcePico);
    sideways_forward.wheels[1].angle_mdeg   = 90000;
    gatr2::RobotProfileDoc same_port        = perpendicular(gatr2::kImuSourcePico);
    same_port.wheels[1].encoder_port        = 0;
    gatr2::RobotProfileDoc unwired          = perpendicular(gatr2::kImuSourcePico);
    unwired.wheels[1].encoder_port          = 3;
    gatr2::RobotProfileDoc imu_port         = perpendicular(gatr2::kImuSourcePico);
    imu_port.imu_port                       = 1;
    gatr2::RobotProfileDoc camera           = perpendicular(gatr2::kImuSourcePico);
    camera.camera_count                     = 1;

    for (const Refusal& refusal : std::vector<Refusal>{
             {"two wheels without an IMU", two_none, gatr2::kProfileReasonImuCombination, 0},
             {"three wheels with the Brain clock IMU", three_vex,
              gatr2::kProfileReasonImuCombination, 0},
             {"collinear wheels", collinear, gatr2::kProfileReasonObservability, 0},
             {"a sideways wheel in the forward topology", sideways_forward,
              gatr2::kProfileReasonObservability, 0},
             {"one port twice", same_port, gatr2::kProfileReasonEncoderPort, 1},
             {"a port this Pi does not have", unwired, gatr2::kProfileReasonEncoderPort, 1},
             {"a Pico IMU port that does not exist", imu_port, gatr2::kProfileReasonImuPort, 0},
             {"a camera slot this Pi does not have", camera, gatr2::kProfileReasonCamera, 0},
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
            port                       = port->NextSiblingElement("Port")->NextSiblingElement("Port");
            encoders->DeleteChild(port);
        });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"port 2 unwired", threeWheel(gatr2::kImuSourceNone),
                          gatr2::kProfileReasonEncoderPort, 2});
    }
    {
        Rig r([](tinyxml2::XMLElement* root) { removeChild(root, kProfilePath, "Imu"); });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"no Pico IMU", perpendicular(gatr2::kImuSourcePico),
                          gatr2::kProfileReasonImuSource, 0});
    }
    {
        Rig r([](tinyxml2::XMLElement* root) {
            removeChild(root, kProfilePath, "BrainImu");
            removeChild(root, "Pipeline/CommandCollection", "BenchImu");
        });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"no Brain IMU mailbox", perpendicular(gatr2::kImuSourceBrainVex),
                          gatr2::kProfileReasonImuSource, 0});
    }
    {
        Rig r([](tinyxml2::XMLElement* root) { removeChild(root, kProfilePath, "Fusion"); });
        ASSERT_TRUE(r.ok()) << r.build_error;
        r.hello();
        expectRefused(r, {"no fusion tuning", threeWheel(gatr2::kImuSourcePico),
                          gatr2::kProfileReasonImuCombination, 0});
    }
}

TEST(BrainProfile, ARefusedProfileLeavesTheRunningOneRunning) {
    Rig                          r;
    const gatr2::RobotProfileDoc a = perpendicular(gatr2::kImuSourceBrainVex);
    ready(r, a);
    const gatr2::BrainState before = r.still(1);
    gatr2::RobotProfileDoc  bad    = a;
    bad.camera_count               = 1;
    const gatr2::BrainReply reply  = r.stageAndApply(bytesOf(bad));
    EXPECT_EQ(reply.profile_reason, gatr2::kProfileReasonCamera);
    const gatr2::BrainState after = r.translate({kCpr, 0, 0}, 10);
    EXPECT_EQ(after.profile_state, gatr2::kProfileRejected);
    EXPECT_EQ(after.odometry_epoch, before.odometry_epoch);
    EXPECT_NE(after.robot_flags & gatr2::kRobotLocalized, 0);
    EXPECT_EQ(r.system->profileBinding()->id, idOf(bytesOf(a)));
    r.still(2);
    EXPECT_NEAR(r.pose().x_m, 1.0 + kRevolution, 1e-6);
}

// ---- CONTROL ---------------------------------------------------------------

TEST(BrainProfile, RecalibrateNeedsStillnessAndHoldsThePose) {
    Rig                          r;
    const gatr2::RobotProfileDoc p = perpendicular(gatr2::kImuSourcePico);
    ready(r, p);
    EXPECT_EQ(r.still(1).calibration, gatr2::kCalibrationDone);

    // moving now: nothing starts
    for (int i = 0; i < 4; ++i) {
        r.counts[0] += 200;
        r.cycle();
    }
    r.counts[0] += 200;
    const gatr2::BrainReply moving = r.control(gatr2::kControlRecalibrate);
    EXPECT_EQ(moving.result, gatr2::kResultNotStationary);
    EXPECT_EQ(r.still(1).calibration, gatr2::kCalibrationDone);

    // turning in place counts as moving too
    r.still(20);
    r.rate_mdps               = 5000.0;   // 5 deg/s
    r.still(20);
    EXPECT_EQ(r.control(gatr2::kControlRecalibrate).result, gatr2::kResultNotStationary);
    r.rate_mdps = 0.0;

    // still for longer than the window: it restarts
    r.still(20);
    const Pose2D            held = r.pose();
    const gatr2::BrainReply ok   = r.control(gatr2::kControlRecalibrate);
    EXPECT_EQ(ok.result, gatr2::kResultOk);
    EXPECT_EQ(ok.action, gatr2::kControlRecalibrate);
    EXPECT_EQ(ok.calibration, gatr2::kCalibrationRunning);
    const uint16_t control_rid = static_cast<uint16_t>(r.rid - 1);

    const gatr2::BrainState done = r.still(30);
    EXPECT_EQ(done.calibration, gatr2::kCalibrationDone);
    EXPECT_NE(done.health & gatr2::kHealthBiasCalibrated, 0);
    EXPECT_NEAR(r.pose().x_m, held.x_m, 1e-9);
    EXPECT_NEAR(r.pose().y_m, held.y_m, 1e-9);

    // a lost acknowledgement: the same request id reports, never reruns
    gatr2::BrainRequest dup = r.request(gatr2::kOpControl);
    r.rid--;
    dup.request_id = control_rid;
    dup.action     = gatr2::kControlRecalibrate;
    r.picoFrame();
    const gatr2::BrainReply again = r.one(dup);
    EXPECT_EQ(again.result, gatr2::kResultOk);
    EXPECT_EQ(again.calibration, gatr2::kCalibrationDone);   // not restarted

    // the calibrated bias is the one it measured
    const Pose2D before = r.pose();
    r.turn(25000.0, 20, countsPerRad(p));
    r.still(3);
    EXPECT_NEAR(wrapAngle(r.pose().heading_rad - before.heading_rad), degToRad(10.0), 2e-4);
}

TEST(BrainProfile, ReinitializeStartsANewUnplacedOdometry) {
    Rig                          r;
    const gatr2::RobotProfileDoc p = perpendicular(gatr2::kImuSourcePico);
    ready(r, p);
    r.still(20);
    const gatr2::BrainState before = r.still(1);
    const gatr2::BrainReply reply  = r.control(gatr2::kControlReinitialize);
    EXPECT_EQ(reply.result, gatr2::kResultOk);
    EXPECT_EQ(reply.calibration, gatr2::kCalibrationRunning);
    const gatr2::BrainState after = r.still(30);
    EXPECT_GT(after.odometry_epoch, before.odometry_epoch);
    EXPECT_EQ(after.robot_flags & gatr2::kRobotLocalized, 0);   // the old SET_POSE is withdrawn
    EXPECT_EQ(after.calibration, gatr2::kCalibrationDone);
    EXPECT_EQ(r.place(500, 500, 0).result, gatr2::kResultOk);
    EXPECT_NE(r.still(1).robot_flags & gatr2::kRobotLocalized, 0);
}

TEST(BrainProfile, CalibrationSettingsFromTheProfileReachTheModel) {
    Rig                    r;
    gatr2::RobotProfileDoc p = perpendicular(gatr2::kImuSourcePico);
    p.calibration_window_ms  = 1000;   // longer than the samples the Pi needs
    p.still_travel_um        = 20;     // under one encoder count of travel
    ASSERT_TRUE(r.ok()) << r.build_error;
    r.hello();
    r.still(3);
    ASSERT_TRUE(r.applyProfile(p));
    EXPECT_EQ(r.still(30).calibration, gatr2::kCalibrationRunning);   // 600 ms of samples
    EXPECT_EQ(r.still(25).calibration, gatr2::kCalibrationDone);

    // one count of travel restarts the window under this profile's limit
    ASSERT_EQ(r.control(gatr2::kControlRecalibrate).result, gatr2::kResultOk);
    r.still(30);
    r.counts[0] += 1;
    EXPECT_EQ(r.still(30).calibration, gatr2::kCalibrationRunning);
    EXPECT_EQ(r.still(30).calibration, gatr2::kCalibrationDone);
}

TEST(BrainProfile, VexRecalibrateHasNothingToCalibrateOnThePi) {
    Rig r;
    ready(r, perpendicular(gatr2::kImuSourceBrainVex));
    r.still(20);
    const gatr2::BrainReply reply = r.control(gatr2::kControlRecalibrate);
    EXPECT_EQ(reply.result, gatr2::kResultOk);
    EXPECT_EQ(reply.calibration, gatr2::kCalibrationNone);
    EXPECT_NE(r.still(1).robot_flags & gatr2::kRobotLocalized, 0);   // nothing reset
    // the Pico commands wait for the Pico link stage
    EXPECT_EQ(r.control(gatr2::kControlReinitImu).result, gatr2::kResultNotReady);
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
}

TEST(BrainProfile, InspectionFollowsTheBoundary) {
    Rig r;
    ready(r, perpendicular(gatr2::kImuSourceBrainVex));
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

TEST(BrainProfile, ResetWaitsForAProfileAgain) {
    Rig r;
    ready(r, perpendicular(gatr2::kImuSourceBrainVex));
    const gatr2::BrainState before = r.still(1);
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
    const uint32_t             pi_instance = r.one(r.request(gatr2::kOpGetState)).pi_instance;
    const std::vector<uint8_t> doc         = bytesOf(perpendicular(gatr2::kImuSourceBrainVex));
    r.stage(doc, 0, static_cast<uint16_t>(doc.size()), gatr2::kProfileChunkMax);

    // the APPLY waits in the link; the estimation worker prepares it
    gatr2::BrainRequest apply = r.request(gatr2::kOpProfileApply);
    apply.profile_id          = idOf(doc);
    apply.total_len           = static_cast<uint16_t>(doc.size());
    std::array<uint8_t, gatr2::kMaxFrameLen> bytes{};
    const uint16_t len = gatr2::encodeBrainRequest(apply, bytes.data(), bytes.size());
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
    EXPECT_EQ(r.system->command().profile.state, gatr2::kProfileApplied);
    EXPECT_EQ(r.system->command().session, session);

    // back inline: the same session and pi_instance answer
    r.pi_raw->setClock([&r] { return r.now * 1000; });
    r.pi_raw->output().takeAll();
    r.now += 1000;
    r.step();
    const gatr2::BrainReply state = r.one(r.request(gatr2::kOpGetState));
    EXPECT_EQ(state.result, gatr2::kResultOk);
    EXPECT_EQ(state.pi_instance, pi_instance);
    EXPECT_EQ(state.state.profile_state, gatr2::kProfileApplied);
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
