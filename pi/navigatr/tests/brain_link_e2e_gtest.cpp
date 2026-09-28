// brain_link_e2e_gtest.cpp
// The real Brain stack against the real Pi runtime over the USB console.
// communiGATR (Client, LinkDriver, the NG1 line codec) with the bench robot
// profile, actuGATR Motion and a tank Drive, and the investiGATR planner talk
// to a navigatr System built from brain_profile_usb.xml. A drivetrain sim is
// the truth: it drives the Pico encoder counts of the two perpendicular
// tracking wheels and the Brain VEX IMU heading. Covers profile upload and
// application, placement, direct and avoiding moves on the transferred field
// map, wheel readings, a travel scale change, Brain restart, Pi restart and
// a pulled cable. Every geometry value is a test value.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "actugatr/drive.h"
#include "actugatr/follower.h"
#include "actugatr/kinematics.h"
#include "actugatr/motion.h"
#include "common/frame_codec.h"
#include "communigatr/client.h"
#include "communigatr/link_driver.h"
#include "communigatr/readiness.h"
#include "communigatr/robot_profile.h"
#include "communigatr/usb_line.h"
#include "config/composition.h"
#include "impl/resources/pros_usb_link.h"
#include "impl/resources/serial_links.h"
#include "investigatr/planner.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "sim/drive_sim.h"
#include "sim/sim_motor_output.h"
#include "tinyxml2/tinyxml2.h"

namespace
{

namespace ag = actugatr;
namespace cg = communigatr;
namespace ig = investigatr;
namespace nv = navigatr;

constexpr int64_t kTickUs     = 1000;  // sim step
constexpr int64_t kPiPeriodUs = 10000; // Loop rate_hz 100
constexpr int64_t kPollUs     = 2000;  // Brain link task
constexpr int64_t kControlUs  = 10000; // Motion and Drive
constexpr int64_t kLineUs     = 2000;  // USB line delivery, each way
constexpr int64_t kPiDownUs   = 300000;

constexpr double kRadius = 0.024;
constexpr double kCpr    = 4000;
constexpr double kPi     = ig::kPi;

constexpr uint32_t kNonceA = 0xA11CE001;
constexpr uint32_t kNonceB = 0xB0B0B002;

const ig::Pose      kStart{0.6, 0.6, 0.0};
const ig::Footprint kFootprint{0.23, 0.23, 0.23, 0.23};

// Forward wheel on port 0, 0.15 m left of the origin; sideways wheel on
// port 1, 0.15 m ahead, measuring to the left.
struct Mount {
    uint8_t port;
    double  x;
    double  y;
    double  angle;
};
const Mount kWheels[2] = {{0, 0.0, 0.15, 0.0}, {1, 0.15, 0.0, kPi / 2}};

cg::RobotProfile benchProfile(double forward_scale = 1.0) {
    cg::RobotProfile p;
    p.topology       = cg::LocalizationTopology::kTwoWheelImu;
    p.imu_source     = cg::ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = kFootprint;
    for (const Mount& m : kWheels) {
        cg::TrackingWheel w;
        w.encoder_port   = m.port;
        w.radius         = kRadius;
        w.counts_per_rev = static_cast<uint32_t>(kCpr);
        w.x              = m.x;
        w.y              = m.y;
        w.angle          = m.angle;
        p.wheels.push_back(w);
    }
    p.wheels[0].travel_scale = forward_scale;
    return p;
}

ag::WheelDrive driveWheels() {
    ag::WheelDrive w;
    w.wheel_diameter  = 0.1016;
    w.gear_ratio      = 0.6;
    w.usable_fraction = 0.9;
    return w;
}

ag::MotionConfig motionConfig() {
    ag::MotionConfig c;
    c.model.footprint        = kFootprint;
    c.model.clearance        = 0.06;
    c.model.limits.max_speed = 0.6;
    c.model.limits.max_accel = 1.5;
    c.model.limits.max_omega = 3.0;
    c.model.limits.max_alpha = 8.0;
    c.default_timeout        = 25.0;
    return c;
}

gatr2::BrainRequest requestOf(const std::vector<uint8_t>& frame) {
    gatr2::BrainRequest r;
    EXPECT_TRUE(gatr2::decodeBrainRequest(frame.data(), static_cast<uint16_t>(frame.size()), r));
    return r;
}

// The V5 USB console: every Brain frame becomes an NG1 line (communiGATR
// codec) that the Pi's pros_usb_link reads, and every Pi line is decoded
// back to frame bytes for the client, kLineUs each way. Unplugged, lines in
// flight and new ones are lost; the Brain port still accepts writes.
class UsbWire : public cg::BytePort
{
public:
    explicit UsbWire(const int64_t& now_us) : now_us_(now_us) {}

    int read(uint8_t* buf, int max) override {
        while (!to_brain_.empty() && to_brain_.front().first <= now_us_) {
            if (decoder_.push(static_cast<char>(to_brain_.front().second))) {
                frames_.insert(frames_.end(), decoder_.bytes(), decoder_.bytes() + decoder_.length());
            }
            to_brain_.pop_front();
        }
        int n = 0;
        while (n < max && !frames_.empty()) {
            buf[n++] = frames_.front();
            frames_.pop_front();
        }
        return n;
    }

    bool write(const uint8_t* data, int len) override {
        requests.emplace_back(data, data + len);
        if (!plugged_) {
            return true;
        }
        char              line[cg::kUsbLineMax];
        const std::size_t n = cg::encodeUsbLine(data, static_cast<std::size_t>(len), line, sizeof(line));
        EXPECT_GT(n, 0u);
        for (std::size_t i = 0; i < n; ++i) {
            to_pi_.emplace_back(now_us_ + kLineUs, static_cast<uint8_t>(line[i]));
        }
        return true;
    }

    // Line characters due at the Pi; lost while no Pi runs.
    void deliver(nv::MemoryLink* pi) {
        std::vector<uint8_t> chars;
        while (!to_pi_.empty() && to_pi_.front().first <= now_us_) {
            chars.push_back(to_pi_.front().second);
            to_pi_.pop_front();
        }
        if (pi != nullptr && !chars.empty()) {
            pi->input().feed(chars);
        }
    }

    // What the Pi wrote this cycle, on its way to the Brain.
    void collect(nv::MemoryLink& pi) {
        const std::vector<uint8_t> chars = pi.output().takeAll();
        if (!plugged_) {
            return;
        }
        for (uint8_t c : chars) {
            to_brain_.emplace_back(now_us_ + kLineUs, c);
        }
    }

    void setPlugged(bool plugged) {
        plugged_ = plugged;
        if (!plugged) {
            to_pi_.clear();
            to_brain_.clear();
        }
    }

    void dropToPi() { to_pi_.clear(); }

    // Requests with op from index `from` on.
    int count(uint8_t op, std::size_t from = 0) const {
        int n = 0;
        for (std::size_t i = from; i < requests.size(); ++i) {
            n += requestOf(requests[i]).op == op ? 1 : 0;
        }
        return n;
    }

    std::vector<std::vector<uint8_t>> requests; // every Brain write, in order

private:
    using Timed = std::deque<std::pair<int64_t, uint8_t>>;

    const int64_t&      now_us_;
    bool                plugged_ = true;
    Timed               to_pi_;
    Timed               to_brain_;
    cg::UsbLineDecoder  decoder_;
    std::deque<uint8_t> frames_;
};

// One Brain boot: client, link driver, planner, Motion and the Drive on the
// robot's motors. The link task polls every kPollUs, the control loop runs
// every kControlUs.
struct Brain {
    Brain(cg::BytePort& port, uint32_t nonce, const cg::ClientConfig& config,
          const ag::Kinematics& kinematics, ag::MotorOutput& motors)
        : client(port, [nonce] { return nonce; }, config), driver(client),
          motion(driver, planner, follower, motionConfig()),
          drive(kinematics, driveWheels(), motors) {
        motion.setPathSink(&driver);
    }

    void control(cg::Seconds now) { drive.apply(motion.update(now), now, now); }

    cg::Readiness readiness(cg::Seconds now) const {
        return cg::readinessOf(client, now, false).state;
    }

    cg::Client                client;
    cg::LinkDriver            driver;
    ig::GeometricPlanner      planner;
    ag::DifferentialFollower  follower;
    ag::Motion                motion;
    ag::Drive                 drive;
};

enum class Restart { kProcess, kReset };

// Pi, USB wire, drivetrain truth and at most one Brain on one microsecond
// clock. The Pi is brain_profile_usb.xml with its Pico UART and Brain USB
// device replaced by memory links.
struct Rig {
    int64_t              now_us = 0;
    UsbWire              wire{now_us};
    nv::FunctionRegistry functions;
    std::string          xml;
    std::string          error;

    std::shared_ptr<nv::MemoryLink> pi_raw; // under the Pi's pros_usb_link
    std::shared_ptr<nv::MemoryLink> pico;
    std::unique_ptr<nv::System>     system; // null while the Pi process is down
    int64_t                         up_at_us = 0;

    ag::TankKinematics kinematics{0.30};
    ag::SimMotorOutput motors{2};
    ag::DriveSim       body{kinematics, driveWheels(), motors};
    double             travel[2] = {0, 0}; // true wheel travel per port
    double             rotation  = 0;      // true heading, unwrapped
    uint8_t            pico_seq  = 0;

    std::unique_ptr<Brain> brain; // null while powered off
    cg::RobotProfile       profile = benchProfile();

    Rig() {
        nv::registerAll(functions);
        EXPECT_TRUE(functions.add<nv::ResourceMakeFunction>(
            nv::FunctionKey{"test_usb"},
            [this](const nv::ConfigNode&, nv::ResourceInitializationContext&, std::string&) {
                return nv::ResourceInstance::asContract<nv::SerialLink>(
                    std::make_shared<nv::ProsUsbLink>(pi_raw));
            }));
        nv::ResolvedConfiguration config;
        if (!nv::resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) +
                                          "/override/brain_profile_usb.xml",
                                      config, error)) {
            ADD_FAILURE() << error;
            return;
        }
        tinyxml2::XMLDocument doc;
        EXPECT_EQ(doc.Parse(config.xml.c_str()), tinyxml2::XML_SUCCESS);
        tinyxml2::XMLElement* resources = doc.RootElement()->FirstChildElement("Resources");
        for (auto* r = resources->FirstChildElement("Resource"); r != nullptr;
             r = r->NextSiblingElement("Resource")) {
            const std::string id = r->Attribute("id") != nullptr ? r->Attribute("id") : "";
            if (id == "brain_usb") {
                r->SetAttribute("type", "test_usb");
            } else if (id == "pico_uart") {
                r->SetAttribute("type", "memory_link");
            }
        }
        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        xml = printer.CStr();
        body.setPose(kStart);
        startPi();
    }

    bool ok() const { return system != nullptr && pico != nullptr; }

    double now() const { return static_cast<double>(now_us) * 1e-6; }

    void startPi() {
        pi_raw = std::make_shared<nv::MemoryLink>();
        pi_raw->setClock([this] { return now_us; });
        system = nv::System::buildFromString(xml.c_str(), functions, error);
        ASSERT_NE(system, nullptr) << error;
        std::string err;
        pico = std::dynamic_pointer_cast<nv::MemoryLink>(
            system->resources().require<nv::SerialLink>(nv::ResourceId{"pico_uart"}, err));
        ASSERT_NE(pico, nullptr) << err;
    }

    // kProcess: a new process after kPiDownUs; bytes on their way are lost.
    // kReset: System::reset() in place.
    void restartPi(Restart kind) {
        if (kind == Restart::kReset) {
            system->reset();
            return;
        }
        system.reset();
        pico.reset();
        up_at_us = now_us + kPiDownUs;
        wire.dropToPi();
    }

    cg::ClientConfig clientConfig() {
        cg::ClientConfig c;
        c.profile   = cg::makeProfileDocument(profile);
        c.bench_imu = [this] {
            cg::BenchImuSample s;
            s.valid         = true;
            s.stamp_ms      = static_cast<uint32_t>(now_us / 1000);
            s.rotation_mdeg = static_cast<int32_t>(std::llround(rotation * 180000.0 / kPi));
            return s;
        };
        return c;
    }

    Brain& boot(uint32_t nonce) {
        brain.reset(new Brain(wire, nonce, clientConfig(), kinematics, motors));
        return *brain;
    }

    void powerOff() {
        brain.reset();
        motors.stop(ag::StopMode::kBrake);
    }

    void tick() {
        now_us += kTickUs;
        const ig::Pose before = body.pose();
        body.step(static_cast<double>(kTickUs) * 1e-6);
        accumulate(before, body.pose());

        if (system == nullptr && now_us >= up_at_us) {
            startPi();
        }
        wire.deliver(system != nullptr ? pi_raw.get() : nullptr);
        if (system != nullptr && now_us % kPiPeriodUs == 0) {
            pico->input().feed(sensorFrame());
            system->step(nv::hostTime(now_us / 1000));
            wire.collect(*pi_raw);
        }
        if (brain != nullptr) {
            if (now_us % kPollUs == 0) {
                brain->client.poll(now());
            }
            if (now_us % kControlUs == 0) {
                brain->control(now());
            }
        }
    }

    bool runUntil(const std::function<bool()>& done, double limit_s) {
        const int64_t end = now_us + static_cast<int64_t>(limit_s * 1e6);
        while (now_us < end) {
            tick();
            if (done()) {
                return true;
            }
        }
        return false;
    }

    void run(double s) {
        runUntil([] { return false; }, s);
    }

    // Wheel travel for one body step: body translation along the wheel plus
    // its lever times the rotation, at the mid heading.
    void accumulate(const ig::Pose& a, const ig::Pose& b) {
        const double dth = ig::wrapAngle(b.heading - a.heading);
        const double mid = a.heading + dth / 2.0;
        const double ex  = b.x - a.x;
        const double ey  = b.y - a.y;
        const double dx  = std::cos(mid) * ex + std::sin(mid) * ey;
        const double dy  = -std::sin(mid) * ex + std::cos(mid) * ey;
        for (int i = 0; i < 2; ++i) {
            const Mount& m = kWheels[i];
            travel[i] += dx * std::cos(m.angle) + dy * std::sin(m.angle) +
                         dth * (m.x * std::sin(m.angle) - m.y * std::cos(m.angle));
        }
        rotation += dth;
    }

    std::vector<uint8_t> sensorFrame() {
        gatr2::SensorSample s{};
        s.seq      = pico_seq++;
        s.stamp_ms = static_cast<uint32_t>(5000 + now_us / 1000);
        s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorEnc2;
        for (int i = 0; i < 2; ++i) {
            s.enc[i] = static_cast<int32_t>(std::llround(travel[i] / (2.0 * kPi * kRadius) * kCpr));
        }
        std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
        buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
        return buf;
    }

    nv::Pose2D piPose() const { return system->robot().fieldPose(); }

    // Boots a Brain and waits for the profile, a placement at the truth and
    // a ready pose.
    Brain& ready(uint32_t nonce = kNonceA) {
        Brain& b = boot(nonce);
        EXPECT_TRUE(runUntil([&] { return b.client.profileApplied(); }, 3.0));
        EXPECT_TRUE(runUntil(
            [&] { return b.readiness(now()) == cg::Readiness::kNeedsPlacement; }, 3.0))
            << cg::toString(b.readiness(now()));
        const cg::PlacementTicket ticket = b.driver.place(body.pose());
        EXPECT_NE(ticket, 0u);
        EXPECT_TRUE(runUntil(
            [&] { return b.client.placementResult(ticket) == cg::PlacementResult::kApplied; },
            2.0));
        EXPECT_TRUE(
            runUntil([&] { return b.readiness(now()) == cg::Readiness::kReady; }, 1.0));
        return b;
    }

    ag::MotionState finish(Brain& b, double limit_s) {
        runUntil([&] { return ag::isTerminal(b.motion.status().state); }, limit_s);
        return b.motion.status().state;
    }
};

void expectPiNearTruth(const Rig& rig, double tolerance_m, double tolerance_rad) {
    const nv::Pose2D pi = rig.piPose();
    EXPECT_NEAR(pi.x_m, rig.body.pose().x, tolerance_m);
    EXPECT_NEAR(pi.y_m, rig.body.pose().y, tolerance_m);
    EXPECT_NEAR(ig::wrapAngle(pi.heading_rad - rig.body.pose().heading), 0.0, tolerance_rad);
}

} // namespace

TEST(BrainLinkEndToEnd, ProfilePlacementAndDirectMovesOverUsb) {
    Rig rig;
    ASSERT_TRUE(rig.ok()) << rig.error;
    Brain& b = rig.ready();
    EXPECT_EQ(rig.wire.count(gatr2::kOpSetPose), 1);
    expectPiNearTruth(rig, 1e-3, 1e-3);

    b.motion.goToDirect({1.3, 0.6, 0.0});
    ASSERT_EQ(rig.finish(b, 15.0), ag::MotionState::kCompleted)
        << ag::toString(b.motion.status().reason);
    b.motion.goToDirect({1.3, 1.1, kPi / 2});
    ASSERT_EQ(rig.finish(b, 15.0), ag::MotionState::kCompleted)
        << ag::toString(b.motion.status().reason);

    EXPECT_NEAR(rig.body.pose().x, 1.3, 0.04);
    EXPECT_NEAR(rig.body.pose().y, 1.1, 0.04);
    EXPECT_NEAR(ig::wrapAngle(rig.body.pose().heading - kPi / 2), 0.0, 0.06);
    expectPiNearTruth(rig, 0.01, 0.01);
    EXPECT_EQ(b.client.stats().timeouts, 0u);
    EXPECT_EQ(b.client.stats().session_losses, 0u);
}

TEST(BrainLinkEndToEnd, AvoidingMovePlansOnTheTransferredFieldMap) {
    Rig rig;
    ASSERT_TRUE(rig.ok()) << rig.error;
    Brain&    b = rig.ready();
    ig::Field field;
    ASSERT_TRUE(rig.runUntil([&] { return b.driver.field(field) && !field.objects.empty(); }, 5.0));
    EXPECT_EQ(field.map.id, b.client.field().map_id);
    EXPECT_EQ(field.map.id, b.client.state().state.map_id);

    b.motion.goToAvoiding({1.4, 1.2, kPi / 2});
    double worst      = 1e9;
    int    collisions = 0;
    rig.runUntil(
        [&] {
            const ig::Clearance c = ig::footprintClearance(rig.body.pose(), kFootprint, 0.0, field);
            worst                 = std::min(worst, c.distance);
            collisions += c.clear ? 0 : 1;
            return ag::isTerminal(b.motion.status().state);
        },
        25.0);
    ASSERT_EQ(b.motion.status().state, ag::MotionState::kCompleted)
        << ag::toString(b.motion.status().reason);
    EXPECT_EQ(b.motion.status().mode, ig::PlanMode::kAvoiding);
    EXPECT_EQ(collisions, 0) << "closest " << worst;
    EXPECT_NEAR(rig.body.pose().x, 1.4, 0.04);
    EXPECT_NEAR(rig.body.pose().y, 1.2, 0.04);
    expectPiNearTruth(rig, 0.01, 0.01);
}

TEST(BrainLinkEndToEnd, WheelReadingsReportRawTravel) {
    Rig rig;
    ASSERT_TRUE(rig.ok()) << rig.error;
    rig.profile.wheels[0].travel_scale = 1.02; // readings never include it
    Brain& b = rig.ready();

    auto read = [&](double out[2]) {
        const uint32_t before = b.client.wheelReadings().sequence;
        ASSERT_TRUE(b.client.requestWheels());
        ASSERT_TRUE(rig.runUntil([&] { return b.client.wheelReadings().sequence != before; }, 1.0));
        const cg::WheelReadings& r = b.client.wheelReadings();
        ASSERT_EQ(r.result, gatr2::kResultOk);
        ASSERT_EQ(r.count, 2);
        for (int i = 0; i < 2; ++i) {
            EXPECT_EQ(r.wheels[i].port, kWheels[i].port);
            out[i] = r.wheels[i].travel_um * 1e-6;
        }
    };
    double start[2];
    read(start);
    const double truth0[2] = {rig.travel[0], rig.travel[1]};

    b.motion.goToDirect({1.2, 0.6, 0.0});
    ASSERT_EQ(rig.finish(b, 15.0), ag::MotionState::kCompleted);
    rig.run(0.3);
    double end[2];
    read(end);
    for (int i = 0; i < 2; ++i) {
        EXPECT_NEAR(end[i] - start[i], rig.travel[i] - truth0[i], 1e-4) << "port " << i;
    }
    EXPECT_GT(end[0] - start[0], 0.3);
}

TEST(BrainLinkEndToEnd, TravelScaleChangeIsANewProfileAndNeedsPlacement) {
    Rig rig;
    ASSERT_TRUE(rig.ok()) << rig.error;
    Brain&         b          = rig.ready();
    const uint32_t old_id     = b.client.profile().id;
    const uint32_t old_epoch  = b.client.state().state.odometry_epoch;
    const size_t   mark       = rig.wire.requests.size();

    ASSERT_TRUE(b.driver.setProfile(benchProfile(1.02)));
    ASSERT_TRUE(rig.runUntil(
        [&] { return b.client.profileApplied() && b.client.profile().id != old_id; }, 3.0));
    ASSERT_TRUE(rig.runUntil(
        [&] { return b.readiness(rig.now()) == cg::Readiness::kNeedsPlacement; }, 1.0));
    EXPECT_NE(b.client.state().state.odometry_epoch, old_epoch);
    rig.run(1.0);
    EXPECT_EQ(b.readiness(rig.now()), cg::Readiness::kNeedsPlacement);
    EXPECT_EQ(rig.wire.count(gatr2::kOpSetPose, mark), 0); // never placed by itself

    // Placed again, the Pi counts 2 percent more forward travel than the truth.
    const cg::PlacementTicket ticket = b.driver.place(rig.body.pose());
    ASSERT_TRUE(rig.runUntil(
        [&] { return b.client.placementResult(ticket) == cg::PlacementResult::kApplied; }, 2.0));
    const double truth_x = rig.body.pose().x;
    const double pi_x    = rig.piPose().x_m;
    b.motion.goToDirect({pi_x + 0.5, rig.piPose().y_m, 0.0});
    ASSERT_EQ(rig.finish(b, 15.0), ag::MotionState::kCompleted);
    rig.run(0.3);
    EXPECT_NEAR((rig.piPose().x_m - pi_x) / (rig.body.pose().x - truth_x), 1.02, 0.002);
}

TEST(BrainLinkEndToEnd, BrainRestartKeepsProfilePlacementAndPose) {
    Rig rig;
    ASSERT_TRUE(rig.ok()) << rig.error;
    Brain& a = rig.ready(kNonceA);
    a.motion.goToDirect({1.1, 0.6, 0.0});
    ASSERT_EQ(rig.finish(a, 15.0), ag::MotionState::kCompleted);
    const uint32_t pi_instance = a.client.piInstance();
    const uint32_t session     = a.client.session();
    const auto     state       = a.client.state().state;

    rig.powerOff();
    rig.run(0.5);
    const size_t mark = rig.wire.requests.size();
    Brain&       b    = rig.boot(kNonceB);
    ASSERT_TRUE(rig.runUntil([&] { return b.client.profileApplied(); }, 2.0));
    ASSERT_TRUE(
        rig.runUntil([&] { return b.readiness(rig.now()) == cg::Readiness::kReady; }, 1.0))
        << cg::toString(b.readiness(rig.now()));

    EXPECT_EQ(b.client.piInstance(), pi_instance);
    EXPECT_NE(b.client.session(), session);
    EXPECT_EQ(rig.wire.count(gatr2::kOpProfileWrite, mark), 0); // the Pi already runs it
    EXPECT_EQ(rig.wire.count(gatr2::kOpSetPose, mark), 0);
    EXPECT_EQ(b.client.state().state.odometry_epoch, state.odometry_epoch);
    EXPECT_EQ(b.client.state().state.anchor_revision, state.anchor_revision);
    EXPECT_EQ(b.client.state().state.profile_id, state.profile_id);
    expectPiNearTruth(rig, 0.01, 0.01);
}

TEST(BrainLinkEndToEnd, PiRestartNeedsTheProfileAndAPlacementAgain) {
    for (Restart kind : {Restart::kProcess, Restart::kReset}) {
        SCOPED_TRACE(kind == Restart::kProcess ? "process" : "reset");
        Rig rig;
        ASSERT_TRUE(rig.ok()) << rig.error;
        Brain&         b           = rig.ready();
        const uint32_t pi_instance = b.client.piInstance();
        const size_t   mark        = rig.wire.requests.size();

        rig.restartPi(kind);
        if (kind == Restart::kProcess) {
            ASSERT_TRUE(rig.runUntil([&] { return b.client.piInstance() != pi_instance; }, 3.0));
            EXPECT_GT(rig.wire.count(gatr2::kOpProfileWrite, mark), 0);
        }
        ASSERT_TRUE(rig.runUntil([&] { return b.client.profileApplied(); }, 3.0));
        ASSERT_TRUE(rig.runUntil(
            [&] { return b.readiness(rig.now()) == cg::Readiness::kNeedsPlacement; }, 3.0))
            << cg::toString(b.readiness(rig.now()));

        // Nothing places by itself, and a move waits for a placement.
        b.motion.goToDirect({1.0, 0.6, 0.0});
        EXPECT_EQ(rig.finish(b, 5.0), ag::MotionState::kFailed);
        EXPECT_EQ(b.motion.status().reason, ag::MotionReason::kPlacementRequired);
        EXPECT_EQ(rig.wire.count(gatr2::kOpSetPose, mark), 0);
        EXPECT_NEAR(rig.body.pose().x, kStart.x, 1e-6);
    }
}

TEST(BrainLinkEndToEnd, PulledCableStopsTheMoveAndTheLinkResumes) {
    Rig rig;
    ASSERT_TRUE(rig.ok()) << rig.error;
    Brain& b = rig.ready();
    b.motion.goToDirect({2.0, 0.6, 0.0});
    rig.run(0.8);
    ASSERT_EQ(b.motion.status().state, ag::MotionState::kRunning);

    rig.wire.setPlugged(false);
    rig.run(1.0);
    EXPECT_EQ(b.motion.status().state, ag::MotionState::kFailed);
    EXPECT_FALSE(b.client.connected(rig.now()));
    const ig::Pose stopped = rig.body.pose();
    rig.run(0.5);
    EXPECT_NEAR(rig.body.pose().x, stopped.x, 1e-3); // the drive stays stopped

    rig.wire.setPlugged(true);
    ASSERT_TRUE(
        rig.runUntil([&] { return b.readiness(rig.now()) == cg::Readiness::kReady; }, 3.0))
        << cg::toString(b.readiness(rig.now()));
    EXPECT_EQ(rig.wire.count(gatr2::kOpSetPose), 1); // the placement held
    expectPiNearTruth(rig, 0.01, 0.01);
    EXPECT_EQ(b.motion.status().state, ag::MotionState::kFailed); // never resumes
}
