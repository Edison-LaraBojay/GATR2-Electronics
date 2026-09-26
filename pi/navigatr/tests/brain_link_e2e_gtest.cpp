// brain_link_e2e_gtest.cpp
// The real Brain stack against the real Pi runtime. A communiGATR Client and
// Driver under an investiGATR Navigator talk to a navigatr System with
// brain_link on both slots over a timed half-duplex wire. A differential
// drive sim closes the loop through Pico tracking wheel packets. Covers Brain
// reboot, Pi restart (new process and System::reset), placement retries and
// a placement followed at once by a goTo.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "common/frame_codec.h"
#include "communigatr/client.h"
#include "communigatr/driver.h"
#include "impl/resources/cameras.h"
#include "impl/resources/serial_links.h"
#include "investigatr/drive.h"
#include "investigatr/navigator.h"
#include "math/angles.h"
#include "resources/camera.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "sim/differential_drive_sim.h"

using namespace navigatr;
using communigatr::PlacementResult;
using communigatr::PlacementStatus;
using communigatr::PlacementTicket;
using investigatr::CommandId;
using investigatr::DriveCommand;
using investigatr::FrameGeneration;
using investigatr::InputSnapshot;
using investigatr::MotionReason;
using investigatr::MotionState;
using investigatr::Pose;

namespace
{

constexpr int64_t  kTickUs     = 1000;     // sim, wire and drivetrain step
constexpr int64_t  kPiPeriodUs = 10000;    // Loop rate_hz 100
constexpr int64_t  kPollUs     = 2000;     // Brain client task period
constexpr int64_t  kControlUs  = 10000;    // Navigator period
constexpr int64_t  kByteUs     = 87;       // 10 bits at 115200 baud
constexpr int64_t  kGuardUs    = 1000;     // Pi reply start after its cycle
constexpr int64_t  kDowntimeUs = 300000;   // Pi process restart
constexpr double   kCountsPerM = 4000.0 / (2.0 * kPi * 0.0254);
constexpr uint8_t  kLandmark   = 1;   // center_goal
constexpr uint32_t kNonceA     = 0xA11CE001;
constexpr uint32_t kNonceB     = 0xB0B0B002;

const Pose kStart{0.61, 0.457, kPi / 2.0};
const Pose kApproach{-0.6, 0.0, 0.0};   // landmark frame: (1.2, 1.8, 0) in the field

// Three tracking wheels, a map-seeded world estimator on a camera that never
// delivers a frame (landmarks are nominal), one session-owned target on wire
// id 1, and the brain link.
const char* kPiHead = R"(
<System>
    <Loop rate_hz="100"/>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
        </Resource>
        <Resource id="brain_uart" type="memory_link"/>
        <Resource id="game_field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified" x_m="1.8" y_m="1.8" heading_deg="0"/>
            </Landmark>
        </Resource>
        <Resource id="robot_geometry" type="robot_frame_map"/>
        <Resource id="targets" type="target_set">
            <FieldMap resource_id="game_field"/>
            <RobotFrames resource_id="robot_geometry"/>
            <Target id="approach" type="robot_relative" wire_id="1"
                    controlled_frame_id="robot_body">
                <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
                <Delta x_m="0.3" y_m="0" heading_deg="0"/>
                <VisionCorrection type="none"/>
            </Target>
        </Resource>
        <Resource id="camera_device" type="idle_camera"/>
        <Resource id="tag_detector" type="apriltag_detector">
            <Family name="tag36h11" detection_size_m="0.06"/>
            <Detector quad_decimate="1.0" nthreads="1"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="front_camera" type="camera_frame">
            <Source resource_id="camera_device" output_id="frame"/>
        </Sensor>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="brain_link">
            <Serial resource_id="brain_uart"/>
        </CommandCollection>
)";

const char* kOdometry = R"(
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <TrackingWheel sensor_id="enc_a" label="left" radius_m="0.0254"
                               position_x_m="0" position_y_m="0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_b" label="right" radius_m="0.0254"
                               position_x_m="0" position_y_m="-0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_c" label="rear" radius_m="0.0254"
                               position_x_m="-0.12" position_y_m="0"
                               measurement_angle_deg="90" direction="positive"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
            </Estimator>
        </Localization>
)";

// An estimator that never applies a placement: SET_POSE stays Pending.
const char* kNoLocalization = R"(
        <Localization><Estimator type="noop"/></Localization>
)";

const char* kPiTail = R"(
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="tag_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never"/>
            </Estimator>
        </WorldEstimation>
        <TargetResolution type="configured_targets">
            <Targets resource_id="targets"/>
        </TargetResolution>
        <Publishing type="brain_link">
            <Serial resource_id="brain_uart"/>
            <FieldObject object_id="center_goal" wire_id="1"/>
        </Publishing>
    </Pipeline>
</System>
)";

class IdleCamera : public CameraDevice
{
public:
    bool                    alive() const override { return true; }
    const CameraIntrinsics* intrinsics() const override { return nullptr; }
    FrameId engineeringFrame() const override { return FrameId{"front_camera_engineering"}; }
    std::optional<CameraFrameData> latestFrame(uint64_t, uint32_t) override {
        return std::nullopt;
    }
};

gatr2::BrainRequest requestOf(const std::vector<uint8_t>& frame) {
    gatr2::BrainRequest r;
    EXPECT_TRUE(
        gatr2::decodeBrainRequest(frame.data(), static_cast<uint16_t>(frame.size()), r));
    return r;
}

gatr2::BrainReply replyOf(const std::vector<uint8_t>& frame) {
    gatr2::BrainReply r;
    EXPECT_TRUE(gatr2::decodeBrainReply(frame.data(), static_cast<uint16_t>(frame.size()), r));
    return r;
}

bool zero(const DriveCommand& d) { return d.forward == 0.0 && d.turn == 0.0; }

// The RS-485 pair between the Brain port and the Pi memory link. Each
// direction carries one byte per kByteUs, in order. A Pi reply starts one
// guard after its cycle. Brain requests and Pi replies are logged; a Brain
// and a Pi transmission that overlap count as a collision. Injected frames
// stand for delayed traffic of an earlier boot and are not counted.
class Wire : public communigatr::BytePort
{
public:
    explicit Wire(const int64_t& now_us) : now_us_(now_us) {}

    int read(uint8_t* buf, int max) override {
        int n = 0;
        while (n < max && !to_brain_.empty() && to_brain_.front().first <= now_us_) {
            buf[n++] = to_brain_.front().second;
            to_brain_.pop_front();
        }
        return n;
    }

    bool write(const uint8_t* data, int len) override {
        if (now_us_ < pi_tx_end_) {
            ++collisions;
        }
        const std::vector<uint8_t> frame(data, data + len);
        brain_tx_end_ = queue(to_pi_, pi_rx_free_, frame, now_us_);
        requests.push_back(frame);
        if (on_request) {
            on_request(requestOf(frame));
        }
        return true;
    }

    // Request bytes that reached the Pi by now; lost while no Pi runs.
    void deliver(MemoryLink* pi) {
        std::vector<uint8_t> bytes;
        while (!to_pi_.empty() && to_pi_.front().first <= now_us_) {
            bytes.push_back(to_pi_.front().second);
            to_pi_.pop_front();
        }
        if (pi != nullptr && !bytes.empty()) {
            pi->input().feed(bytes);
        }
    }

    // The reply the Pi wrote this cycle, if any, onto the wire.
    void transmit(MemoryLink& pi) {
        const std::vector<uint8_t> frame = pi.output().takeAll();
        if (frame.empty()) {
            return;
        }
        const int64_t start = std::max(now_us_ + kGuardUs, brain_rx_free_);
        if (start < brain_tx_end_) {
            ++collisions;
        }
        replies.push_back(frame);
        if (replyOf(frame).op == gatr2::kOpSetPose && drop_set_pose_replies > 0) {
            --drop_set_pose_replies;   // on the bus, never received
            pi_tx_end_     = start + static_cast<int64_t>(frame.size()) * kByteUs;
            brain_rx_free_ = pi_tx_end_;
            return;
        }
        pi_tx_end_ = queue(to_brain_, brain_rx_free_, frame, start);
    }

    void sendToPi(const std::vector<uint8_t>& frame) {
        queue(to_pi_, pi_rx_free_, frame, now_us_);
    }
    void sendToBrain(const std::vector<uint8_t>& frame) {
        queue(to_brain_, brain_rx_free_, frame, now_us_);
    }

    // A crashed Pi process never reads what was on its way.
    void dropToPi() { to_pi_.clear(); }

    std::vector<std::vector<uint8_t>>               requests;   // every Brain write, in order
    std::vector<std::vector<uint8_t>>               replies;    // every Pi reply, in order
    int                                             drop_set_pose_replies = 0;
    int                                             collisions            = 0;
    std::function<void(const gatr2::BrainRequest&)> on_request;   // after each Brain write

private:
    using Timed = std::deque<std::pair<int64_t, uint8_t>>;

    // Bytes follow each other and whatever is already queued; returns the end.
    static int64_t queue(Timed& q, int64_t& free_us, const std::vector<uint8_t>& frame,
                         int64_t start_us) {
        int64_t t = std::max(start_us, free_us);
        for (uint8_t b : frame) {
            t += kByteUs;
            q.emplace_back(t, b);
        }
        free_us = t;
        return t;
    }

    const int64_t& now_us_;
    Timed          to_pi_;
    Timed          to_brain_;
    int64_t        pi_rx_free_    = 0;
    int64_t        brain_rx_free_ = 0;
    int64_t        brain_tx_end_  = 0;
    int64_t        pi_tx_end_     = 0;
};

// One Brain boot: new client, driver and Navigator, request ids from 1. The
// drivetrain holds demand between Navigator updates.
struct Brain {
    Brain(communigatr::BytePort& port, uint32_t nonce)
        : client(port, [nonce] { return nonce; }), driver(client), navigator(driver) {}

    communigatr::Client    client;
    communigatr::Driver    driver;
    investigatr::Navigator navigator;
    DriveCommand           demand;
    bool                   polling = true;   // false: the client task is stalled
};

enum class Restart { kProcess, kReset };

// Pi, wire, drivetrain and at most one Brain on one microsecond clock.
struct Rig {
    std::string                       xml;
    FunctionRegistry                  functions;
    int64_t                           now_us = 0;
    Wire                              wire{now_us};
    std::unique_ptr<System>           system;   // null while the Pi process is down
    MemoryLink*                       pico     = nullptr;
    MemoryLink*                       link     = nullptr;   // brain_uart
    int64_t                           up_at_us = 0;
    investigatr::DifferentialDriveSim drive{{}, kStart};
    double                            travel[3] = {0.0, 0.0, 0.0};
    uint8_t                           pico_seq  = 0;
    std::unique_ptr<Brain>            brain;   // null while powered off
    std::function<void()>             each_tick;

    explicit Rig(bool odometry = true)
        : xml(std::string(kPiHead) + (odometry ? kOdometry : kNoLocalization) + kPiTail) {
        registerAll(functions);
        functions.add(FunctionKey{"idle_camera"},
                      ResourceMakeFunction([](const ConfigNode&, ResourceInitializationContext&,
                                              std::string&) {
                          return cameraResource(std::make_shared<IdleCamera>(),
                                                OutputId{"frame"});
                      }));
        startPi();
    }

    double now() const { return static_cast<double>(now_us) * 1e-6; }

    void startPi() {
        std::string err;
        system = System::buildFromString(xml.c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system == nullptr) {
            return;
        }
        auto brain_link = system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
        auto pico_link  = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
        link            = dynamic_cast<MemoryLink*>(brain_link.get());
        pico            = dynamic_cast<MemoryLink*>(pico_link.get());
        EXPECT_NE(link, nullptr);
        EXPECT_NE(pico, nullptr);
        link->setClock([this] { return now_us; });
    }

    // kProcess: a new System after kDowntimeUs, bytes on their way are lost.
    // kReset: System::reset() in place.
    void restartPi(Restart kind) {
        if (kind == Restart::kReset) {
            system->reset();
            return;
        }
        system.reset();
        link     = nullptr;
        pico     = nullptr;
        up_at_us = now_us + kDowntimeUs;
        wire.dropToPi();
    }

    Brain& bootBrain(uint32_t nonce) {
        brain = std::make_unique<Brain>(wire, nonce);
        return *brain;
    }

    void tick() {
        now_us += kTickUs;

        // drivetrain; stopped while the Brain is off
        const investigatr::TankOutput out =
            brain != nullptr ? investigatr::mixTank(brain->demand) : investigatr::TankOutput{};
        const Pose before = drive.pose();
        drive.step(out, static_cast<double>(kTickUs) * 1e-6);
        accumulate(before, drive.pose());

        if (system == nullptr && now_us >= up_at_us) {
            startPi();
        }
        wire.deliver(link);
        if (system != nullptr && now_us % kPiPeriodUs == 0) {
            pico->input().feed(sensorPacket());
            system->step(hostTime(now_us / 1000));
            wire.transmit(*link);
        }

        if (brain != nullptr) {
            if (brain->polling && now_us % kPollUs == 0) {
                brain->client.poll(now());
            }
            if (now_us % kControlUs == 0) {
                brain->demand = brain->navigator.update(now());
            }
        }
        if (each_tick) {
            each_tick();
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

    // Tracking wheel travel for one body step: left (0, 0.13) and right
    // (0, -0.13) measure along x, rear (-0.12, 0) along y.
    void accumulate(const Pose& a, const Pose& b) {
        const double dth = investigatr::wrapAngle(b.heading - a.heading);
        const double mid = a.heading + dth / 2.0;
        const double ex  = b.x - a.x;
        const double ey  = b.y - a.y;
        const double dx  = std::cos(mid) * ex + std::sin(mid) * ey;
        const double dy  = -std::sin(mid) * ex + std::cos(mid) * ey;
        travel[0] += dx - 0.13 * dth;
        travel[1] += dx + 0.13 * dth;
        travel[2] += dy - 0.12 * dth;
    }

    std::vector<uint8_t> sensorPacket() {
        gatr2::SensorSample s{};
        s.seq      = pico_seq++;
        s.stamp_ms = static_cast<uint32_t>(5000 + now_us / 1000);
        s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorEnc2;
        for (int i = 0; i < 3; ++i) {
            s.enc[i] = static_cast<int32_t>(std::llround(travel[i] * kCountsPerM));
        }
        std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
        buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
        return buf;
    }
};

// Boots a Brain that places the robot and waits for kApplied.
Brain& connect(Rig& rig, uint32_t nonce, const Pose& pose) {
    Brain&                b      = rig.bootBrain(nonce);
    const PlacementTicket ticket = b.driver.submitPlacement(pose);
    EXPECT_NE(ticket, 0u);
    EXPECT_TRUE(rig.runUntil(
        [&] { return b.driver.placementResult(ticket) == PlacementResult::kApplied; }, 2.0));
    return b;
}

// Logged Brain requests from index `from` with this op.
std::vector<std::vector<uint8_t>> withOp(const std::vector<std::vector<uint8_t>>& requests,
                                         std::size_t from, uint8_t op) {
    std::vector<std::vector<uint8_t>> out;
    for (std::size_t i = from; i < requests.size(); ++i) {
        if (requestOf(requests[i]).op == op) {
            out.push_back(requests[i]);
        }
    }
    return out;
}

std::vector<uint8_t> firstWithOp(const std::vector<std::vector<uint8_t>>& requests,
                                 std::size_t from, uint8_t op) {
    const std::vector<std::vector<uint8_t>> found = withOp(requests, from, op);
    if (found.empty()) {
        ADD_FAILURE() << "no request with op " << static_cast<int>(op);
        return {};
    }
    return found.front();
}

void expectNear(const Pose2D& pose, const Pose& want, double tolerance_m) {
    EXPECT_NEAR(pose.x_m, want.x, tolerance_m);
    EXPECT_NEAR(pose.y_m, want.y, tolerance_m);
}

// Pi state a delayed request of another session must never change.
struct Applied {
    uint32_t session;
    uint64_t init_sequence;
    uint32_t init_session;
    bool     object_requested;
    uint8_t  object_wire_id;
    uint64_t object_sequence;
    uint64_t anchor_revision;
    uint32_t placement_session;
    uint64_t placement_sequence;
    bool     target_active;
    uint64_t target_generation;

    static Applied of(const System& s) {
        const CommandState& c = s.command();
        const RobotState&   r = s.robot();
        return {c.session,          c.init_sequence,      c.init_session,
                c.object_requested, c.object_wire_id,     c.object_sequence,
                r.anchor_revision,  r.placement_session,  r.placement_sequence,
                s.target().active,  s.target().generation};
    }

    bool operator==(const Applied& o) const {
        return session == o.session && init_sequence == o.init_sequence &&
               init_session == o.init_session && object_requested == o.object_requested &&
               object_wire_id == o.object_wire_id && object_sequence == o.object_sequence &&
               anchor_revision == o.anchor_revision && placement_session == o.placement_session &&
               placement_sequence == o.placement_sequence && target_active == o.target_active &&
               target_generation == o.target_generation;
    }
};

} // namespace

// ---- Brain reboot ------------------------------------------------------------

TEST(BrainLinkEndToEnd, BrainRebootStartsANewSessionWithoutAPiReset) {
    Rig rig;
    ASSERT_NE(rig.system, nullptr);
    investigatr::MotionOptions nominal;
    nominal.require_observed_landmark = false;

    // session A places the robot and drives toward the landmark
    Brain&                a        = rig.bootBrain(kNonceA);
    const PlacementTicket a_ticket = a.driver.submitPlacement(kStart);
    a.navigator.goToRelative(kLandmark, kApproach, nominal);
    ASSERT_TRUE(
        rig.runUntil([&] { return a.navigator.status().state == MotionState::kDriving; }, 3.0))
        << investigatr::toString(a.navigator.status().reason);
    rig.run(0.5);
    ASSERT_EQ(a.navigator.status().state, MotionState::kDriving);
    EXPECT_EQ(a.driver.placementResult(a_ticket), PlacementResult::kApplied);
    EXPECT_EQ(a.client.selection(), communigatr::SelectionState::kActive);
    EXPECT_TRUE(rig.system->target().latched);   // session A's latch

    const uint32_t a_session   = a.client.session();
    const uint32_t pi_instance = a.client.piInstance();
    const uint64_t a_target    = rig.system->target().generation;
    const auto     a_requests  = rig.wire.requests;
    const auto     a_replies   = rig.wire.replies;
    const auto     a_set_pose  = firstWithOp(a_requests, 0, gatr2::kOpSetPose);
    const uint64_t a_anchor    = rig.system->robot().anchor_revision;
    const uint64_t a_init_seq  = rig.system->command().init_sequence;

    // A powers off mid-drive; the Pi keeps running and sends nothing unasked
    rig.brain.reset();
    const uint64_t cycles = rig.system->cycle();
    rig.run(1.0);
    EXPECT_GE(rig.system->cycle(), cycles + 99);
    EXPECT_EQ(rig.wire.replies.size(), a_replies.size());
    EXPECT_EQ(rig.system->command().session, a_session);
    const Pose2D rest = rig.system->robot().odom_pose;

    // B asks with the ids A used; A's delayed replies arrive first
    std::map<uint16_t, std::vector<uint8_t>> a_reply_by_id;
    for (const auto& frame : a_replies) {
        a_reply_by_id.emplace(replyOf(frame).request_id, frame);
    }
    int injected        = 0;
    rig.wire.on_request = [&](const gatr2::BrainRequest& r) {
        const auto it = a_reply_by_id.find(r.request_id);
        if (it != a_reply_by_id.end()) {
            rig.wire.sendToBrain(it->second);
            a_reply_by_id.erase(it);
            ++injected;
        }
    };
    // B's Navigator stays idle with zero output and the robot stays put
    rig.each_tick = [&] {
        if (rig.brain != nullptr) {
            EXPECT_EQ(rig.brain->navigator.status().state, MotionState::kIdle);
            EXPECT_TRUE(zero(rig.brain->demand));
        }
        EXPECT_NEAR(rig.system->robot().odom_pose.x_m, rest.x_m, 1e-6);
        EXPECT_NEAR(rig.system->robot().odom_pose.y_m, rest.y_m, 1e-6);
    };

    const std::size_t     b_first  = rig.wire.requests.size();
    Brain&                b        = rig.bootBrain(kNonceB);
    const PlacementTicket b_ticket = b.driver.submitPlacement(kStart);
    ASSERT_TRUE(rig.runUntil(
        [&] { return b.driver.placementResult(b_ticket) == PlacementResult::kApplied; }, 2.0));
    rig.wire.on_request = nullptr;

    // same request id and pose as A's placement, a new session: applied
    const gatr2::BrainRequest hello = requestOf(rig.wire.requests[b_first]);
    const gatr2::BrainRequest place =
        requestOf(firstWithOp(rig.wire.requests, b_first, gatr2::kOpSetPose));
    const gatr2::BrainRequest old = requestOf(a_set_pose);
    EXPECT_EQ(hello.op, gatr2::kOpHello);
    EXPECT_EQ(hello.request_id, 1u);
    EXPECT_EQ(place.request_id, old.request_id);
    EXPECT_EQ(place.x_mm, old.x_mm);
    EXPECT_EQ(place.y_mm, old.y_mm);
    EXPECT_EQ(place.heading_cdeg, old.heading_cdeg);
    EXPECT_NE(place.session, old.session);

    const uint32_t b_session = b.client.session();
    EXPECT_NE(b_session, a_session);
    EXPECT_EQ(rig.system->command().session, b_session);
    EXPECT_EQ(rig.system->robot().anchor_revision, a_anchor + 1);
    EXPECT_EQ(rig.system->robot().placement_origin, "command");
    EXPECT_EQ(rig.system->robot().placement_session, b_session);
    EXPECT_EQ(rig.system->command().init_sequence, a_init_seq + 1);
    EXPECT_FALSE(rig.system->target().active);   // the new session released A's latch
    EXPECT_FALSE(rig.system->command().object_requested);

    // A's replies were dropped (A's last reply may still have been on the
    // wire at power-off: one more); B's Ok is its own; nothing reset the Pi
    EXPECT_GE(injected, 3);
    EXPECT_GE(b.client.stats().uncorrelated, static_cast<uint32_t>(injected));
    EXPECT_LE(b.client.stats().uncorrelated, static_cast<uint32_t>(injected) + 1);
    EXPECT_EQ(b.driver.placementStatus(b_ticket).anchor_revision, a_anchor + 1);
    EXPECT_EQ(b.client.state().session, b_session);
    EXPECT_EQ(b.client.stats().sessions, 1u);
    EXPECT_EQ(b.client.stats().session_losses, 0u);
    EXPECT_EQ(b.client.piInstance(), pi_instance);
    EXPECT_EQ(rig.system->resetCount(), 0u);

    // A's delayed requests after B opened: refused, nothing changes
    const Applied before = Applied::of(*rig.system);
    b.polling            = false;
    rig.run(0.03);   // B's last exchange completes
    for (uint8_t op : {gatr2::kOpHello, gatr2::kOpSetPose, gatr2::kOpSelectLandmark,
                       gatr2::kOpGetState}) {
        const std::vector<uint8_t> frame = firstWithOp(a_requests, 0, op);
        const std::size_t          n     = rig.wire.replies.size();
        rig.wire.sendToPi(frame);
        rig.run(0.02);
        ASSERT_EQ(rig.wire.replies.size(), n + 1) << "op " << static_cast<int>(op);
        const gatr2::BrainReply reply = replyOf(rig.wire.replies.back());
        EXPECT_EQ(reply.request_id, requestOf(frame).request_id);
        EXPECT_EQ(reply.result, op == gatr2::kOpHello ? gatr2::kResultStale
                                                      : gatr2::kResultUnknownSession);
    }
    b.polling = true;
    EXPECT_TRUE(Applied::of(*rig.system) == before);
    rig.run(0.3);
    EXPECT_TRUE(b.client.connected(rig.now()));
    EXPECT_EQ(b.client.session(), b_session);
    EXPECT_EQ(b.client.stats().session_losses, 0u);
    rig.each_tick = nullptr;

    // a new command from B selects the landmark again and completes
    const CommandId id = b.navigator.goToRelative(kLandmark, kApproach, nominal);
    ASSERT_TRUE(
        rig.runUntil([&] { return investigatr::isTerminal(b.navigator.status().state); }, 12.0));
    EXPECT_EQ(b.navigator.status().state, MotionState::kCompleted)
        << investigatr::toString(b.navigator.status().reason);
    EXPECT_EQ(b.navigator.status().command_id, id);
    EXPECT_TRUE(rig.system->command().object_requested);
    EXPECT_EQ(rig.system->command().object_wire_id, kLandmark);
    EXPECT_TRUE(rig.system->target().active);
    EXPECT_GT(rig.system->target().generation, a_target);
    expectNear(rig.system->robot().fieldPose(), Pose{1.2, 1.8, 0.0}, 0.035);
    EXPECT_EQ(rig.wire.collisions, 0);
}

// ---- Pi restart ----------------------------------------------------------------

namespace
{

// An active command fails and never resumes, even once a new placement
// makes the input usable again in a new frame.
void activeCommandFailsOnPiRestart(Restart kind) {
    Rig rig;
    ASSERT_NE(rig.system, nullptr);
    Brain&          b  = connect(rig, kNonceA, kStart);
    const CommandId id = b.navigator.goTo(Pose{2.4, 0.457, 0.0});
    ASSERT_TRUE(
        rig.runUntil([&] { return b.navigator.status().state == MotionState::kDriving; }, 3.0));
    rig.run(0.3);
    const FrameGeneration frame    = b.driver.latest(rig.now()).frame;
    const uint32_t        session  = b.client.session();
    const uint32_t        instance = b.client.piInstance();
    ASSERT_NE(frame, 0u);

    rig.restartPi(kind);

    // detected from a correlated reply: input cleared at once, then a new session
    bool cleared = false;
    ASSERT_TRUE(rig.runUntil(
        [&] {
            if (!cleared && b.client.stats().pi_restarts > 0) {
                const InputSnapshot s = b.driver.latest(rig.now());
                EXPECT_EQ(s.frame, 0u);
                EXPECT_FALSE(s.robot.valid);
                EXPECT_FALSE(s.connected);
                cleared = true;
            }
            if (investigatr::isTerminal(b.navigator.status().state)) {
                EXPECT_TRUE(zero(b.demand));
            }
            return cleared && b.client.ready();
        },
        3.0));
    EXPECT_EQ(b.client.stats().pi_restarts, 1u);
    EXPECT_NE(b.client.session(), session);
    EXPECT_NE(b.client.piInstance(), instance);
    EXPECT_EQ(b.navigator.status().state, MotionState::kFailed);
    const MotionReason reason = b.navigator.status().reason;
    EXPECT_TRUE(reason == MotionReason::kInputLost || reason == MotionReason::kFrameChanged)
        << investigatr::toString(reason);
    EXPECT_FALSE(rig.system->robot().initialized);   // the new runtime has no anchor

    const PlacementTicket ticket = b.driver.submitPlacement(kStart);
    ASSERT_TRUE(rig.runUntil(
        [&] { return b.driver.placementResult(ticket) == PlacementResult::kApplied; }, 2.0));
    const InputSnapshot s = b.driver.latest(rig.now());
    EXPECT_TRUE(s.robot.valid);
    EXPECT_NE(s.frame, 0u);
    EXPECT_NE(s.frame, frame);

    rig.each_tick = [&] { EXPECT_TRUE(zero(b.demand)); };
    rig.run(1.0);
    EXPECT_EQ(b.navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(b.navigator.status().reason, reason);
    EXPECT_EQ(b.navigator.status().command_id, id);
    EXPECT_EQ(rig.wire.collisions, 0);
}

// A placement in flight when the Pi restarts ends kSessionLost and is never
// sent again; the new runtime never applies it.
void inFlightPlacementLostOnPiRestart(Restart kind) {
    Rig rig;
    ASSERT_NE(rig.system, nullptr);
    Brain& b = connect(rig, kNonceA, kStart);

    bool sent           = false;
    rig.wire.on_request = [&](const gatr2::BrainRequest& r) {
        sent = sent || r.op == gatr2::kOpSetPose;
    };
    const std::size_t     first  = rig.wire.requests.size();
    const PlacementTicket ticket = b.driver.submitPlacement(Pose{1.0, 1.0, 0.0});
    ASSERT_TRUE(rig.runUntil([&] { return sent; }, 1.0));
    rig.wire.on_request = nullptr;
    rig.restartPi(kind);

    ASSERT_TRUE(rig.runUntil(
        [&] { return b.client.stats().pi_restarts > 0 && b.client.ready(); }, 3.0));
    EXPECT_EQ(b.driver.placementResult(ticket), PlacementResult::kSessionLost);
    rig.run(1.0);
    EXPECT_EQ(b.driver.placementResult(ticket), PlacementResult::kSessionLost);

    // every SET_POSE since the submit is the same frame, all before the new HELLO
    const std::vector<uint8_t> lost  = firstWithOp(rig.wire.requests, first, gatr2::kOpSetPose);
    bool                       hello = false;
    for (std::size_t i = first; i < rig.wire.requests.size(); ++i) {
        const gatr2::BrainRequest r = requestOf(rig.wire.requests[i]);
        hello                       = hello || r.op == gatr2::kOpHello;
        if (r.op == gatr2::kOpSetPose) {
            EXPECT_FALSE(hello) << "resent after the session was lost";
            EXPECT_EQ(rig.wire.requests[i], lost);
        }
    }
    EXPECT_TRUE(hello);
    EXPECT_EQ(rig.system->command().init_sequence, 0u);
    EXPECT_FALSE(rig.system->robot().initialized);
    EXPECT_FALSE(b.driver.latest(rig.now()).robot.valid);
    EXPECT_EQ(rig.wire.collisions, 0);
}

} // namespace

TEST(BrainLinkEndToEnd, PiProcessRestartFailsTheActiveCommand) {
    activeCommandFailsOnPiRestart(Restart::kProcess);
}

TEST(BrainLinkEndToEnd, PiResetFailsTheActiveCommand) {
    activeCommandFailsOnPiRestart(Restart::kReset);
}

TEST(BrainLinkEndToEnd, PiProcessRestartLosesTheInFlightPlacement) {
    inFlightPlacementLostOnPiRestart(Restart::kProcess);
}

TEST(BrainLinkEndToEnd, PiResetLosesTheInFlightPlacement) {
    inFlightPlacementLostOnPiRestart(Restart::kReset);
}

// ---- placement retries ---------------------------------------------------------

TEST(BrainLinkEndToEnd, DroppedPlacementRepliesRetryTheSameBytesAndApplyOnce) {
    Rig rig;
    ASSERT_NE(rig.system, nullptr);
    Brain& b = rig.bootBrain(kNonceA);
    ASSERT_TRUE(rig.runUntil([&] { return b.client.ready(); }, 1.0));

    rig.wire.drop_set_pose_replies = 2;
    const std::size_t     first    = rig.wire.requests.size();
    const PlacementTicket ticket   = b.driver.submitPlacement(kStart);
    int64_t               acked_at = -1;
    ASSERT_TRUE(rig.runUntil(
        [&] {
            const PlacementStatus s = b.driver.placementStatus(ticket);
            if (s.state == PlacementResult::kPending) {
                EXPECT_FALSE(b.driver.latest(rig.now()).robot.valid);
                if (s.anchor_revision != 0 && acked_at < 0) {
                    acked_at = rig.now_us;
                }
            }
            return s.state != PlacementResult::kPending;
        },
        2.0));
    const PlacementStatus status = b.driver.placementStatus(ticket);
    EXPECT_EQ(status.state, PlacementResult::kApplied);
    EXPECT_EQ(status.anchor_revision, 1u);
    ASSERT_GE(acked_at, 0);
    EXPECT_LT(acked_at, rig.now_us);   // applied on a state reply after the Ok
    EXPECT_EQ(b.client.state().state.anchor_revision, 1u);

    // three sends of the same bytes, one application
    const auto sends = withOp(rig.wire.requests, first, gatr2::kOpSetPose);
    ASSERT_EQ(sends.size(), 3u);
    EXPECT_EQ(sends[1], sends[0]);
    EXPECT_EQ(sends[2], sends[0]);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);
    EXPECT_EQ(rig.system->command().init_sequence, 1u);
    EXPECT_EQ(rig.system->diagnostics().links.at("brain_uart").duplicates, 2u);
    const InputSnapshot s = b.driver.latest(rig.now());
    ASSERT_TRUE(s.robot.valid);
    EXPECT_NEAR(s.robot.pose.x, kStart.x, 0.002);
    EXPECT_NEAR(s.robot.pose.y, kStart.y, 0.002);

    // a late copy after GET_STATE polls: answered Ok, never applied again
    b.polling = false;
    rig.run(0.03);
    const std::size_t n = rig.wire.replies.size();
    rig.wire.sendToPi(sends[0]);
    rig.run(0.02);
    ASSERT_EQ(rig.wire.replies.size(), n + 1);
    const gatr2::BrainReply late = replyOf(rig.wire.replies.back());
    EXPECT_EQ(late.result, gatr2::kResultOk);
    EXPECT_EQ(late.anchor_revision, 1u);
    b.polling = true;
    rig.run(0.2);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);
    EXPECT_EQ(rig.system->command().init_sequence, 1u);
    EXPECT_EQ(rig.system->diagnostics().links.at("brain_uart").duplicates, 3u);
    EXPECT_EQ(b.driver.placementResult(ticket), PlacementResult::kApplied);
    EXPECT_TRUE(b.client.connected(rig.now()));
    EXPECT_EQ(b.client.stats().session_losses, 0u);
    EXPECT_EQ(rig.wire.collisions, 0);
}

TEST(BrainLinkEndToEnd, PendingPlacementRetriesBetweenStatePollsNeverReapply) {
    Rig rig(false);   // localization never applies it: every answer is Pending
    ASSERT_NE(rig.system, nullptr);
    Brain& b = rig.bootBrain(kNonceA);
    ASSERT_TRUE(rig.runUntil([&] { return b.client.ready(); }, 1.0));

    const std::size_t     first  = rig.wire.requests.size();
    const PlacementTicket ticket = b.driver.submitPlacement(kStart);
    ASSERT_TRUE(rig.runUntil(
        [&] {
            EXPECT_LE(rig.system->command().init_sequence, 1u);
            return !b.driver.placementPending();
        },
        2.0));
    const PlacementStatus status = b.driver.placementStatus(ticket);
    EXPECT_EQ(status.state, PlacementResult::kTimedOut);   // outcome unknown
    EXPECT_EQ(status.result, gatr2::kResultPending);

    // the same bytes up to the attempt bound, with state polls in between
    const auto sends = withOp(rig.wire.requests, first, gatr2::kOpSetPose);
    ASSERT_EQ(sends.size(), static_cast<std::size_t>(b.client.config().placement_attempts));
    int         polls_between = 0;
    std::size_t seen          = 0;
    for (std::size_t i = first; i < rig.wire.requests.size() && seen < sends.size(); ++i) {
        const gatr2::BrainRequest r = requestOf(rig.wire.requests[i]);
        if (r.op == gatr2::kOpSetPose) {
            EXPECT_EQ(rig.wire.requests[i], sends.front());
            ++seen;
        } else if (r.op == gatr2::kOpGetState && seen > 0) {
            ++polls_between;
        }
    }
    EXPECT_GE(polls_between, 3);
    EXPECT_EQ(rig.system->command().init_sequence, 1u);
    EXPECT_EQ(rig.system->diagnostics().links.at("brain_uart").duplicates,
              static_cast<uint32_t>(sends.size() - 1));
    EXPECT_FALSE(rig.system->robot().initialized);
    EXPECT_FALSE(b.driver.latest(rig.now()).robot.valid);
    EXPECT_EQ(rig.wire.collisions, 0);
}

// ---- placement then goTo -------------------------------------------------------

TEST(BrainLinkEndToEnd, PlacementThenImmediateGoToCapturesTheNewFrame) {
    Rig rig;
    ASSERT_NE(rig.system, nullptr);

    // an earlier boot localized the Pi and moved the robot
    {
        Brain& a = connect(rig, kNonceA, kStart);
        a.navigator.goTo(Pose{0.9, 0.9, 0.0});
        ASSERT_TRUE(rig.runUntil(
            [&] { return investigatr::isTerminal(a.navigator.status().state); }, 8.0));
        EXPECT_EQ(a.navigator.status().state, MotionState::kCompleted);
        rig.brain.reset();
    }
    rig.run(0.5);

    // B takes the inherited command anchor as usable input before placing
    Brain& b = rig.bootBrain(kNonceB);
    ASSERT_TRUE(rig.runUntil([&] { return b.driver.latest(rig.now()).robot.valid; }, 1.0));
    const FrameGeneration inherited = b.driver.latest(rig.now()).frame;

    const PlacementTicket ticket = b.driver.submitPlacement(kStart);
    const Pose            goal{1.0, 1.2, 0.0};
    const CommandId       id = b.navigator.goTo(goal);
    ASSERT_TRUE(rig.runUntil(
        [&] {
            EXPECT_NE(b.navigator.status().reason, MotionReason::kFrameChanged);
            return investigatr::isTerminal(b.navigator.status().state);
        },
        10.0));
    EXPECT_EQ(b.navigator.status().state, MotionState::kCompleted)
        << investigatr::toString(b.navigator.status().reason);
    EXPECT_EQ(b.navigator.status().command_id, id);
    EXPECT_EQ(b.driver.placementResult(ticket), PlacementResult::kApplied);
    EXPECT_NE(b.driver.latest(rig.now()).frame, inherited);
    expectNear(rig.system->robot().fieldPose(), goal, 0.035);
    EXPECT_EQ(rig.wire.collisions, 0);
}
