// workers_gtest.cpp
// The two-worker runtime through the synthetic rig: the bounded handoff and
// the worker counters on their own, then start, stop, reset and destroy
// with real threads, a detector slow enough that the field worker falls
// behind the frame rate, a camera that dies mid run, and the snapshot
// readers the other worker and inspection depend on. Every timing
// assertion polls against a deadline or accepts a range: Windows rounds
// sleeps up to its timer tick, so exact counts would only test the
// scheduler.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "core/host_clock.h"
#include "impl/resources/cameras.h"
#include "impl/resources/tag_detectors.h"
#include "payloads/camera_frames.h"
#include "payloads/tag_observations.h"
#include "resources/camera.h"
#include "resources/tag_detector.h"
#include "runtime/handoff.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// Polls pred every few ms until it holds or the deadline passes.
bool waitFor(Ms deadline, const std::function<bool()>& pred) {
    const auto until = Clock::now() + deadline;
    while (!pred()) {
        if (Clock::now() >= until) {
            return pred();
        }
        std::this_thread::sleep_for(Ms(5));
    }
    return true;
}

// Detector that holds every frame for delay before delegating to the real
// one, so the field worker cannot keep up with a 10 Hz camera. entered and
// left say whether a detection is in flight.
struct DetectorProbe {
    std::atomic<uint64_t> entered{0};
    std::atomic<uint64_t> left{0};
    bool                  inFlight() const { return entered.load() > left.load(); }
};

class SlowDetector : public TagDetector
{
public:
    SlowDetector(std::shared_ptr<TagDetector> inner, Ms delay,
                 std::shared_ptr<DetectorProbe> probe)
        : inner_(std::move(inner)), delay_(delay), probe_(std::move(probe)) {}

    bool detect(const CameraFrameData& frame, const CameraIntrinsics* intrinsics,
                std::vector<NativeTagDetection>& out, std::string& err) override {
        ++probe_->entered;
        std::this_thread::sleep_for(delay_);
        const bool ok = inner_->detect(frame, intrinsics, out, err);
        ++probe_->left;
        return ok;
    }

private:
    std::shared_ptr<TagDetector>   inner_;
    Ms                             delay_;
    std::shared_ptr<DetectorProbe> probe_;
};

// Camera that serves a fresh blank frame on every poll until the test
// tells it to die; from then on it is dead with a reason. Polled on the
// estimation worker, controlled from the test thread.
struct CameraProbe {
    std::atomic<bool>     die{false};
    std::atomic<bool>     alive{true};
    std::atomic<uint64_t> polls{0};
};

class FailingCamera : public CameraDevice
{
public:
    explicit FailingCamera(std::shared_ptr<CameraProbe> probe) : probe_(std::move(probe)) {
        intrinsics_.model                = "brown_conrady";
        intrinsics_.calibrated_width_px  = kWidth;
        intrinsics_.calibrated_height_px = kHeight;
        intrinsics_.fx_px                = 150;
        intrinsics_.fy_px                = 150;
        intrinsics_.cx_px                = kWidth / 2;
        intrinsics_.cy_px                = kHeight / 2;
    }

    bool        alive() const override { return probe_->alive.load(); }
    std::string diagnostic() const override {
        return probe_->alive.load() ? std::string{} : "camera unplugged during the run";
    }
    const CameraIntrinsics* intrinsics() const override { return &intrinsics_; }
    FrameId                 engineeringFrame() const override {
        return FrameId{"front_camera_engineering"};
    }
    std::optional<CameraFrameData> latestFrame(uint64_t, uint32_t) override {
        ++probe_->polls;
        if (probe_->die.load()) {
            probe_->alive.store(false);
            return std::nullopt;
        }
        CameraFrameData frame;
        frame.sequence   = ++sequence_;
        frame.exposureAt = HostClock::now();
        frame.receivedAt = frame.exposureAt;
        frame.width_px   = kWidth;
        frame.height_px  = kHeight;
        frame.y8         = pixels_;
        return frame;
    }

private:
    static constexpr int kWidth  = 160;
    static constexpr int kHeight = 120;

    std::shared_ptr<CameraProbe>                probe_;
    CameraIntrinsics                            intrinsics_;
    uint32_t                                    sequence_ = 0;
    std::shared_ptr<const std::vector<uint8_t>> pixels_ =
        std::make_shared<const std::vector<uint8_t>>(kWidth * kHeight, 128);
};

// The synthetic rig configuration from synthetic_rig_gtest.cpp with the
// detector type and the camera the front_camera sensor reads left open.
struct RigOptions {
    std::string detector_type   = "apriltag_detector";
    std::string camera_resource = "rig";
    std::string extra_resources;   // xml appended inside Resources
};

std::string replaceToken(std::string xml, const std::string& token, const std::string& value) {
    const auto at = xml.find(token);
    if (at != std::string::npos) {
        xml.replace(at, token.size(), value);
    }
    return xml;
}

std::string rigConfig(const RigOptions& options) {
    std::string xml = R"(
<System>
    <Loop rate_hz="100"/>
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
        <Resource id="tag_detector" type="@DETECTOR@">
            <Family name="tagCircle21h7" detection_size_m="0.03"/>
            <Detector quad_decimate="1.0" nthreads="2"/>
        </Resource>
        @EXTRA@
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
            <Source resource_id="@CAMERA@" output_id="frame"/>
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
    xml = replaceToken(std::move(xml), "@DETECTOR@", options.detector_type);
    xml = replaceToken(std::move(xml), "@CAMERA@", options.camera_resource);
    xml = replaceToken(std::move(xml), "@EXTRA@", options.extra_resources);
    return xml;
}

const SensorId kCamera{"front_camera"};

struct Rig {
    FunctionRegistry               functions;
    std::shared_ptr<DetectorProbe> detector = std::make_shared<DetectorProbe>();
    std::shared_ptr<CameraProbe>   camera   = std::make_shared<CameraProbe>();
    std::unique_ptr<System>        system;

    explicit Rig(const RigOptions& options = {}) {
        registerAll(functions);
        // the slow detector wraps the real one built from the same node, so
        // the configuration stays the rig's
        auto probe = detector;
        functions.add(FunctionKey{"slow_apriltag_detector"},
                      ResourceMakeFunction([probe](const ConfigNode&              node,
                                                   ResourceInitializationContext& context,
                                                   std::string&                   err) {
                          ResourceInstance inner = make_apriltag_detector(node, context, err);
                          if (inner.empty()) {
                              return ResourceInstance{};
                          }
                          std::shared_ptr<TagDetector> real = inner.require<TagDetector>(err);
                          if (real == nullptr) {
                              return ResourceInstance{};
                          }
                          return ResourceInstance::asContract<TagDetector>(
                              std::make_shared<SlowDetector>(real, Ms(150), probe));
                      }));
        auto cam = camera;
        functions.add(FunctionKey{"failing_camera"},
                      ResourceMakeFunction([cam](const ConfigNode&, ResourceInitializationContext&,
                                                 std::string&) {
                          return cameraResource(std::make_shared<FailingCamera>(cam),
                                                OutputId{"frame"});
                      }));
        std::string err;
        system = System::buildFromString(rigConfig(options).c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;   // ASSERT cannot live in a ctor
    }

    bool start() {
        std::string err;
        const bool  ok = system->start(err);
        EXPECT_TRUE(ok) << err;
        return ok;
    }
};

const SourceHealthEntry* findEntry(const SourceHealthSnapshot& health, const char* kind,
                                   const std::string& id) {
    for (const SourceHealthEntry& e : health.entries) {
        if (e.kind == kind && e.id == id) {
            return &e;
        }
    }
    return nullptr;
}

// The newest detection frame for the camera, if any was published.
std::shared_ptr<const DetectionFrameSnapshot> cameraFrame(const System& system) {
    const auto frames = system.detectionFrames();
    const auto it     = frames.find(kCamera);
    return it == frames.end() ? nullptr : it->second;
}

} // namespace

// ---- handoff and counters --------------------------------------------------

TEST(Workers, LatestSlotHandsOverTheNewestEntryAndCountsDisplaced) {
    LatestSlot<int> slot;
    int             out = 0;
    EXPECT_FALSE(slot.take(out));
    EXPECT_FALSE(slot.pending());

    slot.put(1);
    slot.put(2);
    slot.put(3);
    EXPECT_TRUE(slot.pending());
    EXPECT_EQ(slot.offered(), 3u);
    EXPECT_EQ(slot.replaced(), 2u);   // 1 and 2 were displaced before anyone saw them
    EXPECT_EQ(slot.taken(), 0u);

    ASSERT_TRUE(slot.take(out));
    EXPECT_EQ(out, 3);
    EXPECT_EQ(slot.taken(), 1u);
    EXPECT_FALSE(slot.pending());
    EXPECT_FALSE(slot.take(out));

    // a consumed entry is never counted as displaced
    slot.put(4);
    ASSERT_TRUE(slot.take(out));
    EXPECT_EQ(out, 4);
    EXPECT_EQ(slot.replaced(), 2u);
    EXPECT_EQ(slot.taken(), 2u);

    // clear drops the pending entry without counting it as replaced
    slot.put(5);
    slot.clear();
    EXPECT_FALSE(slot.pending());
    EXPECT_FALSE(slot.take(out));
    EXPECT_EQ(slot.offered(), 5u);
    EXPECT_EQ(slot.replaced(), 2u);
    EXPECT_EQ(slot.taken(), 2u);
}

TEST(Workers, LatestSlotWaitTakeTimesOutWakesOnPutAndHonorsStop) {
    LatestSlot<int> slot;
    int             out = 0;

    // nothing offered: the wait runs to its timeout
    auto t0 = Clock::now();
    EXPECT_FALSE(slot.waitTake(out, Ms(50)));
    EXPECT_GE(msSince(t0), 20.0);
    EXPECT_LT(msSince(t0), 1000.0);

    // a put from another thread wakes the waiter well before the timeout
    std::thread producer([&] {
        std::this_thread::sleep_for(Ms(30));
        slot.put(7);
    });
    t0 = Clock::now();
    EXPECT_TRUE(slot.waitTake(out, Ms(3000)));
    EXPECT_EQ(out, 7);
    EXPECT_LT(msSince(t0), 1500.0);
    producer.join();

    // stop wakes a waiter with false
    bool   got    = true;
    double waited = 0.0;
    std::thread waiter([&] {
        const auto started = Clock::now();
        got                = slot.waitTake(out, Ms(3000));
        waited             = msSince(started);
    });
    std::this_thread::sleep_for(Ms(30));
    slot.stop();
    waiter.join();
    EXPECT_FALSE(got);
    EXPECT_LT(waited, 1500.0);

    // while stopped, later waits return at once instead of timing out
    t0 = Clock::now();
    EXPECT_FALSE(slot.waitTake(out, Ms(1000)));
    EXPECT_LT(msSince(t0), 300.0);

    // resume restores waiting, including the wake-up on put
    slot.resume();
    t0 = Clock::now();
    EXPECT_FALSE(slot.waitTake(out, Ms(50)));
    EXPECT_GE(msSince(t0), 20.0);
    std::thread producer2([&] {
        std::this_thread::sleep_for(Ms(30));
        slot.put(9);
    });
    EXPECT_TRUE(slot.waitTake(out, Ms(3000)));
    EXPECT_EQ(out, 9);
    producer2.join();
}

TEST(Workers, WorkerStatsTrackCyclesOverrunsAndReset) {
    WorkerStats stats("estimation");
    stats.setPeriodTarget(10.0);
    stats.setRunning(true);

    stats.cycleDone(4.0, 100, 0, 0);
    WorkerStatsSnapshot s = stats.snapshot();
    EXPECT_EQ(s.name, "estimation");
    EXPECT_TRUE(s.running);
    EXPECT_EQ(s.cycles, 1u);
    EXPECT_DOUBLE_EQ(s.last_cycle_ms, 4.0);
    EXPECT_DOUBLE_EQ(s.mean_cycle_ms, 4.0);   // the first sample seeds the mean
    EXPECT_DOUBLE_EQ(s.max_cycle_ms, 4.0);
    EXPECT_DOUBLE_EQ(s.period_target_ms, 10.0);
    EXPECT_EQ(s.overruns, 0u);
    EXPECT_EQ(s.last_cycle_host_ms, 100);

    stats.cycleDone(12.0, 110, 3, 1);   // over the period
    s = stats.snapshot();
    EXPECT_EQ(s.cycles, 2u);
    EXPECT_DOUBLE_EQ(s.last_cycle_ms, 12.0);
    EXPECT_NEAR(s.mean_cycle_ms, 0.9 * 4.0 + 0.1 * 12.0, 1e-9);
    EXPECT_DOUBLE_EQ(s.max_cycle_ms, 12.0);
    EXPECT_EQ(s.overruns, 1u);
    EXPECT_EQ(s.dropped, 3u);
    EXPECT_EQ(s.pending, 1u);
    EXPECT_EQ(s.last_cycle_host_ms, 110);

    stats.cycleDone(6.0, 120, 3, 0);
    s = stats.snapshot();
    EXPECT_EQ(s.cycles, 3u);
    EXPECT_DOUBLE_EQ(s.last_cycle_ms, 6.0);
    EXPECT_DOUBLE_EQ(s.max_cycle_ms, 12.0);   // max is held, not the last
    EXPECT_EQ(s.overruns, 1u);
    EXPECT_EQ(s.pending, 0u);

    // no period target: nothing can overrun
    WorkerStats event_driven("field");
    event_driven.cycleDone(500.0, 1, 0, 0);
    EXPECT_EQ(event_driven.snapshot().overruns, 0u);
    EXPECT_DOUBLE_EQ(event_driven.snapshot().period_target_ms, 0.0);

    stats.resetCounters();
    s = stats.snapshot();
    EXPECT_EQ(s.name, "estimation");
    EXPECT_TRUE(s.running);
    EXPECT_DOUBLE_EQ(s.period_target_ms, 10.0);
    EXPECT_EQ(s.cycles, 0u);
    EXPECT_DOUBLE_EQ(s.last_cycle_ms, 0.0);
    EXPECT_DOUBLE_EQ(s.mean_cycle_ms, 0.0);
    EXPECT_DOUBLE_EQ(s.max_cycle_ms, 0.0);
    EXPECT_EQ(s.overruns, 0u);
    EXPECT_EQ(s.dropped, 0u);
    EXPECT_EQ(s.last_cycle_host_ms, -1);

    stats.setRunning(false);
    EXPECT_FALSE(stats.snapshot().running);
}

TEST(Workers, WorkerStatsRateWindowAndStallDecay) {
    WorkerStats stats("field");
    stats.setRunning(true);
    EXPECT_DOUBLE_EQ(stats.snapshot().rate_hz, 0.0);   // nothing measured yet

    // drive at roughly 100 Hz through the first one second window; the
    // exact call count depends on the scheduler, so the rate is checked
    // against what was actually delivered
    const auto t0         = Clock::now();
    uint64_t   calls      = 0;
    double     young_rate = -1.0;
    while (msSince(t0) < 1100.0) {
        stats.cycleDone(1.0, static_cast<int64_t>(++calls), 0, 0);
        if (young_rate < 0.0 && msSince(t0) > 150.0) {
            young_rate = stats.snapshot().rate_hz;   // before any window closed
        }
        std::this_thread::sleep_for(Ms(10));
    }
    const double delivered_hz = calls / (msSince(t0) / 1000.0);
    const WorkerStatsSnapshot live = stats.snapshot();
    RecordProperty("delivered_hz", static_cast<int>(delivered_hz));
    RecordProperty("live_rate_hz", static_cast<int>(live.rate_hz));
    EXPECT_GT(young_rate, 0.0);
    EXPECT_GT(live.rate_hz, 0.0);
    EXPECT_GT(live.rate_hz, 0.4 * delivered_hz);
    EXPECT_LT(live.rate_hz, 2.0 * delivered_hz);
    EXPECT_EQ(live.cycles, calls);

    // no cycles for over two seconds: the advertised rate collapses
    // instead of holding the last good window, and keeps falling
    std::this_thread::sleep_for(Ms(2200));
    const WorkerStatsSnapshot stalled = stats.snapshot();
    RecordProperty("stalled_rate_hz_x100", static_cast<int>(stalled.rate_hz * 100.0));
    EXPECT_LT(stalled.rate_hz, 0.25 * live.rate_hz);
    EXPECT_LT(stalled.rate_hz, 15.0);
    std::this_thread::sleep_for(Ms(300));
    EXPECT_LE(stats.snapshot().rate_hz, stalled.rate_hz);
    EXPECT_EQ(stats.snapshot().cycles, calls);
}

// ---- workers on the synthetic rig ------------------------------------------

TEST(Workers, StopAndResetWakeAWorkerWaitingForALongPeriod) {
    // No detector or hardware work: a two-second period makes an
    // uninterruptible scheduling sleep visible without timing CPU work.
    const char* xml = R"(
        <System>
            <Loop rate_hz="0.5"/>
            <Pipeline>
                <CommandCollection type="noop"/>
                <Localization><Estimator type="noop"/></Localization>
                <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
                <TargetResolution type="noop"/>
                <Publishing type="noop"/>
            </Pipeline>
        </System>)";
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;
    ASSERT_TRUE(system->start(err)) << err;
    ASSERT_TRUE(waitFor(Ms(1000), [&] {
        return system->estimationStats().cycles >= 1 && system->fieldStats().cycles >= 1;
    }));
    EXPECT_DOUBLE_EQ(system->estimationStats().period_target_ms, 2000.0);

    auto before = Clock::now();
    system->stop();
    const double stop_ms = msSince(before);
    RecordProperty("low_rate_stop_ms", static_cast<int>(stop_ms));
    EXPECT_LT(stop_ms, 1000.0);   // generous scheduler margin, below the period
    EXPECT_FALSE(system->running());
    EXPECT_FALSE(system->estimationStats().running);
    EXPECT_FALSE(system->fieldStats().running);

    const uint64_t prior_cycles = system->estimationStats().cycles;
    ASSERT_TRUE(system->start(err)) << err;
    ASSERT_TRUE(waitFor(Ms(1000), [&] {
        return system->estimationStats().cycles > prior_cycles;
    }));
    before = Clock::now();
    ASSERT_NO_THROW(system->reset());
    const double reset_ms = msSince(before);
    RecordProperty("low_rate_reset_ms", static_cast<int>(reset_ms));
    EXPECT_LT(reset_ms, 1000.0);
    EXPECT_EQ(system->resetCount(), 1u);
    EXPECT_TRUE(system->running());
    ASSERT_TRUE(waitFor(Ms(1000), [&] {
        return system->estimationStats().cycles >= 1 && system->fieldStats().cycles >= 1;
    }));

    // Reset restarts both threads and must not leave the stop predicate
    // latched, or reintroduce an uninterruptible wait in the restart path.
    before = Clock::now();
    system->stop();
    EXPECT_LT(msSince(before), 1000.0);
    EXPECT_FALSE(system->running());
}

TEST(Workers, WorkersRunPublishAndRestart) {
    Rig f;
    ASSERT_NE(f.system, nullptr);
    System& s = *f.system;
    EXPECT_FALSE(s.running());
    ASSERT_TRUE(f.start());
    EXPECT_TRUE(s.running());
    EXPECT_TRUE(s.estimationStats().running);
    EXPECT_TRUE(s.fieldStats().running);
    ASSERT_TRUE(f.start());   // idempotent

    ASSERT_TRUE(waitFor(Ms(3000), [&] { return s.cycle() >= 100; })) << s.cycle();
    EXPECT_TRUE(waitFor(Ms(1000), [&] { return s.fieldStats().cycles > 0; }));
    EXPECT_NEAR(s.estimationStats().period_target_ms, 10.0, 1e-9);

    const uint64_t publication = s.robotFeed()->publication();
    EXPECT_TRUE(waitFor(Ms(1000), [&] { return s.robotFeed()->publication() > publication; }));

    // the field worker binds every observation set to the exact frame it
    // decoded, pixels included
    std::shared_ptr<const DetectionFrameSnapshot> frame;
    ASSERT_TRUE(waitFor(Ms(3000), [&] {
        frame = cameraFrame(s);
        return frame != nullptr;
    }));
    EXPECT_EQ(frame->camera, kCamera);
    ASSERT_TRUE(frame->has_observations);
    EXPECT_EQ(frame->observations.camera, kCamera);
    EXPECT_EQ(frame->frame_epoch, frame->observations.frame_epoch);
    EXPECT_EQ(frame->frame_sequence, frame->observations.frame_sequence);
    EXPECT_EQ(frame->width_px, 640);
    EXPECT_EQ(frame->height_px, 480);
    ASSERT_NE(frame->y8, nullptr);
    EXPECT_EQ(frame->y8->size(), 640u * 480u);
    EXPECT_NE(frame->intrinsics, nullptr);
    EXPECT_GT(frame->field_invocation, 0u);
    EXPECT_EQ(frame->processedAt.domain, ClockDomain::kHost);

    const uint64_t invocation = s.fieldSnapshot()->invocation;
    EXPECT_GT(invocation, 0u);
    EXPECT_TRUE(waitFor(Ms(1000), [&] { return s.fieldSnapshot()->invocation > invocation; }));

    // every configured sensor is visible in the health snapshot
    const std::shared_ptr<const SourceHealthSnapshot> health = s.sourceHealth();
    ASSERT_NE(health, nullptr);
    EXPECT_GT(health->cycle, 0u);
    EXPECT_EQ(s.sensorCatalog().size(), 6u);
    for (const SensorCatalog::Entry& entry : s.sensorCatalog().entries()) {
        EXPECT_NE(findEntry(*health, "sensor", entry.id.value), nullptr) << entry.id.value;
    }

    s.stop();
    EXPECT_FALSE(s.running());
    EXPECT_FALSE(s.estimationStats().running);
    EXPECT_FALSE(s.fieldStats().running);
    const uint64_t stopped_cycle      = s.cycle();
    const uint64_t stopped_invocation = s.fieldSnapshot()->invocation;
    std::this_thread::sleep_for(Ms(100));
    EXPECT_EQ(s.cycle(), stopped_cycle);
    EXPECT_EQ(s.fieldSnapshot()->invocation, stopped_invocation);
    s.stop();   // idempotent

    // a stopped system restarts and picks up where its counters were
    ASSERT_TRUE(f.start());
    EXPECT_TRUE(s.running());
    EXPECT_TRUE(waitFor(Ms(2000), [&] { return s.cycle() >= stopped_cycle + 20; }));
    EXPECT_TRUE(waitFor(Ms(2000), [&] {
        return s.fieldSnapshot()->invocation > stopped_invocation;
    }));
    s.stop();
    EXPECT_FALSE(s.running());
    EXPECT_EQ(s.resetCount(), 0u);
}

TEST(Workers, SlowFieldWorkNeverStallsEstimation) {
    RigOptions options;
    options.detector_type = "slow_apriltag_detector";
    Rig f(options);
    ASSERT_NE(f.system, nullptr);
    System& s = *f.system;
    ASSERT_TRUE(f.start());

    // sample localization publications while the run covers at least one
    // full rate window and 150 estimation cycles
    const auto            t0 = Clock::now();
    std::vector<uint64_t> publications;
    while (msSince(t0) < 2000.0 || (s.cycle() < 150 && msSince(t0) < 3500.0)) {
        publications.push_back(s.robotFeed()->publication());
        std::this_thread::sleep_for(Ms(50));
    }
    const double              elapsed_ms = msSince(t0);
    const WorkerStatsSnapshot estimation = s.estimationStats();
    const WorkerStatsSnapshot field      = s.fieldStats();
    s.stop();
    RecordProperty("elapsed_ms", static_cast<int>(elapsed_ms));
    RecordProperty("estimation_rate_hz", static_cast<int>(estimation.rate_hz));
    RecordProperty("estimation_cycles", static_cast<int>(estimation.cycles));
    RecordProperty("field_cycles", static_cast<int>(field.cycles));
    RecordProperty("field_dropped", static_cast<int>(field.dropped));

    EXPECT_GE(estimation.cycles, 150u) << elapsed_ms;
    EXPECT_GE(estimation.rate_hz, 60.0);
    EXPECT_LE(estimation.rate_hz, 130.0);

    // every field cycle held a frame for 150 ms, so the count is bounded by
    // the run length while the handoff displaced far more than it consumed
    EXPECT_GT(field.cycles, 0u);
    EXPECT_LE(field.cycles, static_cast<uint64_t>(elapsed_ms / 150.0) + 2);
    EXPECT_GT(field.dropped, 0u);
    EXPECT_GT(field.dropped, field.cycles);
    EXPECT_GT(f.detector->left.load(), 0u);
    EXPECT_FALSE(f.detector->inFlight());   // stop joined the last detection

    ASSERT_GE(publications.size(), 2u);
    for (std::size_t i = 1; i < publications.size(); ++i) {
        EXPECT_GE(publications[i], publications[i - 1]);
    }
    EXPECT_GT(publications.back(), publications.front() + 100);
}

TEST(Workers, ResetWhileRunningRestartsStagesOnce) {
    Rig f;
    ASSERT_NE(f.system, nullptr);
    System& s = *f.system;
    ASSERT_TRUE(f.start());
    ASSERT_TRUE(waitFor(Ms(3000), [&] { return s.cycle() >= 80; }));
    std::shared_ptr<const DetectionFrameSnapshot> before;
    ASSERT_TRUE(waitFor(Ms(3000), [&] {
        before = cameraFrame(s);
        return before != nullptr;
    }));
    const uint64_t epoch_before      = s.robotFeed()->latest().odometry_epoch;
    const uint64_t invocation_before = s.fieldSnapshot()->invocation;
    const uint64_t cycles_before     = s.cycle();
    EXPECT_GT(invocation_before, 0u);
    EXPECT_EQ(s.resetCount(), 0u);

    s.reset();

    EXPECT_EQ(s.resetCount(), 1u);
    EXPECT_TRUE(s.running());
    EXPECT_TRUE(s.detectionFrames().empty());
    const uint64_t invocation_after = s.fieldSnapshot()->invocation;
    EXPECT_LT(invocation_after, invocation_before);   // the field stage restarted
    EXPECT_LE(invocation_after, 25u);
    EXPECT_GE(s.cycle(), cycles_before);   // the cycle counter is kept

    // odometry continues in exactly the next epoch, once
    EXPECT_TRUE(waitFor(Ms(2000), [&] {
        return s.robotFeed()->latest().odometry_epoch == epoch_before + 1;
    })) << s.robotFeed()->latest().odometry_epoch;
    EXPECT_TRUE(waitFor(Ms(2000), [&] { return s.cycle() >= cycles_before + 20; }));
    EXPECT_TRUE(waitFor(Ms(2000), [&] { return s.fieldSnapshot()->invocation >= 10; }));

    // detection frames come back from the camera's new epoch; nothing from
    // before the reset survives
    std::shared_ptr<const DetectionFrameSnapshot> after;
    ASSERT_TRUE(waitFor(Ms(2000), [&] {
        after = cameraFrame(s);
        return after != nullptr;
    }));
    EXPECT_EQ(after->frame_epoch, before->frame_epoch + 1);
    EXPECT_GT(after->field_invocation, 0u);
    EXPECT_EQ(s.robotFeed()->latest().odometry_epoch, epoch_before + 1);
    EXPECT_EQ(s.resetCount(), 1u);
    EXPECT_TRUE(s.running());

    s.stop();
    EXPECT_FALSE(s.running());
}

TEST(Workers, DeadCameraIsReportedWhileWorkersKeepCycling) {
    RigOptions options;
    options.camera_resource = "failing_camera_device";
    options.extra_resources = R"(<Resource id="failing_camera_device" type="failing_camera"/>)";
    Rig f(options);
    ASSERT_NE(f.system, nullptr);
    System& s = *f.system;
    ASSERT_TRUE(f.start());

    // frames flow first
    ASSERT_TRUE(waitFor(Ms(3000), [&] {
        const auto health = s.sourceHealth();
        if (health == nullptr) {
            return false;
        }
        const SourceHealthEntry* e = findEntry(*health, "sensor", kCamera.value);
        return e != nullptr && e->state == SourceState::kValid && e->has_sample;
    }));
    EXPECT_TRUE(waitFor(Ms(2000), [&] { return s.fieldStats().cycles > 0; }));

    // the device dies on its next poll
    f.camera->die.store(true);
    ASSERT_TRUE(waitFor(Ms(2000), [&] { return !f.camera->alive.load(); }));

    SourceHealthEntry sensor;
    ASSERT_TRUE(waitFor(Ms(2000), [&] {
        const auto health = s.sourceHealth();
        if (health == nullptr) {
            return false;
        }
        const SourceHealthEntry* e = findEntry(*health, "sensor", kCamera.value);
        if (e == nullptr ||
            (e->state != SourceState::kUnavailable && e->state != SourceState::kFault)) {
            return false;
        }
        sensor = *e;
        return true;
    }));
    EXPECT_EQ(sensor.state, SourceState::kUnavailable);
    EXPECT_FALSE(sensor.diagnostic.empty());
    EXPECT_NE(sensor.diagnostic.find("unplugged"), std::string::npos) << sensor.diagnostic;

    // the resource output says the same thing
    const auto               health = s.sourceHealth();
    const SourceHealthEntry* output =
        findEntry(*health, "resource", "failing_camera_device.frame");
    ASSERT_NE(output, nullptr);
    EXPECT_EQ(output->state, SourceState::kUnavailable);
    EXPECT_FALSE(output->diagnostic.empty());

    // neither worker stalls on a dead source
    const uint64_t cycles = s.cycle();
    const uint64_t field  = s.fieldStats().cycles;
    EXPECT_TRUE(waitFor(Ms(2000), [&] {
        return s.cycle() >= cycles + 20 && s.fieldStats().cycles >= field + 10;
    }));
    EXPECT_TRUE(s.running());
    s.stop();
    EXPECT_FALSE(s.running());
}

TEST(Workers, DestroyingARunningSystemJoinsInFlightWork) {
    RigOptions options;
    options.detector_type = "slow_apriltag_detector";
    {
        Rig f(options);
        ASSERT_NE(f.system, nullptr);
        ASSERT_TRUE(f.start());
        ASSERT_TRUE(waitFor(Ms(3000), [&] { return f.detector->inFlight(); }));

        // destroyed through the owner while a detection sleeps inside the
        // field worker: the worker finishes its cycle and is joined
        const auto t0 = Clock::now();
        f.system.reset();
        EXPECT_LT(msSince(t0), 2000.0);
        EXPECT_GT(f.detector->left.load(), 0u);
        EXPECT_EQ(f.detector->entered.load(), f.detector->left.load());
    }
    {
        Rig f(options);
        ASSERT_NE(f.system, nullptr);
        ASSERT_TRUE(f.start());
        EXPECT_TRUE(waitFor(Ms(2000), [&] { return f.system->cycle() >= 5; }));
        f.system->stop();
        EXPECT_FALSE(f.system->running());
        EXPECT_EQ(f.detector->entered.load(), f.detector->left.load());
        const auto t0 = Clock::now();
        f.system.reset();
        EXPECT_LT(msSince(t0), 2000.0);
    }
}

TEST(Workers, SnapshotReadersAreMonotonicAndStayValid) {
    Rig f;
    ASSERT_NE(f.system, nullptr);
    System& s = *f.system;
    ASSERT_TRUE(f.start());
    ASSERT_TRUE(waitFor(Ms(3000), [&] {
        return s.reportingSnapshot() != nullptr && s.fieldSnapshot()->invocation > 0;
    }));

    const std::shared_ptr<const FieldSnapshot>     held_field     = s.fieldSnapshot();
    const std::shared_ptr<const ReportingSnapshot> held_reporting = s.reportingSnapshot();
    const uint64_t                                 held_invocation = held_field->invocation;
    const uint64_t                                 held_cycle      = held_reporting->cycle;

    uint64_t last_invocation = 0;
    uint64_t last_cycle      = 0;
    for (int i = 0; i < 200; ++i) {
        const std::shared_ptr<const FieldSnapshot> field = s.fieldSnapshot();
        ASSERT_NE(field, nullptr);
        EXPECT_GE(field->invocation, last_invocation);
        last_invocation = field->invocation;
        const std::shared_ptr<const ReportingSnapshot> reporting = s.reportingSnapshot();
        ASSERT_NE(reporting, nullptr);
        EXPECT_GE(reporting->cycle, last_cycle);
        last_cycle = reporting->cycle;
        if (i % 4 == 3) {
            std::this_thread::sleep_for(Ms(1));   // spread the reads over real cycles
        }
    }
    EXPECT_GT(last_invocation, held_invocation);
    EXPECT_GT(last_cycle, held_cycle);

    // snapshots are immutable values: the ones held across many later
    // cycles still read exactly as they were published
    std::this_thread::sleep_for(Ms(500));
    EXPECT_EQ(held_field->invocation, held_invocation);
    EXPECT_EQ(held_field->at.domain, ClockDomain::kHost);
    for (const auto& kv : held_field->observations) {
        EXPECT_FALSE(kv.first.value.empty());
    }
    for (const auto& kv : held_field->field.objects) {
        EXPECT_FALSE(kv.first.value.empty());
    }
    EXPECT_EQ(held_reporting->cycle, held_cycle);
    EXPECT_EQ(held_reporting->at.domain, ClockDomain::kHost);
    EXPECT_FALSE(held_reporting->target.active);

    s.stop();
}

TEST(Workers, InlineStepStillSplitsDiagnosticsByWorker) {
    Rig f;
    ASSERT_NE(f.system, nullptr);
    System& s = *f.system;
    for (int i = 1; i <= 30; ++i) {
        s.step(hostTime(10 * i));
    }
    EXPECT_FALSE(s.running());
    EXPECT_EQ(s.cycle(), 30u);
    EXPECT_EQ(s.diagnostics().cycles, 30u);
    EXPECT_EQ(s.fieldDiagnostics().cycles, 30u);

    // estimation labels on one side, field estimation on the other
    EXPECT_EQ(s.diagnostics().functions.count("Localization/planar_motion_integrator"), 1u);
    EXPECT_EQ(s.diagnostics().functions.count("CommandCollection/noop"), 1u);
    EXPECT_EQ(s.fieldDiagnostics().functions.count("WorldEstimation/goals"), 1u);
    for (const auto& kv : s.diagnostics().functions) {
        EXPECT_NE(kv.first.rfind("WorldEstimation/", 0), 0u) << kv.first;
    }
    for (const auto& kv : s.fieldDiagnostics().functions) {
        EXPECT_NE(kv.first.rfind("Localization/", 0), 0u) << kv.first;
    }
    EXPECT_EQ(s.diagnostics().functions.at("Localization/planar_motion_integrator").runs, 30u);
    EXPECT_EQ(s.fieldDiagnostics().functions.at("WorldEstimation/goals").runs, 30u);

    // the published snapshots carry the same cycle
    EXPECT_EQ(s.fieldSnapshot()->invocation, 30u);
    ASSERT_NE(s.reportingSnapshot(), nullptr);
    EXPECT_EQ(s.reportingSnapshot()->cycle, 30u);
    const std::shared_ptr<const DiagnosticsSnapshot> d = s.diagnosticsSnapshot();
    ASSERT_NE(d, nullptr);
    EXPECT_EQ(d->estimation.cycles, 30u);
    EXPECT_EQ(d->field.cycles, 30u);
    EXPECT_EQ(s.estimationStats().cycles, 0u);   // inline mode runs no worker
}
