// inspection_document.cpp

#include "inspection/inspection_document.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

#include "common/link_documents.h"
#include "config/field_map.h"
#include "impl/publishing/field_documents.h"
#include "impl/resources/tag_detectors.h"
#include "inspection/json_writer.h"
#include "math/angles.h"
#include "payloads/camera_frames.h"
#include "resources/camera.h"
#include "resources/robot_frames.h"
#include "runtime/brain_profile_builder.h"
#include "runtime/pico_control_ref.h"
#include "state/attitude.h"

namespace navigatr
{

namespace
{

const char* clockName(ClockDomain d) {
    switch (d) {
    case ClockDomain::kUnset: return "unset";
    case ClockDomain::kDevice: return "device";
    case ClockDomain::kHost: return "host";
    }
    return "unknown";
}

std::string hex64(uint64_t v) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
    return buf;
}

// host_ms or null when the time is not on the host clock
void hostMs(JsonWriter& w, const char* key, MonotonicTime t) {
    w.key(key);
    if (t.domain == ClockDomain::kHost) {
        w.value(static_cast<int64_t>(t.ms));
    } else {
        w.null();
    }
}

// age in ms against now, null when not comparable
void ageMs(JsonWriter& w, const char* key, MonotonicTime t, MonotonicTime now) {
    w.key(key);
    if (t.domain == ClockDomain::kHost && now.domain == ClockDomain::kHost) {
        w.value(static_cast<int64_t>(now.ms - t.ms));
    } else {
        w.null();
    }
}

void stamp(JsonWriter& w, const char* key, MonotonicTime t) {
    w.key(key);
    w.beginObject();
    w.field("clock", clockName(t.domain));
    w.key("ms");
    if (t.isSet()) {
        w.value(static_cast<int64_t>(t.ms));
    } else {
        w.null();
    }
    w.endObject();
}

void pose2(JsonWriter& w, const char* key, const Pose2D& p) {
    w.key(key);
    w.beginObject();
    w.field("x_m", p.x_m);
    w.field("y_m", p.y_m);
    w.field("heading_deg", radToDeg(p.heading_rad));
    w.endObject();
}

void transform3(JsonWriter& w, const char* key, const Transform3& T) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    eulerFromRotation(T.R, roll, pitch, yaw);
    w.key(key);
    w.beginObject();
    w.field("x_m", T.x_m);
    w.field("y_m", T.y_m);
    w.field("z_m", T.z_m);
    w.field("roll_deg", radToDeg(roll));
    w.field("pitch_deg", radToDeg(pitch));
    w.field("yaw_deg", radToDeg(yaw));
    w.key("R");
    w.beginArray();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            w.value(T.R.m[r][c]);
        }
    }
    w.endArray();
    w.endObject();
}

void intrinsics(JsonWriter& w, const char* key, const CameraIntrinsics* K) {
    w.key(key);
    if (K == nullptr) {
        w.null();
        return;
    }
    w.beginObject();
    w.field("model", K->model);
    w.field("width_px", K->calibrated_width_px);
    w.field("height_px", K->calibrated_height_px);
    w.field("fx_px", K->fx_px);
    w.field("fy_px", K->fy_px);
    w.field("cx_px", K->cx_px);
    w.field("cy_px", K->cy_px);
    w.field("k1", K->k1);
    w.field("k2", K->k2);
    w.field("p1", K->p1);
    w.field("p2", K->p2);
    w.field("k3", K->k3);
    w.field("rms_reprojection_px", K->rms_reprojection_px);
    w.endObject();
}

void attitude(JsonWriter& w, const char* key, const Attitude& a, MonotonicTime now) {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    attitudeEuler(a, roll, pitch, yaw);
    w.key(key);
    w.beginObject();
    w.field("valid", a.valid);
    w.field("assumed_level", a.assumed_level);
    w.field("reference", a.reference);
    w.field("source", a.source);
    w.field("epoch", a.epoch);
    w.field("quality", a.quality);
    hostMs(w, "measured_at_host_ms", a.measuredAt);
    stamp(w, "measured_at_source", a.measuredAtSource);
    ageMs(w, "age_ms", a.measuredAt, now);
    w.field("roll_deg", radToDeg(roll));
    w.field("pitch_deg", radToDeg(pitch));
    w.field("yaw_deg", radToDeg(yaw));
    w.key("q_wxyz");
    w.beginArray();
    w.value(a.q_reference_body.w);
    w.value(a.q_reference_body.x);
    w.value(a.q_reference_body.y);
    w.value(a.q_reference_body.z);
    w.endArray();
    w.endObject();
}

// ---- brain link, profile and Pico names --------------------------------

// A profile or map id as the Brain prints it, null for 0.
void wireId32(JsonWriter& w, const char* key, uint32_t id) {
    w.key(key);
    if (id == 0) {
        w.null();
        return;
    }
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%08x", static_cast<unsigned>(id));
    w.value(buf);
}

void wireId16(JsonWriter& w, const char* key, uint16_t id) {
    w.key(key);
    if (id == 0) {
        w.null();
    } else {
        w.value(static_cast<int64_t>(id));
    }
}

const char* profileStateName(uint8_t s) {
    switch (s) {
    case gatr2::kProfileNone: return "none";
    case gatr2::kProfileApplying: return "applying";
    case gatr2::kProfileApplied: return "applied";
    case gatr2::kProfileRejected: return "rejected";
    default: return "unknown";
    }
}

const char* calibrationName(uint8_t s) {
    return s <= gatr2::kCalibrationFailed ? toString(static_cast<BiasCalibration>(s)) : "unknown";
}

const char* imuSourceName(uint8_t s) {
    switch (s) {
    case gatr2::kImuSourceNone: return "none";
    case gatr2::kImuSourcePico: return "pico";
    case gatr2::kImuSourceBrainVex: return "brain_vex";
    default: return "unknown";
    }
}

const char* controlActionName(uint8_t a) {
    switch (a) {
    case gatr2::kControlRecalibrate: return "recalibrate";
    case gatr2::kControlReinitialize: return "reinitialize";
    case gatr2::kControlReinitImu: return "reinit_imu";
    case gatr2::kControlRestartAcquisition: return "restart_acquisition";
    default: return "unknown";
    }
}

const char* resultName(uint8_t r) {
    switch (r) {
    case gatr2::kResultOk: return "ok";
    case gatr2::kResultPending: return "pending";
    case gatr2::kResultNotReady: return "not_ready";
    case gatr2::kResultNotStationary: return "not_stationary";
    case gatr2::kResultFailed: return "failed";
    case gatr2::kResultInvalidArgument: return "invalid_argument";
    default: return "other";
    }
}

const char* controlDetailName(uint8_t d) {
    switch (d) {
    case gatr2::kControlDetailNone: return "none";
    case gatr2::kControlDetailPicoLink: return "pico_link";
    case gatr2::kControlDetailImuAbsent: return "imu_absent";
    case gatr2::kControlDetailImuUnused: return "imu_unused";
    case gatr2::kControlDetailPicoRefused: return "pico_refused";
    case gatr2::kControlDetailTimedOut: return "timed_out";
    case gatr2::kControlDetailCalibration: return "calibration";
    default: return "unknown";
    }
}

const char* pathModeName(uint8_t m) {
    switch (m) {
    case gatr2::kPathNone: return "none";
    case gatr2::kPathDirect: return "direct";
    case gatr2::kPathAvoiding: return "avoiding";
    default: return "unknown";
    }
}

const char* picoImuStateName(uint8_t s) {
    switch (s) {
    case gatr2::kPicoImuDisabled: return "disabled";
    case gatr2::kPicoImuInitializing: return "initializing";
    case gatr2::kPicoImuAligning: return "aligning";
    case gatr2::kPicoImuReady: return "ready";
    case gatr2::kPicoImuRetrying: return "retrying";
    case gatr2::kPicoImuFailed: return "failed";
    default: return "unknown";
    }
}

const char* picoImuReasonName(uint8_t r) {
    switch (r) {
    case gatr2::kPicoImuReasonNone: return "none";
    case gatr2::kPicoImuReasonNoResponse: return "no_response";
    case gatr2::kPicoImuReasonBoot: return "boot";
    case gatr2::kPicoImuReasonFeatures: return "features";
    case gatr2::kPicoImuReasonStream: return "stream";
    default: return "unknown";
    }
}

const char* picoFirmwareName(uint8_t f) {
    switch (f) {
    case gatr2::kPicoFirmwareBno08x: return "bno08x";
    case gatr2::kPicoFirmwareAsm330: return "asm330";
    default: return "unknown";
    }
}

const char* picoOpName(uint8_t op) {
    switch (op) {
    case 0: return "none";
    case gatr2::kPicoOpConfigure: return "configure";
    case gatr2::kPicoOpReinitImu: return "reinit_imu";
    case gatr2::kPicoOpRestartAcquisition: return "restart_acquisition";
    default: return "unknown";
    }
}

const char* picoCommandStatusName(uint8_t s) {
    switch (s) {
    case gatr2::kPicoCommandNone: return "none";
    case gatr2::kPicoCommandRunning: return "running";
    case gatr2::kPicoCommandCompleted: return "completed";
    case gatr2::kPicoCommandFailed: return "failed";
    default: return "unknown";
    }
}

const char* picoDetailName(uint8_t d) {
    switch (d) {
    case gatr2::kPicoDetailNone: return "none";
    case gatr2::kPicoDetailWrongTarget: return "wrong_target";
    case gatr2::kPicoDetailUnknownOp: return "unknown_op";
    case gatr2::kPicoDetailBadBody: return "bad_body";
    case gatr2::kPicoDetailImuAbsent: return "imu_absent";
    case gatr2::kPicoDetailImuDisabled: return "imu_disabled";
    case gatr2::kPicoDetailNoSuchPort: return "no_such_port";
    default: return "unknown";
    }
}

void collisionBox(JsonWriter& w, const char* key, const CollisionBoxDecl& b) {
    w.key(key);
    if (!b.declared) {
        w.null();
        return;
    }
    w.beginObject();
    w.field("x_m", b.center.x_m);
    w.field("y_m", b.center.y_m);
    w.field("yaw_deg", radToDeg(b.center.heading_rad));
    w.field("size_x_m", b.size_x_m);
    w.field("size_y_m", b.size_y_m);
    w.field("note", b.note);
    w.endObject();
}

void stillness(JsonWriter& w, const char* key, const StillnessStatus& s) {
    w.key(key);
    w.beginObject();
    w.field("monitored", s.monitored);
    w.field("stationary", s.stationary);
    w.field("calibration", toString(s.calibration));
    w.field("reason", s.reason);
    w.field("progress_ms", static_cast<int64_t>(s.progress_ms));
    w.field("window_ms", static_cast<int64_t>(s.window_ms));
    w.field("windows", s.windows);
    w.field("restarts", s.restarts);
    w.field("attempts", static_cast<uint64_t>(s.attempts));
    w.field("steps", static_cast<uint64_t>(s.steps));
    w.key("bias_dps");
    if (s.has_bias) {
        w.value(radToDeg(s.bias_rad_s));
    } else {
        w.null();
    }
    w.endObject();
}

// The running profile as the Pi applied it: SI units, degrees.
void runningProfile(JsonWriter& w, const ProfileBinding& b) {
    const gatr2::RobotProfileDoc& p = b.profile;
    w.beginObject();
    wireId32(w, "id", b.id);
    w.field("generation", b.generation);
    w.field("topology", profileTopologyName(p.topology));
    w.field("summary", b.summary);
    w.key("wheels");
    w.beginArray();
    for (uint8_t i = 0; i < p.wheel_count && i < gatr2::kProfileMaxWheels; ++i) {
        const gatr2::ProfileWheel& wh = p.wheels[i];
        w.beginObject();
        w.field("port", static_cast<int64_t>(wh.encoder_port));
        w.field("sensor_id", i < b.encoders.size() ? b.encoders[i].value : std::string());
        w.field("counts_per_rev", static_cast<uint64_t>(wh.counts_per_rev));
        w.field("gear", wh.gear_micro / static_cast<double>(gatr2::kUnitMicro));
        w.field("reversed", (wh.flags & gatr2::kWheelReversed) != 0);
        w.field("radius_m", wh.radius_um * 1e-6);
        w.field("x_m", wh.x_um * 1e-6);
        w.field("y_m", wh.y_um * 1e-6);
        w.field("angle_deg", wh.angle_mdeg / 1000.0);
        w.field("travel_scale", wh.travel_scale_ppm / static_cast<double>(gatr2::kUnitMicro));
        w.endObject();
    }
    w.endArray();
    w.key("imu");
    w.beginObject();
    w.field("source", imuSourceName(p.imu_source));
    w.field("port", static_cast<int64_t>(p.imu_port));
    w.field("vex_smart_port", static_cast<int64_t>(p.vex_smart_port));
    w.field("inverted", (p.imu_flags & gatr2::kImuInvert) != 0);
    w.field("bias_function", b.bias_function);
    w.endObject();
    w.key("footprint");
    w.beginObject();
    w.field("front_m", p.footprint_front_um * 1e-6);
    w.field("back_m", p.footprint_back_um * 1e-6);
    w.field("left_m", p.footprint_left_um * 1e-6);
    w.field("right_m", p.footprint_right_um * 1e-6);
    w.endObject();
    // 0 = the Pi default
    w.key("calibration");
    w.beginObject();
    w.field("window_ms", static_cast<int64_t>(p.calibration_window_ms));
    w.field("still_rate_dps", p.still_rate_cdps / 100.0);
    w.field("still_travel_m", p.still_travel_um * 1e-6);
    w.endObject();
    w.endObject();
}

// The state block a Brain GET_STATE would read now.
void brainState(JsonWriter& w, const gatr2::BrainState& s) {
    const auto bit = [](uint8_t v, uint8_t b) { return (v & b) != 0; };
    w.key("state");
    w.beginObject();
    w.key("flags");
    w.beginObject();
    w.field("pose_valid", bit(s.robot_flags, gatr2::kRobotPoseValid));
    w.field("localized", bit(s.robot_flags, gatr2::kRobotLocalized));
    w.field("age_known", bit(s.robot_flags, gatr2::kRobotAgeKnown));
    w.field("anchor_command", bit(s.robot_flags, gatr2::kRobotAnchorCommand));
    w.field("anchor_configured", bit(s.robot_flags, gatr2::kRobotAnchorConfigured));
    w.endObject();
    w.key("health");
    w.beginObject();
    w.field("encoders_fresh", bit(s.health, gatr2::kHealthEncodersFresh));
    w.field("gyro_fresh", bit(s.health, gatr2::kHealthGyroFresh));
    w.field("vision_alive", bit(s.health, gatr2::kHealthVisionAlive));
    w.field("bias_calibrated", bit(s.health, gatr2::kHealthBiasCalibrated));
    w.field("pico_link", bit(s.health, gatr2::kHealthPicoLink));
    w.field("imu_initializing", bit(s.health, gatr2::kHealthImuInitializing));
    w.field("imu_failed", bit(s.health, gatr2::kHealthImuFailed));
    w.field("stationary", bit(s.health, gatr2::kHealthStationary));
    w.endObject();
    w.field("calibration", calibrationName(s.calibration));
    w.field("profile_state", profileStateName(s.profile_state));
    wireId32(w, "profile_id", s.profile_id);
    wireId32(w, "map_id", s.map_id);
    w.field("estimate_id", static_cast<uint64_t>(s.estimate_id));
    w.field("odometry_epoch", static_cast<uint64_t>(s.odometry_epoch));
    w.field("anchor_revision", static_cast<uint64_t>(s.anchor_revision));
    w.endObject();
}

// The Pico link of a pico_telemetry resource (or a stand-in), any thread.
void picoLink(JsonWriter& w, const std::string& resource_id, const PicoLinkState& l,
              MonotonicTime now) {
    w.beginObject();
    w.field("resource_id", resource_id);
    w.field("frames_fresh", l.frames_fresh);
    // false with fresh frames: v1 firmware, no identity or commands
    w.field("identity", l.identity);
    w.field("boot_id", static_cast<int64_t>(l.boot_id));
    w.field("acq_epoch", static_cast<int64_t>(l.acq_epoch));
    w.field("imu_epoch", static_cast<int64_t>(l.imu_epoch));
    w.field("reboots", l.reboots);
    w.field("restarts", l.restarts);
    w.field("imu_restarts", l.imu_restarts);
    ageMs(w, "last_frame_age_ms", l.last_frame, now);
    w.field("status_known", l.status_known);
    if (l.status_known) {
        const gatr2::PicoStatus& s = l.status;
        ageMs(w, "last_status_age_ms", l.last_status, now);
        w.field("firmware", picoFirmwareName(s.firmware));
        w.field("uptime_ms", static_cast<uint64_t>(s.uptime_ms));
        w.key("imu");
        w.beginObject();
        w.field("enabled", (s.flags & gatr2::kPicoImuEnabled) != 0);
        w.field("state", picoImuStateName(s.imu_state));
        w.field("reason", picoImuReasonName(s.imu_reason));
        w.field("attempts", static_cast<int64_t>(s.imu_attempts));
        w.endObject();
        w.key("last_command");
        w.beginObject();
        w.field("request_id", static_cast<int64_t>(s.last_request_id));
        w.field("op", picoOpName(s.last_op));
        w.field("status", picoCommandStatusName(s.last_status));
        w.field("detail", picoDetailName(s.last_detail));
        w.endObject();
    } else {
        w.fieldNull("last_status_age_ms");
        w.fieldNull("firmware");
        w.fieldNull("uptime_ms");
        w.fieldNull("imu");
        w.fieldNull("last_command");
    }
    w.endObject();
}

// The Pico link inspection shows: the one the brain_link CommandCollection
// names, else the first resource that is one.
std::shared_ptr<PicoControl> findPico(const System& system, std::string& resource_id) {
    for (const ResourceStore::Record& r : system.resources().records()) {
        std::string                  err;
        std::shared_ptr<PicoControl> pico = requirePicoControl(system.resources(), r.id, err);
        if (pico == nullptr) {
            continue;
        }
        if (system.picoControl() == nullptr || pico == system.picoControl()) {
            resource_id = r.id.value;
            return pico;
        }
    }
    resource_id.clear();
    return system.picoControl();
}

// The mounting transform of a frame from any configured robot frame map.
bool findRobotFrame(const System& system, const FrameId& frame, Transform3& out) {
    if (frame.empty()) {
        return false;
    }
    for (const ResourceStore::Record& r : system.resources().records()) {
        std::string err;
        auto        map = r.value.require<const RobotFrameMap>(err);
        if (map == nullptr) {
            continue;
        }
        if (const Transform3* T = map->find(frame)) {
            out = *T;
            return true;
        }
    }
    return false;
}

void workerStats(JsonWriter& w, const char* key, const WorkerStatsSnapshot& s) {
    w.key(key);
    w.beginObject();
    w.field("running", s.running);
    w.field("cycles", s.cycles);
    w.field("rate_hz", s.rate_hz);
    w.field("last_cycle_ms", s.last_cycle_ms);
    w.field("mean_cycle_ms", s.mean_cycle_ms);
    w.field("max_cycle_ms", s.max_cycle_ms);
    w.field("period_target_ms", s.period_target_ms);
    w.field("overruns", s.overruns);
    w.field("dropped", s.dropped);
    w.field("pending", s.pending);
    w.key("last_cycle_host_ms");
    if (s.last_cycle_host_ms >= 0) {
        w.value(static_cast<int64_t>(s.last_cycle_host_ms));
    } else {
        w.null();
    }
    w.endObject();
}

void diagnostics(JsonWriter& w, const char* key, const Diagnostics& d) {
    w.key(key);
    w.beginObject();
    w.field("cycles", d.cycles);
    w.key("functions");
    w.beginArray();
    for (const auto& kv : d.functions) {
        w.beginObject();
        w.field("label", kv.first);
        w.field("runs", kv.second.runs);
        w.field("ok", kv.second.ok);
        w.field("no_data", kv.second.no_data);
        w.field("fault", kv.second.fault);
        w.field("last", functionStatusName(kv.second.last));
        w.endObject();
    }
    w.endArray();
    w.key("links");
    w.beginArray();
    for (const auto& kv : d.links) {
        w.beginObject();
        w.field("id", kv.first);
        w.field("bytes", kv.second.bytes);
        w.field("packets", kv.second.packets);
        w.field("decode_errors", kv.second.decode_errors);
        w.field("seq_gaps", kv.second.seq_gaps);
        w.field("requests", kv.second.requests);
        w.field("duplicates", kv.second.duplicates);
        w.field("superseded", kv.second.superseded);
        w.field("stale", kv.second.stale);
        w.field("unknown_session", kv.second.unknown_session);
        w.field("unanswered", kv.second.unanswered);
        w.field("replies", kv.second.replies);
        w.field("expired", kv.second.expired);
        w.field("input_pending", kv.second.input_pending);
        w.field("late_release", kv.second.late_release);
        w.field("tx_errors", kv.second.tx_errors);
        w.endObject();
    }
    w.endArray();
    w.endObject();
}

void detectionFrame(JsonWriter& w, const System& system, const DetectionFrameSnapshot& f,
                    MonotonicTime now) {
    w.beginObject();
    w.field("camera", f.camera.value);
    w.field("frame_id", f.engineering_frame.value);
    w.field("epoch", f.frame_epoch);
    w.field("sequence", static_cast<uint64_t>(f.frame_sequence));
    hostMs(w, "exposure_host_ms", f.exposureAt);
    hostMs(w, "received_host_ms", f.receivedAt);
    hostMs(w, "processed_host_ms", f.processedAt);
    ageMs(w, "exposure_age_ms", f.exposureAt, now);
    w.field("exposure_uncertainty_ms", static_cast<int64_t>(f.exposure_uncertainty_ms));
    w.field("exposure_time_reliable", f.exposure_time_reliable);
    w.field("width_px", f.width_px);
    w.field("height_px", f.height_px);
    w.field("has_image", f.y8 != nullptr && !f.y8->empty());
    intrinsics(w, "intrinsics", f.intrinsics.get());
    Transform3 T_robot_camera;
    const bool mounted = findRobotFrame(system, f.engineering_frame, T_robot_camera);
    w.field("mounted", mounted);
    if (mounted) {
        transform3(w, "T_robot_camera", T_robot_camera);
    } else {
        w.fieldNull("T_robot_camera");
    }
    w.field("field_invocation", f.field_invocation);

    w.key("pose_at_exposure");
    w.beginObject();
    w.field("status", lookupStatusName(f.pose_at_exposure.status));
    w.field("exact", f.pose_at_exposure.exact);
    w.field("odometry_epoch", f.pose_at_exposure.odometry_epoch);
    pose2(w, "odom", f.pose_at_exposure.odom_pose);
    pose2(w, "field", compose(f.field_from_odom, f.pose_at_exposure.odom_pose));
    w.endObject();
    w.key("attitude_at_exposure");
    w.beginObject();
    w.field("status", lookupStatusName(f.attitude_at_exposure.status));
    attitude(w, "attitude", f.attitude_at_exposure.attitude, now);
    w.endObject();
    pose2(w, "field_from_odom", f.field_from_odom);
    w.field("anchor_revision", f.anchor_revision);

    // false: preview only, detection did not run on this frame
    w.field("has_observations", f.has_observations);
    w.field("has_trace", f.has_trace);
    w.field("detector_processing_ms",
            f.has_observations ? f.observations.detector_processing_ms : 0.0);
    w.field("frame_note", f.has_trace ? f.trace.frame_note : std::string());
    w.key("tags");
    w.beginArray();
    if (f.has_observations) {
        for (std::size_t i = 0; i < f.observations.tags.size(); ++i) {
            const TagObservation& t = f.observations.tags[i];
            w.beginObject();
            w.field("index", static_cast<uint64_t>(i));
            w.field("family", t.family);
            w.field("id", t.observed_id);
            w.field("hamming", t.hamming);
            w.field("decision_margin", t.decision_margin);
            w.key("corners_px");
            w.beginArray();
            for (int c = 0; c < 4; ++c) {
                w.beginArray();
                w.value(t.corners_px[c][0]);
                w.value(t.corners_px[c][1]);
                w.endArray();
            }
            w.endArray();
            w.key("center_px");
            w.beginArray();
            w.value(t.center_px[0]);
            w.value(t.center_px[1]);
            w.endArray();
            w.field("has_pose", t.has_pose);
            if (t.has_pose) {
                transform3(w, "T_camera_tag", t.T_camera_tag);
            } else {
                w.fieldNull("T_camera_tag");
            }
            w.key("reprojection_error_px");
            if (t.has_reprojection_error) {
                w.value(t.reprojection_error_px);
            } else {
                w.null();
            }
            w.key("alternate_pose_ambiguity");
            if (t.has_alternate_pose_ambiguity) {
                w.value(t.alternate_pose_ambiguity);
            } else {
                w.null();
            }
            w.key("association");
            const TagAssociationTrace* trace = nullptr;
            if (f.has_trace) {
                for (const TagAssociationTrace& tr : f.trace.tags) {
                    if (tr.tag_index == i) {
                        trace = &tr;
                        break;
                    }
                }
            }
            if (trace == nullptr) {
                w.null();
            } else {
                w.beginObject();
                w.field("accepted", trace->accepted);
                w.field("rejection", trace->rejection);
                w.field("object", trace->object.value);
                w.field("mount", trace->mount);
                w.key("candidates");
                w.beginArray();
                for (const TagAssociationCandidate& c : trace->candidates) {
                    w.beginObject();
                    w.field("object", c.object.value);
                    w.field("mount", c.mount);
                    pose2(w, "implied_odom", c.implied);
                    pose2(w, "implied_field", compose(f.field_from_odom, c.implied));
                    w.field("translation_error_m", c.translation_error_m);
                    w.field("heading_error_deg", radToDeg(c.heading_error_rad));
                    w.field("score", c.score);
                    w.endObject();
                }
                w.endArray();
                w.endObject();
            }
            w.endObject();
        }
    }
    w.endArray();
    w.endObject();
}

} // namespace

std::string helloDocument(const System& system, MonotonicTime now) {
    // one consistent binding, safe while a profile boundary runs
    const std::shared_ptr<const BindingView> binding = system.bindingView();
    JsonWriter                               w;
    w.beginObject();
    w.field("type", "hello");
    w.field("contract", kInspectionContract);
    w.field("host_ms", static_cast<int64_t>(now.ms));
    w.key("session");
    w.beginObject();
    w.field("id", system.sessionId());
    w.field("reset_count", system.resetCount());
    w.endObject();
    w.key("configuration");
    w.beginObject();
    w.field("id", system.configurationId());
    w.field("name", system.configurationName());
    w.field("digest", hex64(system.configurationDigest()));
    w.field("loop_rate_hz", system.loopRateHz());
    w.field("commands_type", system.commandsType());
    // localization comes from a Brain robot profile
    w.field("brain_profile", binding->brain_profile);
    w.endObject();
    const InspectionConfig& ic = system.inspection();
    w.key("inspection");
    w.beginObject();
    w.field("snapshot_hz", ic.snapshot_hz);
    w.field("preview_hz", ic.preview_hz);
    w.field("preview_quality", static_cast<int64_t>(ic.preview_quality));
    w.field("preview_max_width", static_cast<int64_t>(ic.preview_max_width));
    w.endObject();
    w.key("robot_body");
    w.beginObject();
    w.field("length_m", ic.robot_body.length_m);
    w.field("width_m", ic.robot_body.width_m);
    w.field("height_m", ic.robot_body.height_m);
    w.field("origin_x_m", ic.robot_body.origin_x_m);
    w.field("origin_y_m", ic.robot_body.origin_y_m);
    w.endObject();

    // every configured field map, normally one
    w.key("fields");
    w.beginArray();
    std::set<std::string> families;
    for (const ResourceStore::Record& r : system.resources().records()) {
        std::string err;
        auto        map = r.value.require<const FieldMap>(err);
        if (map == nullptr) {
            continue;
        }
        w.beginObject();
        w.field("resource_id", r.id.value);
        w.field("name", map->name);
        // planning data: what the Brain receives as the map document
        w.field("revision", static_cast<int64_t>(map->revision));
        {
            FieldMapDocument doc;
            std::string      why;
            if (buildFieldMapDocument(*map, doc, why)) {
                wireId32(w, "map_id", doc.map_id);
                w.fieldNull("map_error");
            } else {
                w.fieldNull("map_id");
                w.field("map_error", why);
            }
        }
        w.key("boundary");
        if (!map->boundary.declared) {
            w.null();
        } else {
            const FieldBoundaryDecl& b = map->boundary;
            w.beginObject();
            w.field("min_x_m", b.min_x_m);
            w.field("min_y_m", b.min_y_m);
            w.field("max_x_m", b.max_x_m);
            w.field("max_y_m", b.max_y_m);
            w.field("note", b.note);
            w.endObject();
        }
        w.key("obstacles");
        w.beginArray();
        for (const ObstacleDecl& o : map->obstacles) {
            w.beginObject();
            w.field("id", o.id);
            wireId16(w, "wire_id", o.wire_id);
            pose2(w, "pose", o.pose);
            collisionBox(w, "collision_box", o.box);   // obstacle frame
            w.field("note", o.note);
            w.endObject();
        }
        w.endArray();
        w.key("dimensions");
        if (!map->dimensions.declared) {
            w.null();
        } else {
            const FieldDimensions& d = map->dimensions;
            w.beginObject();
            w.field("inside_x_m", d.inside_x_m);
            w.field("inside_y_m", d.inside_y_m);
            w.field("wall_height_m", d.wall_height_m);
            w.field("wall_thickness_m", d.wall_thickness_m);
            w.field("tile_m", d.tile_m);
            w.field("source", d.source);
            w.field("revision", d.revision);
            w.field("units_note", d.units_note);
            w.endObject();
        }
        w.key("features");
        w.beginArray();
        for (const FieldFeatureDecl& f : map->features) {
            w.beginObject();
            w.field("id", f.id);
            w.field("kind", f.kind);
            w.field("x_m", f.x_m);
            w.field("y_m", f.y_m);
            w.field("z_m", f.z_m);
            w.field("size_x_m", f.size_x_m);
            w.field("size_y_m", f.size_y_m);
            w.field("size_z_m", f.size_z_m);
            w.field("yaw_deg", radToDeg(f.yaw_rad));
            w.field("color", f.color);
            w.field("note", f.note);
            w.endObject();
        }
        w.endArray();
        w.key("landmarks");
        w.beginArray();
        for (const LandmarkDecl& lm : map->landmarks) {
            w.beginObject();
            w.field("id", lm.id.value);
            wireId16(w, "wire_id", lm.wire_id);
            pose2(w, "nominal", lm.nominal);
            collisionBox(w, "collision_box", lm.box);   // landmark frame
            w.key("visual");
            if (!lm.visual.declared) {
                w.null();
            } else {
                const LandmarkVisualDecl& v = lm.visual;
                w.beginObject();
                w.field("shape", v.shape);
                w.field("height_m", v.height_m);
                w.field("base_across_flats_m", v.base_across_flats_m);
                w.field("top_across_flats_m", v.top_across_flats_m);
                w.field("size_x_m", v.size_x_m);
                w.field("size_y_m", v.size_y_m);
                w.field("size_z_m", v.size_z_m);
                w.field("tag_plate_width_m", v.tag_plate_width_m);
                w.field("tag_plate_height_m", v.tag_plate_height_m);
                w.field("tag_plate_thickness_m", v.tag_plate_thickness_m);
                w.field("color", v.color);
                w.field("note", v.note);
                w.endObject();
            }
            w.key("mounts");
            w.beginArray();
            for (const TagMountDecl& m : lm.mounts) {
                families.insert(m.family);
                w.beginObject();
                w.field("instance_id", m.instance_id);
                w.field("family", m.family);
                w.field("observed_id", m.observed_id);
                w.field("detection_size_m", m.detection_size_m);
                transform3(w, "T_landmark_tag", m.T_landmark_tag_surface);
                w.endObject();
            }
            w.endArray();
            w.key("approaches");
            w.beginArray();
            for (const ApproachFrameDecl& a : lm.approaches) {
                w.beginObject();
                w.field("id", a.id.value);
                transform3(w, "T_landmark_approach", a.T_landmark_approach);
                w.endObject();
            }
            w.endArray();
            w.endObject();
        }
        w.endArray();
        w.endObject();
    }
    w.endArray();

    w.key("tag_families");
    w.beginObject();
    for (const std::string& family : families) {
        int         width_at_border = 0, total_width = 0;
        std::string err;
        if (!AprilTagDetector::familyGeometry(family, width_at_border, total_width, err)) {
            continue;
        }
        w.key(family);
        w.beginObject();
        w.field("width_at_border", width_at_border);
        w.field("total_width", total_width);
        w.endObject();
    }
    w.endObject();

    // camera devices and the sensors that forward them
    w.key("cameras");
    w.beginArray();
    for (const ResourceStore::Record& r : system.resources().records()) {
        std::string err;
        auto        device = r.value.require<CameraDevice>(err);
        if (device == nullptr) {
            continue;
        }
        w.beginObject();
        w.field("resource_id", r.id.value);
        w.field("implementation", r.implementationType.value);
        w.field("frame_id", device->engineeringFrame().value);
        w.field("alive", device->alive());
        w.field("diagnostic", device->diagnostic());
        intrinsics(w, "intrinsics", device->intrinsics());
        Transform3 T;
        const bool mounted = findRobotFrame(system, device->engineeringFrame(), T);
        w.field("mounted", mounted);
        if (mounted) {
            transform3(w, "T_robot_camera", T);
        } else {
            w.fieldNull("T_robot_camera");
        }
        w.endObject();
    }
    w.endArray();
    w.key("camera_sensors");
    w.beginArray();
    for (const auto& e : binding->sensors.entries()) {
        if (e.payload.stable_name == payload_names::kCameraFrame) {
            w.value(e.id.value);
        }
    }
    w.endArray();

    w.key("localization");
    w.beginObject();
    w.field("estimator_type", binding->estimator_type);
    w.key("functions");
    w.beginArray();
    for (const ObservationFunctionStatus& f : system.robotFeed()->status().functions) {
        w.beginObject();
        w.field("id", f.id);
        w.field("type", f.type);
        w.endObject();
    }
    w.endArray();
    const PoseHistoryConfig& h = system.robotFeed()->historyConfig();
    w.key("history");
    w.beginObject();
    w.field("retention_ms", static_cast<int64_t>(h.retention_ms));
    w.field("capacity", static_cast<uint64_t>(h.capacity));
    w.field("max_interpolation_gap_ms", static_cast<int64_t>(h.max_interpolation_gap_ms));
    w.field("attitude_gap_ms", static_cast<int64_t>(h.attitude_gap_ms));
    w.endObject();
    w.endObject();

    w.key("warnings");
    w.beginArray();
    for (const std::string& s : binding->warnings) {
        w.value(s);
    }
    w.endArray();
    w.endObject();
    return w.take();
}

std::string snapshotDocument(const System& system, const InspectionServiceStats& service,
                             MonotonicTime now, std::size_t trail_max_entries) {
    JsonWriter w;
    w.beginObject();
    w.field("type", "snapshot");
    w.field("contract", kInspectionContract);
    w.field("host_ms", static_cast<int64_t>(now.ms));
    w.key("session");
    w.beginObject();
    w.field("id", system.sessionId());
    w.field("reset_count", system.resetCount());
    w.endObject();
    w.field("cycle", system.cycle());
    w.field("running", system.running());

    const std::shared_ptr<RobotStateFeed> feed   = system.robotFeed();
    const LocalizationSnapshot localization = feed->snapshot(trail_max_entries);
    const RobotState& robot = localization.robot;
    const LocalizationStatus& status = localization.status;

    w.key("robot");
    w.beginObject();
    w.field("valid", robot.valid);
    w.field("initialized", robot.initialized);
    // "command" (Brain), "configuration", or empty before any placement
    w.field("placement_origin", robot.placement_origin);
    w.field("placement_session", static_cast<uint64_t>(robot.placement_session));
    w.field("placement_sequence", robot.placement_sequence);
    w.field("odometry_epoch", robot.odometry_epoch);
    w.field("anchor_revision", robot.anchor_revision);
    pose2(w, "odom", robot.odom_pose);
    pose2(w, "field", robot.fieldPose());
    pose2(w, "field_from_odom", robot.field_from_odom);
    w.field("vx_m_s", robot.vx_m_s);
    w.field("vy_m_s", robot.vy_m_s);
    w.field("yaw_rate_deg_s", radToDeg(robot.yaw_rate_rad_s));
    w.field("confidence", robot.confidence);
    w.field("has_covariance", robot.has_covariance);
    if (robot.has_covariance) {
        // odometry frame, m^2, m rad, rad^2
        const PoseCovariance& c = robot.odom_covariance;
        w.key("odom_covariance");
        w.beginObject();
        w.field("xx", c.xx);
        w.field("xy", c.xy);
        w.field("xh", c.xh);
        w.field("yy", c.yy);
        w.field("yh", c.yh);
        w.field("hh", c.hh);
        w.endObject();
    }
    stamp(w, "measured_at", robot.measuredAt);
    hostMs(w, "measured_at_host_ms", robot.measuredAtHost);
    ageMs(w, "age_ms", robot.measuredAtHost, now);
    attitude(w, "attitude", robot.attitude, now);
    w.endObject();

    w.key("localization");
    w.beginObject();
    w.field("estimator_type", status.estimator_type);
    w.field("updates", status.updates);
    w.field("history_size", status.history_size);
    w.field("clock_mapped", status.clock_mapped);
    w.field("publication", localization.publication);
    w.field("all_ready", status.allReady());
    // gated stationary handling: velocity reported zero, the pose untouched
    w.field("stationary", status.stationary());
    w.field("continuity_breaks", status.continuity_breaks);
    w.field("last_break", status.last_break);
    w.key("functions");
    w.beginArray();
    for (const ObservationFunctionStatus& f : status.functions) {
        w.beginObject();
        w.field("id", f.id);
        w.field("type", f.type);
        w.field("ready", f.ready);
        w.field("note", f.note);
        stillness(w, "stillness", f.stillness);
        w.endObject();
    }
    w.endArray();
    w.endObject();

    // oldest first, bounded
    w.key("trail");
    w.beginArray();
    {
        const auto& recent = localization.trail;
        for (auto it = recent.rbegin(); it != recent.rend(); ++it) {
            w.beginObject();
            w.field("host_ms", static_cast<int64_t>(it->at.ms));
            w.field("x_m", it->odom_pose.x_m);
            w.field("y_m", it->odom_pose.y_m);
            w.field("heading_deg", radToDeg(it->odom_pose.heading_rad));
            w.field("epoch", it->odometry_epoch);
            w.field("attitude_valid", it->attitude.valid);
            if (it->attitude.valid) {
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                attitudeEuler(it->attitude, roll, pitch, yaw);
                w.field("roll_deg", radToDeg(roll));
                w.field("pitch_deg", radToDeg(pitch));
            }
            w.endObject();
        }
    }
    w.endArray();

    const std::shared_ptr<const FieldSnapshot> field = system.fieldSnapshot();
    w.key("field_snapshot");
    w.beginObject();
    w.field("invocation", field->invocation);
    hostMs(w, "at_host_ms", field->at);
    ageMs(w, "age_ms", field->at, now);
    w.field("status", functionStatusName(field->status));
    w.field("diagnostic", field->diagnostic);
    w.endObject();

    // nominal poses come from the configured maps, for heading error
    std::map<std::string, Pose2D> nominal;
    for (const ResourceStore::Record& r : system.resources().records()) {
        std::string err;
        auto        map = r.value.require<const FieldMap>(err);
        if (map == nullptr) {
            continue;
        }
        for (const LandmarkDecl& lm : map->landmarks) {
            nominal.emplace(lm.id.value, lm.nominal);
        }
    }
    w.key("field_objects");
    w.beginArray();
    {
        // deterministic order for clients
        std::vector<const std::pair<const FieldObjectId, FieldObjectState>*> ordered;
        for (const auto& kv : field->field.objects) {
            ordered.push_back(&kv);
        }
        std::sort(ordered.begin(), ordered.end(),
                  [](const auto* a, const auto* b) { return a->first.value < b->first.value; });
        for (const auto* kv : ordered) {
            const FieldObjectState& o = kv->second;
            w.beginObject();
            w.field("id", kv->first.value);
            w.field("valid", o.valid);
            w.field("observed", o.observed);
            w.field("source", estimateSourceName(o.source));
            w.field("confidence", o.confidence);
            w.field("frame", o.pose.frame.value);
            pose2(w, "pose", o.pose.pose);
            const auto n = nominal.find(kv->first.value);
            if (n != nominal.end()) {
                pose2(w, "nominal", n->second);
                w.field("heading_error_deg",
                        radToDeg(wrapAngle(o.pose.pose.heading_rad - n->second.heading_rad)));
                w.field("displacement_m",
                        std::hypot(o.pose.pose.x_m - n->second.x_m,
                                   o.pose.pose.y_m - n->second.y_m));
            } else {
                w.fieldNull("nominal");
                w.fieldNull("heading_error_deg");
                w.fieldNull("displacement_m");
            }
            w.field("anchor_revision", o.anchor_revision);
            w.field("odometry_epoch", o.odometry_epoch);
            hostMs(w, "last_observed_host_ms", o.lastObservedAt);
            ageMs(w, "age_ms", o.lastObservedAt, now);
            w.field("last_source", o.last_source);
            w.field("last_feature", o.last_feature);
            w.field("last_source_sequence", static_cast<uint64_t>(o.last_source_sequence));
            w.endObject();
        }
    }
    w.endArray();

    w.key("detection_frames");
    w.beginArray();
    for (const auto& kv : system.detectionFrames()) {
        detectionFrame(w, system, *kv.second, now);
    }
    w.endArray();

    const std::shared_ptr<const ReportingSnapshot> reporting = system.reportingSnapshot();
    w.key("target");
    if (reporting == nullptr) {
        w.null();
    } else {
        const TargetState& t = reporting->target;
        w.beginObject();
        w.field("active", t.active);
        w.field("target_id", t.target_id);
        w.field("wire_id", static_cast<int64_t>(t.wire_id));
        w.field("generation", t.generation);
        w.field("status", targetStatusName(t.status));
        w.field("latched", t.latched);
        pose2(w, "T_odom_robot_target", t.T_odom_robot_target);
        w.field("odometry_epoch", t.odometry_epoch);
        hostMs(w, "activated_host_ms", t.activatedAt);
        w.endObject();
    }
    w.key("command");
    if (reporting == nullptr) {
        w.null();
    } else {
        const CommandState& c = reporting->command;
        w.beginObject();
        w.field("session", c.session);
        w.field("init_sequence", c.init_sequence);
        w.field("init_session", c.init_session);
        pose2(w, "init_pose", c.init_pose);
        w.field("object_requested", c.object_requested);
        w.field("object_wire_id", static_cast<int64_t>(c.object_wire_id));
        w.field("object_sequence", c.object_sequence);
        w.endObject();
    }

    // what the Brain sees and what the Pi runs for it
    w.key("brain_link");
    if (system.commandsType() != "brain_link") {
        w.null();
    } else {
        const std::shared_ptr<const BindingView> binding = system.bindingView();
        const ProfileBinding* running = binding != nullptr ? binding->profile.get() : nullptr;
        w.beginObject();
        w.field("takes_profile", binding != nullptr && binding->brain_profile);
        if (reporting == nullptr) {
            w.field("link_open", false);
            w.field("session", static_cast<uint64_t>(0));
            w.field("pi_instance", static_cast<uint64_t>(0));
            w.fieldNull("last_request_host_ms");
            w.fieldNull("last_request_age_ms");
            w.fieldNull("state");
        } else {
            const CommandState& c = reporting->command;
            w.field("link_open", c.link_open);
            w.field("session", static_cast<uint64_t>(c.session));
            w.field("pi_instance", static_cast<uint64_t>(c.pi_instance));
            hostMs(w, "last_request_host_ms", c.last_request);
            ageMs(w, "last_request_age_ms", c.last_request, now);
            if (reporting->brain_state.has_value()) {
                brainState(w, *reporting->brain_state);
            } else {
                w.fieldNull("state");
            }
        }

        w.key("profile");
        w.beginObject();
        const ProfileStatus ps =
            reporting != nullptr ? reporting->command.profile : ProfileStatus{};
        w.field("state", profileStateName(ps.state));
        wireId32(w, "id", ps.id);
        wireId32(w, "applied_id", ps.applied_id);
        w.field("reason", profileReasonName(ps.reason));
        w.field("detail", static_cast<int64_t>(ps.detail));
        w.key("running");
        if (running == nullptr) {
            w.null();
        } else {
            runningProfile(w, *running);
        }
        w.endObject();

        // the model that owns the IMU bias; null when nothing calibrates on the Pi
        const ObservationFunctionStatus* bias =
            running != nullptr && !running->bias_function.empty()
                ? status.find(running->bias_function)
                : nullptr;
        w.key("calibration");
        if (bias == nullptr) {
            w.null();
        } else {
            w.beginObject();
            w.field("function", bias->id);
            w.field("ready", bias->ready);
            stillness(w, "stillness", bias->stillness);
            w.endObject();
        }

        // READ_WHEELS as the Brain would read it, with the active corrections
        w.key("wheels");
        w.beginArray();
        if (reporting != nullptr) {
            for (const gatr2::WheelReading& r : reporting->wheels) {
                const gatr2::ProfileWheel* p = nullptr;
                for (uint8_t i = 0; running != nullptr && i < running->profile.wheel_count; ++i) {
                    if (running->profile.wheels[i].encoder_port == r.port) {
                        p = &running->profile.wheels[i];
                    }
                }
                w.beginObject();
                w.field("port", static_cast<int64_t>(r.port));
                w.field("valid", (r.flags & gatr2::kWheelValid) != 0);
                w.field("fresh", (r.flags & gatr2::kWheelFresh) != 0);
                w.field("counts", static_cast<int64_t>(r.counts));
                w.field("travel_m", r.travel_um * 1e-6);
                w.field("discontinuity", static_cast<int64_t>(r.discontinuity));
                w.field("age_ms", static_cast<int64_t>(r.age_ms));
                if (p != nullptr) {
                    w.field("counts_per_rev", static_cast<uint64_t>(p->counts_per_rev));
                    w.field("gear", p->gear_micro / static_cast<double>(gatr2::kUnitMicro));
                    w.field("reversed", (p->flags & gatr2::kWheelReversed) != 0);
                    w.field("radius_m", p->radius_um * 1e-6);
                    w.field("travel_scale",
                            p->travel_scale_ppm / static_cast<double>(gatr2::kUnitMicro));
                }
                w.endObject();
            }
        }
        w.endArray();

        w.key("operation");
        if (reporting == nullptr || !reporting->pico_operation.active) {
            w.null();
        } else {
            const PicoOperation& op = reporting->pico_operation;
            w.beginObject();
            w.field("action", controlActionName(op.action));
            w.field("result", resultName(op.result));
            w.field("detail", controlDetailName(op.detail));
            hostMs(w, "started_host_ms", op.started);
            ageMs(w, "age_ms", op.started, now);
            w.endObject();
        }

        // the Brain's latest PATH_REPORT, field frame, inspection only
        w.key("path");
        const PathReport* path = reporting != nullptr ? &reporting->command.path : nullptr;
        if (path == nullptr || path->mode == gatr2::kPathNone || path->count == 0) {
            w.null();
        } else {
            w.beginObject();
            w.field("session", static_cast<uint64_t>(path->session));
            w.field("command_id", static_cast<uint64_t>(path->command_id));
            w.field("mode", pathModeName(path->mode));
            hostMs(w, "received_host_ms", path->received);
            ageMs(w, "age_ms", path->received, now);
            w.key("points");
            w.beginArray();
            for (uint8_t i = 0; i < path->count && i < path->points.size(); ++i) {
                w.beginObject();
                w.field("x_m", path->points[i].x_m);
                w.field("y_m", path->points[i].y_m);
                w.endObject();
            }
            w.endArray();
            w.endObject();
        }
        w.endObject();
    }

    std::string                        pico_id;
    const std::shared_ptr<PicoControl> pico = findPico(system, pico_id);
    w.key("pico");
    if (pico == nullptr) {
        w.null();
    } else {
        picoLink(w, pico_id, pico->link(), now);
    }

    // bounded lifecycle log, oldest first
    w.key("events");
    w.beginArray();
    for (const RuntimeEvent& e : system.events()) {
        w.beginObject();
        w.field("sequence", e.sequence);
        hostMs(w, "host_ms", e.at);
        w.field("text", e.text);
        w.endObject();
    }
    w.endArray();

    const std::shared_ptr<const SourceHealthSnapshot> health = system.sourceHealth();
    w.key("sources");
    w.beginArray();
    if (health != nullptr) {
        for (const SourceHealthEntry& e : health->entries) {
            w.beginObject();
            w.field("kind", e.kind);
            w.field("id", e.id);
            w.field("state", sourceStateName(e.state));
            w.field("diagnostic", e.diagnostic);
            w.field("payload", e.payload);
            w.field("has_sample", e.has_sample);
            stamp(w, "measured_at", e.measuredAt);
            hostMs(w, "received_host_ms", e.receivedAt);
            ageMs(w, "receipt_age_ms", e.receivedAt, now);
            hostMs(w, "last_polled_host_ms", e.lastPolledAt);
            w.field("sequence", e.sequence);
            w.field("epoch", e.epoch);
            w.field("upstream_source", e.upstream_source);
            w.field("upstream_clock", e.upstream_clock);
            w.field("upstream_sequence", e.upstream_sequence);
            w.field("upstream_epoch", e.upstream_epoch);
            w.endObject();
        }
    }
    w.endArray();

    w.key("workers");
    w.beginObject();
    workerStats(w, "estimation", system.estimationStats());
    workerStats(w, "field", system.fieldStats());
    w.key("inspection");
    w.beginObject();
    w.field("running", service.running);
    w.field("port", service.port);
    w.field("clients", service.clients);
    w.field("clients_total", service.clients_total);
    w.field("client_disconnects", service.client_disconnects);
    w.field("snapshots_sent", service.snapshots_sent);
    w.field("snapshots_skipped", service.snapshots_skipped);
    w.field("frames_sent", service.frames_sent);
    w.field("frames_skipped", service.frames_skipped);
    w.field("bytes_sent", service.bytes_sent);
    w.field("encodes", service.encodes);
    w.field("last_encode_ms", service.last_encode_ms);
    w.field("mean_encode_ms", service.mean_encode_ms);
    w.field("snapshot_rate_hz", service.snapshot_rate_hz);
    w.field("frame_rate_hz", service.frame_rate_hz);
    w.endObject();
    w.endObject();

    const std::shared_ptr<const DiagnosticsSnapshot> diag = system.diagnosticsSnapshot();
    w.key("diagnostics");
    if (diag == nullptr) {
        w.null();
    } else {
        w.beginObject();
        diagnostics(w, "estimation", diag->estimation);
        diagnostics(w, "field", diag->field);
        w.endObject();
    }
    w.endObject();
    return w.take();
}

std::string frameHeaderDocument(const DetectionFrameSnapshot& frame, int preview_width_px,
                                int preview_height_px, long quality, double encode_ms,
                                MonotonicTime now) {
    JsonWriter w;
    w.beginObject();
    w.field("type", "frame");
    w.field("contract", kInspectionContract);
    w.field("host_ms", static_cast<int64_t>(now.ms));
    w.field("camera", frame.camera.value);
    w.field("epoch", frame.frame_epoch);
    w.field("sequence", static_cast<uint64_t>(frame.frame_sequence));
    hostMs(w, "exposure_host_ms", frame.exposureAt);
    w.field("width_px", frame.width_px);
    w.field("height_px", frame.height_px);
    w.field("preview_width_px", preview_width_px);
    w.field("preview_height_px", preview_height_px);
    w.field("quality", static_cast<int64_t>(quality));
    w.field("encode_ms", encode_ms);
    w.field("format", "image/jpeg");
    w.endObject();
    return w.take();
}

} // namespace navigatr
