// inspection_document.cpp

#include "inspection/inspection_document.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

#include "translaGATR/frame_codec.h"
#include "translaGATR/link_documents.h"
#include "capture/capture_service.h"
#include "config/field_map.h"
#include "diagnostics/hub.h"
#include "diagnostics/instrumentation.h"
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

// Times, ages and stamps are varying: a diag identity check leaves them out.

// host_ms or null when the time is not on the host clock
void hostMs(JsonWriter& w, const char* key, MonotonicTime t) {
    const std::size_t from = w.mark();
    w.key(key);
    if (t.domain == ClockDomain::kHost) {
        w.value(static_cast<int64_t>(t.ms));
    } else {
        w.null();
    }
    w.varying(from);
}

// age in ms against now, null when not comparable
void ageMs(JsonWriter& w, const char* key, MonotonicTime t, MonotonicTime now) {
    const std::size_t from = w.mark();
    w.key(key);
    if (t.domain == ClockDomain::kHost && now.domain == ClockDomain::kHost) {
        w.value(static_cast<int64_t>(now.ms - t.ms));
    } else {
        w.null();
    }
    w.varying(from);
}

void stamp(JsonWriter& w, const char* key, MonotonicTime t) {
    const std::size_t from = w.mark();
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
    w.varying(from);
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
    // measured, stale, assumed_level or unavailable
    w.field("status", attitudeStatus(a, now));
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
    case translagatr::kProfileNone: return "none";
    case translagatr::kProfileApplying: return "applying";
    case translagatr::kProfileApplied: return "applied";
    case translagatr::kProfileRejected: return "rejected";
    default: return "unknown";
    }
}

const char* calibrationName(uint8_t s) {
    return s <= translagatr::kCalibrationFailed ? toString(static_cast<BiasCalibration>(s)) : "unknown";
}

const char* imuSourceName(uint8_t s) {
    switch (s) {
    case translagatr::kImuSourceNone: return "none";
    case translagatr::kImuSourcePico: return "pico";
    case translagatr::kImuSourceBrainVex: return "brain_vex";
    default: return "unknown";
    }
}

const char* controlActionName(uint8_t a) {
    switch (a) {
    case translagatr::kControlRecalibrate: return "recalibrate";
    case translagatr::kControlReinitialize: return "reinitialize";
    case translagatr::kControlReinitImu: return "reinit_imu";
    case translagatr::kControlRestartAcquisition: return "restart_acquisition";
    default: return "unknown";
    }
}

const char* resultName(uint8_t r) {
    switch (r) {
    case translagatr::kResultOk: return "ok";
    case translagatr::kResultPending: return "pending";
    case translagatr::kResultNotReady: return "not_ready";
    case translagatr::kResultNotStationary: return "not_stationary";
    case translagatr::kResultFailed: return "failed";
    case translagatr::kResultInvalidArgument: return "invalid_argument";
    default: return "other";
    }
}

const char* controlDetailName(uint8_t d) {
    switch (d) {
    case translagatr::kControlDetailNone: return "none";
    case translagatr::kControlDetailPicoLink: return "pico_link";
    case translagatr::kControlDetailImuAbsent: return "imu_absent";
    case translagatr::kControlDetailImuUnused: return "imu_unused";
    case translagatr::kControlDetailPicoRefused: return "pico_refused";
    case translagatr::kControlDetailTimedOut: return "timed_out";
    case translagatr::kControlDetailCalibration: return "calibration";
    default: return "unknown";
    }
}

const char* pathModeName(uint8_t m) {
    switch (m) {
    case translagatr::kPathNone: return "none";
    case translagatr::kPathDirect: return "direct";
    case translagatr::kPathAvoiding: return "avoiding";
    default: return "unknown";
    }
}

const char* picoImuStateName(uint8_t s) {
    switch (s) {
    case translagatr::kPicoImuDisabled: return "disabled";
    case translagatr::kPicoImuInitializing: return "initializing";
    case translagatr::kPicoImuAligning: return "aligning";
    case translagatr::kPicoImuReady: return "ready";
    case translagatr::kPicoImuRetrying: return "retrying";
    case translagatr::kPicoImuFailed: return "failed";
    default: return "unknown";
    }
}

const char* picoImuReasonName(uint8_t r) {
    switch (r) {
    case translagatr::kPicoImuReasonNone: return "none";
    case translagatr::kPicoImuReasonNoResponse: return "no_response";
    case translagatr::kPicoImuReasonBoot: return "boot";
    case translagatr::kPicoImuReasonFeatures: return "features";
    case translagatr::kPicoImuReasonStream: return "stream";
    default: return "unknown";
    }
}

const char* picoFirmwareName(uint8_t f) {
    switch (f) {
    case translagatr::kPicoFirmwareBno08x: return "bno08x";
    case translagatr::kPicoFirmwareAsm330: return "asm330";
    default: return "unknown";
    }
}

const char* picoOpName(uint8_t op) {
    switch (op) {
    case 0: return "none";
    case translagatr::kPicoOpConfigure: return "configure";
    case translagatr::kPicoOpReinitImu: return "reinit_imu";
    case translagatr::kPicoOpRestartAcquisition: return "restart_acquisition";
    case translagatr::kPicoOpDiagnostics: return "diagnostics";
    default: return "unknown";
    }
}

const char* picoCommandStatusName(uint8_t s) {
    switch (s) {
    case translagatr::kPicoCommandNone: return "none";
    case translagatr::kPicoCommandRunning: return "running";
    case translagatr::kPicoCommandCompleted: return "completed";
    case translagatr::kPicoCommandFailed: return "failed";
    default: return "unknown";
    }
}

const char* picoDetailName(uint8_t d) {
    switch (d) {
    case translagatr::kPicoDetailNone: return "none";
    case translagatr::kPicoDetailWrongTarget: return "wrong_target";
    case translagatr::kPicoDetailUnknownOp: return "unknown_op";
    case translagatr::kPicoDetailBadBody: return "bad_body";
    case translagatr::kPicoDetailImuAbsent: return "imu_absent";
    case translagatr::kPicoDetailImuDisabled: return "imu_disabled";
    case translagatr::kPicoDetailNoSuchPort: return "no_such_port";
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
    // progress counters move every cycle while the robot moves
    w.varyingField("progress_ms", static_cast<int64_t>(s.progress_ms));
    w.field("window_ms", static_cast<int64_t>(s.window_ms));
    w.varyingField("windows", s.windows);
    w.varyingField("restarts", s.restarts);
    w.varyingField("movements", s.movements);
    w.varyingField("attempts", static_cast<uint64_t>(s.attempts));
    w.varyingField("steps", static_cast<uint64_t>(s.steps));
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
    const translagatr::RobotProfileDoc& p = b.profile;
    w.beginObject();
    wireId32(w, "id", b.id);
    w.field("generation", b.generation);
    w.field("topology", profileTopologyName(p.topology));
    w.field("summary", b.summary);
    w.key("wheels");
    w.beginArray();
    for (uint8_t i = 0; i < p.wheel_count && i < translagatr::kProfileMaxWheels; ++i) {
        const translagatr::ProfileWheel& wh = p.wheels[i];
        w.beginObject();
        w.field("port", static_cast<int64_t>(wh.encoder_port));
        w.field("sensor_id", i < b.encoders.size() ? b.encoders[i].value : std::string());
        w.field("counts_per_rev", static_cast<uint64_t>(wh.counts_per_rev));
        w.field("gear", wh.gear_micro / static_cast<double>(translagatr::kUnitMicro));
        w.field("reversed", (wh.flags & translagatr::kWheelReversed) != 0);
        w.field("radius_m", wh.radius_um * 1e-6);
        w.field("x_m", wh.x_um * 1e-6);
        w.field("y_m", wh.y_um * 1e-6);
        w.field("angle_deg", wh.angle_mdeg / 1000.0);
        w.field("travel_scale", wh.travel_scale_ppm / static_cast<double>(translagatr::kUnitMicro));
        w.endObject();
    }
    w.endArray();
    w.key("imu");
    w.beginObject();
    w.field("source", imuSourceName(p.imu_source));
    w.field("port", static_cast<int64_t>(p.imu_port));
    w.field("vex_smart_port", static_cast<int64_t>(p.vex_smart_port));
    w.field("inverted", (p.imu_flags & translagatr::kImuInvert) != 0);
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
void brainState(JsonWriter& w, const translagatr::BrainState& s) {
    const auto bit = [](uint8_t v, uint8_t b) { return (v & b) != 0; };
    w.key("state");
    w.beginObject();
    w.key("flags");
    w.beginObject();
    w.field("pose_valid", bit(s.robot_flags, translagatr::kRobotPoseValid));
    w.field("localized", bit(s.robot_flags, translagatr::kRobotLocalized));
    w.field("age_known", bit(s.robot_flags, translagatr::kRobotAgeKnown));
    w.field("anchor_command", bit(s.robot_flags, translagatr::kRobotAnchorCommand));
    w.field("anchor_configured", bit(s.robot_flags, translagatr::kRobotAnchorConfigured));
    w.endObject();
    w.key("health");
    w.beginObject();
    w.field("encoders_fresh", bit(s.health, translagatr::kHealthEncodersFresh));
    w.field("gyro_fresh", bit(s.health, translagatr::kHealthGyroFresh));
    w.field("vision_alive", bit(s.health, translagatr::kHealthVisionAlive));
    w.field("bias_calibrated", bit(s.health, translagatr::kHealthBiasCalibrated));
    w.field("pico_link", bit(s.health, translagatr::kHealthPicoLink));
    w.field("imu_initializing", bit(s.health, translagatr::kHealthImuInitializing));
    w.field("imu_failed", bit(s.health, translagatr::kHealthImuFailed));
    w.field("stationary", bit(s.health, translagatr::kHealthStationary));
    w.endObject();
    w.field("calibration", calibrationName(s.calibration));
    w.field("profile_state", profileStateName(s.profile_state));
    wireId32(w, "profile_id", s.profile_id);
    wireId32(w, "map_id", s.map_id);
    w.varyingField("estimate_id", static_cast<uint64_t>(s.estimate_id));
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
        const translagatr::PicoStatus& s = l.status;
        ageMs(w, "last_status_age_ms", l.last_status, now);
        w.field("firmware", picoFirmwareName(s.firmware));
        w.varyingField("uptime_ms", static_cast<uint64_t>(s.uptime_ms));
        w.key("imu");
        w.beginObject();
        w.field("enabled", (s.flags & translagatr::kPicoImuEnabled) != 0);
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

// Timing and throughput are varying; running and overruns are content.
// dropped is the field worker's latest-frame policy, normal with a camera.
void workerStats(JsonWriter& w, const char* key, const WorkerStatsSnapshot& s) {
    w.key(key);
    w.beginObject();
    w.field("running", s.running);
    w.varyingField("cycles", s.cycles);
    w.varyingField("rate_hz", s.rate_hz);
    w.varyingField("last_cycle_ms", s.last_cycle_ms);
    w.varyingField("mean_cycle_ms", s.mean_cycle_ms);
    w.varyingField("max_cycle_ms", s.max_cycle_ms);
    w.field("period_target_ms", s.period_target_ms);
    w.field("overruns", s.overruns);
    w.varyingField("dropped", s.dropped);
    w.varyingField("pending", s.pending);
    const std::size_t from = w.mark();
    w.key("last_cycle_host_ms");
    if (s.last_cycle_host_ms >= 0) {
        w.value(static_cast<int64_t>(s.last_cycle_host_ms));
    } else {
        w.null();
    }
    w.varying(from);
    w.endObject();
}

// Run and traffic counts are varying; faults and error counts are content.
void diagnostics(JsonWriter& w, const char* key, const Diagnostics& d) {
    w.key(key);
    w.beginObject();
    w.varyingField("cycles", d.cycles);
    w.key("functions");
    w.beginArray();
    for (const auto& kv : d.functions) {
        w.beginObject();
        w.field("label", kv.first);
        w.varyingField("runs", kv.second.runs);
        w.varyingField("ok", kv.second.ok);
        w.varyingField("no_data", kv.second.no_data);
        w.field("fault", kv.second.fault);
        // flips between ok and no_data as sensors arrive at their own rates
        w.varyingField("last", functionStatusName(kv.second.last));
        w.endObject();
    }
    w.endArray();
    w.key("links");
    w.beginArray();
    for (const auto& kv : d.links) {
        w.beginObject();
        w.field("id", kv.first);
        w.varyingField("bytes", kv.second.bytes);
        w.varyingField("packets", kv.second.packets);
        w.field("decode_errors", kv.second.decode_errors);
        w.field("seq_gaps", kv.second.seq_gaps);
        w.varyingField("requests", kv.second.requests);
        w.field("duplicates", kv.second.duplicates);
        w.varyingField("superseded", kv.second.superseded);
        w.field("stale", kv.second.stale);
        w.field("unknown_session", kv.second.unknown_session);
        w.field("unanswered", kv.second.unanswered);
        w.varyingField("replies", kv.second.replies);
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
    w.varyingField("sequence", static_cast<uint64_t>(f.frame_sequence));
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
    w.varyingField("field_invocation", f.field_invocation);

    // the robot's pose and tilt at each exposure: varying like robot itself
    w.key("pose_at_exposure");
    w.beginObject();
    w.field("status", lookupStatusName(f.pose_at_exposure.status));
    std::size_t from = w.mark();
    w.field("exact", f.pose_at_exposure.exact);
    w.varying(from);
    w.field("odometry_epoch", f.pose_at_exposure.odometry_epoch);
    from = w.mark();
    pose2(w, "odom", f.pose_at_exposure.odom_pose);
    pose2(w, "field", compose(f.field_from_odom, f.pose_at_exposure.odom_pose));
    w.varying(from);
    w.endObject();
    w.key("attitude_at_exposure");
    w.beginObject();
    w.field("status", lookupStatusName(f.attitude_at_exposure.status));
    from = w.mark();
    attitude(w, "attitude", f.attitude_at_exposure.attitude, now);
    w.varying(from);
    w.endObject();
    pose2(w, "field_from_odom", f.field_from_odom);
    w.field("anchor_revision", f.anchor_revision);

    // false: preview only, detection did not run on this frame
    w.field("has_observations", f.has_observations);
    w.field("has_trace", f.has_trace);
    w.varyingField("detector_processing_ms",
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

std::string helloDocument(const System& system, MonotonicTime now, uint64_t client_id) {
    // one consistent binding, safe while a profile boundary runs
    const std::shared_ptr<const BindingView> binding = system.bindingView();
    JsonWriter                               w;
    w.beginObject();
    w.field("type", "hello");
    w.field("contract", kInspectionContract);
    w.field("host_ms", static_cast<int64_t>(now.ms));
    if (client_id != 0) {
        // this client's row in diag.inspection.clients
        w.field("client_id", client_id);
    }
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
    w.field("state_hz", ic.state_hz);
    w.field("diag_hz", ic.diag_hz);
    w.field("preview_hz", ic.preview_hz);
    w.field("preview_quality", static_cast<int64_t>(ic.preview_quality));
    w.field("preview_max_width", static_cast<int64_t>(ic.preview_max_width));
    w.endObject();
    // what this server can send; a client subscribes to the optional parts
    w.key("features");
    w.beginObject();
    w.field("state_hz", ic.state_hz);
    w.field("max_state_hz", kMaxStateHz);
    w.field("diag_hz", ic.diag_hz);
    w.field("capture", captureAvailable(system));
    w.field("instrumentation", true);
    // TELEMETRY arrives only over a Brain link
    w.field("telemetry", system.commandsType() == "brain_link");
    w.field("history_max", static_cast<uint64_t>(kHistoryMaxEntries));
    w.field("attitude_fresh_ms", static_cast<int64_t>(kAttitudeFreshMs));
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

namespace
{

// reset_count is the count the document's data was read under (see
// underOneReset), not a fresh read.
void session(JsonWriter& w, const System& system, uint64_t reset_count) {
    w.key("session");
    w.beginObject();
    w.field("id", system.sessionId());
    w.field("reset_count", reset_count);
    w.endObject();
}

// System::reset publishes the power-on state (invalid, the next odometry
// epoch) and only then counts the reset, with the workers paused between.
// So the count is read before the data and checked after: when it held, the
// document carries nothing of an earlier session, at most the power-on
// state of the next one. A reset between the reads builds it again; stable
// is false when resets kept landing, and the last build is returned.
constexpr int kResetAttempts = 3;

template <typename Build>
std::string underOneReset(const System& system, Build build, bool* stable = nullptr) {
    std::string doc;
    for (int attempt = 0; attempt < kResetAttempts; ++attempt) {
        const uint64_t reset = system.resetCount();
        doc                  = build(reset);
        if (system.resetCount() == reset) {
            if (stable != nullptr) {
                *stable = true;
            }
            return doc;
        }
    }
    if (stable != nullptr) {
        *stable = false;
    }
    return doc;
}

// Snapshot robot fields; the state message carries exactly these.
void writeRobot(JsonWriter& w, const RobotState& robot, MonotonicTime now) {
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
}

// Trail entries oldest first; the snapshot holds them newest first.
void writeTrail(JsonWriter& w, const std::vector<PoseHistoryEntry>& recent) {
    w.key("trail");
    w.beginArray();
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
    w.endArray();
}

// Everything of the snapshot after its header. The trail only when asked.
// Values that change on every build (clocks, ages, counters, the robot,
// which state carries anyway) are marked varying for the diag identity
// check (JsonWriter::recordVarying).
void writeSnapshotBody(JsonWriter& w, const System& system, const InspectionServiceStats& service,
                       MonotonicTime now, const LocalizationSnapshot& localization,
                       uint64_t reset_count, bool with_trail) {
    const RobotState&         robot  = localization.robot;
    const LocalizationStatus& status = localization.status;

    session(w, system, reset_count);
    w.varyingField("cycle", system.cycle());
    w.field("running", system.running());
    std::size_t from = w.mark();
    writeRobot(w, robot, now);
    w.varying(from);

    w.key("localization");
    w.beginObject();
    w.field("estimator_type", status.estimator_type);
    w.varyingField("updates", status.updates);
    w.varyingField("history_size", status.history_size);
    w.field("clock_mapped", status.clock_mapped);
    w.varyingField("publication", localization.publication);
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
        // rises every cycle while the robot moves during a bias calibration;
        // dropped_why names the reason and is content
        w.varyingField("dropped_intervals", f.dropped_intervals);
        w.field("dropped_why", f.dropped_why);
        stillness(w, "stillness", f.stillness);
        w.endObject();
    }
    w.endArray();
    w.endObject();

    // oldest first, bounded
    if (with_trail) {
        writeTrail(w, localization.trail);
    }

    const std::shared_ptr<const FieldSnapshot> field = system.fieldSnapshot();
    w.key("field_snapshot");
    w.beginObject();
    w.varyingField("invocation", field->invocation);
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
            w.varyingField("last_source_sequence", static_cast<uint64_t>(o.last_source_sequence));
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
            for (const translagatr::WheelReading& r : reporting->wheels) {
                const translagatr::ProfileWheel* p = nullptr;
                for (uint8_t i = 0; running != nullptr && i < running->profile.wheel_count; ++i) {
                    if (running->profile.wheels[i].encoder_port == r.port) {
                        p = &running->profile.wheels[i];
                    }
                }
                w.beginObject();
                w.field("port", static_cast<int64_t>(r.port));
                w.field("valid", (r.flags & translagatr::kWheelValid) != 0);
                w.field("fresh", (r.flags & translagatr::kWheelFresh) != 0);
                // readings move with the robot; flags and discontinuities are content
                w.varyingField("counts", static_cast<int64_t>(r.counts));
                w.varyingField("travel_m", r.travel_um * 1e-6);
                w.field("discontinuity", static_cast<int64_t>(r.discontinuity));
                w.varyingField("age_ms", static_cast<int64_t>(r.age_ms));
                if (p != nullptr) {
                    w.field("counts_per_rev", static_cast<uint64_t>(p->counts_per_rev));
                    w.field("gear", p->gear_micro / static_cast<double>(translagatr::kUnitMicro));
                    w.field("reversed", (p->flags & translagatr::kWheelReversed) != 0);
                    w.field("radius_m", p->radius_um * 1e-6);
                    w.field("travel_scale",
                            p->travel_scale_ppm / static_cast<double>(translagatr::kUnitMicro));
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
        if (path == nullptr || path->mode == translagatr::kPathNone || path->count == 0) {
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
            w.varyingField("sequence", e.sequence);
            w.field("epoch", e.epoch);
            w.field("upstream_source", e.upstream_source);
            w.field("upstream_clock", e.upstream_clock);
            w.varyingField("upstream_sequence", e.upstream_sequence);
            w.field("upstream_epoch", e.upstream_epoch);
            w.endObject();
        }
    }
    w.endArray();

    w.key("workers");
    w.beginObject();
    workerStats(w, "estimation", system.estimationStats());
    workerStats(w, "field", system.fieldStats());
    // transport counters: they change with every message sent
    const std::size_t inspection_begin = w.mark();
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
    w.field("diag_rate_hz", service.diag_rate_hz);
    w.field("states_queued", service.states_queued);
    w.field("states_replaced", service.states_replaced);
    w.field("diags_queued", service.diags_queued);
    w.field("diags_replaced", service.diags_replaced);
    w.field("diags_skipped", service.diags_skipped);
    w.field("hellos_queued", service.hellos_queued);
    w.field("histories_queued", service.histories_queued);
    w.field("events_queued", service.events_queued);
    w.field("captures_queued", service.captures_queued);
    w.field("pongs_queued", service.pongs_queued);
    w.field("telemetry_queued", service.telemetry_queued);
    w.field("instrumentation_queued", service.instrumentation_queued);
    w.field("messages_replaced", service.messages_replaced);
    w.field("messages_refused", service.messages_refused);
    w.field("closed_stalled", service.closed_stalled);
    w.field("closed_reliable_overflow", service.closed_reliable_overflow);
    w.field("closed_protocol", service.closed_protocol);
    w.field("closed_peer", service.closed_peer);
    w.field("flow_control_clients", service.flow_control_clients);
    w.endObject();
    w.varying(inspection_begin);
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
}

void buildStats(JsonWriter& w, const char* key, const DocBuildStats& s) {
    w.key(key);
    w.beginObject();
    w.field("count", s.count);
    w.field("last_us", s.last_us);
    w.field("mean_us", s.mean_us);
    w.field("recent_mean_us", s.recent_mean_us);
    w.field("max_us", s.max_us);
    w.field("last_bytes", s.last_bytes);
    w.field("mean_bytes", s.mean_bytes);
    w.endObject();
}

// A duration, null when never measured (negative).
void optionalMs(JsonWriter& w, const char* key, double ms) {
    w.key(key);
    if (ms >= 0.0) {
        w.value(ms);
    } else {
        w.null();
    }
}

// diag.inspection: per-client queues and document costs.
void feedStats(JsonWriter& w, const InspectionServiceStats& service,
               const InspectionFeedStats& feed) {
    w.key("inspection");
    w.beginObject();
    w.field("contract", kInspectionContract);
    w.field("state_hz", feed.state_hz);
    w.field("diag_hz", feed.diag_hz);
    w.key("build");
    w.beginObject();
    buildStats(w, "state", feed.state);
    buildStats(w, "diag", feed.diag);
    buildStats(w, "history", feed.history);
    buildStats(w, "instrumentation", feed.instrumentation);
    w.endObject();
    w.key("closed");
    w.beginObject();
    w.field("stalled", service.closed_stalled);
    w.field("reliable_overflow", service.closed_reliable_overflow);
    w.field("protocol", service.closed_protocol);
    w.field("peer", service.closed_peer);
    w.endObject();
    w.field("messages_replaced", service.messages_replaced);
    w.field("messages_refused", service.messages_refused);
    w.key("recent_closes");
    w.beginArray();
    for (const WsCloseRecord& c : feed.recent_closes) {
        w.beginObject();
        w.field("id", c.id);
        w.field("reason", c.reason);
        w.field("host_ms", c.host_ms);
        w.endObject();
    }
    w.endArray();
    w.key("clients");
    w.beginArray();
    for (const FeedClientStats& c : feed.clients) {
        const WsClientStats& q = c.queue;
        w.beginObject();
        w.field("id", q.id);
        w.field("connected_ms", q.connected_ms);
        w.field("queued_bytes", static_cast<uint64_t>(q.queued_bytes));
        w.field("in_flight_bytes", static_cast<uint64_t>(q.in_flight_bytes));
        w.field("reliable_backlog", static_cast<uint64_t>(q.reliable_backlog));
        w.field("reliable_bytes", static_cast<uint64_t>(q.reliable_bytes));
        w.field("slots_pending", static_cast<uint64_t>(q.slots_pending));
        w.field("diag_skipped", c.diag_skipped);
        w.field("flow_control", q.flow_control);
        w.field("unacked_bytes", q.unacked_bytes);
        optionalMs(w, "ack_rtt_ms", q.ack_rtt_ms);
        optionalMs(w, "ack_rtt_min_ms", q.ack_rtt_min_ms);
        w.field("pace_gap_ms", q.pace_gap_ms);
        w.key("subscription");
        w.beginObject();
        w.field("state_hz", c.subscription.state_hz);
        w.field("diag", c.subscription.diag);
        w.field("instrumentation", c.subscription.instrumentation);
        w.field("raw", c.subscription.raw);
        w.field("decoded", c.subscription.decoded);
        w.field("telemetry", c.subscription.telemetry);
        w.field("preview_hz", c.subscription.preview_hz);
        w.endObject();
        w.key("channels");
        w.beginObject();
        for (const WsChannelStats& ch : q.channels) {
            w.key(ch.name);
            w.beginObject();
            w.field("replaceable", ch.replaceable);
            w.field("queued", ch.queued);
            w.field("sent", ch.sent);
            w.field("replaced", ch.replaced);
            w.field("refused", ch.refused);
            w.field("dropped", ch.dropped);
            w.field("bytes", ch.bytes);
            optionalMs(w, "last_latency_ms", ch.last_latency_ms);
            optionalMs(w, "last_acked_ms", ch.last_acked_ms);
            w.endObject();
        }
        w.endObject();
        w.endObject();
    }
    w.endArray();
    w.endObject();
}

void hubStats(JsonWriter& w, const DiagnosticsHub& hub) {
    const DiagHubStats s = hub.stats();
    w.key("hub");
    w.beginObject();
    // posted, queued and drained move with every record; drops are content
    std::size_t from = w.mark();
    w.key("posted");
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        w.field(diagKindName(static_cast<DiagKind>(k)), s.posted[static_cast<std::size_t>(k)]);
    }
    w.endObject();
    w.varying(from);
    w.key("dropped");
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        w.field(diagKindName(static_cast<DiagKind>(k)), s.dropped[static_cast<std::size_t>(k)]);
    }
    w.endObject();
    w.varyingField("queued", static_cast<uint64_t>(s.queued));
    w.field("capacity", static_cast<uint64_t>(s.capacity));
    w.varyingField("drained", s.drained);
    w.endObject();
}

// FNV-1a over the document, leaving out the spans. Spans nest (a varying
// age inside the varying robot), so they are sorted by start first.
uint64_t contentHash(const std::string& s, JsonSpans spans) {
    std::sort(spans.begin(), spans.end());
    uint64_t    h   = 1469598103934665603ull;
    std::size_t pos = 0;
    auto        mix = [&](std::size_t from, std::size_t to) {
        for (std::size_t i = from; i < to; ++i) {
            h ^= static_cast<unsigned char>(s[i]);
            h *= 1099511628211ull;
        }
    };
    for (const auto& span : spans) {
        mix(pos, std::min(span.first, s.size()));
        pos = std::max(pos, std::min(span.second, s.size()));
    }
    mix(pos, s.size());
    return h;
}

// Wire enum names of the Brain's motion telemetry (actuGATR MotionState,
// MotionReason, DriveFault; investiGATR PlanMode). Keep in step with
// brain/actuGATR/include/actugatr/{motion,drive}.h; unknown values print
// "unknown" next to the number.
const char* motionStateName(uint8_t v) {
    static const char* const kNames[] = {"idle",      "waiting",   "running", "settling",
                                         "completed", "cancelled", "failed"};
    return v < sizeof(kNames) / sizeof(kNames[0]) ? kNames[v] : "unknown";
}

const char* motionReasonName(uint8_t v) {
    static const char* const kNames[] = {
        "none",           "invalid_command",       "invalid_config",     "input_unavailable",
        "no_profile",     "calibrating",           "placement_required", "input_lost",
        "frame_changed",  "field_unavailable",     "map_mismatch",       "unknown_reference",
        "not_reference",  "reference_unavailable", "unsupported_model",  "start_out_of_bounds",
        "start_blocked",  "goal_out_of_bounds",    "goal_blocked",       "no_path",
        "tracking_error", "plan_limit",            "timed_out",          "source_changed",
        "cancelled_by_caller"};
    return v < sizeof(kNames) / sizeof(kNames[0]) ? kNames[v] : "unknown";
}

const char* planModeName(uint8_t v) {
    switch (v) {
    case 0: return "direct";
    case 1: return "avoiding";
    default: return "unknown";
    }
}

const char* driveFaultName(uint8_t v) {
    static const char* const kNames[] = {"none", "wrong_frame", "non_finite", "unsupported_motion",
                                         "stale"};
    return v < sizeof(kNames) / sizeof(kNames[0]) ? kNames[v] : "unknown";
}

void codeAndName(JsonWriter& w, const char* key, const char* name_key, uint8_t v,
                 const char* name) {
    w.field(key, static_cast<int64_t>(v));
    w.field(name_key, name);
}

// A document an object writer produced, with the message header in front:
// {"type":...,"seq":...,"host_ms":..., <the object's own fields>}.
std::string flatMessage(const char* type, uint64_t seq, MonotonicTime now,
                        const std::string& object) {
    JsonWriter w;
    w.beginObject();
    w.field("type", type);
    w.field("seq", seq);
    w.field("host_ms", static_cast<int64_t>(now.ms));
    std::string out = w.take();
    if (object.size() > 2 && object.front() == '{' && object.back() == '}') {
        out += ',';
        out.append(object, 1, std::string::npos);
    } else {
        out += '}';
    }
    return out;
}

} // namespace

const char* attitudeStatus(const Attitude& a, MonotonicTime now) {
    if (a.valid) {
        const bool fresh = a.measuredAt.domain == ClockDomain::kHost &&
                           now.domain == ClockDomain::kHost &&
                           now.ms - a.measuredAt.ms <= kAttitudeFreshMs;
        return fresh ? "measured" : "stale";
    }
    // the estimator keeps the old measurement time when a tilt ages out
    if (a.measuredAt.isSet()) {
        return "stale";
    }
    // a source is configured but nothing was measured or could be timed
    if (!a.source.empty()) {
        return "unavailable";
    }
    return a.assumed_level ? "assumed_level" : "unavailable";
}

bool captureAvailable(const System& system) {
    const CaptureService* capture = system.capture();
    if (capture == nullptr) {
        return false;
    }
    JsonWriter w;
    capture->writeStatus(w);
    return w.str().find("\"available\":true") != std::string::npos;
}

std::string snapshotDocument(const System& system, const InspectionServiceStats& service,
                             MonotonicTime now, std::size_t trail_max_entries) {
    return underOneReset(system, [&](uint64_t reset) {
        const LocalizationSnapshot localization = system.robotFeed()->snapshot(trail_max_entries);
        JsonWriter                 w;
        w.beginObject();
        w.field("type", "snapshot");
        w.field("contract", kSnapshotContract);
        w.field("host_ms", static_cast<int64_t>(now.ms));
        writeSnapshotBody(w, system, service, now, localization, reset, true);
        w.endObject();
        return w.take();
    });
}

std::string stateDocument(const System& system, uint64_t seq, MonotonicTime now, int64_t now_us,
                          uint64_t* publication) {
    uint64_t          doc_publication = 0;
    bool              stable          = false;
    const std::string doc             = underOneReset(
        system,
        [&](uint64_t reset) {
            const LocalizationSnapshot localization = system.robotFeed()->snapshot(0);
            const LocalizationStatus&  status       = localization.status;
            doc_publication                         = localization.publication;
            JsonWriter w;
            w.beginObject();
            w.field("type", "state");
            w.field("seq", seq);
            w.field("host_ms", static_cast<int64_t>(now.ms));
            w.field("host_us", now_us);
            session(w, system, reset);
            w.field("publication", localization.publication);
            writeRobot(w, localization.robot, now);
            w.key("localization");
            w.beginObject();
            w.field("all_ready", status.allReady());
            w.field("stationary", status.stationary());
            w.field("continuity_breaks", status.continuity_breaks);
            w.field("last_break", status.last_break);
            w.endObject();
            w.endObject();
            return w.take();
        },
        &stable);
    if (!stable) {
        return {};   // the next tick sends a state under one reset count
    }
    if (publication != nullptr) {
        *publication = doc_publication;
    }
    return doc;
}

std::string historyDocument(const System& system, uint64_t seq, MonotonicTime now,
                            std::size_t max_entries) {
    // unstable only when resets keep landing; a new hello and history follow each
    return underOneReset(system, [&](uint64_t reset) {
        const LocalizationSnapshot localization = system.robotFeed()->snapshot(max_entries);
        JsonWriter                 w;
        w.beginObject();
        w.field("type", "history");
        w.field("seq", seq);
        w.field("host_ms", static_cast<int64_t>(now.ms));
        session(w, system, reset);
        w.field("publication", localization.publication);
        w.field("max_entries", static_cast<uint64_t>(max_entries));
        writeTrail(w, localization.trail);
        w.endObject();
        return w.take();
    });
}

std::string diagDocument(const System& system, const InspectionServiceStats& service,
                         const InspectionFeedStats& feed, uint64_t seq, MonotonicTime now,
                         uint64_t* content_hash) {
    JsonSpans         spans;
    bool              stable = false;
    const std::string doc    = underOneReset(
        system,
        [&](uint64_t reset) {
            spans.clear();
            const LocalizationSnapshot localization = system.robotFeed()->snapshot(0);
            JsonWriter                 w;
            w.recordVarying(&spans);
            w.beginObject();
            w.field("type", "diag");
            w.field("contract", kInspectionContract);
            w.varyingField("seq", seq);
            w.varyingField("host_ms", static_cast<int64_t>(now.ms));
            writeSnapshotBody(w, system, service, now, localization, reset, false);
            const std::size_t from = w.mark();
            feedStats(w, service, feed);
            w.varying(from);
            hubStats(w, system.diagHub());
            w.endObject();
            return w.take();
        },
        &stable);
    if (!stable) {
        return {};   // the next scheduled diag goes under one reset count
    }
    if (content_hash != nullptr) {
        *content_hash = contentHash(doc, spans);
    }
    return doc;
}

std::string eventDocument(const RuntimeEvent& e) {
    JsonWriter w;
    w.beginObject();
    w.field("type", "event");
    w.field("seq", e.sequence);
    hostMs(w, "host_ms", e.at);
    w.field("text", e.text);
    w.endObject();
    return w.take();
}

std::string pongDocument(const std::string& id_token, const std::string& client_ms_token,
                         MonotonicTime now, int64_t now_us) {
    JsonWriter w;
    w.beginObject();
    w.field("type", "pong");
    w.key("id");
    if (id_token.empty()) {
        w.null();
    } else {
        w.raw(id_token);
    }
    w.key("client_ms");
    if (client_ms_token.empty()) {
        w.null();
    } else {
        w.raw(client_ms_token);
    }
    w.field("host_ms", static_cast<int64_t>(now.ms));
    w.field("host_us", now_us);
    w.endObject();
    return w.take();
}

bool decodeTelemetryRecord(const DiagBrainTelemetry& rec, translagatr::BrainTelemetry& out) {
    if (rec.len > sizeof(rec.body)) {
        return false;
    }
    return translagatr::decodeTelemetryBody(rec.body, rec.len, out);
}

std::string telemetryDocument(uint64_t seq, MonotonicTime now, const DiagRecord& record,
                              const DiagBrainTelemetry& rec,
                              const translagatr::BrainTelemetry& t) {
    using namespace translagatr;
    JsonWriter w;
    w.beginObject();
    w.field("type", "telemetry");
    w.field("seq", seq);
    w.field("host_ms", static_cast<int64_t>(now.ms));
    // Pi arrival of the report; stamp_ms is the Brain clock
    const int64_t received_ms = record.host_us / 1000;
    w.field("received_host_ms", received_ms);
    w.field("age_ms", static_cast<int64_t>(now.ms) - received_ms);
    w.field("session", static_cast<uint64_t>(rec.session));
    w.field("stamp_ms", static_cast<uint64_t>(t.stamp_ms));
    w.field("flags", static_cast<int64_t>(t.flags));
    w.key("attitude");
    if ((t.flags & kTelemetryAttitude) != 0) {
        w.beginObject();
        w.field("roll_deg", t.roll_cdeg / 100.0);
        w.field("pitch_deg", t.pitch_cdeg / 100.0);
        w.endObject();
    } else {
        w.null();
    }
    w.key("motion");
    if ((t.flags & kTelemetryMotion) != 0) {
        w.beginObject();
        w.field("command_id", static_cast<uint64_t>(t.command_id));
        codeAndName(w, "state", "state_name", t.motion_state, motionStateName(t.motion_state));
        codeAndName(w, "reason", "reason_name", t.motion_reason,
                    motionReasonName(t.motion_reason));
        codeAndName(w, "mode", "mode_name", t.plan_mode, planModeName(t.plan_mode));
        w.field("segment", static_cast<int64_t>(t.segment));
        w.field("segment_count", static_cast<int64_t>(t.segment_count));
        w.key("target");
        // without the target bit the Brain has no resolved destination
        if ((t.flags & kTelemetryTarget) != 0) {
            w.beginObject();
            w.field("x_m", t.target_x_mm / 1000.0);
            w.field("y_m", t.target_y_mm / 1000.0);
            w.field("heading_deg", t.target_heading_cdeg / 100.0);
            w.endObject();
        } else {
            w.null();
        }
        w.key("cmd");
        w.beginObject();
        w.field("vx_m_s", t.cmd_vx_mm_s / 1000.0);
        w.field("vy_m_s", t.cmd_vy_mm_s / 1000.0);
        w.field("omega_deg_s", t.cmd_omega_cdeg_s / 100.0);
        w.endObject();
        w.field("cross_track_m", t.cross_track_mm / 1000.0);
        w.field("distance_error_m", t.distance_error_mm / 1000.0);
        w.field("heading_error_deg", t.heading_error_cdeg / 100.0);
        codeAndName(w, "drive_fault", "drive_fault_name", t.drive_fault,
                    driveFaultName(t.drive_fault));
        w.endObject();
    } else {
        w.null();
    }
    w.key("wheels");
    if ((t.flags & kTelemetryWheels) != 0) {
        w.beginObject();
        w.key("rpm");
        w.beginArray();
        for (uint8_t i = 0; i < t.wheel_count && i < kTelemetryWheelsMax; ++i) {
            w.value(t.wheel_rpm_x10[i] / 10.0);
        }
        w.endArray();
        w.endObject();
    } else {
        w.null();
    }
    w.endObject();
    return w.take();
}

std::string instrumentationDocument(const System& system, uint64_t seq, MonotonicTime now,
                                    const InstrumentationOptions& options) {
    JsonWriter w;
    writeInstrumentation(w, system, options);
    return flatMessage("instrumentation", seq, now, w.str());
}

std::string captureDocument(const CaptureService& capture, uint64_t seq, MonotonicTime now) {
    JsonWriter w;
    capture.writeStatus(w);
    return flatMessage("capture", seq, now, w.str());
}

std::string frameHeaderDocument(const DetectionFrameSnapshot& frame, int preview_width_px,
                                int preview_height_px, long quality, double encode_ms,
                                MonotonicTime now, const System* system) {
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
    if (system != nullptr) {
        // the detection_frames entry of exactly this frame, so overlays bind
        // without waiting for a diag that names it
        w.key("detection");
        detectionFrame(w, *system, frame, now);
    }
    w.endObject();
    return w.take();
}

} // namespace navigatr
