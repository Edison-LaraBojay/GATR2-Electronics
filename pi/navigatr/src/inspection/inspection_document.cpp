// inspection_document.cpp

#include "inspection/inspection_document.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>

#include "config/field_map.h"
#include "impl/resources/tag_detectors.h"
#include "inspection/json_writer.h"
#include "math/angles.h"
#include "payloads/camera_frames.h"
#include "resources/camera.h"
#include "resources/robot_frames.h"
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
    JsonWriter w;
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
            pose2(w, "nominal", lm.nominal);
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
    for (const auto& e : system.sensorCatalog().entries()) {
        if (e.payload.stable_name == payload_names::kCameraFrame) {
            w.value(e.id.value);
        }
    }
    w.endArray();

    w.key("localization");
    w.beginObject();
    w.field("estimator_type", system.localization().estimatorType());
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
    for (const std::string& s : system.warnings()) {
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
    w.key("functions");
    w.beginArray();
    for (const ObservationFunctionStatus& f : status.functions) {
        w.beginObject();
        w.field("id", f.id);
        w.field("type", f.type);
        w.field("ready", f.ready);
        w.field("note", f.note);
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
        w.field("stream_on", c.stream_on);
        w.field("init_sequence", c.init_sequence);
        pose2(w, "init_pose", c.init_pose);
        w.field("mode", static_cast<int64_t>(c.mode));
        w.field("object_requested", c.object_requested);
        w.field("object_wire_id", static_cast<int64_t>(c.object_wire_id));
        w.field("object_sequence", c.object_sequence);
        w.endObject();
    }

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
