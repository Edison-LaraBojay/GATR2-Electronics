// synthetic_rig.cpp

#include "impl/resources/synthetic_rig.h"

#include <algorithm>
#include <cmath>

#include "impl/resources/tag_detectors.h"
#include "math/angles.h"
#include "math/camera_model.h"
#include "math/quaternion.h"
#include "payloads/attitude_samples.h"
#include "payloads/camera_frames.h"
#include "payloads/pico_telemetry_samples.h"

namespace navigatr
{

RigTruth SyntheticRig::truthAt(MonotonicTime host) const {
    const double t   = std::max(0.0, host.ms / 1000.0 - trajectory_.hold_s);
    const double phi = trajectory_.start_rad + 2.0 * kPi * t / trajectory_.period_s;
    RigTruth     out;
    out.pose.x_m = trajectory_.center_x_m + trajectory_.radius_m * std::cos(phi);
    out.pose.y_m = trajectory_.center_y_m + trajectory_.radius_m * std::sin(phi);
    if (trajectory_.facing == "tangent") {
        out.pose.heading_rad = wrapAngle(phi + kPi / 2.0);
    } else {
        out.pose.heading_rad = wrapAngle(std::atan2(trajectory_.center_y_m - out.pose.y_m,
                                                    trajectory_.center_x_m - out.pose.x_m));
    }
    if (attitude_.rock_rad > 0.0 && attitude_.period_s > 0.0) {
        const double w = 2.0 * kPi * t / attitude_.period_s;
        out.roll_rad   = attitude_.rock_rad * std::sin(w);
        out.pitch_rad  = 0.5 * attitude_.rock_rad * std::cos(w);
    }
    return out;
}

Pose2D SyntheticRig::landmarkTruth(const FieldObjectId& id) const {
    Pose2D nominal;
    if (field_ != nullptr) {
        if (const LandmarkDecl* lm = field_->find(id)) {
            nominal = lm->nominal;
        }
    }
    const auto it = displacements_.find(id.value);
    if (it == displacements_.end()) {
        return nominal;
    }
    Pose2D out;
    out.x_m         = nominal.x_m + it->second.x_m;
    out.y_m         = nominal.y_m + it->second.y_m;
    out.heading_rad = wrapAngle(nominal.heading_rad + it->second.heading_rad);
    return out;
}

void SyntheticRig::render(const RigTruth& truth, std::vector<uint8_t>& y8) const {
    const CameraIntrinsics& K = *camera_.intrinsics;
    const int               W = K.calibrated_width_px;
    const int               H = K.calibrated_height_px;
    y8.assign(static_cast<std::size_t>(W) * H, camera_.background);
    if (field_ == nullptr) {
        return;
    }

    // camera pose in the field: the physical rocking is in the chain
    // whether or not anything reports it
    Transform3 T_field_robot;
    T_field_robot.R   = rotationFromEuler(truth.roll_rad, truth.pitch_rad, truth.pose.heading_rad);
    T_field_robot.x_m = truth.pose.x_m;
    T_field_robot.y_m = truth.pose.y_m;
    T_field_robot.z_m = 0.0;
    const Transform3 T_field_camera = compose(T_field_robot, T_robot_camera_);
    const Transform3 T_camera_field = inverse(T_field_camera);

    for (const LandmarkDecl& lm : field_->landmarks) {
        const Pose2D     landmark = landmarkTruth(lm.id);
        const Transform3 T_field_landmark = transform3FromPlanar(landmark);
        for (const TagMountDecl& mount : lm.mounts) {
            const Transform3 T_camera_tag =
                compose(T_camera_field, compose(T_field_landmark, mount.T_landmark_tag_surface));
            // in front of the lens and facing it
            if (T_camera_tag.x_m <= 0.05 || T_camera_tag.R.m[0][0] > -0.15) {
                continue;
            }
            const std::string key = mount.family + "/" + std::to_string(mount.observed_id);
            auto              cached = tag_cache_.find(key);
            if (cached == tag_cache_.end()) {
                std::vector<uint8_t> cells;
                int                  total = 0;
                std::string          err;
                if (!AprilTagDetector::renderTag(mount.family, mount.observed_id, cells, total,
                                                 err)) {
                    continue;   // unknown family: nothing to draw
                }
                cached = tag_cache_.emplace(key, std::make_pair(total, std::move(cells))).first;
            }
            const int   total_width = cached->second.first;
            const auto& cells       = cached->second.second;
            int         width_at_border = 0, tw = 0;
            std::string geometry_err;
            if (!AprilTagDetector::familyGeometry(mount.family, width_at_border, tw,
                                                  geometry_err)) {
                continue;
            }
            // physical cell size from the detector-corner square
            const double cell_m  = mount.detection_size_m / width_at_border;
            const double quiet_m = cell_m * (total_width / 2.0 + 1.0);   // one cell of white

            // bounding box of the quiet square in the captured image
            double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
            bool   visible = true;
            for (int c = 0; c < 4 && visible; ++c) {
                const double y_s = (c == 0 || c == 3) ? -quiet_m : quiet_m;
                const double z_s = (c < 2) ? -quiet_m : quiet_m;
                double       x_e = 0.0, y_e = 0.0, z_e = 0.0;
                transformPoint3(T_camera_tag, 0.0, y_s, z_s, x_e, y_e, z_e);
                double u = 0.0, v = 0.0;
                if (x_e <= 0.02 || !projectEngineering(K, x_e, y_e, z_e, u, v)) {
                    visible = false;
                    break;
                }
                umin = std::min(umin, u);
                umax = std::max(umax, u);
                vmin = std::min(vmin, v);
                vmax = std::max(vmax, v);
            }
            if (!visible) {
                continue;
            }
            const int u0 = std::max(0, static_cast<int>(std::floor(umin)) - 1);
            const int u1 = std::min(W - 1, static_cast<int>(std::ceil(umax)) + 1);
            const int v0 = std::max(0, static_cast<int>(std::floor(vmin)) - 1);
            const int v1 = std::min(H - 1, static_cast<int>(std::ceil(vmax)) + 1);
            if (u0 > u1 || v0 > v1) {
                continue;
            }

            const Transform3 T_tag_camera = inverse(T_camera_tag);
            for (int v = v0; v <= v1; ++v) {
                for (int u = u0; u <= u1; ++u) {
                    // ray of this pixel center in the engineering camera frame
                    double xn = 0.0, yn = 0.0;
                    undistortPixel(K, u + 0.5, v + 0.5, xn, yn);
                    double dx_e = 0.0, dy_e = 0.0, dz_e = 0.0;
                    engineeringFromOptical(xn, yn, 1.0, dx_e, dy_e, dz_e);
                    // into the tag surface frame: origin and direction
                    const double ox = T_tag_camera.x_m, oy = T_tag_camera.y_m,
                                 oz = T_tag_camera.z_m;
                    const Rotation3& R  = T_tag_camera.R;
                    const double     dx = R.m[0][0] * dx_e + R.m[0][1] * dy_e + R.m[0][2] * dz_e;
                    const double     dy = R.m[1][0] * dx_e + R.m[1][1] * dy_e + R.m[1][2] * dz_e;
                    const double     dz = R.m[2][0] * dx_e + R.m[2][1] * dy_e + R.m[2][2] * dz_e;
                    if (std::fabs(dx) < 1e-9) {
                        continue;
                    }
                    const double t = -ox / dx;   // plane x_S = 0
                    if (t <= 0.0) {
                        continue;
                    }
                    const double y_s = oy + t * dy;
                    const double z_s = oz + t * dz;
                    // canonical surface to native tag image: right = y_S, down = -z_S
                    const double x_sd = y_s;
                    const double y_sd = -z_s;
                    if (std::fabs(x_sd) > quiet_m || std::fabs(y_sd) > quiet_m) {
                        continue;
                    }
                    const double half = cell_m * total_width / 2.0;
                    uint8_t      value = 255;   // quiet zone
                    if (std::fabs(x_sd) < half && std::fabs(y_sd) < half) {
                        const int col = static_cast<int>(std::floor((x_sd + half) / cell_m));
                        const int row = static_cast<int>(std::floor((y_sd + half) / cell_m));
                        const int cc  = std::min(std::max(col, 0), total_width - 1);
                        const int rr  = std::min(std::max(row, 0), total_width - 1);
                        value = cells[static_cast<std::size_t>(rr) * total_width + cc];
                    }
                    y8[static_cast<std::size_t>(v) * W + u] = value;
                }
            }
        }
    }
}

namespace
{

struct EncoderOutput {
    OutputId         id;
    const WheelDecl* wheel = nullptr;
    double           ux = 1.0, uy = 0.0, k_m = 0.0, sign = 1.0;
    double           fractional = 0.0;   // counts not yet emitted
    int64_t          counts     = 0;
};

} // namespace

class SyntheticRigExecutable
{
public:
    SyntheticRigExecutable(std::shared_ptr<SyntheticRig> rig) : rig_(std::move(rig)) {}

    std::shared_ptr<SyntheticRig> rig_;
    std::vector<EncoderOutput>    encoders_;
    OutputId                      imu_;        // empty when not configured
    OutputId                      attitude_;   // empty when not configured
    OutputId                      camera_;     // empty when not configured

    // telemetry state
    bool          have_tick_    = false;
    MonotonicTime last_tick_;
    Pose2D        prev_pose_;
    double        prev_rate_mdps_ = 0.0;
    double        accum_mdeg_     = 0.0;
    uint64_t      accum_epoch_    = 0;
    uint64_t      device_epoch_   = 0;
    MonotonicTime last_stamp_;
    int32_t       last_rate_mdps_ = 0;
    RigTruth      last_truth_;

    // camera state
    MonotonicTime last_frame_;
    uint32_t      frame_sequence_ = 0;

    ResourcePollResult execute(const ExecutionContext& context) {
        ResourcePollResult result;
        result.state = SourceState::kValid;

        const int64_t tick_ms = static_cast<int64_t>(std::llround(1000.0 / rig_->tick_hz_));
        bool          ticked  = false;
        if (!have_tick_) {
            have_tick_ = true;
            last_tick_ = hostTime(context.now.ms - tick_ms);
            prev_pose_ = rig_->truthAt(last_tick_).pose;
        }
        while (context.now.ms - last_tick_.ms >= tick_ms) {
            last_tick_ = hostTime(last_tick_.ms + tick_ms);
            const RigTruth truth = rig_->truthAt(last_tick_);
            const Pose2D   delta = compose(inverse(prev_pose_), truth.pose);
            const double   dt_s  = tick_ms / 1000.0;
            for (EncoderOutput& e : encoders_) {
                const double travel_m =
                    e.ux * delta.x_m + e.uy * delta.y_m + e.k_m * delta.heading_rad;
                const double counts =
                    e.sign * travel_m / (2.0 * kPi * e.wheel->radius_m) *
                    rig_->counts_per_revolution_;
                e.fractional += counts;
                const double whole = std::trunc(e.fractional);
                e.counts += static_cast<int64_t>(whole);
                e.fractional -= whole;
            }
            const double rate_mdps =
                radToDeg(delta.heading_rad / dt_s) * 1000.0 + rig_->gyro_bias_mdps_;
            if (rig_->ticks_ > 0) {
                accum_mdeg_ += 0.5 * (prev_rate_mdps_ + rate_mdps) * dt_s;
            }
            prev_rate_mdps_ = rate_mdps;
            last_rate_mdps_ = static_cast<int32_t>(std::llround(rate_mdps));
            prev_pose_      = truth.pose;
            last_truth_     = truth;
            last_stamp_     = deviceTime(last_tick_.ms + rig_->device_offset_ms_);
            ++rig_->ticks_;
            ticked = true;
        }

        const auto provenance = [&](Publication& p) {
            p.upstream.source   = "synthetic_rig";
            p.upstream.clock    = "synthetic_device";
            p.upstream.sequence = rig_->ticks_;
            p.upstream.epoch    = device_epoch_;
            p.receivedAt        = context.now;
        };

        for (const EncoderOutput& e : encoders_) {
            OutputPoll poll;
            poll.id = e.id;
            if (ticked) {
                poll.result.state = SourceState::kValid;
                Publication p;
                p.measuredAt = last_stamp_;
                provenance(p);
                p.payload = TypedPayload::store(
                    PicoEncoderCounts{static_cast<int32_t>(static_cast<uint32_t>(e.counts))},
                    payload_names::kPicoEncoderCounts);
                poll.result.publication = std::move(p);
            } else {
                poll.result.state = rig_->ticks_ > 0 ? SourceState::kValid : SourceState::kNoDataYet;
            }
            result.outputs.push_back(std::move(poll));
        }
        if (!imu_.empty()) {
            OutputPoll poll;
            poll.id = imu_;
            if (ticked) {
                poll.result.state = SourceState::kValid;
                Publication p;
                p.measuredAt = last_stamp_;
                provenance(p);
                PicoGyroRate rate;
                rate.rate_mdps         = last_rate_mdps_;
                rate.accumulated_mdeg  = accum_mdeg_;
                rate.accumulated_epoch = accum_epoch_;
                p.payload = TypedPayload::store(rate, payload_names::kPicoGyroRate);
                poll.result.publication = std::move(p);
            } else {
                poll.result.state = rig_->ticks_ > 0 ? SourceState::kValid : SourceState::kNoDataYet;
            }
            result.outputs.push_back(std::move(poll));
        }
        if (!attitude_.empty()) {
            OutputPoll poll;
            poll.id = attitude_;
            if (!rig_->attitude_.measured) {
                poll.result.state      = SourceState::kUnavailable;
                poll.result.diagnostic = "no attitude source in this rig";
            } else if (ticked) {
                poll.result.state = SourceState::kValid;
                Publication p;
                p.measuredAt = last_stamp_;
                provenance(p);
                AttitudeSample sample;
                sample.q_reference_body =
                    quaternionFromEuler(last_truth_.roll_rad, last_truth_.pitch_rad, 0.0);
                sample.reference = "gravity";
                sample.has_yaw   = false;
                sample.quality   = 1.0;
                sample.epoch     = device_epoch_;
                p.payload = TypedPayload::store(sample, payload_names::kAttitudeSample);
                poll.result.publication = std::move(p);
            } else {
                poll.result.state = rig_->ticks_ > 0 ? SourceState::kValid : SourceState::kNoDataYet;
            }
            result.outputs.push_back(std::move(poll));
        }
        if (!camera_.empty()) {
            OutputPoll poll;
            poll.id = camera_;
            const int64_t period_ms =
                static_cast<int64_t>(std::llround(1000.0 / rig_->camera_.frame_rate_hz));
            if (!last_frame_.isSet() || context.now.ms - last_frame_.ms >= period_ms) {
                last_frame_ = context.now;
                const MonotonicTime exposure =
                    hostTime(context.now.ms - rig_->camera_.latency_ms);
                const RigTruth truth = rig_->truthAt(exposure);
                auto           pixels = std::make_shared<std::vector<uint8_t>>();
                rig_->render(truth, *pixels);
                ++rig_->frames_rendered_;

                CameraFramePayload payload;
                payload.frame.sequence   = ++frame_sequence_;
                payload.frame.epoch      = device_epoch_;
                payload.frame.exposureAt = exposure;
                payload.frame.receivedAt = context.now;
                payload.frame.exposure_uncertainty_ms = 0;
                payload.frame.exposure_time_reliable  = true;
                payload.frame.width_px   = rig_->camera_.intrinsics->calibrated_width_px;
                payload.frame.height_px  = rig_->camera_.intrinsics->calibrated_height_px;
                payload.frame.y8         = pixels;
                payload.engineering_frame  = rig_->camera_.frame_id;
                payload.intrinsics         = rig_->camera_.intrinsics;

                poll.result.state = SourceState::kValid;
                Publication p;
                p.measuredAt        = exposure;
                p.receivedAt        = context.now;
                p.upstream.source   = "synthetic_rig.camera";
                p.upstream.clock    = "host";
                p.upstream.sequence = frame_sequence_;
                p.upstream.epoch    = device_epoch_;
                p.payload = TypedPayload::store(std::move(payload), payload_names::kCameraFrame);
                poll.result.publication = std::move(p);
            } else {
                poll.result.state = frame_sequence_ > 0 ? SourceState::kValid : SourceState::kNoDataYet;
            }
            result.outputs.push_back(std::move(poll));
        }
        return result;
    }

    void reset() {
        // a rig restart: counters and accumulators start over in a new
        // device epoch; the truth keeps moving
        for (EncoderOutput& e : encoders_) {
            e.counts     = 0;
            e.fractional = 0.0;
        }
        have_tick_      = false;
        accum_mdeg_     = 0.0;
        accum_epoch_    = 0;
        prev_rate_mdps_ = 0.0;
        ++device_epoch_;
        last_frame_     = MonotonicTime{};
        frame_sequence_ = 0;
        rig_->ticks_    = 0;
    }
};

ResourceInstance make_synthetic_rig(const ConfigNode&              node,
                                    ResourceInitializationContext& context,
                                    std::string&                   err) {
    auto rig  = std::make_shared<SyntheticRig>();
    auto exec = std::make_shared<SyntheticRigExecutable>(rig);

    const ConfigNode field = node.child("Field");
    if (field.valid()) {
        const ResourceId id{field.attr("resource_id")};
        if (id.empty()) {
            err = field.path() + ": Field needs resource_id";
            return ResourceInstance{};
        }
        rig->field_ = context.require<const FieldMap>(id, err);
        if (rig->field_ == nullptr) {
            return ResourceInstance{};
        }
    }

    const ConfigNode wheels = node.child("Wheels");
    if (wheels.valid()) {
        const ResourceId id{wheels.attr("resource_id")};
        if (id.empty()) {
            err = wheels.path() + ": Wheels needs resource_id";
            return ResourceInstance{};
        }
        rig->wheels_ = context.require<const WheelGeometryMap>(id, err);
        if (rig->wheels_ == nullptr) {
            return ResourceInstance{};
        }
        if (!wheels.requireDouble("counts_per_revolution", rig->counts_per_revolution_, err)) {
            return ResourceInstance{};
        }
        if (rig->counts_per_revolution_ <= 0.0) {
            err = wheels.path() + ": counts_per_revolution must be positive";
            return ResourceInstance{};
        }
    }

    const ConfigNode trajectory = node.child("Trajectory");
    {
        SyntheticRig::TrajectoryConfig& t = rig->trajectory_;
        double                          start_deg = 0.0;
        if (!trajectory.getDouble("center_x_m", 0.0, t.center_x_m, err) ||
            !trajectory.getDouble("center_y_m", 0.0, t.center_y_m, err) ||
            !trajectory.getDouble("radius_m", 0.5, t.radius_m, err) ||
            !trajectory.getDouble("period_s", 40.0, t.period_s, err) ||
            !trajectory.getDouble("start_deg", 0.0, start_deg, err) ||
            !trajectory.getDouble("hold_s", 0.0, t.hold_s, err)) {
            return ResourceInstance{};
        }
        t.facing    = trajectory.attr("facing", "center");
        t.start_rad = degToRad(start_deg);
        if (t.radius_m < 0.0 || t.period_s <= 0.0 || t.hold_s < 0.0) {
            err = trajectory.path() + ": radius_m and hold_s cannot be negative and period_s must be positive";
            return ResourceInstance{};
        }
        if (t.facing != "center" && t.facing != "tangent") {
            err = trajectory.path() + ": facing must be center or tangent";
            return ResourceInstance{};
        }
    }

    bool ok = true;
    node.forEach("Displace", [&](const ConfigNode& d) {
        if (!ok) {
            return;
        }
        std::string id;
        Pose2D      delta;
        double      dyaw_deg = 0.0;
        if (!d.requireAttr("landmark_id", id, err) ||
            !d.getDouble("dx_m", 0.0, delta.x_m, err) ||
            !d.getDouble("dy_m", 0.0, delta.y_m, err) ||
            !d.getDouble("dyaw_deg", 0.0, dyaw_deg, err)) {
            ok = false;
            return;
        }
        if (rig->field_ == nullptr || rig->field_->find(FieldObjectId{id}) == nullptr) {
            err = d.path() + ": landmark " + id + " is not in the Field resource";
            ok  = false;
            return;
        }
        delta.heading_rad          = degToRad(dyaw_deg);
        rig->displacements_[id]    = delta;
    });
    if (!ok) {
        return ResourceInstance{};
    }

    const ConfigNode telemetry = node.child("Telemetry");
    {
        long offset = 5000;
        if (!telemetry.getDouble("tick_hz", 50.0, rig->tick_hz_, err) ||
            !telemetry.getInt("device_offset_ms", 5000, offset, err) ||
            !telemetry.getDouble("gyro_bias_mdps", 0.0, rig->gyro_bias_mdps_, err)) {
            return ResourceInstance{};
        }
        rig->device_offset_ms_ = offset;
        if (rig->tick_hz_ <= 0.0) {
            err = telemetry.path() + ": tick_hz must be positive";
            return ResourceInstance{};
        }
    }

    const ConfigNode attitude = node.child("Attitude");
    if (attitude.valid()) {
        const std::string mode = attitude.attr("mode", "unavailable");
        if (mode != "measured" && mode != "unavailable") {
            err = attitude.path() + ": mode must be measured or unavailable";
            return ResourceInstance{};
        }
        double rock_deg = 0.0;
        if (!attitude.getDouble("rock_deg", 0.0, rock_deg, err) ||
            !attitude.getDouble("period_s", 2.5, rig->attitude_.period_s, err)) {
            return ResourceInstance{};
        }
        rig->attitude_.measured = mode == "measured";
        rig->attitude_.rock_rad = degToRad(rock_deg);
    }

    const ConfigNode camera = node.child("Camera");
    if (camera.valid()) {
        SyntheticRig::CameraConfig& c = rig->camera_;
        std::string                 frame_raw, frames_id_raw;
        long                        w = 0, h = 0, latency = 30, background = 110;
        auto                        K = std::make_shared<CameraIntrinsics>();
        if (!camera.requireAttr("frame_id", frame_raw, err) ||
            !camera.requireAttr("robot_frames_resource_id", frames_id_raw, err) ||
            !camera.requireInt("width_px", w, err) || !camera.requireInt("height_px", h, err) ||
            !camera.requireDouble("fx_px", K->fx_px, err) ||
            !camera.requireDouble("fy_px", K->fy_px, err) ||
            !camera.requireDouble("cx_px", K->cx_px, err) ||
            !camera.requireDouble("cy_px", K->cy_px, err) ||
            !camera.getDouble("k1", 0.0, K->k1, err) || !camera.getDouble("k2", 0.0, K->k2, err) ||
            !camera.getDouble("p1", 0.0, K->p1, err) || !camera.getDouble("p2", 0.0, K->p2, err) ||
            !camera.getDouble("k3", 0.0, K->k3, err) ||
            !camera.getDouble("frame_rate_hz", 10.0, c.frame_rate_hz, err) ||
            !camera.getInt("latency_ms", 30, latency, err) ||
            !camera.getInt("background", 110, background, err)) {
            return ResourceInstance{};
        }
        if (w <= 0 || h <= 0 || K->fx_px <= 0.0 || K->fy_px <= 0.0 || c.frame_rate_hz <= 0.0 ||
            latency < 0 || background < 0 || background > 255) {
            err = camera.path() + ": camera geometry, rate, latency and background out of range";
            return ResourceInstance{};
        }
        if (rig->field_ == nullptr) {
            err = camera.path() + ": a Camera needs a <Field resource_id=.../> to look at";
            return ResourceInstance{};
        }
        rig->frames_ = context.require<const RobotFrameMap>(ResourceId{frames_id_raw}, err);
        if (rig->frames_ == nullptr) {
            return ResourceInstance{};
        }
        const Transform3* T_robot_camera = rig->frames_->find(FrameId{frame_raw});
        if (T_robot_camera == nullptr) {
            err = camera.path() + ": frame " + frame_raw + " is not in resource " + frames_id_raw;
            return ResourceInstance{};
        }
        rig->T_robot_camera_     = *T_robot_camera;
        K->model                 = "brown_conrady";
        K->calibrated_width_px   = static_cast<int>(w);
        K->calibrated_height_px  = static_cast<int>(h);
        c.configured             = true;
        c.frame_id               = FrameId{frame_raw};
        c.intrinsics             = K;
        c.latency_ms             = latency;
        c.background             = static_cast<uint8_t>(background);
    }

    ResourceExecutable executable;
    node.forEach("Output", [&](const ConfigNode& o) {
        if (!ok) {
            return;
        }
        const OutputId id{o.attr("id")};
        if (id.empty()) {
            err = o.path() + ": Output needs id";
            ok  = false;
            return;
        }
        for (const ResourceOutputDecl& seen : executable.outputs) {
            if (seen.id == id) {
                err = o.path() + ": duplicate Output id " + id.value;
                ok  = false;
                return;
            }
        }
        const std::string wheel_id = o.attr("wheel_id");
        const std::string channel  = o.attr("channel");
        if (!wheel_id.empty()) {
            if (rig->wheels_ == nullptr) {
                err = o.path() + ": encoder outputs need <Wheels resource_id=.../>";
                ok  = false;
                return;
            }
            const WheelDecl* decl = rig->wheels_->find(wheel_id);
            if (decl == nullptr) {
                err = o.path() + ": wheel " + wheel_id + " is not declared";
                ok  = false;
                return;
            }
            EncoderOutput e;
            e.id    = id;
            e.wheel = decl;
            const double angle = degToRad(decl->measurement_angle_deg);
            e.ux    = std::cos(angle);
            e.uy    = std::sin(angle);
            e.k_m   = decl->position_x_m * e.uy - decl->position_y_m * e.ux;
            e.sign  = decl->direction_positive ? 1.0 : -1.0;
            exec->encoders_.push_back(e);
            executable.outputs.push_back(ResourceOutputDecl{
                id, PayloadDescriptor::of<PicoEncoderCounts>(payload_names::kPicoEncoderCounts)});
        } else if (channel == "imu") {
            exec->imu_ = id;
            executable.outputs.push_back(
                ResourceOutputDecl{id, PayloadDescriptor::of<PicoGyroRate>(payload_names::kPicoGyroRate)});
        } else if (channel == "attitude") {
            exec->attitude_ = id;
            executable.outputs.push_back(ResourceOutputDecl{
                id, PayloadDescriptor::of<AttitudeSample>(payload_names::kAttitudeSample)});
        } else if (channel == "camera") {
            if (!rig->camera_.configured) {
                err = o.path() + ": camera output needs a <Camera .../> element";
                ok  = false;
                return;
            }
            exec->camera_ = id;
            executable.outputs.push_back(ResourceOutputDecl{
                id, PayloadDescriptor::of<CameraFramePayload>(payload_names::kCameraFrame)});
        } else {
            err = o.path() + ": Output needs wheel_id or channel imu|attitude|camera";
            ok  = false;
            return;
        }
    });
    if (!ok) {
        return ResourceInstance{};
    }
    if (executable.outputs.empty()) {
        err = node.path() + ": synthetic_rig declares no outputs";
        return ResourceInstance{};
    }

    executable.execute = [exec](const ExecutionContext& context) { return exec->execute(context); };
    executable.reset   = [exec] { exec->reset(); };

    auto instance = ResourceInstance::asContract<const SyntheticRig>(rig);
    instance.setExecutable(std::move(executable));
    return instance;
}

} // namespace navigatr
