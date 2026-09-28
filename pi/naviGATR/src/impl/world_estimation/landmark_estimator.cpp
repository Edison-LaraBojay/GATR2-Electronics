// landmark_estimator.cpp

#include "impl/world_estimation/landmark_estimator.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "math/angles.h"

namespace navigatr
{

std::optional<LandmarkEstimator> LandmarkEstimator::create(const ConfigNode& node,
                                                           std::shared_ptr<const FieldMap> field,
                                                           std::string& err) {
    LandmarkEstimator estimator;
    estimator.field_ = std::move(field);

    std::string commit;
    if (!node.requireAttr("commit", commit, err)) {
        return std::nullopt;
    }
    if (commit == "always") {
        estimator.commit_ = CommitPolicy::kAlways;
    } else if (commit == "never") {
        estimator.commit_ = CommitPolicy::kNever;
    } else {
        err = node.path() + ": commit must be always or never, not \"" + commit + "\"";
        return std::nullopt;
    }
    if (!node.getDouble("blend", 1.0, estimator.blend_, err)) {
        return std::nullopt;
    }
    if (estimator.blend_ < 0.0 || estimator.blend_ > 1.0) {
        err = node.path() + ": blend must be within 0..1";
        return std::nullopt;
    }
    return estimator;
}

FieldState LandmarkEstimator::run(const FieldState&                 previous,
                                  const FieldObjectPoseEvidenceSet* evidence,
                                  const RobotState& robot, MonotonicTime now) const {
    FieldState field = previous;

    // Seed every mapped landmark, then keep observed entries coherent with
    // the current coordinate context.
    for (const auto& decl : field_->landmarks) {
        if (field.objects.find(decl.id) != field.objects.end()) {
            continue;
        }
        FieldObjectState obj;
        obj.pose.frame      = FrameId{"field"};
        obj.pose.pose       = decl.nominal;
        obj.pose.measuredAt = now;
        obj.confidence      = 0.5;
        obj.valid           = true;
        obj.source          = EstimateSource::kFieldMap;
        field.objects.emplace(decl.id, obj);
    }
    for (auto& kv : field.objects) {
        FieldObjectState& obj = kv.second;
        obj.observed          = false;
        if (obj.source != EstimateSource::kObserved) {
            continue;
        }
        if (obj.odometry_epoch != robot.odometry_epoch) {
            // the frame the estimate was measured in no longer exists: back
            // to the nominal definition, never a silently stale pose
            const LandmarkDecl* decl = field_->find(kv.first);
            obj.source               = EstimateSource::kFieldMap;
            obj.confidence           = 0.5;
            obj.valid                = decl != nullptr;
            if (decl != nullptr) {
                obj.pose.pose = decl->nominal;
            }
            obj.pose.measuredAt = now;
            continue;
        }
        if (obj.anchor_revision != robot.anchor_revision) {
            // a re-anchor re-expresses the same measurement consistently
            obj.pose.pose       = compose(robot.field_from_odom, obj.T_odom_object);
            obj.anchor_revision = robot.anchor_revision;
        }
    }

    if (commit_ != CommitPolicy::kAlways || evidence == nullptr) {
        return field;
    }

    // Deterministic fusion in the odometry frame: several accepted
    // measurements of one object in one invocation combine by
    // confidence-weighted planar mean and circular heading mean, so
    // container or detector iteration order can never pick the result.
    // Non-finite evidence and evidence from another odometry epoch never
    // reach state.
    struct Accumulated {
        double        wx = 0.0, wy = 0.0, wsin = 0.0, wcos = 0.0;
        double        weight = 0.0, wconf = 0.0;
        MonotonicTime newest;
        std::string   source, feature;
        uint32_t      sequence = 0;
    };
    std::map<FieldObjectId, Accumulated> merged;
    for (const auto& entry : evidence->entries) {
        if (entry.frame != FrameId{"odometry"}) {
            continue;   // this estimator folds odometry-frame evidence
        }
        if (entry.odometry_epoch != robot.odometry_epoch) {
            continue;
        }
        const auto& p = entry.T_frame_object;
        if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) || !std::isfinite(p.heading_rad) ||
            !std::isfinite(entry.confidence) || entry.confidence < 0.0) {
            continue;
        }
        const auto w = std::max(entry.confidence, 1e-6);
        auto&      a = merged[entry.object];
        a.wx += w * p.x_m;
        a.wy += w * p.y_m;
        a.wsin += w * std::sin(p.heading_rad);
        a.wcos += w * std::cos(p.heading_rad);
        a.weight += w;
        a.wconf += w * entry.confidence;
        if (!a.newest.isSet() || entry.measuredAt > a.newest) {
            a.newest   = entry.measuredAt;
            a.source   = entry.source.value;
            a.feature  = entry.feature_instance;
            a.sequence = entry.source_sequence;
        }
    }
    for (const auto& kv : merged) {
        const auto&  a = kv.second;
        const Pose2D fused{a.wx / a.weight, a.wy / a.weight, std::atan2(a.wsin, a.wcos)};
        auto&        obj = field.objects[kv.first];
        if (!obj.valid) {
            obj.T_odom_object = fused;
        } else {
            if (obj.source != EstimateSource::kObserved) {
                // first evidence blends away from the seeded nominal,
                // expressed in the same frame as the measurement
                obj.T_odom_object = compose(inverse(robot.field_from_odom), obj.pose.pose);
            }
            obj.T_odom_object.x_m += (fused.x_m - obj.T_odom_object.x_m) * blend_;
            obj.T_odom_object.y_m += (fused.y_m - obj.T_odom_object.y_m) * blend_;
            obj.T_odom_object.heading_rad = wrapAngle(
                obj.T_odom_object.heading_rad +
                wrapAngle(fused.heading_rad - obj.T_odom_object.heading_rad) * blend_);
        }
        obj.pose.frame           = FrameId{"field"};
        obj.pose.pose            = compose(robot.field_from_odom, obj.T_odom_object);
        obj.pose.measuredAt      = a.newest;
        obj.lastObservedAt       = a.newest;
        obj.valid                = true;
        obj.observed             = true;
        obj.source               = EstimateSource::kObserved;
        obj.confidence           = a.wconf / a.weight;
        obj.odometry_epoch       = robot.odometry_epoch;
        obj.anchor_revision      = robot.anchor_revision;
        obj.last_source          = a.source;
        obj.last_feature         = a.feature;
        obj.last_source_sequence = a.sequence;
    }
    return field;
}

} // namespace navigatr
